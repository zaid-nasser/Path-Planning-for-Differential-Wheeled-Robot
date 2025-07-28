import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
import numpy as np
from robot_interface.msg import Path2D
import matplotlib.pyplot as plt
import os
from ament_index_python.packages import get_package_share_directory


class advanced_controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.k1 = 9
        self.k2 = 5
        self.k3 = 3
        self.max_speed_linear = 3.0
        self.max_speed_angular = 2.0
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_supscriper = self.create_subscription(Path2D,'path',self.path_callback,10)
        self.act_path = []


    def filter_path(self,path):
        x, y = path
        running_index = 0
        window = 0
        considered_points = np.zeros(x.shape,dtype=bool)
        considered_points[0] = True
        while running_index <= len(considered_points) - 3:
            m1 = (y[running_index+1+window] - y[running_index]) / ((x[running_index+1+window] - x[running_index])+1e-5)
            m2 = (y[running_index+2+window] - y[running_index]) / ((x[running_index+2+window] - x[running_index])+1e-5)
            if abs(m1 -m2) > 0.01:
                considered_points[running_index+1+window] = 1
                running_index = running_index+1+window
                window = 0
                if running_index == len(considered_points) - 2:
                    considered_points[len(considered_points) -1] = True
                    break
            else:
                considered_points[running_index+1+window] = 0
                window +=1
                if (running_index+window) == len(considered_points) -2:
                    considered_points[len(considered_points) -1] = True
                    break
        filtered_path = (x[considered_points],y[considered_points])
        return filtered_path



    def draw_map(self,graph, path,actual_path):
        plt.imshow(graph, cmap='binary', interpolation='none')
        if path:
            x_vals, y_vals = zip(*path)
            x_vals_act, y_vals_act = zip(*actual_path)
            plt.plot(x_vals, y_vals, color='red', linewidth=2,label='Desired')
            plt.plot(x_vals_act, y_vals_act, color='green', linewidth=2, label='Actual')
            plt.legend()
        plt.axis('off')  
        plt.show()
    
    def path_callback (self,msg):
        x = []
        y = []
        for pose in msg.points:
            x.append(pose.x)
            y.append(pose.y)
        x = np.array(x)
        y = np.array(y)
        x,y = self.filter_path((x,y))
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        times = np.concatenate(([0], np.cumsum(distances)))
        times = times/ 0.7
        f_x = CubicSpline(times, x)
        f_y = CubicSpline(times,y)
        self.times_new = np.arange(start=times[0], stop=times[-1], step=0.05)
        self.x_d = f_x(self.times_new)
        self.vx_d = f_x(self.times_new,1)
        self.ax_d = f_x(self.times_new,2)
        self.y_d = f_y(self.times_new)
        self.vy_d = f_y(self.times_new,1)
        self.ay_d = f_y(self.times_new,2)
        self.current_goal_state = 0
        self.goal_x = self.x_d[0]
        self.goal_y = self.y_d[0]
        self.goal_vx = self.vx_d[0]
        self.goal_vy = self.vy_d[0]
        self.goal_ax = self.ax_d[0]
        self.goal_ay = self.ay_d[0]
        package_share = get_package_share_directory('robot_interface')
        map_path = os.path.join(package_share,'maps','binary_map2.txt')
        self.binary_map = self.read_map(map_path)
        self.path = []
        for x,y in zip(self.x_d,self.y_d):
            self.path.append(self.gz2np(x,y))
        self.timer = self.create_timer(0.05, self.on_timer)
        self.goal_reached = False


    def gz2np(self,gz_x,gz_y):
        np_x = (gz_x+10) * 4
        np_y = (-gz_y+10) * 4
        return np_x, np_y
    
    def read_map(self, path='binary_map.txt'):
        m = []
        with open(path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                m.append([int(x) for x in line.strip().split()])
        m = np.array(m)
        
        m = np.flipud(m)
        return m
    def draw_error(self,path,actual_path):
        if path:
            x_vals, y_vals = zip(*path)
            x_vals_act, y_vals_act = zip(*actual_path)
            e_x = np.array(x_vals) - np.array(x_vals_act)
            e_y = np.array(y_vals) - np.array(y_vals_act)
            plt.plot(self.times_new[10:], e_x[10:], color='red', linewidth=2,label='Error x')
            plt.plot(self.times_new[10:], e_y[10:], color='blue', linewidth=2, label='Error y')
            plt.legend()
            plt.xlabel('time (s)',fontsize=16)
            plt.ylabel('Error (m)',fontsize=16)
            plt.title('The Error Signal between the actual and planned path',fontsize=16,fontweight='bold')
        #plt.axis('off')  
        plt.show()

    def on_timer(self):
        from_frame_rel = 'odom'
        to_frame_rel = 'base_footprint'
        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
            return

        # self.get_logger().info(f"Current position: x={t.transform.translation.x}, y={t.transform.translation.y}")
        
        quaternion = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=False)
        self.act_path.append(self.gz2np(t.transform.translation.x,t.transform.translation.y))
        vel_command = Twist()
        vd_linear = np.sqrt(self.goal_vx**2 + self.goal_vy**2)
        theta_d = np.arctan2(self.goal_vy, self.goal_vx)
        omega_d = (self.goal_ay * self.goal_vx - self.goal_ax * self.goal_vy) / (self.goal_vx**2 + self.goal_vy**2 + 1e-6)
        e_x_w = self.goal_x - t.transform.translation.x
        e_y_w = self.goal_y - t.transform.translation.y
        e_x_r = np.cos(euler[2]) * e_x_w + np.sin(euler[2]) * e_y_w
        e_y_r = -np.sin(euler[2]) * e_x_w + np.cos(euler[2]) * e_y_w
        e_theta_r = (theta_d - euler[2] + np.pi) % (2 * np.pi) - np.pi
        vel_command.linear.x = vd_linear * np.cos(e_theta_r) + self.k1 * e_x_r
        vel_command.linear.x = np.clip(vel_command.linear.x, -self.max_speed_linear, self.max_speed_linear)
        vel_command.angular.z = omega_d + self.k2 * e_y_r + self.k3 * np.sin(e_theta_r)
        vel_command.angular.z = np.clip(vel_command.angular.z, -self.max_speed_angular, self.max_speed_angular)


        self.current_goal_state +=1
        if (self.current_goal_state >= len(self.x_d)):    
                vel_command.linear.x = 0.0
                vel_command.angular.z = 0.0
                if not self.goal_reached:
                    self.get_logger().info("Goal reached!")
                    self.goal_reached = True
                    # Uncomment this for plotting results
                    #self.draw_map(self.binary_map,self.path,self.act_path)
                    #self.draw_error(self.path,self.act_path)
        else:
            self.goal_x = self.x_d[self.current_goal_state]
            self.goal_y = self.y_d[self.current_goal_state]
            self.goal_vx = self.vx_d[self.current_goal_state]
            self.goal_vy = self.vy_d[self.current_goal_state]
            self.goal_ax = self.ax_d[self.current_goal_state]
            self.goal_ay = self.ay_d[self.current_goal_state]

        self.vel_publisher.publish(vel_command)



def main():
    rclpy.init()
    node = advanced_controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
