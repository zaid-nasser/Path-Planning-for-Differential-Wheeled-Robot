import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
import heapq
from robot_interface.msg import Path2D
from vision_msgs.msg import Point2D
from ament_index_python.packages import get_package_share_directory
import os


class potential_field_planner(Node):

    def __init__(self):
        super().__init__('potential_field_planner')
        self.path_publisher = self.create_publisher(Path2D, 'path', 10)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)

        self.goal_x_param = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y_param = self.get_parameter('goal_y').get_parameter_value().double_value


    def read_map(self, path='binary_map.txt'):
        m = []
        with open(path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                m.append([int(x) for x in line.strip().split()])
        m = np.array(m)
        
        m = np.flipud(m)
        return m

    
    def draw_map(self,graph, path):
        plt.imshow(graph, cmap='binary', interpolation='none')
        if path:
            x_vals, y_vals = zip(*path)
            plt.plot(x_vals, y_vals, color='red', linewidth=1)
        plt.axis('off')  
        plt.show()


    def direction_vector(self,theta_degrees):
        rad = math.radians(theta_degrees)
        dx = round(math.cos(rad))
        dy = round(math.sin(rad))
        return dx, dy
    
    def heuristic(self,goal, current_state) -> float:
        current_pos, current_angle = current_state
        goal_x, goal_y = goal
        cur_x, cur_y = current_pos

        dx = goal_x - cur_x
        dy = goal_y - cur_y

        goal_angle = (np.degrees(np.arctan2(dy, dx)) + 360) % 360

        angle_diff = abs(current_angle - goal_angle) % 360
        turns = min(angle_diff // 45, (360 - angle_diff) // 45)
        turn_cost = turns * 5

        move_cost = (max(abs(dx), abs(dy)) + (np.sqrt(2) - 1) * min(abs(dx), abs(dy))) * 5

        return turn_cost + move_cost
    
    def gz2np(self,gz_x,gz_y):
        np_x = (gz_x+10) * 4
        np_y = (-gz_y+10) * 4
        return np_x, np_y
    

    def np2gz(self,np_x,np_y):
        gz_x = (np_x*0.25) - 10
        gz_y = (-np_y*0.25) + 10
        return gz_x, gz_y

    def compute_potential_field(self, binary_map, start, goal,
                            k_att=1.0, k_rep=100.0, repulsive_radius=10):
        """
        Compute the combined attractive and repulsive potential for each cell.
        """
        h, w = binary_map.shape
        U = np.zeros((h, w), dtype=float)

        # Precompute attractive potential
        for y in range(h):
            for x in range(w):
                U_att = 0.5 * k_att * ((x - goal[0])**2 + (y - goal[1])**2)
                U[y, x] = U_att

        # Add repulsive potential from obstacles
        obs_indices = np.argwhere(binary_map == 1)
        for oy, ox in obs_indices:
            for y in range(max(0, oy - repulsive_radius), min(h, oy + repulsive_radius)):
                for x in range(max(0, ox - repulsive_radius), min(w, ox + repulsive_radius)):
                    d = np.hypot(x - ox, y - oy)
                    if d <= repulsive_radius and d > 0:
                        U_rep = 0.5 * k_rep * (1.0 / d - 1.0 / repulsive_radius)**2
                        U[y, x] += U_rep
        return U
    
    def simulate_path(self, U, binary_map, start_pos, start_angle, dtheta, move_flag, depth=3):
        """Simulate a short rollout to evaluate cumulative potential."""
        pos = start_pos
        angle = (start_angle + dtheta) % 360
        total_potential = 0.0
        h, w = binary_map.shape

        for _ in range(depth):
            if move_flag:
                dx, dy = self.direction_vector(angle)
                nx, ny = pos[0] + dx, pos[1] + dy
                if 0 <= nx < w and 0 <= ny < h and binary_map[ny, nx] == 0:
                    pos = (nx, ny)
            total_potential += U[pos[1], pos[0]]
        return total_potential

    def potential_field_planner(self, binary_map, start, goal, max_iters=10000):
        """
        Follow negative gradient of potential field using discretized control.
        Lookahead used to choose smoother local path.
        Returns path of positions.
        """
        # Compute potential field
        U = self.compute_potential_field(binary_map, start, goal)

        pos = start
        angle = 0
        path = [pos]
        h, w = binary_map.shape

        action_set = [
            (0,  1),    # forward
            (180, 1),   # backward
            (-45, 0),   # turn left
            (45, 0),    # turn right
        ]


        for _ in range(max_iters):
            if pos == goal:
                break
            best_action = None
            best_value = float('inf')
            # Evaluate each action with lookahead
            for dtheta, move_flag in action_set:
                angle_try = (angle + dtheta) % 360
                dx, dy = self.direction_vector(angle_try) if move_flag else (0, 0)
                nx, ny = pos[0] + dx, pos[1] + dy

                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if move_flag and binary_map[ny, nx] == 1:
                    continue

                lookahead_val = self.simulate_path(U, binary_map, pos, angle, dtheta, move_flag)
                if lookahead_val < best_value:
                    best_value = lookahead_val
                    best_action = (angle_try, (nx, ny) if move_flag else pos)

            if best_action is None:
                print("Stuck in local minimum at", pos)
                break

            angle, pos = best_action
            path.append(pos)
        return path


def main():
    rclpy.init()
    node = potential_field_planner()
    node.get_clock().sleep_for(rclpy.duration.Duration(seconds=3.0))
    package_share = get_package_share_directory('robot_interface')
    map_path = os.path.join(package_share,'maps','binary_map.txt')
    binary_map = node.read_map(map_path)

    goal_x = node.get_parameter('goal_x').get_parameter_value().double_value
    goal_y = node.get_parameter('goal_y').get_parameter_value().double_value
    path = node.astar(binary_map = binary_map, start = node.gz2np(-8,1), goal = node.gz2np(goal_x,goal_y))

    if path is None:
        node.get_logger().info("The goal that you provided is an obstacle!!")
    path_msg = Path2D()
    path_to_be_send = []
    for element in path:
        point = Point2D()
        x, y = element
        x, y = node.np2gz(x,y)
        point.x, point.y = float(x), float(y)
        path_to_be_send.append(point)
    path_msg.points = path_to_be_send
    node.path_publisher.publish(path_msg)
    #node.draw_map(binary_map,path)

    rclpy.shutdown()


if __name__ == '__main__':
    main()