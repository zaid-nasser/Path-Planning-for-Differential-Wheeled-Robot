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


class planner(Node):

    def __init__(self):
        super().__init__('astar_planner')
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
    
    def heuristic(self,goal: tuple[int, int], current_state: tuple[tuple[int, int], int]) -> float:
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

    def astar(self,binary_map, start, goal):
        height, width = binary_map.shape
        visited = set()

        start_state = (start, 0)  # position, angle
        goal_pos = goal

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(goal_pos, start_state), 0, start_state, []))

        while open_set:
            f, g, current_state, path = heapq.heappop(open_set)
            current_pos, current_angle = current_state

            if current_pos == goal_pos:
                return path + [current_pos]

            if current_state in visited:
                continue
            visited.add(current_state)

            # action space
            actions = [
                (0,  5),   # move forward
                (180, 5),  # move backward
                (-45, 5),  # turn left
                (45,  5),  # turn right
            ]

            for action, cost in actions:
                if action in [-45, 45]:  # in place turn
                    new_angle = (current_angle + action) % 360
                    new_state = (current_pos, new_angle)
                    if new_state not in visited:
                        heapq.heappush(open_set, (g + cost + self.heuristic(goal_pos, new_state), g + cost, new_state, path))

                else:  # move forward/backward
                    move_angle = (current_angle + action) % 360
                    dx, dy = self.direction_vector(move_angle)
                    new_x, new_y = current_pos[0] + dx, current_pos[1] + dy

                    if 0 <= new_x < width and 0 <= new_y < height and binary_map[new_y, new_x] == 0:
                        move_cost = 7 if move_angle % 90 != 0 else 5 # diagonal moves costs more
                        new_state = ((new_x, new_y), current_angle)
                        if new_state not in visited:
                            heapq.heappush(open_set, (g + move_cost + self.heuristic(goal_pos, new_state), g + move_cost, new_state, path + [current_pos]))

        return None
    

def main():
    rclpy.init()
    node = planner()
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