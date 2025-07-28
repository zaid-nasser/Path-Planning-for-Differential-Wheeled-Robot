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
import random

class Node:
    def __init__(self, state, parent=None):
        self.state = state # ((x, y), angle)
        self.parent = parent

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
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

    def distance(self, a, b):
        # Euclidean + small angle penalty
        (x1, y1), th1 = a
        (x2, y2), th2 = b
        d_pos = math.hypot(x2 - x1, y2 - y1)
        d_angle = min(abs(th1 - th2), 360 - abs(th1 - th2)) / 45
        return d_pos + d_angle

    def rrt_planner(self, binary_map, start, goal, max_iter=10000, step_actions=None, threshold=5):
        height, width = binary_map.shape
        start_state = (start, 0)
        goal_state = (goal, None)

        # default action set: move forward/backward or turn
        if step_actions is None:
            step_actions = [
                (0, 1),   # forward
                (180, 1), # backward
                (-45, 0), # turn left
                (45, 0)   # turn right
            ]

        tree = [Node(start_state)]

        def sample_state():
            # random free grid cell and random orientation
            while True:
                x = random.randrange(0, width)
                y = random.randrange(0, height)
                if binary_map[y, x] == 0:
                    th = random.choice([i * 45 for i in range(8)])
                    return ((x, y), th)

        for i in range(max_iter):
            rand_state = sample_state() if random.random() < 0.9 else (goal, random.choice([i*45 for i in range(8)]))

            # find nearest node
            nearest = min(tree, key=lambda node: self.distance(node.state, rand_state))
            cur_state = nearest.state

            # attempt extension: pick action that reduces distance
            best = None
            best_state = None
            best_dist = float('inf')
            for action, move in step_actions:
                if action in [-45, 45]:  # turn in place
                    new_angle = (cur_state[1] + action) % 360
                    new_state = (cur_state[0], new_angle)
                else:
                    move_angle = (cur_state[1] + action) % 360
                    dx, dy = self.direction_vector(move_angle)
                    new_xy = (cur_state[0][0] + dx*move, cur_state[0][1] + dy*move)
                    if 0 <= new_xy[0] < width and 0 <= new_xy[1] < height and binary_map[new_xy[1], new_xy[0]] == 0:
                        new_state = (new_xy, cur_state[1])
                    else:
                        continue
                d = self.distance(new_state, rand_state)
                if d < best_dist:
                    best_dist = d
                    best_state = new_state
            if best_state is None:
                continue

            new_node = Node(best_state, parent=nearest)
            tree.append(new_node)

            # check if close to goal
            if math.hypot(best_state[0][0]-goal[0], best_state[0][1]-goal[1]) < threshold:
                # attach goal
                goal_node = Node((goal, best_state[1]), parent=new_node)
                # reconstruct path
                path = []
                node = goal_node
                while node:
                    path.append(node.state[0])
                    node = node.parent
                return list(reversed(path))

        return None


def main():
    rclpy.init()
    node = RRTNode()
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