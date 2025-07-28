import os
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import numpy as np
import math

class MapToGraph(Node):
    def __init__(self):
        super().__init__('map_to_graph')

        # Path to world file
        world_path = os.path.join(
            os.getenv("HOME"),
            'ros2_ws', 'src', 'my_robot_bringup', 'worlds',
            'Obstacle_Course', 'new_map_obstacle.world'
        )
        
        # Map configuration
        map_bounds = (-10, 10, -10, 10)  # xmin, xmax, ymin, ymax
        resolution = 0.25  # meters per cell
        grid_width = int((map_bounds[1] - map_bounds[0]) / resolution)
        grid_height = int((map_bounds[3] - map_bounds[2]) / resolution)
        grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

        self.get_logger().info(f"Parsing: {world_path}")
        tree = ET.parse(world_path)
        root = tree.getroot()

        def world_to_grid(x, y):
            gx = int((x - map_bounds[0]) / resolution)
            gy = int((y - map_bounds[2]) / resolution)
            return gx, gy

        def parse_pose(pose_elem):
            # get full pose of an element
            vals = list(map(float, pose_elem.text.strip().split())) if pose_elem is not None else [0.0] * 6
            while len(vals) < 6:
                vals.append(0.0)
            x, y, z, roll, pitch, yaw = vals
            # Ignore z, roll, pitch for 2D projection
            return x, y, yaw

        def fill_rotated_box(grid, cx, cy, yaw, w, d):
            # Get 4 corners of box in world coordinates
            dx = w / 2
            dy = d / 2
            corners = [
                (+dx, +dy),
                (-dx, +dy),
                (-dx, -dy),
                (+dx, -dy)
            ]
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # Transform corners
            world_corners = []
            for ox, oy in corners:
                wx = cx + ox * cos_yaw - oy * sin_yaw
                wy = cy + ox * sin_yaw + oy * cos_yaw
                world_corners.append((wx, wy))

            # Bounding box in world coords
            xs, ys = zip(*world_corners)
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)

            # Convert to grid indices
            gx_min, gy_min = world_to_grid(x_min, y_min)
            gx_max, gy_max = world_to_grid(x_max, y_max)

            gx_min = max(0, gx_min)
            gy_min = max(0, gy_min)
            gx_max = min(grid_width - 1, gx_max)
            gy_max = min(grid_height - 1, gy_max)

            # Check each cell center within this bounding box
            for gy in range(gy_min, gy_max + 1):
                for gx in range(gx_min, gx_max + 1):
                    # Convert cell center to world coordinates
                    cell_x = map_bounds[0] + (gx + 0.5) * resolution
                    cell_y = map_bounds[2] + (gy + 0.5) * resolution

                    # Transform into box-local space
                    rel_x =  (cell_x - cx) * cos_yaw + (cell_y - cy) * sin_yaw
                    rel_y = -(cell_x - cx) * sin_yaw + (cell_y - cy) * cos_yaw

                    if abs(rel_x) <= w/2 and abs(rel_y) <= d/2:
                        grid[gy, gx] = 1

        def process_model(model, link_prefixes):
            count = 0
            model_x, model_y, model_yaw = parse_pose(model.find("pose"))

            for link in model.findall("link"):
                link_name = link.attrib.get("name", "")
                if not any(link_name.startswith(pfx) for pfx in link_prefixes):
                    continue

                # Link pose (local to model)
                lx, ly, lyaw = parse_pose(link.find("pose"))

                # World position + rotation
                cx = model_x + lx
                cy = model_y + ly
                yaw = model_yaw + lyaw

                size_elem = link.find(".//geometry/box/size")
                if size_elem is None:
                    continue
                size_vals = list(map(float, size_elem.text.strip().split()))
                w, d = size_vals[0], size_vals[1]

                fill_rotated_box(grid, cx, cy, yaw, w, d)
                count += 1
            return count

       
        walls_model = root.find(".//model[@name='map_obstacle']")
        wall_count = process_model(walls_model, link_prefixes=['Wall_']) if walls_model is not None else 0

        boxes_model = root.find(".//model[@name='boxes']")
        box_count = process_model(boxes_model, link_prefixes=['link_']) if boxes_model is not None else 0

        
        np.savetxt('/tmp/binary_map.txt', grid, fmt='%d')
        np.save('/tmp/binary_map.npy', grid)

        self.get_logger().info(f"Done: {wall_count} walls, {box_count} boxes mapped.")
        self.get_logger().info("Saved: /tmp/binary_map.txt and binary_map.npy")


def main(args=None):
    rclpy.init(args=args)
    node = MapToGraph()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    