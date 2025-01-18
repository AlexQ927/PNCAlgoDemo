import rclpy
from rclpy.node import Node

from astar_planner.msg import AstarPath

import numpy as np
import matplotlib.pyplot as plt

class PlotAstarPath(Node):
    def __init__(self):
        super().__init__('plot_map_subscriber')
        self.subscriber = self.create_subscription(
            AstarPath,
            'astar_path',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        start_pose = msg.start_pose
        end_pose = msg.end_pose
        grid_msg = msg.occupancy_grid
        path = msg.final_path
        grid_map_height = grid_msg.info.height
        grid_map_width = grid_msg.info.width
        grid_map_resolution = grid_msg.info.resolution
        grid_map_x0 = grid_msg.info.origin.position.x
        grid_map_y0 = grid_msg.info.origin.position.y
        grid_map_data = np.array(grid_msg.data, dtype=np.int8)
        cost_map = grid_map_data.reshape(grid_map_height, grid_map_width)
        start_x = round((start_pose.position.x - grid_map_x0) / grid_map_resolution)
        start_y = round((start_pose.position.y - grid_map_y0) / grid_map_resolution)
        end_x = round((end_pose.position.x - grid_map_x0) / grid_map_resolution)
        end_y = round((end_pose.position.y - grid_map_y0) / grid_map_resolution)
        px, py = [], []
        for p in path:
            px.append(round((p.position.x - grid_map_x0) / grid_map_resolution))
            py.append(round((p.position.y - grid_map_y0) / grid_map_resolution))
        print(len(path))
        print(len(px))
        print(len(py))
        # 可视化地图
        plt.clf()
        plt.grid(which='both', linestyle='--', linewidth=0.5, color='gray')
        plt.imshow(cost_map, cmap='gray_r', origin='lower')
        plt.plot(start_x, start_y, marker='o', markersize=5, color='red')
        plt.plot(end_x, end_y, marker='*', markersize=5, color='red')
        plt.plot(px, py, color='red')
        px.clear()
        py.clear()
        plt.title('Astar Path')
        plt.colorbar()
        plt.pause(0.01)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    plot_astar_path = PlotAstarPath()
    rclpy.spin(plot_astar_path)
    plot_astar_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()