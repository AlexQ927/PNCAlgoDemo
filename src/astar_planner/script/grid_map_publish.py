import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class GridMapPublisher(Node):
    def __init__(self):
        super().__init__('grid_map_publisher')
        self.grid_publisher = self.create_publisher(OccupancyGrid, 'grid_map', 10)
        self.start_publisher = self.create_publisher(PoseStamped, 'start_pose', 10)
        self.end_publisher = self.create_publisher(PoseStamped, 'end_pose', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.grid = OccupancyGrid()
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid.header.frame_id = 'map_' + str(self.count)
        self.grid.info.height = 1000            # y轴
        self.grid.info.width = 500             # x轴
        self.grid.info.resolution = 0.1
        self.grid.info.origin.position.x = -1.0
        self.grid.info.origin.position.y = -1.0
        self.grid.data = [0] * (self.grid.info.height * self.grid.info.width)
        self.set_obstacle(100, 200, 50, 100)
        self.set_obstacle(200, 700, 60, 200)
        self.set_obstacle(350, 500, 100, 400)
        self.set_obstacle(100, 300, 200, 20)
        self.grid_publisher.publish(self.grid)
        start_pose = PoseStamped()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.header.frame_id = 'start_' + str(self.count)
        start_pose.pose.position.x = 10 * self.grid.info.resolution
        start_pose.pose.position.y = 20 * self.grid.info.resolution
        self.start_publisher.publish(start_pose)
        end_pose = PoseStamped()
        end_pose.header.stamp = self.get_clock().now().to_msg()
        end_pose.header.frame_id = 'end_' + str(self.count)
        end_pose.pose.position.x = 480 * self.grid.info.resolution
        end_pose.pose.position.y = 960 * self.grid.info.resolution
        self.end_publisher.publish(end_pose)
        self.count += 1
        self.get_logger().info('Published grid' + self.grid.header.frame_id)
        self.get_logger().info('Published start: ("%lf", "%lf")' \
                               % (start_pose.pose.position.x, start_pose.pose.position.y))
        self.get_logger().info('Published end: ("%lf", "%lf")' \
                        % (end_pose.pose.position.x, end_pose.pose.position.y))
        

    def set_obstacle(self, center_x, center_y, width, height):
        left = center_x - int(width / 2)
        right = left + width
        low = center_y - int(height / 2)
        high = low + height
        if left < 0 or right >= self.grid.info.width:
            return
        if low < 0 or high >= self.grid.info.height:
            return
        # print("left: " + str(left))
        # print("right: " + str(right))
        # print("low: " + str(low))
        # print("high: " + str(high))
        for i in range(low, high):
            for j in range(left, right):
                self.grid.data[i * self.grid.info.width + j] = 100

def main(args = None):
    rclpy.init(args=args)
    grid_map_publisher = GridMapPublisher()
    rclpy.spin(grid_map_publisher)
    grid_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()