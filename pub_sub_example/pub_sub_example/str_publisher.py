import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPublisher(Node):
    def __init__(self):
        super().__init__('str_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Laptop: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)                   # Initialize ROS2 middleware
    string_publisher = StringPublisher()
    rclpy.spin(string_publisher)            # Keep node alive
    string_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
