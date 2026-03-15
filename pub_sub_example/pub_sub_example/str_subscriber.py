import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringSubscriber(Node):
    def __init__(self):
        super().__init__('str_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription # Prevents unused variable warning

    # This function executes whenever a message is received
    def listener_callback(self, msg):
        self.get_logger().info(f'Pi heard: "{msg.data}"')

def main():
    print('Hi from pub_sub_example.')


if __name__ == '__main__':
    main()
