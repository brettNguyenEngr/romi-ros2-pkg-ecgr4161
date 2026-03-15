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

def main():
    print('Hi from pub_sub_example.')


if __name__ == '__main__':
    main()
