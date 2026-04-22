import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RoombaNode(Node):
    def __init__(self):
        super().__init__('roomba_node')
        
        # --- ROS 2 INTERFACES ---
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # --- CONFIGURABLE PARAMETERS ---
        self.declare_parameter('forward_speed', 0.20)  # m/s
        self.declare_parameter('turning_speed', 1.0)   # rad/s
        self.declare_parameter('safe_dist', 0.5)       # meters (threshold to stop)
        
        self.get_logger().info("Roomba Node Template Initialized. Waiting for Lidar data...")

    def lidar_callback(self, msg):
        """
        TODO: Implement the Reactive Obstacle Avoidance Logic
        
        Game Plan:
        1. Slice the 'msg.ranges' array to isolate the front of the robot. 
           (Note: In many lidars, 0 degrees is the front. You may need to 
           combine slices from the beginning and end of the array).
        2. Filter the data. Real lidars return 0.0 or 'inf' for failed 
           readings. Remove these so they don't count as "0 meters away."
        3. Find the minimum distance (min_dist) in your filtered front arc.
        4. Compare min_dist to your 'safe_dist' parameter.
        5. Publish a Twist message: 
           - If the path is clear: Drive forward.
           - If an obstacle is detected: Stop and spin until it's clear.
        """

        # =========================================================================
        # TODO: YOUR CODE HERE
        # =========================================================================
        
        move_cmd = Twist()
        
        # 1. & 2. Slicing and Filtering
        # front_arc = ...
        
        # 3. Find Closest Point
        # min_dist = ...
        
        # 4. & 5. Decision Making and Publishing
        # if min_dist < safe_threshold:
        #     ...
        # else:
        #     ...

        # self.publisher.publish(move_cmd)
        
        # Optional: Add a logger to help you debug what the robot sees
        # self.get_logger().info(f"Min Dist: {min_dist:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = RoombaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Publish a stop command before shutting down
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()