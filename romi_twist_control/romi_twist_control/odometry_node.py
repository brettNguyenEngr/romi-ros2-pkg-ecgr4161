import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- ROBOT CONSTANTS ---
        self.wheel_circumference = math.pi * 0.07  # 70mm diameter
        self.ticks_per_rev = 1440.0                # 12 cts/rev * 120 gear ratio
        self.wheel_base = 0.141                    # Distance between wheels (m)

        # --- STATE VARIABLES (Global Position) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading in radians

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_time = self.get_clock().now()

        # --- ROS 2 INTERFACES ---
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odometry State Estimator Initialized.")

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        current_left_ticks = msg.data[0]
        current_right_ticks = msg.data[1]

        # Initialize on first pass
        if self.prev_left_ticks is None:
            self.prev_left_ticks = current_left_ticks
            self.prev_right_ticks = current_right_ticks
            return

        # Calculate Delta Ticks
        delta_left_ticks = self.calculate_delta(current_left_ticks, self.prev_left_ticks)
        delta_right_ticks = self.calculate_delta(current_right_ticks, self.prev_right_ticks)

        self.prev_left_ticks = current_left_ticks
        self.prev_right_ticks = current_right_ticks

        # Convert Ticks to Distance (m)
        d_left = (delta_left_ticks / self.ticks_per_rev) * self.wheel_circumference
        d_right = (delta_right_ticks / self.ticks_per_rev) * self.wheel_circumference

        # Calculate Center Displacement and Heading Change
        delta_s = (d_right + d_left) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        # Update Global Position (Dead Reckoning)
        # Use (theta + delta_theta / 2) to approximate the arc of the turn
        self.x += delta_s * math.cos(self.theta + (delta_theta / 2.0))
        self.y += delta_s * math.sin(self.theta + (delta_theta / 2.0))
        self.theta += delta_theta

        # Calculate Instantaneous Velocities
        # (Prevent division by zero on the very first loop if dt is extremely small)
        v = delta_s / dt if dt > 0 else 0.0
        w = delta_theta / dt if dt > 0 else 0.0

        # Publish the Data
        self.publish_odometry(current_time, v, w)

    def publish_odometry(self, current_time, v, w):
        # Create Quaternion from Euler angle (theta)
        # Since robot only rotates on the Z axis (yaw), x and y are 0
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        # --- PUBLISH TRANSFORM (TF) ---
        # Tells tools like RViz where the robot is globally
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)

        # --- PUBLISH ODOMETRY MESSAGE ---
        # Provides exact coordinates and current speeds
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set Pose (Position & Orientation)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        # Set Twist (Instantaneous Velocity)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    def calculate_delta(self, current, prev):
        """ Handles the 16-bit integer rollover from the AStar board. """
        delta = current - prev
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()