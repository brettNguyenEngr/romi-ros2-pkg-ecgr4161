import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int16MultiArray
import math

class PINode(Node):
    def __init__(self):
        super().__init__('pi_controller')
        
        self.create_subscription(Float32MultiArray, '/target_speeds', self.target_callback, 10)
        self.create_subscription(Int32MultiArray, '/encoder_ticks', self.encoder_callback, 10)
        self.pwm_pub = self.create_publisher(Int16MultiArray, '/wheel_pwm', 10)
        
        # --- TUNING PARAMETERS ---
        # You will need to tune these values!
        self.Kp = 400.0  
        self.Ki = 100.0   
        
        # --- ROBOT CONSTANTS ---
        self.wheel_circumference = math.pi * 0.07 # 70mm diameter
        gear_ratio = 120.0
        motor_counts_per_rev = 12.0
        self.ticks_per_rev_wheel = motor_counts_per_rev * gear_ratio # 1440 ticks
        
        # --- STATE VARIABLES ---
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        
        self.integral_left = 0.0
        self.integral_right = 0.0
        
        self.last_time = self.get_clock().now()
        self.get_logger().info("PI Controller Initialized.")

    def target_callback(self, msg):
        self.target_left_mps = msg.data[0]
        self.target_right_mps = msg.data[1]

    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Convert to seconds
        self.last_time = current_time
        
        current_left_ticks = msg.data[0]
        current_right_ticks = msg.data[1]
        
        # Need two points of data to calculate a velocity
        if self.prev_left_ticks is None:
            self.prev_left_ticks = current_left_ticks
            self.prev_right_ticks = current_right_ticks
            return 
            
        # 1. Calculate Delta Ticks (handling 16-bit integer wrap from AStar)
        delta_left = self.calculate_delta(current_left_ticks, self.prev_left_ticks)
        delta_right = self.calculate_delta(current_right_ticks, self.prev_right_ticks)
        
        self.prev_left_ticks = current_left_ticks
        self.prev_right_ticks = current_right_ticks
        
        # 2. Convert raw ticks into Actual Meters per Second
        actual_left_mps = (delta_left / self.ticks_per_rev_wheel) * self.wheel_circumference / dt
        actual_right_mps = (delta_right / self.ticks_per_rev_wheel) * self.wheel_circumference / dt
        
        # 3. Calculate Error
        error_left = self.target_left_mps - actual_left_mps
        error_right = self.target_right_mps - actual_right_mps
        
        # 4. PI Algorithm & Clamping
        pwm_left = self.calculate_pi_pwm(error_left, dt, 'left')
        pwm_right = self.calculate_pi_pwm(error_right, dt, 'right')
            
        # 5. Publish to I2C Node
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(pwm_left), int(pwm_right)]
        self.pwm_pub.publish(pwm_msg)
        
    def calculate_pi_pwm(self, error, dt, side):
        # Apply Integral math
        if side == 'left':
            self.integral_left += error * dt
            integral = self.integral_left
        else:
            self.integral_right += error * dt
            integral = self.integral_right
            
        pwm = (self.Kp * error) + (self.Ki * integral)
        
        # Clamp to 300 and apply Anti-Windup
        if pwm > 300:
            pwm = 300.0
            if side == 'left': self.integral_left -= error * dt
            else: self.integral_right -= error * dt
        elif pwm < -300:
            pwm = -300.0
            if side == 'left': self.integral_left -= error * dt
            else: self.integral_right -= error * dt
            
        return pwm

    def calculate_delta(self, current, prev):
        # AStar returns 16-bit signed ints, so we must handle hardware rollovers
        delta = current - prev
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

def main(args=None):
    rclpy.init(args=args)
    node = PINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()