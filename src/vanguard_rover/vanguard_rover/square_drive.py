import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SquareDriveNode(Node):
    def __init__(self):
        super().__init__('square_drive')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        
        # States: 0 = DRIVE, 1 = TURN, 2 = STOP
        self.state = 0
        self.sides_completed = 0
        self.target_distance = 2.0  # Drive 2 meters per side
        self.target_angle = math.pi / 2  # Turn 90 degrees (1.57 rad)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Square Drive Node initialized using /odom.')

    def odom_callback(self, msg):
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Initialize start poses on first reading
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw

    def control_loop(self):
        if self.start_x is None:
            return

        twist = Twist()

        if self.sides_completed >= 4:
            self.get_logger().info('Completed square maneuver! Stopping rover.')
            self.cmd_pub.publish(Twist())
            self.destroy_timer(self.timer)
            return

        if self.state == 0:  # DRIVE FORWARD
            dist = math.hypot(self.x - self.start_x, self.y - self.start_y)
            if dist < self.target_distance:
                twist.linear.x = 0.3  # Forward speed m/s
            else:
                twist.linear.x = 0.0
                self.state = 1  # Switch to TURN
                self.start_yaw = self.yaw
                self.get_logger().info(f'Side {self.sides_completed + 1} done. Turning...')

        elif self.state == 1:  # TURN 90 DEGREES
            angle_diff = self.normalize_angle(self.yaw - self.start_yaw)
            if abs(angle_diff) < self.target_angle:
                twist.angular.z = 0.4  # Turning angular speed rad/s
            else:
                twist.angular.z = 0.0
                self.state = 0  # Switch back to DRIVE
                self.sides_completed += 1
                self.start_x = self.x
                self.start_y = self.y
                self.get_logger().info(f'Turn {self.sides_completed} done. Driving next side...')

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
