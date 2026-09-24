#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareDrive(Node):
    def __init__(self):
        super().__init__('square_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- Tunable parameters ----
        side_length = 2.0        # meters
        linear_speed = 0.5       # m/s
        angular_speed = 0.5      # rad/s
        turn_scale = 1.0         # tune this if the turn isn't exactly 90 degrees
        pause = 1.0              # seconds to let the rover settle

        drive_time = side_length / linear_speed                        # time = distance / speed
        turn_time = (math.pi / 2) / angular_speed * turn_scale         # time = angle / angular speed

        # ---- The "program": (linear.x, angular.z, duration) ----
        self.steps = []
        for _ in range(4):
            self.steps.append((linear_speed, 0.0, drive_time))   # drive one side
            self.steps.append((0.0, 0.0, pause))                 # stop and settle
            self.steps.append((0.0, angular_speed, turn_time))   # turn 90 degrees
            self.steps.append((0.0, 0.0, pause))                 # stop and settle

        self.index = 0
        self.step_start = None   # set on first timer tick so it works with sim time
        self.timer = self.create_timer(0.1, self.loop)   # 10 Hz
        self.get_logger().info('Square drive started')

    def loop(self):
        now = self.get_clock().now()
        if self.step_start is None:
            self.step_start = now

        # All steps done: stop and shut the timer off
        if self.index >= len(self.steps):
            self.pub.publish(Twist())
            self.get_logger().info('Square complete')
            self.timer.cancel()
            return

        linear, angular, duration = self.steps[self.index]
        elapsed = (now - self.step_start).nanoseconds / 1e9

        if elapsed >= duration:
            self.index += 1          # move to the next step
            self.step_start = now
            return

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # make sure the rover stops
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
