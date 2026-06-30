#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class RoverOdometry(Node):
    def __init__(self):
        super().__init__('rover_odometry')

        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.44)
        self.declare_parameter('encoder_cpr', 4000.0) # Counts per revolution (LPR * 4)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.CPR = self.get_parameter('encoder_cpr').value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_ticks_left = None
        self.last_ticks_right = None
        self.last_time = self.get_clock().now()
        self.callback_count = 0

        # Subscribers & Publishers
        self.sub_ticks = self.create_subscription(
            Point32, '/wheel_ticks', self.ticks_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Rover Odometry Node Started.")
        self.get_logger().info(f"Using CPR: {self.CPR}, Radius: {self.R}m, Separation: {self.L}m")

    def ticks_callback(self, msg: Point32):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        ticks_left = msg.x
        ticks_right = msg.y

        # Initialize on first message
        if self.last_ticks_left is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.last_time = current_time
            return

        # Calculate deltas
        d_ticks_left = ticks_left - self.last_ticks_left
        d_ticks_right = ticks_right - self.last_ticks_right

        # Handle massive jumps (e.g., ESP32 reset)
        if abs(d_ticks_left) > 100000 or abs(d_ticks_right) > 100000:
            self.get_logger().warn("Massive tick jump detected. Resetting tick baseline.")
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.last_time = current_time
            return

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right
        self.last_time = current_time

        # Convert ticks to radians
        d_rad_left = d_ticks_left * (2.0 * math.pi / self.CPR)
        d_rad_right = d_ticks_right * (2.0 * math.pi / self.CPR)

        # Convert to distance traveled by each wheel
        d_left = d_rad_left * self.R
        d_right = d_rad_right * self.R

        # Kinematics
        d_s = (d_right + d_left) / 2.0
        d_th = (d_right - d_left) / self.L

        if dt > 0:
            v = d_s / dt
            w = d_th / dt
        else:
            v = 0.0
            w = 0.0

        # Update pose
        # Using Runge-Kutta 2nd order integration
        self.x += d_s * math.cos(self.th + (d_th / 2.0))
        self.y += d_s * math.sin(self.th + (d_th / 2.0))
        self.th += d_th

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = self.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.pub_odom.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publish Joint States (for Foxglove animation)
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [
            ticks_left * (2.0 * math.pi / self.CPR),
            ticks_right * (2.0 * math.pi / self.CPR)
        ]
        self.pub_joints.publish(js)

        # Periodic logging for debugging
        self.callback_count += 1
        if self.callback_count % 50 == 0:
            self.get_logger().info(
                f"Ticks: L={ticks_left:.1f}, R={ticks_right:.1f} | "
                f"Deltas: dL={d_ticks_left:.1f}, dR={d_ticks_right:.1f} | "
                f"Pose: x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f}"
            )

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = RoverOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
