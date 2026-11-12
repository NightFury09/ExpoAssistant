#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

def extrapolate(x, y, th, v, w, h):
    """Where the rover will be h seconds after its last measured pose, if it
    keeps its last measured speed (v m/s forward, w rad/s turning).

    The same midpoint rule the tick callback integrates with, so a prediction
    that turns out right is indistinguishable from a measurement.
    """
    th_mid = th + w * h / 2.0
    return (x + v * h * math.cos(th_mid),
            y + v * h * math.sin(th_mid),
            th + w * h)


class RoverOdometry(Node):
    def __init__(self):
        super().__init__('rover_odometry')

        # Parameters
        # wheel_radius is now the TRUE physical value: wheels measure 10 cm
        # diameter -> 0.05 m radius. The old 0.0257 was a fudge factor absorbing
        # a wrong encoder_cpr (4000 vs the real 1336 steps/motor-rev) while
        # /wheel_ticks was open-loop dead reckoning from commanded RPM. Two
        # compensating errors only agree at one speed, which is why odometry
        # broke as soon as the motors were made faster. Keep this physical and
        # put the tick scale in encoder_cpr, where it belongs.
        self.declare_parameter('wheel_radius', 0.05)
        # Recalibrated 2026-09-07 on REAL encoder feedback. The previous 0.4621
        # was fitted against the old open-loop odometry and is void.
        # Scripted spin: odometry reported 725.6 deg, actual was ~755 deg
        # (tick deltas dL=-25576 dR=+25407, 0.3% apart -> a clean in-place spin).
        # Odometry UNDER-reported, so the separation must shrink:
        # Two scripted spins, each read by eye to about +/-10 deg:
        #   run 1: tick diff 50983, reported 725.6 deg, actual ~755 -> L = 0.4441
        #   run 2: tick diff 49111, reported 727.3 deg, actual ~710 -> L = 0.4549
        #   mean -> 0.4495
        # Sanity: deck is 44 cm wide with wheels ~2.5 cm proud each side, so the
        # physical track is ~45 cm. Three independent estimates land on ~0.45,
        # which is as tight as an eyeball reading gets. Remaining error is well
        # under 1% and scan matching absorbs that easily.
        # Refine: L_new = L_old * (reported_deg / actual_deg)
        self.declare_parameter('wheel_separation', 0.4495)
        # Counts per WHEEL revolution, measured 2026-09-07: a scripted straight
        # drive produced 3921 ticks (avg of both wheels, 1.6% skew) over a
        # tape-measured 0.45 m, with wheel_radius 0.05 ->
        #   cpr = 2*pi*0.05 / (0.45/3921) = 2737
        # Cross-check: the drive's encoder is 1336 steps per MOTOR revolution
        # (334 lines x 4, per the RMCS-2303 manual), and 2737/1336 = 2.05, so
        # the gearing is ~2:1. That also explains why a commanded 0.12 m/s only
        # yielded ~0.056 m/s -- the firmware's MS_TO_RPM assumes direct drive.
        # Refine with a longer drive if needed: cpr_new = cpr * (odom_x / real_x)
        self.declare_parameter('encoder_cpr', 2737.0)
        # publish_tf: True for SLAM (odometry owns the odom->base_footprint TF).
        # Set False under Nav2, where the EKF owns odom->base_footprint instead,
        # so we don't get two publishers fighting over the same transform.
        self.declare_parameter('publish_tf', True)
        # The odom->base_footprint transform is published on a timer, not once
        # per /wheel_ticks message. Ticks arrive only ~3.5 times a second (two
        # blocking Modbus reads per update, 9600 baud), but laser scans arrive
        # ~7.8 times a second and reach their consumers ~0.13 s after they were
        # measured. A scan can be used only once odometry has been published for
        # an instant AT OR AFTER the scan's timestamp, so with odometry every
        # 0.27 s about half of all scans arrived "ahead" of it. SLAM Toolbox
        # copes; AMCL did not -- it handled one scan after a pose was set and
        # then published nothing, so map->odom went stale and Nav2 aborted every
        # goal. Publishing at tf_rate_hz keeps the transform ahead of every scan.
        self.declare_parameter('tf_rate_hz', 20.0)
        # Between ticks the pose is predicted from the last measured speed. Cap
        # how far ahead it may run: a rover that stops mid-interval is still
        # "moving" in the last measurement, so the prediction overshoots by up to
        # v * this (about 6 cm at 0.2 m/s) until the next tick corrects it.
        self.declare_parameter('tf_extrapolate_s', 0.3)
        # If ticks stop arriving (ESP32 gone), do NOT keep publishing a pose that
        # looks fresh: stop after this long, as the transform always did when the
        # ticks stopped, so consumers see it go stale.
        self.declare_parameter('tf_stale_s', 1.0)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.CPR = self.get_parameter('encoder_cpr').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_rate = float(self.get_parameter('tf_rate_hz').value)
        self.tf_extrapolate = float(self.get_parameter('tf_extrapolate_s').value)
        self.tf_stale = float(self.get_parameter('tf_stale_s').value)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_ticks_left = None
        self.last_ticks_right = None
        self.last_time = self.get_clock().now()
        self.callback_count = 0
        # Last measured pose time and speed, for the TF timer's prediction.
        self.pose_time = None
        self.v = 0.0
        self.w = 0.0

        # Subscribers & Publishers
        self.sub_ticks = self.create_subscription(
            Point32, '/wheel_ticks', self.ticks_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        # Accumulated (unwrapped) heading in degrees — for rotation calibration.
        self.pub_heading = self.create_publisher(Float64, '/heading_deg', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.publish_tf and self.tf_rate > 0:
            self.create_timer(1.0 / self.tf_rate, self.publish_odom_tf)

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
            self.pose_time, self.v, self.w = current_time, 0.0, 0.0
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
            self.pose_time, self.v, self.w = current_time, 0.0, 0.0
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
        self.pose_time, self.v, self.w = current_time, v, w

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

        # Covariance. Previously left unset, which means an all-zero matrix --
        # i.e. "this pose is infinitely certain". Anything that actually reads
        # it (an EKF, a fusion node) then either trusts it absolutely or trips
        # over the singular matrix. AMCL does not read this, but publishing an
        # honest estimate is correct and matters the moment a second source is
        # fused. Values reflect measured encoder odometry: good over short
        # distances, drifting in yaw over long ones.
        # Row-major 6x6 over [x, y, z, roll, pitch, yaw].
        pc = [0.0] * 36
        pc[0]  = 0.01    # x
        pc[7]  = 0.01    # y
        pc[14] = 1e6     # z      - not measured
        pc[21] = 1e6     # roll   - not measured
        pc[28] = 1e6     # pitch  - not measured
        pc[35] = 0.02    # yaw
        odom.pose.covariance = pc

        tc = [0.0] * 36
        tc[0]  = 0.01    # vx
        tc[7]  = 1e6     # vy     - non-holonomic, cannot move sideways
        tc[14] = 1e6
        tc[21] = 1e6
        tc[28] = 1e6
        tc[35] = 0.02    # vyaw
        odom.twist.covariance = tc

        self.pub_odom.publish(odom)

        # Publish accumulated heading in degrees (unwrapped) for calibration
        self.pub_heading.publish(Float64(data=math.degrees(self.th)))

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

    def publish_odom_tf(self):
        """odom->base_footprint at tf_rate_hz: the last measured pose carried
        forward at the last measured speed, stamped now."""
        if self.pose_time is None:
            return
        now = self.get_clock().now()
        age = (now - self.pose_time).nanoseconds / 1e9
        if age > self.tf_stale:
            return                      # no fresh odometry: let it go stale
        x, y, th = extrapolate(self.x, self.y, self.th, self.v, self.w,
                               min(max(age, 0.0), self.tf_extrapolate))
        q = self.quaternion_from_euler(0, 0, th)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

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
