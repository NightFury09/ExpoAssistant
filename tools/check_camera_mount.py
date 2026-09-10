#!/usr/bin/env python3
"""Verify the D455 mount transform using the floor as ground truth.

Run with the camera up and the rover on flat ground, a few metres of clear
floor ahead:

    python3 check_camera_mount.py
    python3 check_camera_mount.py --frame base_link

What it does
------------
Transforms the depth cloud into the robot frame and looks at where the floor
lands. If the mount TF is right, floor points sit at z ~ 0 and stay there as
range increases. Errors show up as:

  * floor height offset from 0        -> cam_z is wrong by that amount
  * floor height DRIFTING with range  -> cam_pitch is wrong; this is the
                                         dangerous one, because a nose-down
                                         camera declared level makes the floor
                                         project upward and Nav2 sees a wall
                                         rising in front of the robot
  * floor tilting left/right          -> roll, or the camera is not seated flat

Why it matters more than it sounds: the costmap keeps points in a height band
(0.10-1.20 m). Get the mount wrong and either the floor climbs into the band and
becomes a phantom obstacle, or real obstacles fall out of it and vanish.

Mount geometry reminder (see realsense.launch.py): level at 0.82 m with a 58 deg
vertical FOV, the camera cannot see the floor closer than ~1.5 m. Expect no
floor samples nearer than that -- their absence is correct, not a fault.
"""
import argparse, math, statistics, sys, time
import rclpy, tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Mount(Node):
    def __init__(self, topic):
        super().__init__('check_camera_mount')
        self.cloud = None
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(PointCloud2, topic, self._cb, 1)

    def _cb(self, m):
        self.cloud = m


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/camera/camera/depth/color/points')
    ap.add_argument('--frame', default='base_link',
                    help='robot frame to evaluate in (default base_link, at floor level)')
    ap.add_argument('--max-range', type=float, default=5.0)
    ap.add_argument('--vfov-deg', type=float, default=58.0,
                    help="camera vertical FOV; sets the floor blind zone (D455 depth = 58)")
    a = ap.parse_args()

    rclpy.init()
    n = Mount(a.topic)
    t0 = time.time()
    while time.time() - t0 < 20 and n.cloud is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.cloud is None:
        sys.exit(f"No cloud on {a.topic}.\n"
                 "Is realsense.launch.py running? Check:  ros2 topic hz " + a.topic)

    src = n.cloud.header.frame_id
    tf = None
    for _ in range(60):
        try:
            tf = n.buf.lookup_transform(a.frame, src, rclpy.time.Time())
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.1)
    if tf is None:
        sys.exit(f"No TF {a.frame} <- {src}.\n"
                 "The camera_mount_tf static publisher in realsense.launch.py\n"
                 "connects base_link -> camera_link; without it the cloud is\n"
                 "unusable to Nav2.")

    t = tf.transform.translation
    q = tf.transform.rotation
    # quaternion -> rpy
    roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
    sp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    print(f"TF {a.frame} <- {src}")
    print(f"  xyz = ({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}) m")
    print(f"  rpy = ({math.degrees(roll):+.1f}, {math.degrees(pitch):+.1f}, "
          f"{math.degrees(yaw):+.1f}) deg")

    # rotate points by the quaternion, then translate
    def xf(p):
        x, y, z = p
        # q * v * q^-1
        tx = 2*(q.y*z - q.z*y); ty = 2*(q.z*x - q.x*z); tz = 2*(q.x*y - q.y*x)
        rx = x + q.w*tx + (q.y*tz - q.z*ty)
        ry = y + q.w*ty + (q.z*tx - q.x*tz)
        rz = z + q.w*tz + (q.x*ty - q.y*tx)
        return rx + t.x, ry + t.y, rz + t.z

    pts = []
    for p in point_cloud2.read_points(n.cloud, field_names=('x', 'y', 'z'),
                                      skip_nans=True):
        X, Y, Z = xf((float(p[0]), float(p[1]), float(p[2])))
        d = math.hypot(X, Y)
        if 0.3 < d < a.max_range:
            pts.append((d, X, Y, Z))
    if len(pts) < 200:
        sys.exit(f"Only {len(pts)} usable points. Point the camera at open floor.")
    print(f"\n{len(pts)} points within {a.max_range} m")

    # ------------------------------------------------------------------
    # Fit a plane to the floor and read the mount error off it directly.
    #
    # An earlier version binned points by range and fitted a slope through the
    # per-band floor heights. That failed three ways and gave contradictory
    # answers on consecutive runs:
    #   * its blind-zone cutoff assumed a LEVEL camera, so once the camera was
    #     pitched down it discarded the densest, most reliable near bands;
    #   * a single sparse far band reading +0.9 m dragged a least-squares fit
    #     to +26 deg when the true error was a few degrees;
    #   * changing cam_z changed which bands were used, so successive runs were
    #     not comparable and the iteration never converged.
    # Fitting a plane to the floor points avoids all of it: one shot, no
    # iteration, and RANSAC ignores furniture and outliers by construction.
    # ------------------------------------------------------------------
    floor_cands = [p for p in pts if p[3] < 0.35]
    if len(floor_cands) < 500:
        sys.exit(f"Only {len(floor_cands)} candidate floor points. Point the "
                 f"camera at open floor a few metres ahead.")
    print(f"{len(floor_cands)} candidate floor points (z < 0.35 m)")

    import random
    random.seed(0)
    best_inl, best = None, None
    for _ in range(400):
        try:
            s3 = random.sample(floor_cands, 3)
        except ValueError:
            break
        (_, x1, y1, z1), (_, x2, y2, z2), (_, x3, y3, z3) = s3
        # plane z = A x + B y + C
        det = (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)
        if abs(det) < 1e-6:
            continue
        A = ((z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)) / det
        B = ((z3-z1)*(x2-x1) - (z2-z1)*(x3-x1)) / det
        C = z1 - A*x1 - B*y1
        if abs(A) > 0.8 or abs(B) > 0.8:
            continue
        inl = [q for q in floor_cands if abs(q[3] - (A*q[1] + B*q[2] + C)) < 0.03]
        if best_inl is None or len(inl) > len(best_inl):
            best_inl, best = inl, (A, B, C)
    if best is None or len(best_inl) < 300:
        sys.exit("Could not fit a floor plane. Is there open floor in view?")

    # least squares refit on the inliers
    n_i = len(best_inl)
    sx = sum(q[1] for q in best_inl); sy = sum(q[2] for q in best_inl)
    sz = sum(q[3] for q in best_inl)
    sxx = sum(q[1]*q[1] for q in best_inl); syy = sum(q[2]*q[2] for q in best_inl)
    sxy = sum(q[1]*q[2] for q in best_inl)
    sxz = sum(q[1]*q[3] for q in best_inl); syz = sum(q[2]*q[3] for q in best_inl)
    M = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n_i)]]
    V = [sxz, syz, sz]
    for i in range(3):
        pv = max(range(i, 3), key=lambda r: abs(M[r][i]))
        M[i], M[pv] = M[pv], M[i]; V[i], V[pv] = V[pv], V[i]
        for r in range(i+1, 3):
            f = M[r][i] / M[i][i]
            for c in range(i, 3):
                M[r][c] -= f * M[i][c]
            V[r] -= f * V[i]
    A = [0.0]*3
    for i in (2, 1, 0):
        A[i] = (V[i] - sum(M[i][c]*A[c] for c in range(i+1, 3))) / M[i][i]
    gx, gy, c0 = A

    print(f"floor plane fitted on {n_i} inliers "
          f"({100*n_i/len(floor_cands):.0f}% of candidates)")
    print(f"  z = {gx:+.4f}*x {gy:+.4f}*y {c0:+.4f}")
    resid = [abs(q[3] - (gx*q[1] + gy*q[2] + c0)) for q in best_inl]
    print(f"  residual: mean {statistics.mean(resid)*1000:.0f} mm")

    pitch_err = math.atan(gx)      # floor rising with x -> more nose-down needed
    roll_err  = math.atan(gy)
    print(f"\nMOUNT ERROR")
    print(f"  pitch : {math.degrees(pitch_err):+.2f} deg")
    print(f"  roll  : {math.degrees(roll_err):+.2f} deg")
    print(f"  height: {c0:+.3f} m  (floor should read 0)")

    # IMPORTANT: read the MOUNT transform (base_link -> camera_link) for the
    # current pitch. `pitch` above comes from base_link -> camera_DEPTH_OPTICAL
    # frame, which carries the optical convention (-90 deg roll/yaw) and whose
    # extracted pitch is meaningless as a mount value -- it reads 0.000 no
    # matter what the mount is actually set to.
    cur_pitch, cur_z, cur_x, cur_y = pitch, t.z, t.x, t.y
    try:
        mt = n.buf.lookup_transform(a.frame, 'camera_link', rclpy.time.Time())
        mq = mt.transform.rotation
        msp = max(-1.0, min(1.0, 2*(mq.w*mq.y - mq.z*mq.x)))
        cur_pitch = math.asin(msp)
        cur_x, cur_y, cur_z = (mt.transform.translation.x,
                               mt.transform.translation.y,
                               mt.transform.translation.z)
        print(f"\nmount TF {a.frame} <- camera_link: "
              f"xyz=({cur_x:+.3f}, {cur_y:+.3f}, {cur_z:+.3f}) "
              f"pitch={math.degrees(cur_pitch):+.2f} deg")
    except Exception:
        print("\n(could not read base_link->camera_link; "
              "reported 'currently' values may be wrong)")

    new_pitch = cur_pitch + pitch_err
    new_z = cur_z - c0
    ok_p = abs(math.degrees(pitch_err)) < 1.0
    ok_z = abs(c0) < 0.04
    print(f"\nRECOMMENDED")
    print(f"  cam_pitch:={new_pitch:.3f}   (currently {cur_pitch:.3f})"
          f"{'   [already good]' if ok_p else ''}")
    print(f"  cam_z:={new_z:.3f}           (currently {cur_z:.3f})"
          f"{'   [already good]' if ok_z else ''}")
    if abs(math.degrees(roll_err)) > 1.5:
        print(f"  roll is {math.degrees(roll_err):+.1f} deg -- the camera is not "
              f"seated level side-to-side; fix mechanically.")
    if ok_p and ok_z:
        print("\nMOUNT IS GOOD. The floor sits flat at z=0 in the robot frame.")
    else:
        print("\nApply both at once, then re-run to confirm:")
        print(f"  ros2 run tf2_ros static_transform_publisher --x {cur_x:.2f} "
              f"--y {cur_y:.2f} --z {new_z:.3f} \\\n"
              f"      --yaw 0 --pitch {new_pitch:.3f} --roll 0 \\\n"
              f"      --frame-id base_link --child-frame-id camera_link")

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
