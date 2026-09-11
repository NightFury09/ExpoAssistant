#!/usr/bin/env python3
"""Ask the planner whether a goal is reachable -- without moving the robot.

    python3 check_reachable.py 2.3 -1.1
    python3 check_reachable.py 2.3 -1.1 --clear     # retry after clearing costmaps

Why this matters more than it sounds
------------------------------------
When Nav2 cannot plan, its default behaviour tree clears costmaps, SPINS, BACKS
UP, waits, and retries — roughly 30 s of thrashing before it aborts, with no
explanation. On a booth floor, spinning and reversing near people and tables is
the worst possible response, and the visitor is told nothing.

Calling the planner directly answers "is there a route?" in under a second, so
the application can decide BEFORE committing:

  reachable          -> send the goal
  blocked, transient -> "someone's in the way, give me a moment" and retry
  blocked, permanent -> "I can't reach that from here" and stay put

The --clear flag is what separates the last two. Costmaps accumulate obstacles
from people who have since walked on; clearing and re-checking distinguishes a
stale mark from a real wall. That distinction is the whole difference between a
rover that waits politely and one that gives up on an empty corridor.
"""
import argparse, math, sys, time
import rclpy, tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class Checker(Node):
    def __init__(self):
        super().__init__('check_reachable')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def here(self):
        for _ in range(60):
            try:
                t = self.buf.lookup_transform('map', 'base_footprint',
                                              rclpy.time.Time())
                return t.transform.translation.x, t.transform.translation.y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def clear_costmaps(self):
        done = []
        for srv in ('/global_costmap/clear_entirely_global_costmap',
                    '/local_costmap/clear_entirely_local_costmap'):
            cli = self.create_client(Empty, srv)
            if cli.wait_for_service(timeout_sec=3.0):
                fut = cli.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                done.append(srv.split('/')[1])
        return done

    def plan(self, x, y, yaw_deg=0.0):
        if not self.ac.wait_for_server(timeout_sec=8.0):
            return None, "planner action server not available"
        g = ComputePathToPose.Goal()
        g.goal = PoseStamped()
        g.goal.header.frame_id = 'map'
        g.goal.header.stamp = self.get_clock().now().to_msg()
        g.goal.pose.position.x = float(x)
        g.goal.pose.position.y = float(y)
        th = math.radians(yaw_deg)
        g.goal.pose.orientation.z = math.sin(th / 2)
        g.goal.pose.orientation.w = math.cos(th / 2)
        g.use_start = False
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, "planner rejected the request"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=15.0)
        res = rf.result()
        if res is None:
            return None, "planner timed out"
        poses = res.result.path.poses
        if not poses:
            return None, "no path found"
        length = sum(
            math.dist((poses[i].pose.position.x, poses[i].pose.position.y),
                      (poses[i+1].pose.position.x, poses[i+1].pose.position.y))
            for i in range(len(poses) - 1))
        return (len(poses), length), None


def report(res, err, start, x, y):
    if err:
        print(f"  NOT REACHABLE  ({err})")
        return False
    n, length = res
    straight = math.dist(start, (x, y)) if start else float('nan')
    detour = length / straight if straight and straight > 0.05 else float('nan')
    print(f"  REACHABLE  path {length:.2f} m over {n} poses")
    if math.isfinite(detour):
        print(f"             straight line {straight:.2f} m -> detour x{detour:.2f}")
        if detour > 2.5:
            print("             NOTE: heavy detour. The direct route is blocked;")
            print("             the robot will take a long way round.")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=0.0,
                    help='arrival heading in degrees (default 0)')
    ap.add_argument('--clear', action='store_true',
                    help='if unreachable, clear the costmaps and try again — '
                         'distinguishes a stale obstacle from a real one')
    a = ap.parse_args()

    rclpy.init()
    n = Checker()
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.1)
    start = n.here()
    if start is None:
        sys.exit("No TF map->base_footprint. Is Nav2 up with an initial pose set?")
    print(f"robot at ({start[0]:+.2f}, {start[1]:+.2f})  ->  goal ({a.x:+.2f}, {a.y:+.2f})")

    res, err = n.plan(a.x, a.y, a.yaw)
    ok = report(res, err, start, a.x, a.y)

    if not ok and a.clear:
        print("\nclearing costmaps and retrying...")
        cleared = n.clear_costmaps()
        print(f"  cleared: {', '.join(cleared) or 'none'}")
        time.sleep(2.0)
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.1)
        res2, err2 = n.plan(a.x, a.y, a.yaw)
        ok2 = report(res2, err2, start, a.x, a.y)
        print()
        if ok2:
            print("  -> was a STALE obstacle. Something was marked that has since")
            print("     moved. Safe to send the goal now.")
        else:
            print("  -> genuinely blocked. Not a stale mark: there is no route")
            print("     even with a clean costmap. Move the obstruction, or pick")
            print("     a different station.")
    elif not ok:
        print("\n  re-run with --clear to tell a stale mark from a real blockage.")

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
