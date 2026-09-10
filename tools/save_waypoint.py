#!/usr/bin/env python3
"""Record the rover's current pose as a named demo station.

Drive the rover to the spot, point it the way it should face a visitor, run:

    python3 save_waypoint.py thermal_camera --label "Thermal Camera" \
        --say "This is our thermal imaging demo."

    python3 save_waypoint.py --list
    python3 save_waypoint.py --delete old_station

Why capture rather than type: map coordinates are meaningless by eye. The only
reliable way to define a station is to put the robot where you want it and read
map -> base_footprint. That also captures the HEADING, which matters -- on
arrival the rover should be facing the visitor, not the wall.

Requires the navigation stack running and AMCL localised. Check first:
    python3 check_localization.py     # want >70%

Waypoints are stored in the MAP frame, so they are tied to the map that was
loaded when you captured them. Re-map the space and you must re-capture.
"""
import argparse, math, os, sys, time
import rclpy, tf2_ros
from rclpy.node import Node

DEFAULT_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'ros2_ws', 'src', 'my_robot_bringup', 'config', 'demo_waypoints.yaml')


def load(path):
    if not os.path.exists(path):
        return {'map': None, 'demos': {}}
    import yaml
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    d.setdefault('demos', {})
    d.setdefault('map', None)
    return d


def save(path, data):
    import yaml
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (
        "# Demo stations for the expo rover.\n"
        "#\n"
        "# Poses are in the MAP frame and are tied to the map named below --\n"
        "# re-survey the space and these must be re-captured.\n"
        "#\n"
        "# Capture with the rover physically placed and facing the visitor:\n"
        "#   python3 tools/save_waypoint.py <id> --label \"Nice Name\"\n"
        "#\n"
        "# yaw is DEGREES, 0 = +X in the map frame.\n"
        "# tolerance_m: how close counts as arrived (default 0.25).\n"
        "# say: what the assistant should announce on arrival.\n\n")
    with open(path, 'w') as fh:
        fh.write(header)
        yaml.safe_dump(data, fh, sort_keys=False, default_flow_style=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('id', nargs='?', help='short id, e.g. thermal_camera')
    ap.add_argument('--label', help='human name shown in the UI and spoken')
    ap.add_argument('--say', default='', help='line to announce on arrival')
    ap.add_argument('--tolerance', type=float, default=0.25,
                    help='arrival tolerance in metres (default 0.25)')
    ap.add_argument('--file', default=DEFAULT_FILE)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--delete', metavar='ID')
    a = ap.parse_args()
    path = os.path.abspath(a.file)
    data = load(path)

    if a.list:
        if not data['demos']:
            print(f"no stations yet in {path}")
            return
        print(f"{len(data['demos'])} station(s) in {path}")
        print(f"  map: {data.get('map')}")
        for k, v in data['demos'].items():
            p = v['pose']
            print(f"  {k:<20} x={p['x']:+.2f} y={p['y']:+.2f} yaw={p['yaw']:+6.1f}°"
                  f"  \"{v.get('label', k)}\"")
        return

    if a.delete:
        if data['demos'].pop(a.delete, None) is None:
            sys.exit(f"no station called '{a.delete}'")
        save(path, data)
        print(f"removed '{a.delete}'")
        return

    if not a.id:
        ap.error('give an id, or use --list / --delete')

    rclpy.init()
    n = rclpy.create_node('save_waypoint')
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    tf = None
    t0 = time.time()
    while time.time() - t0 < 15 and tf is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            pass
    if tf is None:
        sys.exit("No TF map -> base_footprint.\n"
                 "Is master_navigation.launch.py running and an initial pose set?\n"
                 "  python3 tools/set_initial_pose.py <x> <y> <yaw_deg>")

    t, q = tf.transform.translation, tf.transform.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
    data['demos'][a.id] = {
        'label': a.label or a.id.replace('_', ' ').title(),
        'say': a.say,
        'tolerance_m': a.tolerance,
        'pose': {'x': round(t.x, 3), 'y': round(t.y, 3), 'yaw': round(yaw, 1)},
    }
    save(path, data)
    print(f"saved '{a.id}'  x={t.x:+.3f} y={t.y:+.3f} yaw={yaw:+.1f}°")
    print(f"  -> {path}")
    print(f"  {len(data['demos'])} station(s) total")
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
