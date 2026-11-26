#!/usr/bin/env python3
"""What size is the rover really, and does the costmap footprint cover it?

Reads the boxes and wheels in rover.urdf, works out the true outline and the
radii that matter, and compares them with robot_radius in nav2_params.yaml.

    python3 tools/footprint_report.py
    python3 tools/footprint_report.py --extra 0.30,0.0     # something that sticks out

Three numbers matter for a rover that pivots on the spot:
  circumscribed radius   the circle the body sweeps when it turns. The costmap
                         circle must be at least this, or a pivot beside a chair
                         leg clips it (this is what happened on the first roam).
  inscribed radius       the largest circle that fits inside. It sets the narrowest
                         gap a polygon footprint would let the rover drive through.
  padding                what the config adds over the circumscribed radius.

The URDF is a model, not a tape measure. Anything not in it (a screen, a bumper, a
cable loop, the front webcam if it hangs over the edge) goes in --extra as an x,y
offset from base_link in metres, and the numbers below include it.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'ros2_ws', 'src')
URDF = os.path.join(SRC, 'rover_description', 'urdf', 'rover.urdf')
NAV2 = os.path.join(SRC, 'my_robot_bringup', 'config', 'nav2_params.yaml')


def floats(s):
    return [float(v) for v in s.split()]


def outline_points(urdf_path):
    """Corner points (x, y) of every box and wheel, in base_link, from the URDF
    (collision geometry; origins assumed unrotated except wheels' axis)."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {k.get('name'): k for k in root.findall('link')}
    pos = {'base_link': (0.0, 0.0)}
    for _ in range(3):                     # resolve a shallow tree
        for j in joints.values():
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            o = j.find('origin')
            if p in pos and c not in pos and o is not None:
                x, y, _ = floats(o.get('xyz', '0 0 0'))
                pos[c] = (pos[p][0] + x, pos[p][1] + y)
    pts = []
    for name, link in links.items():
        if name not in pos:
            continue
        for col in link.findall('collision'):
            o = col.find('origin')
            ox, oy = (floats(o.get('xyz', '0 0 0'))[:2] if o is not None else (0.0, 0.0))
            cx, cy = pos[name][0] + ox, pos[name][1] + oy
            g = col.find('geometry')
            b, cyl = g.find('box'), g.find('cylinder')
            if b is not None:
                sx, sy, _ = floats(b.get('size'))
                pts += [(cx + dx, cy + dy) for dx in (-sx / 2, sx / 2) for dy in (-sy / 2, sy / 2)]
            elif cyl is not None:
                r, ln = float(cyl.get('radius')), float(cyl.get('length'))
                # wheels: axis along y (rpy 1.5708 about x), so thin in y
                pts += [(cx + dx, cy + dy) for dx in (-r, r) for dy in (-ln / 2, ln / 2)]
    return pts


def measure(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    circ = max(math.hypot(x, y) for x, y in pts)
    return {'x_min': min(xs), 'x_max': max(xs), 'y_min': min(ys), 'y_max': max(ys),
            'circumscribed': circ,
            'inscribed': min(abs(min(xs)), max(xs), abs(min(ys)), max(ys))}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--extra', action='append', default=[], metavar='X,Y',
                    help='a point that sticks out, metres from base_link (repeatable)')
    a = ap.parse_args()
    pts = outline_points(URDF)
    for e in a.extra:
        x, y = (float(v) for v in e.split(','))
        pts.append((x, y))
    m = measure(pts)
    with open(NAV2) as f:
        p = yaml.safe_load(f)
    cfg = {}
    for cm in ('local_costmap', 'global_costmap'):
        rp = p[cm][cm]['ros__parameters']
        cfg[cm] = rp.get('robot_radius'), rp.get('footprint')
    print('outline from the URDF%s:' % (' + %d extra point(s)' % len(a.extra) if a.extra else ''))
    print('  x %+.3f .. %+.3f m   y %+.3f .. %+.3f m   (%.2f long, %.2f wide)' % (
        m['x_min'], m['x_max'], m['y_min'], m['y_max'],
        m['x_max'] - m['x_min'], m['y_max'] - m['y_min']))
    print('  circumscribed radius (the pivot sweep): %.3f m' % m['circumscribed'])
    print('  inscribed radius (narrowest through):   %.3f m  -> gap %.2f m' % (
        m['inscribed'], 2 * m['inscribed']))
    print('\ncostmap footprint:')
    worst = None
    for name, (r, poly) in cfg.items():
        if poly:
            print('  %-15s polygon %s' % (name, poly))
        else:
            pad = r - m['circumscribed']
            worst = pad if worst is None else min(worst, pad)
            print('  %-15s circle %.2f m  (%+.1f cm over the pivot sweep)' % (name, r, 100 * pad))
    if worst is not None:
        if worst < 0:
            print('\nPROBLEM: the costmap circle is SMALLER than the body sweeps when it pivots.')
        elif worst < 0.02:
            print('\nTight: under 2 cm of padding. Fine if the outline above is exactly right; '
                  'measure the real rover, including anything not in the URDF.')
        else:
            print('\nOK: the circle covers the pivot sweep with %.0f cm to spare.' % (100 * worst))
        print('A circle is the common choice for a rover that rotates in place: one number, the '
              'same clearance in every direction. A polygon only narrows how wide a gap it will '
              'drive STRAIGHT through, and it is unproven until the sensing is.')


if __name__ == '__main__':
    main()
