#!/usr/bin/env python3
"""Start and stop the rover's ROS stacks on behalf of the web console.

The console runs OUTSIDE every stack and owns :8080 permanently. That is the
whole point of this module: mode switching means killing one launch tree and
starting another, which the console cannot do if it is itself inside the tree
being killed.

Two rules are enforced here rather than left to the operator, because both
have already cost real debugging time on this rover:

  * ONE STACK AT A TIME. Mapping and navigation both start a micro-ROS agent,
    the lidar and odometry. Two of each fight over the serial port and publish
    competing /tf, which looks like the robot teleporting rather than like a
    duplicate node. start() refuses if anything is already up -- including a
    stack somebody launched by hand in a terminal.

  * NEVER REACH FOR SIGKILL FIRST. A hard kill of the RealSense node wedges
    the V4L2 device until the camera is physically unplugged. Shutdown is
    SIGINT to the whole process group, then a long wait, then SIGTERM, and
    only then SIGKILL with a loud warning in the log.
"""
import os
import signal
import subprocess
import time

LOG_DIR = os.path.expanduser('~/AGX_Orin_Backup/rover_project/logs')

# Escalation timings. Generous on purpose: a Nav2 lifecycle teardown routinely
# takes ten seconds or more, and killing it early is what leaves orphaned
# nodes on the graph.
SIGINT_GRACE = 18.0
SIGTERM_GRACE = 8.0


class Stack:
    """One launchable stack: how to start it, and how to tell it is up."""

    def __init__(self, key, label, launch, nodes, args=(), fixed=None):
        self.key = key
        self.label = label
        self.launch = launch
        self.nodes = nodes          # node names that prove the stack is live
        self.args = args            # operator-chosen, passed only when set
        self.fixed = fixed or {}    # always passed; must be DECLARED by the
        #                             launch file or ros2 launch rejects it


STACKS = {
    'mapping': Stack(
        'mapping', 'Mapping (SLAM)', 'slam_teleop.launch.py',
        ['slam_toolbox', 'rplidar_node', 'rover_odometry']),
    'navigation': Stack(
        'navigation', 'Navigation', 'master_navigation.launch.py',
        ['bt_navigator', 'controller_server', 'planner_server', 'amcl'],
        args=('map', 'use_camera'),
        # The console already owns :8080; a second copy inside the stack would
        # fail to bind and the page would quietly stop updating.
        fixed={'use_dashboard': 'false'}),
}

# Any of these on the graph means SOMETHING is already driving the hardware.
OWNED_NODES = sorted({n for s in STACKS.values() for n in s.nodes} |
                     {'micro_ros_agent', 'map_server'})


class StackSupervisor:
    def __init__(self, node_names, logger):
        """`node_names` is a callable returning the current ROS graph node list.

        Liveness is judged by node names, never by `ros2 action list`: actions
        are advertised while a lifecycle node is still inactive, so an action
        appearing proves nothing about whether the stack can accept a goal.
        """
        self._names = node_names
        self._log = logger
        self.proc = None
        self.mode = 'idle'
        self.since = time.time()
        self.detail = ''
        self.logfile = ''
        self.opts = {}
        self._stopping = False

    # ---- inspection --------------------------------------------------
    def live_nodes(self):
        try:
            return [n for n in self._names() if n.lstrip('/') in OWNED_NODES]
        except Exception:                           # noqa: BLE001
            return []

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def status(self):
        live = self.live_nodes()
        if self.running():
            st = self.mode
            need = STACKS[self.mode].nodes if self.mode in STACKS else []
            up = [n for n in need if any(x.lstrip('/') == n for x in live)]
            if self._stopping:
                st = 'stopping'
            elif len(up) < len(need):
                st = 'starting'
            ready = f'{len(up)}/{len(need)} nodes'
        elif live:
            # Somebody launched a stack from a terminal. Report it honestly
            # rather than offering a Start button that would collide with it.
            st, ready = 'external', f'{len(live)} nodes not started here'
        else:
            st, ready = 'idle', ''
        return {'state': st, 'mode': self.mode, 'detail': self.detail,
                'ready': ready, 'since': self.since, 'opts': dict(self.opts),
                'log': os.path.basename(self.logfile) if self.logfile else '',
                'nodes': sorted(n.lstrip('/') for n in live)}

    # ---- control -----------------------------------------------------
    def start(self, key, opts=None):
        if key not in STACKS:
            return False, f'unknown mode "{key}"'
        if self.running():
            return False, f'{STACKS[self.mode].label} is already running'
        live = self.live_nodes()
        if live:
            return False, ('something is already running: ' +
                           ', '.join(sorted(n.lstrip("/") for n in live)[:4]))

        st = STACKS[key]
        opts = opts or {}
        cmd = ['ros2', 'launch', 'my_robot_bringup', st.launch]
        for a in st.args:
            if opts.get(a) not in (None, ''):
                cmd.append(f'{a}:={opts[a]}')
        cmd += [f'{k}:={v}' for k, v in st.fixed.items()]

        os.makedirs(LOG_DIR, exist_ok=True)
        self.logfile = os.path.join(
            LOG_DIR, f'{key}-{time.strftime("%Y%m%d-%H%M%S")}.log')
        fh = open(self.logfile, 'wb')
        fh.write(f'$ {" ".join(cmd)}\n\n'.encode())
        fh.flush()
        try:
            # start_new_session gives the launch tree its own process group, so
            # one signal reaches every node it spawned. Without it, SIGINT hits
            # only `ros2 launch` and leaves the nodes orphaned and still
            # holding the serial port.
            self.proc = subprocess.Popen(
                cmd, stdout=fh, stderr=subprocess.STDOUT,
                start_new_session=True, cwd=os.path.expanduser('~'))
        except Exception as e:                      # noqa: BLE001
            fh.close()
            return False, f'launch failed: {e}'
        self.mode, self.since, self.opts = key, time.time(), dict(opts)
        self._stopping = False
        self.detail = f'started {st.label}'
        self._log.info(f'supervisor: {" ".join(cmd)} -> log {self.logfile}')
        return True, self.detail

    def stop(self):
        if not self.running():
            self.mode, self.detail, self._stopping = 'idle', 'nothing running', False
            return True, self.detail
        self._stopping = True
        pgid = os.getpgid(self.proc.pid)
        self._log.info(f'supervisor: SIGINT to process group {pgid}')
        for sig, grace in ((signal.SIGINT, SIGINT_GRACE),
                           (signal.SIGTERM, SIGTERM_GRACE)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            if self._wait(grace):
                break
        else:
            self._log.warning(
                'supervisor: stack ignored SIGINT and SIGTERM, sending SIGKILL. '
                'If the RealSense was running it may need a physical replug.')
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._wait(4.0)
        self.proc = None
        self.mode, self.since = 'idle', time.time()
        self._stopping = False
        self.detail = 'stopped'
        return True, self.detail

    def _wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if self.proc.poll() is not None:
                return True
            time.sleep(0.25)
        return self.proc.poll() is not None

    def tail(self, n=60):
        if not self.logfile or not os.path.exists(self.logfile):
            return []
        try:
            with open(self.logfile, 'rb') as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - 16000))
                return fh.read().decode('utf-8', 'replace').splitlines()[-n:]
        except Exception:                           # noqa: BLE001
            return []
