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
# How long to let departed nodes fall out of the discovery graph before
# starting the next stack over them.
GRAPH_SETTLE = 30.0


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


# The camera is deliberately NOT part of either stack. Mapping wants it for the
# operator's view, navigation wants it for 3D obstacles, and switching between
# those two should not make the video drop out and the USB device re-enumerate.
# Running it as its own supervised process means it survives a mode change.
CAMERA = Stack('camera', 'RealSense D455', 'realsense.launch.py', ['camera'])

STACKS = {
    'mapping': Stack(
        'mapping', 'Mapping (SLAM)', 'slam_teleop.launch.py',
        ['slam_toolbox', 'rplidar_node', 'rover_odometry']),
    'navigation': Stack(
        'navigation', 'Navigation', 'master_navigation.launch.py',
        ['bt_navigator', 'controller_server', 'planner_server', 'amcl'],
        args=('map',),
        # The console already owns :8080; a second copy inside the stack would
        # fail to bind and the page would quietly stop updating. The camera is
        # supervised separately, so the stack must never start its own.
        fixed={'use_dashboard': 'false', 'use_camera': 'false'}),
}

# Any of these on the graph means SOMETHING is already driving the hardware.
OWNED_NODES = sorted({n for s in STACKS.values() for n in s.nodes} |
                     {'micro_ros_agent', 'map_server'})


class Adopted:
    """A launch tree this console started before it was itself restarted.

    Restarting the console must not mean losing the ability to stop the camera
    or the stack -- and must certainly not mean killing them on the way out.
    Adoption keeps them running AND keeps them controllable. It quacks like
    Popen for the two things the supervisor asks of one.
    """

    def __init__(self, pid):
        self.pid = pid

    def poll(self):
        try:
            os.kill(self.pid, 0)
        except OSError:
            return 0
        return None


def _find_launch(launch_file):
    """PIDs of `ros2 launch my_robot_bringup <launch_file>` we can signal.

    Matched on the exact launch file name, not a loose substring, so this can
    never adopt some unrelated python process.
    """
    out = []
    needle = f'my_robot_bringup\x00{launch_file}'
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as fh:
                cl = fh.read().decode('utf-8', 'replace')
        except Exception:                           # noqa: BLE001
            continue
        if 'ros2' in cl and needle in cl:
            out.append(int(d))
    return out


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
        # The camera runs on its own, outliving stack restarts.
        self.cam = None
        self.cam_log = ''
        self.cam_detail = ''
        self.adopt()

    def adopt(self):
        """Re-attach to launch trees left running by a previous console."""
        for pid in _find_launch(CAMERA.launch):
            self.cam = Adopted(pid)
            self._log.info(f'supervisor: adopted the camera (pid {pid})')
            break
        for key, st in STACKS.items():
            pids = _find_launch(st.launch)
            if pids:
                self.proc = Adopted(pids[0])
                self.mode = key
                self.detail = f'adopted {st.label}'
                self._log.info(
                    f'supervisor: adopted {st.label} (pid {pids[0]})')
                break

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

    # ---- process plumbing --------------------------------------------
    def _spawn(self, st, opts):
        cmd = ['ros2', 'launch', 'my_robot_bringup', st.launch]
        for a in st.args:
            if opts.get(a) not in (None, ''):
                cmd.append(f'{a}:={opts[a]}')
        cmd += [f'{k}:={v}' for k, v in st.fixed.items()]

        os.makedirs(LOG_DIR, exist_ok=True)
        log = os.path.join(
            LOG_DIR, f'{st.key}-{time.strftime("%Y%m%d-%H%M%S")}.log')
        fh = open(log, 'wb')
        fh.write(f'$ {" ".join(cmd)}\n\n'.encode())
        fh.flush()
        # start_new_session gives the launch tree its own process group, so one
        # signal reaches every node it spawned -- and, just as important, a
        # signal aimed at that group can never travel back up to the console.
        proc = subprocess.Popen(
            cmd, stdout=fh, stderr=subprocess.STDOUT,
            start_new_session=True, cwd=os.path.expanduser('~'))
        self._log.info(f'supervisor: {" ".join(cmd)} -> log {log}')
        return proc, log

    def _kill(self, proc, label, gentle=False):
        """SIGINT the whole group, then escalate. `gentle` doubles the graces.

        The RealSense is the reason for the patience: a hard kill of that node
        wedges the V4L2 device until the camera is physically unplugged.
        """
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        mult = 2.0 if gentle else 1.0
        self._log.info(f'supervisor: SIGINT to {label} process group {pgid}')
        for sig, grace in ((signal.SIGINT, SIGINT_GRACE * mult),
                           (signal.SIGTERM, SIGTERM_GRACE * mult)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            if self._wait_proc(proc, grace):
                return
        self._log.warning(
            f'supervisor: {label} ignored SIGINT and SIGTERM, sending SIGKILL. '
            'If the RealSense was running it may need a physical replug.')
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._wait_proc(proc, 4.0)

    @staticmethod
    def _wait_proc(proc, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if proc.poll() is not None:
                return True
            time.sleep(0.25)
        return proc.poll() is not None

    # ---- camera ------------------------------------------------------
    def camera_running(self):
        return self.cam is not None and self.cam.poll() is None

    def camera(self, on):
        if on:
            if self.camera_running():
                return True, 'camera already running'
            if any(n.lstrip('/') == 'camera' for n in self._names()):
                return False, 'a camera node is already running elsewhere'
            try:
                self.cam, self.cam_log = self._spawn(CAMERA, {})
            except Exception as e:                  # noqa: BLE001
                return False, f'camera launch failed: {e}'
            self.cam_detail = 'starting'
            return True, 'camera starting'
        self._kill(self.cam, 'camera', gentle=True)
        self.cam = None
        self.cam_detail = ''
        return True, 'camera stopped'

    def camera_status(self):
        live = any(n.lstrip('/') == 'camera' for n in self._names())
        if self.camera_running():
            st = 'on' if live else 'starting'
        elif live:
            st = 'external'
        else:
            st = 'off'
        return {'state': st,
                'log': os.path.basename(self.cam_log) if self.cam_log else ''}

    # ---- control -----------------------------------------------------
    def start(self, key, opts=None, force=False):
        if key not in STACKS:
            return False, f'unknown mode "{key}"'
        if self.running():
            return False, f'{STACKS[self.mode].label} is already running'
        # `force` is only ever set by switch(), which has just watched our own
        # launch tree exit. What is left in the graph then is a stale discovery
        # entry, not a process -- and refusing on that would strand the rover
        # with nothing running, which is the opposite of what the guard is for.
        live = [] if force else self.live_nodes()
        if live:
            return False, ('something is already running: ' +
                           ', '.join(sorted(n.lstrip("/") for n in live)[:4]))

        st = STACKS[key]
        opts = opts or {}
        try:
            self.proc, self.logfile = self._spawn(st, opts)
        except Exception as e:                      # noqa: BLE001
            return False, f'launch failed: {e}'
        self.mode, self.since, self.opts = key, time.time(), dict(opts)
        self._stopping = False
        self.detail = f'started {st.label}'
        return True, self.detail

    def switch(self, key, opts=None):
        """Change mode in one action.

        Mapping and navigation are different launch trees, so a change really
        does mean stopping one and starting the other -- but doing that as two
        separate operator actions leaves the rover sitting with nothing running
        and the lidar and ESP32 dark, which is not what "exit mapping" means.
        """
        if key not in STACKS:
            return False, f'unknown mode "{key}"'
        if not self.running():
            return self.start(key, opts)
        if self.mode == key and dict(opts or {}) == dict(self.opts):
            return False, f'already in {STACKS[key].label}'
        # Same mode but different options -- loading a different map -- is a
        # real request. Restarting the stack is the only way to honour it, and
        # making the operator press STOP first just leaves the rover dark.
        self.stop()
        # Let the graph settle. Departed nodes linger in other participants'
        # discovery caches well past process exit -- measured at more than ten
        # seconds for slam_toolbox -- so wait, but do not let a stale entry
        # block the start: stop() has already confirmed the process group is
        # gone, so nothing is actually holding the serial port or the lidar.
        for _ in range(int(GRAPH_SETTLE / 0.5)):
            if not self.live_nodes():
                break
            time.sleep(0.5)
        stale = self.live_nodes()
        if stale:
            self._log.info('supervisor: starting over stale graph entries: ' +
                           ', '.join(sorted(n.lstrip("/") for n in stale)))
        return self.start(key, opts, force=True)

    def stop(self):
        if not self.running():
            self.mode, self.detail, self._stopping = 'idle', 'nothing running', False
            return True, self.detail
        self._stopping = True
        self._kill(self.proc, STACKS.get(self.mode, CAMERA).label)
        self.proc = None
        self.mode, self.since = 'idle', time.time()
        self._stopping = False
        self.detail = 'stopped'
        return True, self.detail

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
