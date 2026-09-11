#!/usr/bin/env python3
"""Talk to the rover console from another program -- the Product_RAG_ hook.

The assistant does not need ROS. It does not need to be on the Jetson. It needs
HTTP and this file, which uses nothing outside the Python standard library, so
it can be dropped straight into an existing project:

    from rover_client import Rover

    rover = Rover()                       # or Rover("http://192.168.3.224:8080")

    if rover.ready():                     # safe to offer someone a walk?
        places = rover.places()           # ['home', 'robotic_arm', ...]
        rover.go("robotic_arm")
        ...
        if rover.arrived():
            say("Here we are.")

WHY `ready()` MATTERS
---------------------
Offering "shall I show you?" and then not moving is worse than not offering.
ready() is true only when navigation is up, the rover is localised, the e-stop
is clear, demo points exist, and those points belong to the map that is loaded.
Check it before you offer, not after the visitor says yes.

ARRIVAL IS NOT INSTANT
----------------------
go() returns as soon as Nav2 accepts the goal, not when the rover gets there.
Poll status() (cheap, a few hundred bytes) or call wait() in a background
thread. The states an assistant should care about:

    sending / active   on the way
    arrived            there -- start talking
    aborted            Nav2 gave up; say so and offer to try again
    idle               nothing running, or the goal was cancelled

ALWAYS GIVE A WAY OUT
---------------------
If the visitor changes their mind, call cancel(). It stops any goal, including
one somebody set from the console. estop(True) is the bigger hammer: it halts
the motors AND cancels the goal, and nothing will move again until
estop(False).
"""
import json
import time
import urllib.error
import urllib.request

DEFAULT_URL = 'http://127.0.0.1:8080'
TIMEOUT = 5.0


class RoverError(RuntimeError):
    """The console refused, or could not be reached."""


class Rover:
    def __init__(self, url=DEFAULT_URL, timeout=TIMEOUT):
        self.url = url.rstrip('/')
        self.timeout = timeout

    # ---- plumbing ----------------------------------------------------
    def _call(self, path, body=None):
        req = urllib.request.Request(
            self.url + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Content-Type': 'application/json'},
            method='POST' if body is not None else 'GET')
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read() or b'{}')
        except urllib.error.HTTPError as e:
            # The console answers refusals with a reason; surface that rather
            # than a bare status code, so the assistant can say something true.
            try:
                return json.loads(e.read() or b'{}')
            except Exception:                       # noqa: BLE001
                raise RoverError(f'{path}: HTTP {e.code}') from None
        except Exception as e:                      # noqa: BLE001
            raise RoverError(f'cannot reach the rover console at {self.url}: {e}')

    # ---- state -------------------------------------------------------
    def status(self):
        """Everything an assistant needs, in one small response."""
        return self._call('/api/places')

    def ready(self):
        """True when it is safe to OFFER to take somebody somewhere."""
        try:
            return bool(self.status().get('ready'))
        except RoverError:
            return False

    def why_not_ready(self):
        """A sentence you can say out loud, or None when it is ready."""
        try:
            s = self.status()
        except RoverError as e:
            return str(e)
        if s.get('ready'):
            return None
        if s.get('estop'):
            return 'the emergency stop is engaged'
        if s.get('mode') != 'navigation':
            return f"the rover is in {s.get('mode', 'an unknown')} mode, not navigation"
        if not s.get('localised'):
            return 'the rover does not know where it is yet'
        if not s.get('places'):
            return 'no demo points have been marked'
        if s.get('places_map') and s.get('map') != s.get('places_map'):
            return (f"the demo points were marked on map "
                    f"'{s['places_map']}' but '{s['map']}' is loaded")
        return 'the rover is not ready'

    def places(self):
        """Names you can pass to go(). Empty if none are marked yet."""
        return self.status().get('places', [])

    def pose(self):
        """Where the rover is on the map: {x, y, yaw} or None."""
        return self.status().get('pose')

    # ---- commands ----------------------------------------------------
    def go(self, place):
        """Send the rover to a named demo point.

        Returns as soon as Nav2 ACCEPTS the goal, not on arrival.
        Raises RoverError if the name is unknown or the rover is e-stopped.
        """
        r = self._call('/api/wp/goto', {'name': place})
        if not r.get('ok'):
            raise RoverError(r.get('err') or f'could not send the rover to {place}')
        return True

    def go_xy(self, x, y, yaw=0.0):
        """Send it to a raw map coordinate. Prefer go() with a named point."""
        r = self._call('/api/goal', {'x': x, 'y': y, 'yaw': yaw})
        if not r.get('ok'):
            raise RoverError(r.get('err') or 'goal refused')
        return True

    def cancel(self):
        """Stop the current goal. Safe to call when nothing is running."""
        self._call('/api/nav_cancel', {})
        return True

    def estop(self, on=True):
        """Halt the motors and cancel the goal. Nothing moves until estop(False)."""
        self._call('/api/estop', {'on': bool(on)})
        return True

    # ---- waiting -----------------------------------------------------
    def nav_state(self):
        return self.status().get('nav', {}).get('state', 'idle')

    def arrived(self):
        return self.nav_state() == 'arrived'

    def wait(self, timeout=180.0, poll=1.0, on_progress=None):
        """Block until the rover stops trying. Returns the final state.

        'arrived', 'aborted', or 'idle' (cancelled). Raises RoverError on
        timeout so a stuck rover cannot hang a conversation for ever.

        on_progress(state, metres_remaining) is called each poll, which is
        where "we're about halfway there" comes from.
        """
        end = time.time() + timeout
        moving = ('sending', 'active', 'cancelling')
        started = False
        while time.time() < end:
            s = self.status().get('nav', {})
            st = s.get('state', 'idle')
            if on_progress:
                on_progress(st, s.get('remaining'))
            if st in moving:
                started = True
            elif started or st in ('arrived', 'aborted'):
                # Only trust 'idle' AFTER movement has been seen: right after
                # go() the goal has not reached the action server yet, and
                # returning 'idle' there would read as "already arrived".
                return st
            time.sleep(poll)
        raise RoverError(f'the rover did not finish within {timeout:.0f} s')


# ---- command line, for trying it without writing any code ------------
def _main(argv):
    import argparse
    ap = argparse.ArgumentParser(
        description='Talk to the rover console.',
        epilog='examples:  rover_client.py places  |  '
               'rover_client.py go robotic_arm --wait  |  rover_client.py cancel')
    ap.add_argument('--url', default=DEFAULT_URL)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status', help='full status as JSON')
    sub.add_parser('places', help='list the demo points')
    sub.add_parser('ready', help='is it safe to offer a walk? exit 0 if yes')
    sub.add_parser('cancel', help='stop the current goal')
    g = sub.add_parser('go', help='send the rover to a demo point')
    g.add_argument('place')
    g.add_argument('--wait', action='store_true', help='block until it arrives')
    e = sub.add_parser('estop', help='halt everything')
    e.add_argument('state', choices=['on', 'off'])
    a = ap.parse_args(argv)

    r = Rover(a.url)
    try:
        if a.cmd == 'status':
            print(json.dumps(r.status(), indent=2))
        elif a.cmd == 'places':
            names = r.places()
            print('\n'.join(names) if names else
                  'no demo points yet -- mark some on the console MAP tab')
        elif a.cmd == 'ready':
            why = r.why_not_ready()
            print('ready' if why is None else f'NOT ready: {why}')
            return 0 if why is None else 1
        elif a.cmd == 'cancel':
            r.cancel(); print('cancelled')
        elif a.cmd == 'estop':
            r.estop(a.state == 'on'); print(f'e-stop {a.state}')
        elif a.cmd == 'go':
            why = r.why_not_ready()
            if why:
                print(f'NOT ready: {why}')
                return 1
            r.go(a.place)
            print(f'on the way to {a.place}')
            if a.wait:
                def show(state, left):
                    print(f'  {state}' + (f'  {left:.2f} m to go'
                                          if left is not None else ''))
                print(r.wait(on_progress=show))
    except RoverError as e:
        print(f'error: {e}')
        return 2
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_main(sys.argv[1:]))
