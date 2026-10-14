#!/usr/bin/python3
"""The rover console as a desktop app, on the rover's own screen.

    /usr/bin/python3 desktop/rover_app.py               # maximised window
    /usr/bin/python3 desktop/rover_app.py --fullscreen  # kiosk

It is the same console page, in its own window instead of a browser tab, loaded
from 127.0.0.1. Loopback works with every network interface down, so this needs
no Wi-Fi, no router and no second device. One page, two ways to open it: a fix
to the console shows up here and in every browser at once.

Why not a native UI: the console is 1,300 lines of page that already drives,
maps, navigates and supervises the stacks. A second copy would drift from the
first within a week, and the one at an expo would be the stale one.

The console itself is the rover-console service, not this window. Closing the
window does not stop the service or the stack it is running -- the page stops
sending commands and the console's deadman zeroes /cmd_vel, exactly as it does
when a browser tab closes. If the service is not running, this starts it.

Keys (the page owns WASD, Space, Shift, 1-3, M and Esc -- Esc cancels a goal,
so it deliberately does NOT leave full screen here):
    F11            full screen on / off
    F5, Ctrl+R     reload
    Ctrl +/-/0     zoom
    Ctrl+Q         close

Uses GTK 3 and WebKit2GTK through the system python3-gi -- already on a stock
JetPack desktop, nothing to pip install. The shebang is /usr/bin/python3 on
purpose: an activated venv's python cannot see the system gi bindings.
"""
import argparse, html, os, subprocess, sys, time

try:
    import gi
    # Gdk too: GTK 4 is also installed, and an unpinned Gdk loads 4.0, which
    # then makes Gtk 3.0 refuse to load.
    gi.require_version('Gdk', '3.0')
    gi.require_version('Gtk', '3.0')
    gi.require_version('WebKit2', '4.0')
    from gi.repository import Gdk, GLib, Gtk, WebKit2
except (ImportError, ValueError) as e:
    sys.exit(f'rover_app: needs GTK 3 and WebKit2GTK 4.0 ({e}).\n'
             '    sudo apt install python3-gi gir1.2-webkit2-4.0')

HERE = os.path.dirname(os.path.abspath(__file__))
ICON = os.path.join(HERE, 'rover.svg')
SERVICE = 'rover-console'
URL = 'http://127.0.0.1:8080/'
RETRY_S = 2
# After this long without an answer, the waiting page says where to look
# instead of only saying "waiting".
HELP_AFTER_S = 20

WAIT_PAGE = """<!doctype html><html><head><meta charset="utf-8"><style>
body{{background:#0b0e12;color:#e8eef5;margin:0;height:100vh;display:flex;
  align-items:center;justify-content:center;
  font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}}
main{{max-width:44em;padding:2em}}
h1{{font-size:20px;font-weight:600;margin:0 0 .4em}}
p{{color:#8998a8;margin:.3em 0}}
code{{font:13px ui-monospace,Menlo,Consolas,monospace;color:#e8eef5;
  background:#161c23;border:1px solid #232c36;border-radius:4px;padding:.1em .4em}}
.help{{margin-top:1.4em;border-top:1px solid #232c36;padding-top:1em}}
</style></head><body><main>
<h1>Connecting to the rover console&hellip;</h1>
<p>{detail}</p>
<p>Waiting {waited} s &mdash; retrying every {retry} s.</p>
{help}
</main></body></html>"""

HELP = """<div class="help">
<p>Is the service up?&ensp;<code>systemctl --user status {svc}</code></p>
<p>Why it failed:&ensp;<code>tail -40 ~/AGX_Orin_Backup/rover_project/logs/console.log</code></p>
<p>Not installed yet:&ensp;<code>~/AGX_Orin_Backup/rover_project/systemd/install.sh</code></p>
</div>"""


class RoverApp:
    def __init__(self, url, fullscreen, zoom):
        self.url = url
        self.full = False
        self.first_fail = None      # monotonic time of the first failed load
        self.service_note = None    # what starting the service said, once
        self.retry_id = None

        GLib.set_prgname(SERVICE)   # WM_CLASS, so the dock matches the launcher
        Gdk.set_program_class(SERVICE)
        self.win = Gtk.Window(title='Rover Control')
        self.win.set_default_size(1280, 800)
        if os.path.exists(ICON):
            self.win.set_icon_from_file(ICON)

        # The console sends no-store, but WebKit still keeps a memory cache of
        # subresources; a document-viewer model keeps that to a minimum.
        WebKit2.WebContext.get_default().set_cache_model(
            WebKit2.CacheModel.DOCUMENT_VIEWER)
        self.view = WebKit2.WebView()
        # Page console output reaches stdout, so running this from a terminal
        # shows the same JavaScript errors a browser's devtools would.
        self.view.get_settings().set_enable_write_console_messages_to_stdout(True)
        self.view.set_zoom_level(zoom)
        self.view.connect('load-failed', self.on_load_failed)
        self.view.connect('load-changed', self.on_load_changed)

        self.win.add(self.view)
        self.win.connect('key-press-event', self.on_key)
        self.win.connect('window-state-event', self.on_state)
        self.win.connect('destroy', Gtk.main_quit)
        if fullscreen:
            self.win.fullscreen()
        else:
            self.win.maximize()
        self.win.show_all()
        self.load()

    # ---- loading -------------------------------------------------------
    # retry_id is set exactly while the waiting page is up: from a failed load
    # until the next real one starts. WebKit reports FINISHED for the failed
    # load and for the waiting page too, but both land inside that window, so
    # a FINISHED with no retry pending can only be the console itself.
    def load(self):
        if self.retry_id is not None:
            GLib.source_remove(self.retry_id)
            self.retry_id = None
        self.view.load_uri(self.url)

    def on_load_changed(self, view, event):
        # The console answered: forget the outage, so a later one starts its
        # own clock and tries the service again.
        if event == WebKit2.LoadEvent.FINISHED and self.retry_id is None:
            self.first_fail = None
            self.service_note = None

    def on_load_failed(self, view, event, uri, error):
        # A load cut short by the next one (reload, the page's own build-change
        # reload) is not an outage.
        if error.matches(WebKit2.network_error_quark(),
                         WebKit2.NetworkError.CANCELLED):
            return False
        now = time.monotonic()
        if self.first_fail is None:
            self.first_fail = now
            self.service_note = self.start_service()
        waited = int(now - self.first_fail)
        page = WAIT_PAGE.format(
            detail=html.escape(self.service_note),
            waited=waited, retry=RETRY_S,
            help=HELP.format(svc=SERVICE) if waited >= HELP_AFTER_S else '')
        # alternate HTML keeps the console's URL as the page URI, so F5 on the
        # waiting page retries the console rather than reloading this text.
        view.load_alternate_html(page, uri, None)
        if self.retry_id is None:
            self.retry_id = GLib.timeout_add_seconds(RETRY_S, self.retry)
        return True

    def retry(self):
        self.retry_id = None        # this source is finishing; nothing to remove
        self.load()
        return False

    def start_service(self):
        """Start the console service if it is down. Returns a line for the page.

        Starting an already-running unit is a no-op, so this is safe to call
        when something else is merely slow to answer.
        """
        try:
            r = subprocess.run(['systemctl', '--user', 'start', SERVICE],
                               capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as e:
            return f'Could not ask systemd to start {SERVICE}: {e}'
        if r.returncode != 0:
            return (f'systemctl --user start {SERVICE} failed: '
                    f'{r.stderr.strip() or r.returncode}')
        return (f'Started the {SERVICE} service; '
                'it takes a few seconds to come up.')

    # ---- window --------------------------------------------------------
    def on_state(self, win, event):
        self.full = bool(event.new_window_state & Gdk.WindowState.FULLSCREEN)

    def on_key(self, win, event):
        k = event.keyval
        ctrl = bool(event.state & Gdk.ModifierType.CONTROL_MASK)
        if k == Gdk.KEY_F11:
            if self.full:
                self.win.unfullscreen()
            else:
                self.win.fullscreen()
        elif k == Gdk.KEY_F5 or (ctrl and k in (Gdk.KEY_r, Gdk.KEY_R)):
            self.load()
        elif ctrl and k in (Gdk.KEY_q, Gdk.KEY_Q):
            self.win.close()
        elif ctrl and k in (Gdk.KEY_plus, Gdk.KEY_equal, Gdk.KEY_KP_Add):
            self.view.set_zoom_level(min(self.view.get_zoom_level() + 0.1, 3.0))
        elif ctrl and k in (Gdk.KEY_minus, Gdk.KEY_KP_Subtract):
            self.view.set_zoom_level(max(self.view.get_zoom_level() - 0.1, 0.3))
        elif ctrl and k in (Gdk.KEY_0, Gdk.KEY_KP_0):
            self.view.set_zoom_level(1.0)
        else:
            return False            # everything else is the page's
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--fullscreen', action='store_true',
                    help='start full screen (F11 toggles)')
    ap.add_argument('--url', default=URL,
                    help=f'console address (default {URL})')
    ap.add_argument('--zoom', type=float, default=1.0,
                    help='page zoom, e.g. 1.25 on a small or distant screen')
    args = ap.parse_args()
    if not Gtk.init_check(sys.argv)[0]:
        sys.exit('rover_app: no display. Run it on the rover\'s desktop, '
                 'or open the console in a browser instead.')
    RoverApp(args.url, args.fullscreen, args.zoom)
    Gtk.main()


if __name__ == '__main__':
    main()
