# Rover Console app

The rover console on the rover's own screen, in its own window, with **no
network needed** — no Wi-Fi, no router, no second device. Optionally opens full
screen every time the rover boots.

It is the same console page you get in a browser at `http://<rover-ip>:8080`,
so everything in [CONSOLE.md](../CONSOLE.md) applies unchanged.

---

## Use it

**Activities → "Rover Console"**. Right-click it for **Open full screen**, or
**Add to Favourites** to pin it to the dock.

From a terminal on the rover:

```bash
~/AGX_Orin_Backup/rover_project/desktop/rover_app.py --fullscreen
```

| option | |
|---|---|
| `--fullscreen` | start full screen |
| `--zoom 1.25` | bigger page, for a small or distant screen |
| `--url URL` | another console, e.g. a test one on `:8081` |

| key | |
|---|---|
| **F11** | full screen on / off |
| **F5**, Ctrl+R | reload |
| Ctrl + / − / 0 | zoom |
| Ctrl+Q | close |
| everything else | goes to the page: WASD, Space, Shift, 1–3, M, Esc |

**Esc does not leave full screen.** The page uses Esc to cancel a navigation
goal, and that must keep working in the one place it is most likely pressed.

Closing the window does **not** stop the console or a running stack. The page
stops sending drive commands and the console's deadman stops the motors, exactly
as when a browser tab is closed.

---

## Open it full screen at boot

Three things have to happen after power-on, each set up separately:

```
power on
 ├─ systemd user manager (linger)  →  rover-console service on :8080   systemd/install.sh
 └─ GDM automatic login            →  GNOME desktop as rptech          Settings → Users
      └─ ~/.config/autostart/rover-console.desktop
           └─ rover_app.py --fullscreen  →  http://127.0.0.1:8080       desktop/install.sh --autostart
```

The order between the two branches does not matter: if the app opens before
the console answers, it shows *Connecting…* and retries every 2 s, and starts
the service itself if it is not running.

### Setting it up

1. **Console service at boot**, once per machine (done on this rover):
   ```bash
   ~/AGX_Orin_Backup/rover_project/systemd/install.sh
   ```
2. **App at login**, full screen:
   ```bash
   ~/AGX_Orin_Backup/rover_project/desktop/install.sh --autostart
   ```
3. **Log in by itself at boot.** Without this, the rover stops at the login
   screen and the app only appears once someone types the password.
   **Settings → Users → Unlock… (your password) → Automatic Login: on.**

   This is a deliberate step for a person, not part of the install script: it
   means anyone who powers on the rover gets a desktop logged in as `rptech`,
   which has sudo. The full-screen app covers it, but the Super key and
   Alt+Tab still reach the desktop. Right for an expo; turn it off when the
   rover is not on show.
4. **Reboot and check:** the console should appear full screen without a
   keyboard being touched.

### Things that would quietly defeat it

All three are already right on this rover (checked 2026-10-14). Check them on
any other machine:

| check | want | why |
|---|---|---|
| `gsettings get org.gnome.desktop.session idle-delay` | `uint32 0` | otherwise the screen blanks after 5 min and then **locks** — the kiosk ends up behind a password prompt. Fix: `gsettings set org.gnome.desktop.session idle-delay 0` |
| `gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type` | `'nothing'` | otherwise the Jetson suspends when idle |
| `nmcli -g 802-11-wireless-security.psk-flags connection show "<wifi name>"` | `0` | `1` keeps the Wi-Fi password in the login keyring, which automatic login does not unlock — a "Unlock keyring" dialog pops up over the app. The app itself does not need Wi-Fi either way |

### Turning it off

```bash
~/AGX_Orin_Backup/rover_project/desktop/install.sh --no-autostart
```

and **Settings → Users → Automatic Login: off**. The autostart entry is also
listed, with an on/off tick, in the *Startup Applications* app.

---

## How it is built

### Two processes, one page

```
 rover's screen                              laptop / phone on Wi-Fi
 ┌─────────────────────────────┐             ┌────────────────────┐
 │ rover_app.py                │             │ any browser        │
 │ GTK 3 window + WebKit view  │             │                    │
 └──────────────┬──────────────┘             └─────────┬──────────┘
   http://127.0.0.1:8080 (loopback)          http://<rover-ip>:8080 (Wi-Fi)
                └──────────────────┬───────────────────┘
                 rover-console service: rover_dashboard (a ROS 2 node)
                 serves the page + JSON API, supervises the stacks
                                   │  ROS 2, all on this machine
                 micro-ROS agent · rplidar · slam_toolbox · Nav2 · RealSense
```

- **The console** is `rover_dashboard` in `ros2_ws/src/rover_core`, run by the
  `rover-console` user service. It is the only part that speaks ROS: it serves
  one self-contained page plus a JSON API on `:8080`, and starts and stops the
  stacks.
- **The app** is [`rover_app.py`](rover_app.py): a GTK 3 window whose only
  content is a WebKit2GTK web view (the engine behind Safari and GNOME Web)
  pointed at `http://127.0.0.1:8080`. It has no ROS code at all. Both libraries
  come with the JetPack desktop through `python3-gi`, so there was nothing to
  install.

### Why wrap the page instead of writing a native app

The page is about 1,300 lines that already drive, map, navigate, mark demo
points and supervise the stacks. A native copy would have to be kept in step
with it forever, and the copy shown at an expo would be the stale one. Wrapped,
there is one UI: a console fix appears in the app and in every browser at once,
and the page's own build check reloads the app after a rebuild too.

### Why it needs no network

- `127.0.0.1` is the **loopback** interface. It is inside the kernel and exists
  with every network adapter down, so the app always reaches the console.
- The page is **self-contained**: inline script and styles, no CDN and no web
  fonts, so nothing has to be fetched from the internet.
- **ROS 2 runs offline too.** Tested on this rover in a network namespace with
  only loopback (what Wi-Fi off looks like): messages flowed between nodes, with
  and without `ROS_LOCALHOST_ONLY`.
- **The service starts without a network.** `logs/console.log` shows it coming
  up as `dashboard on http://127.0.0.1:8080` at a boot where Wi-Fi was not up
  yet.

Wi-Fi is now only needed to use the console from another device.

### What happens when the console is not there

```
open window ─► load http://127.0.0.1:8080 ──ok──► console page
                     │
                   failed
                     ▼
     first failure? ─yes─► systemctl --user start rover-console
                     ▼
     show "Connecting…" page ─► wait 2 s ─► load again
     (after 20 s it also says where to look: service status, console.log)
```

The waiting page is loaded with WebKit's `load_alternate_html`, which keeps
the console's address as the page address — so F5 on the waiting page retries
the console instead of reloading the message. A load cancelled by the next one
(a reload, or the page reloading itself after a rebuild) is not counted as a
failure.

### Details that cost time, so they are written down

- **`/usr/bin/python3`, not `python3`.** An activated venv's Python cannot see
  the system's GTK bindings. The shebang and the launcher both name the system
  interpreter.
- **`Gdk` is pinned to 3.0.** GTK 4 is also installed; an unpinned `Gdk`
  import loads 4.0, after which GTK 3 refuses to load at all.
- **The window's class is `rover-console`** (`GLib.set_prgname`), matching
  `StartupWMClass` in the launcher. Without that, GNOME shows the running
  window as a second, unnamed icon in the dock.
- **The page's JavaScript console goes to stdout.** Run the app from a terminal
  to see page errors, the way browser devtools would show them.

### Files

| file | what it is |
|---|---|
| [`rover_app.py`](rover_app.py) | the app — window, web view, startup and retry, keys |
| [`install.sh`](install.sh) | writes the menu launcher; `--autostart` / `--no-autostart` for login |
| [`rover.svg`](rover.svg) | the icon |
| `~/.local/share/applications/rover-console.desktop` | menu launcher (written by install.sh) |
| `~/.config/autostart/rover-console.desktop` | login autostart (written by `--autostart`) |

---

## When it misbehaves

- **Stuck on *Connecting…*** — the console is not answering. After 20 s the
  page lists the two commands; the reason is almost always at the end of
  `logs/console.log`.
- **`rover_app: no display`** — it was run over SSH. Run it on the rover's own
  desktop, or point it there: `DISPLAY=:1 desktop/rover_app.py` (this rover's
  desktop is display `:1`).
- **Keys do nothing** — the page does not have keyboard focus. Click on it
  once.
- **Blank window** — run it from a terminal and read what WebKit prints. On
  Jetson, `WEBKIT_DISABLE_DMABUF_RENDERER=1 desktop/rover_app.py` is the first
  thing to try. It was not needed on this Orin Nano.
