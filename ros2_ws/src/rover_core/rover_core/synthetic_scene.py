"""Render a floor with posts standing on it, as a camera at a known mount sees it.

A test fixture, kept in the package because the unit tests, the ROS pipeline
test and tools/check_front_cam_mount.py --demo all need the same scene. The
geometry comes from ground_vision's own projection, so it is exactly consistent
with the thing under test: if a post stands 1.00 m ahead here, the pipeline is
asked to find it at 1.00 m. It says nothing about real floors; that needs the
real camera.
"""
import cv2
import numpy as np

from rover_core.ground_vision import point_to_pixel, pixel_to_ground


def render_scene(cam, mount, posts=(), floor='tile', seed=0, noise=2.0,
                 base_bgr=(150, 140, 128), wall_bgr=(205, 205, 200)):
    """BGR frame. posts: iterable of (x, y, diameter_m, height_m, bgr)."""
    rng = np.random.default_rng(seed)
    h, w = cam.height, cam.width
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    px = np.stack([uu.ravel(), vv.ravel()], axis=1)
    g = pixel_to_ground(cam, mount, px)
    X, Y = g[:, 0].reshape(h, w), g[:, 1].reshape(h, w)
    sky = np.isnan(X)

    img = np.empty((h, w, 3), np.float32)
    img[:] = np.asarray(base_bgr, np.float32)
    dist = np.hypot(np.nan_to_num(X, nan=50.0) - mount.x, np.nan_to_num(Y) - mount.y)
    img *= (1.0 - 0.025 * np.minimum(dist, 6.0))[..., None]      # dimmer with distance
    if floor == 'tile':
        # 60 cm tiles with ~1.5 cm grout
        gx = np.abs(((np.nan_to_num(X) % 0.6) + 0.6) % 0.6) < 0.0075
        gy = np.abs(((np.nan_to_num(Y) % 0.6) + 0.6) % 0.6) < 0.0075
        gx |= np.abs((np.nan_to_num(X) % 0.6) - 0.6) < 0.0075
        gy |= np.abs((np.nan_to_num(Y) % 0.6) - 0.6) < 0.0075
        img[(gx | gy) & ~sky] *= 0.80
    elif floor == 'wood':
        planks = np.floor(np.nan_to_num(Y) / 0.12)
        img *= (0.94 + 0.06 * ((planks * 7919) % 5) / 4.0)[..., None]
    img[sky] = np.asarray(wall_bgr, np.float32)
    img += rng.normal(0.0, noise, img.shape).astype(np.float32)
    out = np.clip(img, 0, 255).astype(np.uint8)

    # Posts, far first so near ones cover them.
    plist = sorted(posts, key=lambda p: -np.hypot(p[0] - mount.x, p[1] - mount.y))
    for x, y, diam, height, bgr in plist:
        foot = point_to_pixel(cam, mount, [[x, y, 0.0]])[0]
        top = point_to_pixel(cam, mount, [[x, y, height]])[0]
        if np.isnan(foot).any() or np.isnan(top).any():
            continue
        fwd = max(0.05, np.hypot(x - mount.x, y - mount.y))
        wpx = max(1.0, cam.fx * diam / fwd)
        # A flat-ended bar from foot to top. cv2.line would round the ends and
        # push the foot below the floor point it stands on.
        half = wpx / 2.0
        quad = np.array([[foot[0] - half, foot[1]], [foot[0] + half, foot[1]],
                         [top[0] + half, top[1]], [top[0] - half, top[1]]])
        cv2.fillConvexPoly(out, np.round(quad).astype(np.int32), bgr)
    return out
