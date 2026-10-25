#!/usr/bin/env python3
"""Printable ground-truth sheet for object pose tests (ADR-0005): markers around the edge give the
sheet's pose, and footprints of a cube and of a 2x4 brick are drawn at known positions and yaws.

  python3 tools/generate_placement_sheet.py [--out ~/placement_sheet]

Writes <out>.png, 262 x 197 mm at 300 dpi: print it with the width set to 262 mm (or at 100%), no fit to page and <out>.json with the marker
corners and the footprints in the sheet frame (millimetres, origin at the image centre, x right,
y up as printed, yaw counter-clockwise from +x in degrees). After printing, measure the distance
between two far markers with a ruler and give it to the evaluation as the sheet scale.
"""
import argparse
import json
import math
import os
import sys

_venv = os.path.expanduser("~/venvs/xlerobot-mcp/bin/python")
if os.path.exists(_venv) and os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(_venv))):
    os.execv(_venv, [_venv] + sys.argv)

import cv2  # noqa: E402
import cv2.aruco as aruco  # noqa: E402
import numpy as np  # noqa: E402

DPI = 300
PX_PER_MM = DPI / 25.4
SIZE_MM = (262.0, 197.0)       # the image itself, small enough for the printable area of a Letter or A4 page
MARKER_MM = 24.0
CUBE_MM = 30.0                 # footprint of a 28 mm cube, 1 mm clearance on each side
BRICK_MM = (32.0, 16.0)        # 2x4 Lego brick
COLUMNS = (-70.0, 0.0, 70.0)


def marker_layout(w, h):
    """Seven markers, 5 mm from the image edge: the four corners and three along the top edge (the bottom
    edge has only the corners so the brick footprints keep clear)."""
    mx, my = w / 2 - 5 - MARKER_MM / 2, h / 2 - 5 - MARKER_MM / 2
    return [(-mx, my), (mx, my), (-mx, -my), (mx, -my), (0, my), (-mx / 2, my), (mx / 2, my)]


def footprints():
    """Six cube footprints (yaw in 15 degree steps, a cube repeats every 90) and six 2x4 brick footprints
    (30 degree steps, a brick repeats every 180)."""
    out = []
    for r, y in enumerate((45.0, 5.0)):
        for c, x in enumerate(COLUMNS):
            k = r * 3 + c
            out.append({"id": f"cube{k}", "kind": "cube", "center_mm": [x, y], "yaw_deg": k * 15.0,
                        "size_mm": [CUBE_MM, CUBE_MM]})
    for r, y in enumerate((-33.0, -69.0)):
        for c, x in enumerate(COLUMNS):
            k = r * 3 + c
            out.append({"id": f"brick{k}", "kind": "brick", "center_mm": [x, y], "yaw_deg": k * 30.0,
                        "size_mm": list(BRICK_MM)})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.expanduser("~/placement_sheet"))
    a = ap.parse_args()
    w_mm, h_mm = SIZE_MM
    W, H = int(round(w_mm * PX_PER_MM)), int(round(h_mm * PX_PER_MM))
    img = np.full((H, W), 255, np.uint8)
    to_px = lambda x, y: (int(round(W / 2 + x * PX_PER_MM)), int(round(H / 2 - y * PX_PER_MM)))
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    side = int(round(MARKER_MM * PX_PER_MM))
    markers = []
    for i, (cx, cy) in enumerate(marker_layout(w_mm, h_mm)):
        m = aruco.drawMarker(dic, i, side)
        x0, y0 = to_px(cx, cy)
        img[y0 - side // 2:y0 - side // 2 + side, x0 - side // 2:x0 - side // 2 + side] = m
        half = MARKER_MM / 2
        # marker corners in the order the detector returns them: top-left, top-right, bottom-right, bottom-left
        corners = [[cx - half, cy + half], [cx + half, cy + half], [cx + half, cy - half], [cx - half, cy - half]]
        markers.append({"id": i, "center_mm": [cx, cy], "corners_mm": corners})
    fps = footprints()
    for f in fps:
        cx, cy = f["center_mm"]
        sx, sy = f["size_mm"]
        t = math.radians(f["yaw_deg"])
        loc = np.array([[-sx / 2, -sy / 2], [sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2]])
        rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        pts = (rot @ loc.T).T + np.array([cx, cy])
        cv2.polylines(img, [np.array([to_px(*p) for p in pts], np.int32)], True, 0, 3, cv2.LINE_AA)
        # a tick on the +x edge of the footprint marks its yaw direction
        p0, p1 = (rot @ np.array([sx / 2, 0])) + [cx, cy], (rot @ np.array([sx / 2 + 4, 0])) + [cx, cy]
        cv2.line(img, to_px(*p0), to_px(*p1), 0, 3, cv2.LINE_AA)
        cv2.putText(img, f["id"], to_px(cx - 34, cy - 1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
    cv2.imwrite(a.out + ".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    with open(a.out + ".json", "w") as f:
        json.dump({"size_mm": [w_mm, h_mm], "marker_mm": MARKER_MM, "dictionary": "DICT_4X4_50",
                   "markers": markers, "footprints": fps}, f, indent=1)
    print(f"{a.out}.png and .json: {w_mm:g} x {h_mm:g} mm, {len(markers)} markers of {MARKER_MM:g} mm, "
          f"{sum(f['kind'] == 'cube' for f in fps)} cube and {sum(f['kind'] == 'brick' for f in fps)} brick footprints")


if __name__ == "__main__":
    main()
