#!/usr/bin/env python3
"""Write a ChArUco board as a PNG to print at 100% scale on A4 or US Letter (landscape, 300 dpi).

  python3 tools/generate_charuco.py                       # 8x6 squares of 30 mm, 22 mm markers, A4
  python3 tools/generate_charuco.py --paper letter --squares-x 7 --squares-y 5 --square-length 0.037 --marker-length 0.028
  python3 tools/generate_charuco.py --out board.png --squares-x 8 --squares-y 6 --square-length 0.03

Print without "fit to page", then measure one square with a ruler. A calibration run takes the
sizes on its command line, e.g. tools/calibrate_arm_camera.py left --squares-x 8 --squares-y 6
--square-length 0.03 --marker-length 0.022 (pass the measured square size if it is not exact).
The head camera needs about 20 px per marker at its distance, the 11 mm board is too small for it.
"""
import argparse
import os
import sys

_venv = os.path.expanduser("~/venvs/xlerobot-mcp/bin/python")
if os.path.exists(_venv) and os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(_venv))):
    os.execv(_venv, [_venv] + sys.argv)

import cv2  # noqa: E402
import cv2.aruco as aruco  # noqa: E402
import numpy as np  # noqa: E402

DPI = 300
PAPER_MM = {"a4": (297, 210), "letter": (279.4, 215.9)}   # landscape, width x height


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.expanduser("~/charuco_8x6_30mm_A4.png"))
    ap.add_argument("--paper", choices=list(PAPER_MM), default="a4")
    ap.add_argument("--squares-x", type=int, default=8)
    ap.add_argument("--squares-y", type=int, default=6)
    ap.add_argument("--square-length", type=float, default=0.030)
    ap.add_argument("--marker-length", type=float, default=0.022)
    ap.add_argument("--dict", default="DICT_4X4_250")
    a = ap.parse_args()
    px = lambda m: int(round(m / 0.0254 * DPI))
    board = aruco.CharucoBoard_create(a.squares_x, a.squares_y, a.square_length, a.marker_length,
                                      aruco.getPredefinedDictionary(getattr(aruco, a.dict)))
    w, h = px(a.squares_x * a.square_length), px(a.squares_y * a.square_length)
    page = np.full((px(PAPER_MM[a.paper][1] / 1000), px(PAPER_MM[a.paper][0] / 1000)), 255, np.uint8)
    margin = px(0.010)       # printers cannot reach the page edge
    if w > page.shape[1] - 2 * margin or h > page.shape[0] - 2 * margin:
        sys.exit(f"board {a.squares_x * a.square_length * 1000:.0f} x {a.squares_y * a.square_length * 1000:.0f} mm "
                 f"does not fit {a.paper} with 10 mm margins")
    img = board.draw((w, h))
    y0, x0 = (page.shape[0] - h) // 2, (page.shape[1] - w) // 2
    page[y0:y0 + h, x0:x0 + w] = img
    cv2.imwrite(a.out, page, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f"{a.out}: {a.squares_x}x{a.squares_y} squares of {a.square_length * 1000:g} mm, markers {a.marker_length * 1000:g} mm, "
          f"{a.squares_x * a.square_length * 1000:.0f} x {a.squares_y * a.square_length * 1000:.0f} mm on {a.paper} landscape at {DPI} dpi")


if __name__ == "__main__":
    main()
