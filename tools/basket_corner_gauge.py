#!/usr/bin/env python3
"""Printable paper gauge for the top basket's inside corner radius (the arm boxes' basket_r in the
Fusion design "XLeRobot 0.4 Upper Body Split Shoulders").

  python3 tools/basket_corner_gauge.py [--out docs/basket-corner-gauge.pdf]

One US Letter page, vector, no dependencies. Print it at "Actual size" (100 %), not "fit to page",
and check the 100 mm scale bar with a ruler first: printers often shrink the page by a few percent.

- Cards R10..R32: cut along the solid line (card stock is better than paper), then push the rounded
  corner flat into the basket corner on the floor. The right card touches both walls and the curve
  with no gap; a too-small radius touches the curve and leaves gaps along the walls, a too-large one
  touches the walls and leaves a gap at the curve. Check both front corners (flip the card over).
- Outside method: a square against the outside of the two walls, measure d along the 45 degree line
  from the square's inside corner to the basket, R_outside = d / (sqrt(2) - 1) = 2.414 d.
"""
import argparse
import math
import os

PAGE_W, PAGE_H = 215.9, 279.4          # US Letter, mm
PT = 72.0 / 25.4
K = 4.0 / 3.0 * math.tan(math.pi / 8)  # quarter-circle Bezier constant

RADII = list(range(10, 34, 2))
CARD_W, CARD_H = 56.0, 34.5
COLS, GAP = 3, 4.0


class Page:
    def __init__(self):
        self.ops = []

    def _p(self, x, y):
        return '%.3f %.3f' % (x * PT, (PAGE_H - y) * PT)

    def width(self, w):
        self.ops.append('%.3f w' % (w * PT))

    def dash(self, on=None):
        self.ops.append('[%.2f %.2f] 0 d' % (on * PT, on * PT) if on else '[] 0 d')

    def gray(self, g):
        self.ops.append('%.2f G %.2f g' % (g, g))

    def line(self, x0, y0, x1, y1):
        self.ops.append('%s m %s l S' % (self._p(x0, y0), self._p(x1, y1)))

    def path(self, segs, close=True):
        """segs: ('M', x, y) | ('L', x, y) | ('C', x1, y1, x2, y2, x, y), y down, mm."""
        out = []
        for s in segs:
            if s[0] == 'M':
                out.append('%s m' % self._p(*s[1:]))
            elif s[0] == 'L':
                out.append('%s l' % self._p(*s[1:]))
            else:
                out.append('%s %s %s c' % (self._p(*s[1:3]), self._p(*s[3:5]), self._p(*s[5:7])))
        self.ops.append(' '.join(out) + (' h S' if close else ' S'))

    def text(self, x, y, size, s, align='left', bold=False):
        w = 0.53 * size * len(s) / PT * (1.05 if bold else 1.0)   # rough Helvetica width, mm
        if align == 'center':
            x -= w / 2
        elif align == 'right':
            x -= w
        s = s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')
        self.ops.append('BT /%s %.1f Tf %s Td (%s) Tj ET' % ('F2' if bold else 'F1', size, self._p(x, y), s))


def card(pg, x, y, r):
    """Card with its top-right corner rounded to r (x, y = top-left corner)."""
    w, h = CARD_W, CARD_H
    pg.width(0.3)
    pg.dash()
    pg.path([('M', x, y), ('L', x + w - r, y),
             ('C', x + w - r + K * r, y, x + w, y + r - K * r, x + w, y + r),
             ('L', x + w, y + h), ('L', x, y + h)])
    # the two straight runs the walls should touch, and the tangent points
    pg.width(0.15)
    pg.dash(0.8)
    pg.line(x + w - r, y + 1.5, x + w - r, y + 4)
    pg.line(x + w - 1.5, y + r, x + w - 4, y + r)
    pg.dash()
    pg.text(x + 4, y + h / 2 + 4, 16, 'R%d' % r, bold=True)
    pg.text(x + 4, y + h - 4, 6.5, 'push this corner into the basket corner')


def outside_method(pg, x, y):
    """Diagram: a square outside the basket corner, d on the diagonal."""
    s, R = 34.0, 16.0
    cx, cy = x + s, y            # square's inside corner (virtual corner of the basket's outside)
    pg.width(0.25)
    pg.dash()
    # basket outside: walls along the square's two inside edges, rounded corner
    pg.path([('M', cx - s, cy), ('L', cx - R, cy),
             ('C', cx - R + K * R, cy, cx, cy + R - K * R, cx, cy + R), ('L', cx, cy + s)], close=False)
    pg.text(cx - s + 2, cy + 6, 6.5, 'basket (outside face)')
    # the square: an L outside the basket
    pg.width(0.6)
    pg.gray(0.45)
    pg.path([('M', cx - s, cy - 1), ('L', cx + 1, cy - 1), ('L', cx + 1, cy + s)], close=False)
    pg.gray(0)
    pg.text(cx + 3, cy + s - 2, 6.5, 'square')
    # diagonal and d
    t = R * (1 - 1 / math.sqrt(2))
    pg.width(0.2)
    pg.dash(0.7)
    pg.line(cx, cy, cx - t, cy + t)
    pg.dash()
    pg.text(cx - t / 2 + 1.5, cy + t / 2 - 1.0, 9, 'd', bold=True)


def build(out):
    pg = Page()
    m = 12.0
    pg.text(m, m + 4, 14, 'XLeRobot top basket: corner radius gauge', bold=True)
    pg.text(m, m + 10, 8, 'Print at "Actual size" (100 %), not "fit to page". Check the bar below measures 100 mm before cutting.')

    # 100 mm scale bar
    y = m + 18
    pg.width(0.4)
    pg.line(m, y, m + 100, y)
    for i in range(11):
        pg.line(m + 10 * i, y - (3 if i % 5 == 0 else 1.8), m + 10 * i, y)
    pg.text(m + 103, y + 1, 8, '100 mm', bold=True)

    # cards
    x0, y0 = m, y + 8
    for i, r in enumerate(RADII):
        c, row = i % COLS, i // COLS
        card(pg, x0 + c * (CARD_W + GAP + 6.0), y0 + row * (CARD_H + GAP), r)
    yb = y0 + math.ceil(len(RADII) / COLS) * (CARD_H + GAP) + 4

    # how to read the cards
    lines = [
        'Cards: cut on the solid line (card stock is best). Lay a card flat on the basket floor and push its rounded corner',
        'into a front corner. Right radius: it touches both walls and the curve, no gap anywhere. Too small: touches the curve,',
        'gaps along the walls. Too large: touches the walls, gap at the curve. Flip the card over for the other corner.',
        'The dashed ticks mark where the curve meets the straight edges. Report the radius that fits (or the two it falls between).',
    ]
    for i, s in enumerate(lines):
        pg.text(m, yb + 5 * i, 7.5, s)

    # outside method
    yd = yb + 26
    pg.text(m, yd, 9, 'Or measure from outside the basket', bold=True)
    outside_method(pg, m + 4, yd + 6)
    tx = m + 52
    txt = [
        'Hold a square against the outside of the two walls, just below the rim bead.',
        'Measure d at 45 degrees from the square\'s inside corner to the basket.',
        'R_outside = d / (sqrt 2 - 1) = 2.414 x d',
        'R_inside = R_outside - wall thickness (caliper the mesh, about 2 mm).',
        '',
        'd mm:   4      5      6      7      8      9     10     11     12',
        'R mm: 9.7  12.1  14.5  16.9  19.3  21.7  24.1  26.6  29.0',
    ]
    for i, s in enumerate(txt):
        pg.text(tx, yd + 8 + 5.2 * i, 7.5, s, bold=(i == 2))

    content = '\n'.join(pg.ops).encode('latin-1')
    objs = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        ('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.2f %.2f] /Resources << /Font << /F1 5 0 R /F2 6 0 R >> >> '
         '/Contents 4 0 R >>' % (PAGE_W * PT, PAGE_H * PT)).encode(),
        b'<< /Length %d >>\nstream\n' % len(content) + content + b'\nendstream',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>',
    ]
    data = b'%PDF-1.4\n'
    offs = []
    for i, o in enumerate(objs):
        offs.append(len(data))
        data += b'%d 0 obj\n' % (i + 1) + o + b'\nendobj\n'
    xref = len(data)
    data += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objs) + 1)
    for o in offs:
        data += b'%010d 00000 n \n' % o
    data += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objs) + 1, xref)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, 'wb') as f:
        f.write(data)
    print('wrote', out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'docs', 'basket-corner-gauge.pdf'))
    build(ap.parse_args().out)
