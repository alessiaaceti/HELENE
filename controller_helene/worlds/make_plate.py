#!/usr/bin/env python3
"""Generate plate_with_hole.stl: a square plate with a chamfered square through hole
(binary STL, metres). The chamfer is what lets a real screw (or a countersunk hole) find the
hole once the spiral gets close; without it a flat tool edge wedges against the hole wall.

  python3 make_plate.py [side] [thickness] [hole] [chamfer]
"""
import struct
import sys

side = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
thick = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
hole = float(sys.argv[3]) if len(sys.argv) > 3 else 0.010
chamfer = float(sys.argv[4]) if len(sys.argv) > 4 else 0.002

s, h, t = side / 2, hole / 2, thick
# plate centred on the hole, top face at z = t/2 (model pose lifts it so the top is at z = 0.05)
zt, zb = t / 2, -t / 2
tris = []


def quad(a, b, c, d):          # a, b, c, d counter-clockwise seen from outside
    tris.append((a, b, c))
    tris.append((a, c, d))


def sq(r, z):
    return [(-r, -r, z), (r, -r, z), (r, r, z), (-r, r, z)]


def band(inner, outer, up):      # flat ring between two squares, or a conical band
    for i in range(4):
        j = (i + 1) % 4
        a, b, c, d = outer[i], outer[j], inner[j], inner[i]
        if up:
            quad(a, b, c, d)
        else:
            quad(d, c, b, a)


c = chamfer
band(sq(h + c, zt), sq(s, zt), True)            # top face
band(sq(h, zb), sq(s, zb), False)               # bottom face
for i in range(4):                              # outer walls (normals outwards)
    j = (i + 1) % 4
    o = sq(s, 0)
    quad((o[i][0], o[i][1], zb), (o[j][0], o[j][1], zb), (o[j][0], o[j][1], zt), (o[i][0], o[i][1], zt))
# chamfer (normals up and towards the hole axis) and straight hole wall
top, mid, bot = sq(h + c, zt), sq(h, zt - c), sq(h, zb)
for i in range(4):
    j = (i + 1) % 4
    quad(mid[j], mid[i], top[i], top[j])        # chamfer
    quad(bot[j], bot[i], mid[i], mid[j])        # straight wall


def normal(a, b, c):
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    m = sum(x * x for x in n) ** 0.5 or 1.0
    return [x / m for x in n]


with open('plate_with_hole.stl', 'wb') as f:
    f.write(b'plate with square hole'.ljust(80, b' '))
    f.write(struct.pack('<I', len(tris)))
    for a, b, c in tris:
        f.write(struct.pack('<3f', *normal(a, b, c)))
        for p in (a, b, c):
            f.write(struct.pack('<3f', *p))
        f.write(b'\0\0')
print(len(tris), 'triangles')
