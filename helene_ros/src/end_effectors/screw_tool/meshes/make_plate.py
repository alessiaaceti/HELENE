#!/usr/bin/env python3
"""Generates plate_with_hole.stl: a 60x60x3 mm plate with a round hole
(radius 2 mm) and a 0.5 mm x 45 deg chamfer, centred at the origin, top face
at z = 0. A single closed mesh, so the screw slides over it without catching
on seams between primitives. Run: python3 make_plate.py"""
import math
import os

L = 0.030          # half side
T = 0.003          # thickness
R = 0.002          # hole radius
C = 0.0005         # chamfer
N = 64             # segments (multiple of 8 so the square corners are hit)

tris = []


def quad(a, b, c, d):          # counter-clockwise seen from outside
    tris.append((a, b, c))
    tris.append((a, c, d))


def circle(r, z, i):
    a = 2 * math.pi * i / N
    return (r * math.cos(a), r * math.sin(a), z)


def square(z, i):
    a = 2 * math.pi * i / N
    c, s = math.cos(a), math.sin(a)
    k = L / max(abs(c), abs(s))
    return (k * c, k * s, z)


for i in range(N):
    j = (i + 1) % N
    # top face (normal +z): from chamfer edge out to the square
    quad(circle(R + C, 0, i), square(0, i), square(0, j), circle(R + C, 0, j))
    # chamfer (normal points up and towards the hole axis)
    quad(circle(R, -C, i), circle(R + C, 0, i), circle(R + C, 0, j), circle(R, -C, j))
    # hole wall (normal towards the axis)
    quad(circle(R, -T, i), circle(R, -C, i), circle(R, -C, j), circle(R, -T, j))
    # bottom face (normal -z)
    quad(circle(R, -T, i), circle(R, -T, j), square(-T, j), square(-T, i))
    # outer wall
    quad(square(0, i), square(-T, i), square(-T, j), square(0, j))


def normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(c * c for c in n)) or 1.0
    return tuple(c / m for c in n)


out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plate_with_hole.stl")
with open(out, "w") as f:
    f.write("solid plate\n")
    for t in tris:
        f.write("  facet normal %g %g %g\n    outer loop\n" % normal(t))
        for v in t:
            f.write("      vertex %.6f %.6f %.6f\n" % v)
        f.write("    endloop\n  endfacet\n")
    f.write("endsolid plate\n")
print("wrote", out, len(tris), "triangles")
