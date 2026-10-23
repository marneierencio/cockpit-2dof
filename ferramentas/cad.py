"""CAD solids of the cockpit model (OpenCascade through cadquery).

Turns the parts of `modelo/gerar_modelo.py` into B-rep solids:
  - tubes: hollow rectangular tubes with the profile's wall thickness and rounded
    corners, cut by the end planes that `resolve_joints()` computed (mitre / butt);
  - plates, pins and bought-in items: boxes / cylinders / spheres / tori;
  - mannequin parts are not exported.

`interferencias()` is the weld-joint quality gate: inside one weldment (one node
of the model tree) two tubes may touch but not share volume.

Install: pip install cadquery
"""

import math
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verificar as V  # noqa: E402

ROOT = V.ROOT

# Cold-formed rectangular tube corners: outer radius ~ 1.5 t, inner radius ~ 0.5 t (EN 10219 / NBR 6591 practice)
R_OUT_FACTOR = 1.5


def _vec(p):
    return cq.Vector(p[0], p[1], p[2])


def _perp(d):
    """Any unit vector perpendicular to d."""
    n = math.sqrt(sum(x * x for x in d))
    d = [x / n for x in d]
    ref = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 1.0, 0.0)
    dot = sum(ref[i] * d[i] for i in range(3))
    w = [ref[i] - d[i] * dot for i in range(3)]
    m = math.sqrt(sum(x * x for x in w))
    return tuple(x / m for x in w)


def _plane(origin, xdir, normal):
    return cq.Plane(origin=_vec(origin), xDir=_vec(xdir), normal=_vec(normal))


def _prism(ti, su, sv, r, cut_planes, long=60.0):
    """Section su x sv (corner radius r) extruded along the tube axis and clipped by the end planes."""
    a, u, v = ti["a"], ti["u"], ti["v"]
    L = math.sqrt(sum((ti["p1"][i] - ti["p0"][i]) ** 2 for i in range(3)))
    # start `long` before p0, extrude L + 2 long; the end planes trim it
    start = tuple(ti["p0"][i] - a[i] * long for i in range(3))
    wp = cq.Workplane(_plane(start, u, a))
    if r > 0.01:
        sk = (wp.rect(su, sv).extrude(L + 2 * long).edges(cq.selectors.ParallelDirSelector(_vec(a)))
              .fillet(min(r, 0.49 * min(su, sv))))
    else:
        sk = wp.rect(su, sv).extrude(L + 2 * long)
    solid = sk.val()
    for (Q, n_out) in cut_planes:
        solid = _clip(solid, Q, n_out)
    return solid


def _clip(solid, Q, n_out, big=4000.0):
    """Remove the half space (P - Q).n_out > 0 from `solid`."""
    n = cq.Vector(*n_out).normalized()
    ref = cq.Vector(1, 0, 0) if abs(n.x) < 0.9 else cq.Vector(0, 1, 0)
    xd = ref.sub(n.multiply(ref.dot(n))).normalized()
    pl = cq.Plane(origin=_vec(Q), xDir=xd, normal=n)
    cutter = cq.Workplane(pl).rect(big, big).extrude(big).val()
    return solid.cut(cutter)


def tube_solid(ti, hollow=True):
    """Hollow (or solid) tube with its end cuts."""
    a = ti["a"]
    cuts = []
    for k in (0, 1):
        c = ti["cuts"][k]
        if c is not None and abs(sum(a[i] * c[1][i] for i in range(3))) > 0.05:
            cuts.append(c)
        else:
            Q, n = (ti["p0"], tuple(-x for x in a)) if k == 0 else (ti["p1"], a)
            cuts.append((Q, n))
    t = ti["wall"]
    r_out = R_OUT_FACTOR * t
    outer = _prism(ti, ti["su"], ti["sv"], r_out, cuts)
    if not hollow:
        return outer
    inner = _prism(ti, ti["su"] - 2 * t, ti["sv"] - 2 * t, max(r_out - t, 0.0), cuts, long=70.0)
    return outer.cut(inner)


def part_solid(p):
    """B-rep solid of one model part (in the model's F1 pose), or None."""
    g = p["geom"]
    k = g[0]
    if k == "tube":
        return tube_solid(g[5])
    if k == "box":
        _, o, xa, ya, za, e = g
        sx, sy, sz = abs(e[3] - e[0]), abs(e[4] - e[1]), abs(e[5] - e[2])
        c = [(e[0] + e[3]) / 2, (e[1] + e[4]) / 2, (e[2] + e[5]) / 2]
        ctr = [o[i] + xa[i] * c[0] + ya[i] * c[1] + za[i] * c[2] for i in range(3)]
        pl = cq.Plane(origin=_vec(ctr), xDir=_vec(xa), normal=_vec(za))
        return cq.Workplane(pl).rect(sx, sy).extrude(sz / 2.0, both=True).val()
    if k == "cyl":
        _, p0, p1, r, n = g
        d = [p1[i] - p0[i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        pl = cq.Plane(origin=_vec(p0), xDir=_vec(_perp(d)), normal=_vec(d))
        return cq.Workplane(pl).circle(r).extrude(L).val()
    if k == "ball":
        _, c, r = g
        return cq.Solid.makeSphere(r, _vec(c), angleDegrees1=-90, angleDegrees2=90)
    if k == "torus":
        _, o, xa, ya, za, R, r = g
        return cq.Solid.makeTorus(R, r, _vec(o), _vec(za))
    return None


def interferencias(ns, min_volume=2.0, hollow=False, gap=1.0):
    """Pairs of tubes of one weldment that share volume (mm3 > min_volume), plus the
    weldment-level gap check: tube ends whose end face touches nothing (nearest solid
    farther than `gap` mm; None = nothing nearby). Tubes of other nodes and plates / pins
    count as supports, because pinned joints reach across nodes."""
    by_node = {}
    for p in ns["PARTS"]:
        if p["kind"] == "T":
            by_node.setdefault(p["node"], []).append(p)
    solid_of = {p["id"]: tube_solid(p["geom"][5], hollow=hollow) for ps in by_node.values() for p in ps}
    box_of = {k: v.BoundingBox() for k, v in solid_of.items()}
    supports = [(q["id"], x) for q in ns["PARTS"] if q["kind"] in ("CH", "CO", "PI")
                for x in [part_solid(q)] if x is not None]
    sup_box = {k: x.BoundingBox() for k, x in supports}
    overlaps, floating = [], []

    def apart(a, b, pad):
        return (a.xmax < b.xmin - pad or b.xmax < a.xmin - pad or a.ymax < b.ymin - pad or b.ymax < a.ymin - pad
                or a.zmax < b.zmin - pad or b.zmax < a.zmin - pad)

    for node, tubes in by_node.items():
        for i in range(len(tubes)):
            for j in range(i + 1, len(tubes)):
                pi_, pj = tubes[i], tubes[j]
                if apart(box_of[pi_["id"]], box_of[pj["id"]], 0.1):
                    continue
                try:
                    vol = solid_of[pi_["id"]].intersect(solid_of[pj["id"]]).Volume()
                except Exception:
                    vol = -1.0
                if vol > min_volume or vol < 0:
                    overlaps.append((round(vol, 1), pi_["id"], pi_["desc"], pj["id"], pj["desc"]))
    for node, tubes in by_node.items():
        for p in tubes:
            ti = p["geom"][5]
            for k in (0, 1):
                ring = [_vec(q) for q in ti["pts"][k]]
                face = cq.Face.makeFromWires(cq.Wire.makePolygon(ring + [ring[0]]))
                bb = face.BoundingBox()
                dmin = 1e9
                for q in by_node.values():
                    for p2 in q:
                        if p2 is not p and not apart(bb, box_of[p2["id"]], 5):
                            dmin = min(dmin, face.distance(solid_of[p2["id"]]))
                for pid, x in supports:
                    if not apart(bb, sup_box[pid], 5):
                        dmin = min(dmin, face.distance(x))
                if dmin > gap:
                    floating.append((p["id"], p["desc"][:36], k, round(dmin, 1) if dmin < 1e8 else None))
    overlaps.sort(reverse=True)
    return overlaps, floating


if __name__ == "__main__":
    ns = V.load_model()
    ov, fl = interferencias(ns)
    print("sobreposicoes:", len(ov))
    for r in ov[:60]:
        print("  ", r)
    print("pontas soltas:", len(fl))
    for r in fl:
        print("  ", r)
