"""Geometric verification of the cockpit model (runs locally, no SketchUp).

Loads `modelo/gerar_modelo.py` with EMIT = False and checks:
  1. Interference between parts of different Dynamic Component nodes along
     the F1 <-> truck transition (modo = 0 .. 1), with the door bar closed,
     and with the door bar open in both modes.
  2. Mannequin (driver) against the structure in each mode.
  3. Moving platform against the fixed base over the whole pitch/roll
     envelope allowed by the cranks (rod displacement within +/- crank radius).

Every part is represented by an oriented bounding box (OBB); cylinders,
balls and mannequin segments are boxed as well, so results are conservative
(may report a few mm of overlap at rounded corners). Intended contacts --
hinges, pins, rod ends, carriages on rails -- are filtered by "joint
points": an overlap is ignored when a joint point lies within JOINT_R of
both boxes.

Usage: python3 ferramentas/verificar.py            (prints a report)
       imported by ferramentas/analisar.py          (load_model / run_all)
"""

import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JOINT_R = 45.0
TOL = 1.5


def load_model():
    """Execute the generator in a plain namespace and return it."""
    ns = {"math": math, "EMIT": False}
    path = os.path.join(ROOT, "modelo", "gerar_modelo.py")
    with open(path) as f:
        exec(compile(f.read(), path, "exec"), ns)
    return ns


# --------------------------------------------------------------------------- OBB

def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def part_obb(ns, p):
    """OBB in F1 world coordinates: (centre, (ax, ay, az), (hx, hy, hz))."""
    g = p["geom"]
    t = g[0]
    vn, vs, va, vl = ns["v_norm"], ns["v_sub"], ns["v_add"], ns["v_len"]
    if t == "tube":
        _, p0, p1, prof, up = g
        w, h, _ = ns["PROFILES"][prof]
        xa, ya, za = ns["axes_from"](vs(p1, p0), up)
        return ns["v_lerp"](p0, p1, 0.5), (xa, ya, za), (vl(vs(p1, p0)) / 2, w / 2, h / 2)
    if t == "box":
        _, o, xa, ya, za, e = g
        c = ns["local_to_world"](o, xa, ya, za, ((e[0] + e[3]) / 2, (e[1] + e[4]) / 2, (e[2] + e[5]) / 2))
        return c, (xa, ya, za), (abs(e[3] - e[0]) / 2, abs(e[4] - e[1]) / 2, abs(e[5] - e[2]) / 2)
    if t == "cyl":
        _, p0, p1, r, n = g
        za = vn(vs(p1, p0))
        xa, ya = ns["axis_frame"](za)
        return ns["v_lerp"](p0, p1, 0.5), (xa, ya, za), (r, r, vl(vs(p1, p0)) / 2)
    if t == "ball":
        _, c, r = g
        return c, ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (r, r, r)
    if t == "torus":
        _, o, xa, ya, za, R, r = g
        return o, (xa, ya, za), (R + r, R + r, r)
    if t == "bone":
        _, dn, a, b, sx, sy = g
        rad = {"Manequim_Coxa": 55, "Manequim_Canela": 40, "Manequim_Braco": 36, "Manequim_Antebraco": 27,
               "Manequim_Tronco": 110, "Manequim_Pescoco": 37}[dn]
        if vl(vs(b, a)) < 1:
            return None
        za = vn(vs(b, a))
        xa, ya = ns["axis_frame"](za)
        if dn == "Manequim_Tronco":
            # torso: wide along X, shallow front-to-back
            xa = (1.0, 0.0, 0.0)
            ya = _cross(za, xa)
            return ns["v_lerp"](a, b, 0.5), (xa, ya, za), (rad * sx * 0.95, rad * sy * 0.95, vl(vs(b, a)) / 2)
        return ns["v_lerp"](a, b, 0.5), (xa, ya, za), (rad * sx * 0.9, rad * sy * 0.9, vl(vs(b, a)) / 2)
    if t == "joint":
        _, dn, c, r, sz = g
        return c, ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (r * 0.9, r * 0.9, r * sz * 0.9)
    return None


def transform_obb(ob, fn_pt, fn_vec):
    c, axes, h = ob
    return fn_pt(c), tuple(fn_vec(a) for a in axes), h


def sat_depth(A, B):
    """Penetration depth (>0 overlap, <0 separation estimate) of two OBBs."""
    ca, aa, ha = A
    cb, ab, hb = B
    T = _sub(cb, ca)
    axes = list(aa) + list(ab)
    for u in aa:
        for v in ab:
            c = _cross(u, v)
            l = math.sqrt(_dot(c, c))
            if l > 1e-6:
                axes.append((c[0] / l, c[1] / l, c[2] / l))
    depth = 1e9
    for L in axes:
        ra = sum(ha[i] * abs(_dot(aa[i], L)) for i in range(3))
        rb = sum(hb[i] * abs(_dot(ab[i], L)) for i in range(3))
        d = ra + rb - abs(_dot(T, L))
        if d < depth:
            depth = d
            if depth < -50:
                return depth
    return depth


def point_obb_dist(p, ob):
    c, axes, h = ob
    d = _sub(p, c)
    s = 0.0
    for i in range(3):
        x = abs(_dot(d, axes[i])) - h[i]
        if x > 0:
            s += x * x
    return math.sqrt(s)


def aabb(ob):
    c, axes, h = ob
    e = [sum(abs(axes[i][k]) * h[i] for i in range(3)) for k in range(3)]
    return [c[k] - e[k] for k in range(3)], [c[k] + e[k] for k in range(3)]


def aabb_hit(a, b, pad=2.0):
    return all(a[0][k] - pad <= b[1][k] and b[0][k] - pad <= a[1][k] for k in range(3))


# ------------------------------------------------------------------- state maps

def state_obbs(ns, m, door=0.0, parts=None):
    out = []
    for i, p in enumerate(parts or ns["PARTS"]):
        if ns["node_hidden"](p["node"], m):
            continue
        ob = part_obb(ns, p)
        if ob is None:
            continue
        ang, tx, ty, tz = ns["node_tf"](p["node"], m, door)
        o = ns["NODES"][p["node"]]["origin"]

        def fp(q, ang=ang, o=o, tx=tx, ty=ty, tz=tz):
            r = ns["rot_x"](_sub(q, o), (0.0, 0.0, 0.0), ang)
            return (r[0] + tx, r[1] + ty, r[2] + tz)

        def fv(v, ang=ang):
            return ns["rot_x"](v, (0.0, 0.0, 0.0), ang)
        tob = transform_obb(ob, fp, fv)
        out.append((p, tob, aabb(tob)))
    return out


def joint_points(ns, m, door=0.0):
    pts = []
    for key, nd in ns["NODES"].items():
        if nd["parent"] is None:
            continue
        pts.append(ns["node_point"](key, nd["origin"], m, door))
    for p in ns["PARTS"]:
        # pins and the ends of rods / links (tubes of moving nodes) are joints
        if p["kind"] == "PI" or (p["kind"] == "T" and p["node"].startswith(("HASTE", "ARRASTO", "TIRANTE", "BIELA"))):
            for q in (p["line"] if p["line"] else ()):
                pts.append(ns["node_point"](p["node"], q, m, door))
    return pts


def is_mq(p):
    return p["asm"] == "MQ"


def interferences(ns, m, door=0.0, mannequin=False):
    """Pairs of parts in different nodes (or mannequin vs structure) that overlap."""
    obs = state_obbs(ns, m, door)
    jp = joint_points(ns, m, door)
    res = []
    for i in range(len(obs)):
        pa, A, ba = obs[i]
        for j in range(i + 1, len(obs)):
            pb, B, bb = obs[j]
            if mannequin != (is_mq(pa) != is_mq(pb)):
                continue
            if is_mq(pa) and is_mq(pb):
                continue
            if not mannequin and pa["node"] == pb["node"]:
                continue
            if pa["node"] == "BASE" or pb["node"] == "BASE":
                continue
            if not aabb_hit(ba, bb):
                continue
            d = sat_depth(A, B)
            lim = TOL
            if mannequin:
                other = pb if is_mq(pa) else pa
                lim = 35.0 if other["kind"] == "ES" else 3.0
            if d <= lim:
                continue
            if not mannequin and any(point_obb_dist((A[0][0], q[1], q[2]), A) < JOINT_R and
                                     point_obb_dist((B[0][0], q[1], q[2]), B) < JOINT_R for q in jp):
                continue
            if mannequin:
                mq, other = (pa, pb) if is_mq(pa) else (pb, pa)
                if other["desc"].startswith("Volante G920") and (mq["desc"].startswith("Mao") or mq["desc"].startswith("Antebraco")):
                    continue  # hands gripping the rim
            res.append((round(d, 1), pa["id"] or pa["desc"], pa["desc"], pb["id"] or pb["desc"], pb["desc"]))
    res.sort(reverse=True)
    return res


# ------------------------------------------------------------- motion envelope

def rot_y(v, deg):
    a = math.radians(deg)
    return (v[0] * math.cos(a) + v[2] * math.sin(a), v[1], -v[0] * math.sin(a) + v[2] * math.cos(a))


def platform_fn(ns, pitch, roll):
    P = ns["PIVOT"]

    def fv(v):
        return rot_y(ns["rot_x"](v, (0.0, 0.0, 0.0), pitch), roll)

    def fp(q):
        r = fv(_sub(q, P))
        return (r[0] + P[0], r[1] + P[1], r[2] + P[2])
    return fp, fv


def rod_disp(ns, pitch, roll):
    """Vertical displacement of the two rod top joints for a platform pose."""
    out = []
    for s in (-1, 1):
        q = (s * ns["ROD_X"], ns["ROD_Y"], ns["ZP"] - 83.0)
        fp, _ = platform_fn(ns, pitch, roll)
        out.append(fp(q)[2] - q[2])
    return out


EXCL_BASE = ("Garfo inferior", "Orelha garfo inferior", "Cruzeta eixo Y", "Braco manivela", "Terminal rotular",
             "Eixo saida", "Mola de tracao", "Olhal base mola", "Batente borracha")
EXCL_MOV = ("Garfo superior", "Orelha garfo superior", "Cruzeta eixo X", "Haste ligacao", "Terminal rotular",
            "Suporte haste traseira", "Olhal chassi mola")


def envelope(ns, m, pitch_max=12, roll_max=22, step=1.0):
    """Minimum clearance between moving and base parts over every reachable pose."""
    base = [p for p in ns["PARTS"] if p["node"] == "BASE" and not any(p["desc"].startswith(e) for e in EXCL_BASE)]
    mov = [p for p in ns["PARTS"] if p["node"] != "BASE" and not any(p["desc"].startswith(e) for e in EXCL_MOV)
           and ns["in_tree"](p["node"], "MOV")]
    bobs = state_obbs(ns, m, 0.0, base)
    mobs = state_obbs(ns, m, 0.0, mov)
    stops = [p for p in ns["PARTS"] if p["desc"].startswith("Batente borracha")]
    sobs = state_obbs(ns, m, 0.0, stops)
    worst = {}
    reach = []
    n_p, n_r = int(pitch_max / step), int(roll_max / step)
    for ip in range(-n_p, n_p + 1):
        for ir in range(-n_r, n_r + 1):
            pitch, roll = ip * step, ir * step
            dz = rod_disp(ns, pitch, roll)
            if max(abs(d) for d in dz) > ns["CRANK_R"]:
                continue
            reach.append((pitch, roll))
            fp, fv = platform_fn(ns, pitch, roll)
            moved = []
            for p, ob, _ in mobs:
                t = transform_obb(ob, fp, fv)
                moved.append((p, t, aabb(t)))
            for pb, B, bb in bobs + sobs:
                for pm, A, ba in moved:
                    if not aabb_hit(ba, bb, 60.0):
                        continue
                    d = sat_depth(A, B)
                    key = (pm["id"] or pm["desc"], pb["id"] or pb["desc"])
                    if key not in worst or d > worst[key][0]:
                        worst[key] = (d, pitch, roll, pm["desc"], pb["desc"])
    return worst, reach


def pitch_roll_limits(ns, m):
    """Pure pitch and pure roll reachable with the cranks, and the angle at
    which the rubber bump stops are first touched (pure pitch nose-down)."""
    out = {}
    for name, axis in (("arfagem", 0), ("rolagem", 1)):
        lim = 0.0
        a = 0.0
        while a < 40:
            a += 0.1
            dz = rod_disp(ns, a if axis == 0 else 0.0, a if axis == 1 else 0.0)
            if max(abs(d) for d in dz) > ns["CRANK_R"]:
                break
            lim = a
        out[name + "_max_graus"] = round(lim, 1)
    return out


def run_all(ns):
    rep = {}
    for m in (0.0, 0.25, 0.5, 0.75, 1.0):
        rep["transicao_modo_%.2f" % m] = interferences(ns, m)
    for m in (0.0, 1.0):
        rep["porta_aberta_modo_%d" % m] = interferences(ns, m, door=1.0)
        rep["porta_meio_modo_%d" % m] = interferences(ns, m, door=0.5)
        rep["manequim_modo_%d" % m] = interferences(ns, m, mannequin=True)
    env = {}
    for m in (0.0, 1.0):
        worst, reach = envelope(ns, m)
        lst = sorted(((round(v[0], 1), k[0], v[3], k[1], v[4], v[1], v[2]) for k, v in worst.items()), reverse=True)
        env[m] = (lst[:15], len(reach))
    rep["envoltoria"] = env
    rep["limites"] = pitch_roll_limits(ns, 0.0)
    return rep


if __name__ == "__main__":
    ns = load_model()
    rep = run_all(ns)
    for k, v in rep.items():
        if k == "envoltoria":
            for m, (lst, n) in v.items():
                print("== envoltoria modo %d (%d poses alcancaveis): maiores sobreposicoes (+) / menores folgas (-)" % (m, n))
                for row in lst:
                    print("   %6.1f mm  %s [%s]  x  %s [%s]  @ arfagem %.0f rolagem %.0f" % row)
            continue
        if k == "limites":
            print("==", k, v)
            continue
        print("== %s: %d" % (k, len(v)))
        for row in v[:40]:
            print("   %6.1f mm  %s [%s]  x  %s [%s]" % row)
    print("== hastes", {k: {kk: round(vv, 1) for kk, vv in d.items()} for k, d in ns["GEO"]["rods"].items()})
