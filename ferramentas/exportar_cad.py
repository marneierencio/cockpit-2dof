"""Export of the cockpit to STEP files for SolidWorks.

Output (folder `solidworks/`):
  pecas/<code>_<name>.step   one file per distinct part. A welded assembly of steel
                             tubes (and the plates welded to it) is ONE part: the
                             hollow tubes (wall from PROFILES) are fused into one solid.
  Cockpit_2DOF_F1.step       assembly of every part instance in the F1 pose, nested
                             by the node tree (sub-assemblies = moving groups).
  manifesto_pecas.csv        part code, file, name, quantity, instances, material, mass
  posicoes.csv               placement (4x4 matrix, metres) of each instance
  juntas.csv                 kinematic joints (node, parent, type, axis, travel)

Native .SLDPRT / .SLDASM files cannot be written outside SolidWorks (proprietary
binary format). `solidworks/converter_para_solidworks.bas` converts these STEP
files into native files and builds the .SLDASM.

Run:  python3 ferramentas/exportar_cad.py        (needs cadquery, about 3 minutes)
"""

import csv
import math
import os
import re
import sys
import unicodedata

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cad  # noqa: E402
import verificar as V  # noqa: E402

ROOT = V.ROOT
OUT = os.path.join(ROOT, "solidworks")
DIR_PARTS = os.path.join(OUT, "pecas")
STEEL_KG_MM3 = 7.85e-6

# kinds that are welded into the weldment of their node
WELDED_KINDS = ("T", "CH")
# plates that are NOT welded to the weldment of their node (bolted / pinned items)
LOOSE_PLATE_PREFIXES = ("Braco manivela",)


def slug(text):
    t = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode()
    t = re.sub(r"[^A-Za-z0-9]+", "-", t).strip("-")
    return t[:48]


def vec(p):
    return cq.Vector(p[0], p[1], p[2])


def frame_loc(o, xa, ya, za):
    """cq.Location that maps the local frame (x, y, z) onto the world frame (o, xa, ya, za)."""
    return cq.Location(cq.Plane(origin=vec(o), xDir=vec(xa), normal=vec(za)))


def fuse_all(solids):
    """Fuse touching solids into one. Falls back to a compound when the fuse fails."""
    if len(solids) == 1:
        return solids[0], 1
    try:
        fused = solids[0].fuse(*solids[1:]).clean()
        n = len(fused.Solids())
        return fused, n
    except Exception:
        return cq.Compound.makeCompound(solids), len(solids)


def centred_solid(p):
    """Canonical (origin-centred) solid of a loose part plus the Location that places it."""
    g = p["geom"]
    k = g[0]
    if k == "box":
        _, o, xa, ya, za, e = g
        sx, sy, sz = abs(e[3] - e[0]), abs(e[4] - e[1]), abs(e[5] - e[2])
        c = [(e[0] + e[3]) / 2, (e[1] + e[4]) / 2, (e[2] + e[5]) / 2]
        ctr = [o[i] + xa[i] * c[0] + ya[i] * c[1] + za[i] * c[2] for i in range(3)]
        pl = cq.Plane(origin=cq.Vector(0, 0, 0), xDir=cq.Vector(1, 0, 0), normal=cq.Vector(0, 0, 1))
        solid = cq.Workplane(pl).rect(sx, sy).extrude(sz / 2.0, both=True).val()
        return solid, frame_loc(ctr, xa, ya, za), ("box", round(sx, 2), round(sy, 2), round(sz, 2))
    if k == "cyl":
        _, p0, p1, r, n = g
        d = [p1[i] - p0[i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        za = [x / L for x in d]
        xa = cad._perp(d)
        solid = cq.Workplane("XY").circle(r).extrude(L).val()
        return solid, frame_loc(p0, xa, [za[1] * xa[2] - za[2] * xa[1], za[2] * xa[0] - za[0] * xa[2],
                                        za[0] * xa[1] - za[1] * xa[0]], za), ("cyl", round(r, 2), round(L, 2))
    if k == "ball":
        _, c, r = g
        solid = cq.Solid.makeSphere(r, cq.Vector(0, 0, 0), angleDegrees1=-90, angleDegrees2=90)
        return solid, cq.Location(vec(c)), ("ball", round(r, 2))
    if k == "torus":
        _, o, xa, ya, za, R, r = g
        solid = cq.Solid.makeTorus(R, r, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1))
        return solid, frame_loc(o, xa, ya, za), ("torus", round(R, 2), round(r, 2))
    raise ValueError(k)


def build_parts(ns):
    """Group the model parts into distinct part definitions.

    Returns a list of dicts: code, name, shape, material, mass, nbody, instances
    [(part id, node, Location)], weld (bool), tubes (count)."""
    parts_by_node = {}
    for p in ns["PARTS"]:
        if p["kind"] == "MQ":
            continue
        parts_by_node.setdefault(p["node"], []).append(p)
    defs = []
    sig_index = {}
    ident = cq.Location()

    def is_welded(p):
        if p["kind"] == "T":
            return True
        return p["kind"] == "CH" and not p["desc"].startswith(LOOSE_PLATE_PREFIXES)

    for node, plist in parts_by_node.items():
        welded = [p for p in plist if is_welded(p)]
        if welded:
            members = [(p, cad.tube_solid(p["geom"][5]) if p["kind"] == "T" else cad.part_solid(p)) for p in welded]
            # weld connectivity: members whose solids touch (< 0.5 mm) form one weldment
            n = len(members)
            par = list(range(n))

            def find(i):
                while par[i] != i:
                    par[i] = par[par[i]]
                    i = par[i]
                return i
            boxes = [m[1].BoundingBox() for m in members]
            for i in range(n):
                for j in range(i + 1, n):
                    a, b = boxes[i], boxes[j]
                    if (a.xmax < b.xmin - 1 or b.xmax < a.xmin - 1 or a.ymax < b.ymin - 1 or b.ymax < a.ymin - 1
                            or a.zmax < b.zmin - 1 or b.zmax < a.zmin - 1):
                        continue
                    if members[i][1].distance(members[j][1]) < 0.5:
                        par[find(i)] = find(j)
            comps = {}
            for i in range(n):
                comps.setdefault(find(i), []).append(members[i])
            nm = ns["NODES"][node]["name"]
            main = sorted(comps.values(), key=lambda c: -sum(1 for m in c if m[0]["kind"] == "T"))
            k_w = 0
            for comp in main:
                ntube = sum(1 for m in comp if m[0]["kind"] == "T")
                shape, nb = fuse_all([m[1] for m in comp])
                mass_mm3 = sum(m[1].Volume() for m in comp)
                tube_mm3 = sum(m[1].Volume() for m in comp if m[0]["kind"] == "T")
                ids = ", ".join(m[0]["id"] for m in comp)
                if ntube:
                    k_w += 1
                    code = "SOLD-" + node + ("" if k_w == 1 else "-%d" % k_w)
                    name = "Soldado " + nm + ("" if k_w == 1 else " (%d)" % k_w)
                else:
                    code = comp[0][0]["id"]
                    name = comp[0][0]["desc"] + " [chapa sem tubo de apoio]"
                profs = sorted(set(m[0]["prof"] for m in comp if m[0]["kind"] == "T"))
                defs.append(dict(code=code, name=name, shape=shape,
                                 material="Aco carbono" + (" (tubos %s)" % "/".join(profs) if profs else ""),
                                 mass=mass_mm3 * STEEL_KG_MM3, tube_mass=tube_mm3 * STEEL_KG_MM3, nbody=nb,
                                 instances=[(code, node, ident)], weld=bool(ntube), tubes=ntube, members=ids,
                                 color=ns["COLORS"].get(comp[0][0]["mat"], (128, 128, 128))))
        groups = {}
        for p in plist:
            if is_welded(p):
                continue
            if p["group"]:
                groups.setdefault(p["group"], []).append(p)
                continue
            solid, loc, sig = centred_solid(p)
            key = (sig, p["mat"], p["spec"])
            if key in sig_index:
                d = defs[sig_index[key]]
                d["instances"].append((p["id"], node, loc))
                d["mass"] += 0.0
                continue
            sig_index[key] = len(defs)
            defs.append(dict(code=p["id"], name=p["desc"], shape=solid, material=p["mat"], mass=p["mass"], nbody=1,
                             instances=[(p["id"], node, loc)], weld=False, tubes=0, members=p["id"],
                             color=ns["COLORS"].get(p["mat"], (128, 128, 128)), spec=p["spec"], unit_mass=p["mass"]))
        for gname, gl in groups.items():
            solids = [cad.part_solid(p) for p in gl]
            defs.append(dict(code=gl[0]["id"], name="%s (%s)" % (gname.replace("_", " "), node), shape=cq.Compound.makeCompound(solids),
                             material="; ".join(sorted(set(p["mat"] for p in gl))), mass=sum(p["mass"] for p in gl), nbody=len(solids),
                             instances=[(gl[0]["id"], node, ident)], weld=False, tubes=0,
                             members=", ".join(p["id"] for p in gl), color=ns["COLORS"].get(gl[0]["mat"], (128, 128, 128)),
                             spec=gl[0]["spec"], unit_mass=sum(p["mass"] for p in gl)))
    return defs


def matrix_m(loc):
    """4x4 row-major matrix (metres) of a cq.Location."""
    m = loc.wrapped.Transformation()
    rows = []
    for r in (1, 2, 3):
        rows.append([m.Value(r, c) for c in (1, 2, 3)] + [m.Value(r, 4) / 1000.0])
    rows.append([0.0, 0.0, 0.0, 1.0])
    return rows


def write_outputs(ns, defs):
    os.makedirs(DIR_PARTS, exist_ok=True)
    for f in os.listdir(DIR_PARTS):
        if f.endswith(".step"):
            os.remove(os.path.join(DIR_PARTS, f))
    manifest, places = [], []
    for d in defs:
        fname = "%s_%s.step" % (d["code"], slug(d["name"]))
        d["file"] = fname
        cq.exporters.export(cq.Workplane("XY").newObject([d["shape"]]), os.path.join(DIR_PARTS, fname))
        vol = d["shape"].Volume()
        qty = len(d["instances"])
        d["volume_mm3"] = vol
        manifest.append([d["code"], "pecas/" + fname, d["name"], qty, "; ".join(i[0] for i in d["instances"]),
                         "soldado" if d["weld"] else "avulsa", d["tubes"], d["nbody"], d["material"], "%.1f" % vol,
                         "%.3f" % d["mass"], d["members"]])
        for iid, node, loc in d["instances"]:
            places.append([iid, d["code"], node] + ["%.6f" % x for row in matrix_m(loc) for x in row])
    with open(os.path.join(OUT, "manifesto_pecas.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["codigo", "arquivo_step", "nome", "quantidade", "instancias", "tipo", "tubos", "corpos", "material", "volume_mm3",
                    "massa_kg", "membros"])
        w.writerows(manifest)
    with open(os.path.join(OUT, "posicoes.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["instancia", "codigo_peca", "no"] + ["m%d%d" % (r, c) for r in range(1, 5) for c in range(1, 5)])
        w.writerows(places)
    return manifest


def node_path(ns, key):
    path = []
    while key is not None:
        path.append(key)
        key = ns["NODES"][key]["parent"]
    return list(reversed(path))


def write_assembly(ns, defs):
    """STEP assembly: one product per node (sub-assembly), instances placed in the F1 pose."""
    root = cq.Assembly(name="Cockpit_2DOF_F1")
    subs = {}

    def sub_of(key):
        if key in subs:
            return subs[key]
        a = cq.Assembly(name=ns["NODES"][key]["name"])
        subs[key] = a
        return a

    order = []
    for d in defs:
        for iid, node, loc in d["instances"]:
            order.append((d, iid, node, loc))
    for d, iid, node, loc in order:
        a = sub_of(node)
        r, g, b = d["color"]
        a.add(d["shape"], name="%s %s" % (iid, slug(d["name"])), loc=loc, color=cq.Color(r / 255.0, g / 255.0, b / 255.0, 1.0))
    # nest: every node of the tree becomes a sub-assembly of its parent (parents before children)
    for key in reversed(ns["NODE_ORDER"]):
        if key.startswith("MQ_"):
            continue
        par = ns["NODES"][key]["parent"]
        if key not in subs and not any(k for k in subs if key in node_path(ns, k)):
            continue                      # empty branch
        (sub_of(par) if par else root).add(sub_of(key), name=ns["NODES"][key]["name"])
    root.save(os.path.join(OUT, "Cockpit_2DOF_F1.step"), "STEP")


def write_joints(ns):
    """Joint table (axes and travel from F1 to truck) for the mates / motion study."""
    rows = []
    for key in ns["NODE_ORDER"]:
        nd = ns["NODES"][key]
        if nd["parent"] is None or key.startswith("MQ_"):
            continue
        kind = []
        if abs(nd["rot"]) > 1e-6:
            kind.append("rotacao_X")
        if max(abs(x) for x in nd["slide"]) > 1e-6:
            kind.append("translacao")
        if not kind:
            kind = ["fixo"]
        s = nd["slide"]
        rows.append([key, nd["name"], nd["parent"], "+".join(kind), "%.1f %.1f %.1f" % nd["origin"], "1 0 0",
                     "%.2f" % nd["rot"], "%.1f %.1f %.1f" % s, "porta" if nd["var"] == "aberta" else "modo"])
    with open(os.path.join(OUT, "juntas.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["no", "nome", "pai", "tipo", "origem_mm_xyz", "eixo_rotacao", "giro_graus_F1_para_caminhao", "curso_mm_xyz",
                    "variavel"])
        w.writerows(rows)


def main():
    ns = V.load_model()
    ns["COLORS"] = ns.get("COLORS", {})
    defs = build_parts(ns)
    manifest = write_outputs(ns, defs)
    write_assembly(ns, defs)
    write_joints(ns)
    weld_mass = sum(d["mass"] for d in defs if d["weld"])
    tube_mass = sum(p["mass"] for p in ns["PARTS"] if p["kind"] == "T")
    tube_cad = sum(d.get("tube_mass", 0.0) for d in defs)
    print("pecas:", len(defs), "instancias:", sum(len(d["instances"]) for d in defs))
    for d in defs:
        if d["weld"]:
            print("  %-14s corpos=%d tubos=%d vol=%.0f mm3 (%.2f kg)" % (d["code"], d["nbody"], d["tubes"], d["volume_mm3"], d["mass"]))
    print("tubos ocos no STEP: %.1f kg | tubos pelo gerador: %.1f kg (%.1f %%)" % (tube_cad, tube_mass, 100 * (tube_cad / tube_mass - 1)))
    print("soldados (tubos + chapas soldadas): %.1f kg" % weld_mass)
    for d in defs:
        if not d["weld"] and d["code"] != d["instances"][0][0]:
            pass
    return manifest


if __name__ == "__main__":
    main()
