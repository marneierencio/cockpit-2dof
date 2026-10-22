# =============================================================================
# 2-DOF motion simulator cockpit - parametric model generator (v3)
#
# The file has two layers:
#   1. A pure-Python model (no SketchUp calls): geometry of every part, its
#      identification code, profile, length and mass, the Dynamic Component
#      node tree, mechanism synthesis, mass properties and the automatic
#      balancing of the centre of gravity over the pivot.
#   2. An emission layer (EMIT = True) that turns the model into SketchUp
#      geometry, Dynamic Component attributes, tags and scenes.
#
# Running it
#   - In the SketchUp MCP `build_model` sandbox: send the whole file as `code`
#     (with `clean: true`), then call `save_model`. EMIT defaults to True there.
#   - Locally (analysis, BOM, cut plan): `python3 ferramentas/analisar.py`, which
#     executes this file with EMIT = False and writes the documents.
#   The sandbox forbids `import`, `open`, `exec`, `eval`, `type` and dunder
#   access, so this file uses only `math` (pre-loaded there, injected locally).
#
# Conventions
#   - Millimetres everywhere; SketchUp's inch unit only appears in the emission
#     layer (MM = 1/25.4). Dynamic Component formulas use centimetres.
#   - Axes: X = lateral (driver's right is +X), Y = longitudinal (driver faces
#     +Y, rear of the moving platform is Y = 0), Z = up (floor is Z = 0).
#   - Sagittal angles are measured from +Y towards +Z (right-hand rule about +X).
#   - Tube profiles: "30x50" (30 x 50 x 1.2 mm), "20x30" (20 x 30 x 2.0 mm),
#     "10x20" (10 x 20 x 1.2 mm). The larger side follows the tube's `up` hint.
#   - Part codes: <assembly>-<kind><nn>. Assemblies: B base, M moving chassis,
#     A ring/roll frame, PO door bar, BC seat, PE pedals, VO steering, MO springs.
#     Kinds: T tube, CH plate/bracket, PI pin/shaft, CO bought-in item,
#     ES upholstery. Mannequin parts (MQ) are not part of the bill of materials.
#
# Model tree (model root)
#   Plataforma_Base (group)            fixed frame, pivot tower, cardan, motors, springs
#   Plataforma_Movel (DC, `modo`)      chassis + ring + adjustable modules
#     Carro_Banco (slides) -> Biela_Banco_* (rotate) -> Banco (level sub-frame)
#         -> Assento, Encosto -> Encosto_Superior, Tirante_Apoio_*, Haste_*
#     Tirante_Carro_* (drag links: carriage slide follows the link angle)
#     Carro_Pedaleira (slides) -> Pedaleira_Basculante (tilts; pin in cam slot)
#     Mastro_Volante (slides) -> Braco_Volante (tilts; pin in cam slot)
#     Porta_Barra (DC, own attribute `aberta`, opens upwards)
#     Manequim_F1 / Manequim_Caminhao (shown only at modo = 0 / 1)
# =============================================================================

try:
    EMIT
except Exception:
    EMIT = True

MM = 1.0 / 25.4
ZP = 380.0                    # top of the moving-chassis main rails
STEEL = 7.85e-6               # kg/mm3
PIVOT = (0.0, 700.0, 268.0)   # cardan spider centre (pitch and roll axes)
MZ, MY = 160.0, -40.0         # stepper output shaft height / longitudinal position
CRANK_R = 100.0               # crank radius (mm)
ROD_X, ROD_Y = 310.0, 60.0    # rear rod lateral / longitudinal position
RING_Z = ZP + 550.0           # ring (roll frame) centreline height
RING_X = 295.0                # ring lateral centreline
RING_REAR_Z = ZP + 380.0      # rear frame cross, lowered so the F1 headrest passes above it
DRIVER_KG = 90.0

PROFILES = {"30x50": (30.0, 50.0, 1.2), "20x30": (20.0, 30.0, 1.2), "10x20": (10.0, 20.0, 1.2)}

# ----------------------------------------------------------------------------
# Vector helpers (plain 3-tuples, mm)
# ----------------------------------------------------------------------------

def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def v_dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def v_cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def v_len(a): return math.sqrt(v_dot(a, a))
def v_lerp(a, b, t): return v_add(a, v_mul(v_sub(b, a), t))

def v_norm(a):
    l = v_len(a)
    return (a[0] / l, a[1] / l, a[2] / l)

def sag(deg):
    """Unit vector in the sagittal (Y-Z) plane at `deg` from +Y towards +Z."""
    a = math.radians(deg)
    return (0.0, math.cos(a), math.sin(a))

def sag_angle(v):
    """Sagittal angle (deg) of a vector, from +Y towards +Z."""
    return math.degrees(math.atan2(v[2], v[1]))

def rot_x(p, centre, deg):
    """Rotate p about the X-parallel axis through `centre` (right-hand rule)."""
    a = math.radians(deg)
    dy, dz = p[1] - centre[1], p[2] - centre[2]
    return (p[0], centre[1] + dy * math.cos(a) - dz * math.sin(a), centre[2] + dy * math.sin(a) + dz * math.cos(a))

def local_to_world(o, xa, ya, za, p):
    return v_add(o, v_add(v_mul(xa, p[0]), v_add(v_mul(ya, p[1]), v_mul(za, p[2]))))

def axes_from(d, up=(0, 0, 1)):
    """Right-handed frame with local X along d and local Z as close to `up` as possible."""
    xa = v_norm(d)
    u = v_sub(up, v_mul(xa, v_dot(up, xa)))
    if v_len(u) < 1e-6:
        u = (0, 1, 0) if abs(xa[1]) < 0.9 else (1, 0, 0)
        u = v_sub(u, v_mul(xa, v_dot(u, xa)))
    za = v_norm(u)
    return xa, v_cross(za, xa), za

def axis_frame(za):
    ref = (1, 0, 0) if abs(za[0]) < 0.9 else (0, 1, 0)
    xa = v_norm(v_sub(ref, v_mul(za, v_dot(ref, za))))
    return xa, v_cross(za, xa)

def bisector_point(p1, p2, fix, value):
    """Point on the sagittal perpendicular bisector of p1-p2 with y (fix='y') or
    z (fix='z') set to `value`: a fixed pivot equidistant from both positions."""
    dy, dz = p2[1] - p1[1], p2[2] - p1[2]
    my, mz = 0.5 * (p1[1] + p2[1]), 0.5 * (p1[2] + p2[2])
    if fix == "y":
        return (p1[0], value, mz - dy * (value - my) / dz)
    return (p1[0], my - dz * (value - mz) / dy, value)

def prof_area(p):
    w, h, t = PROFILES[p]
    return w * h - (w - 2 * t) * (h - 2 * t)

def prof_kgm(p):
    return prof_area(p) * STEEL * 1000.0

# ----------------------------------------------------------------------------
# Model registry: Dynamic Component nodes and parts
# ----------------------------------------------------------------------------

NODES = {}
NODE_ORDER = []
PARTS = []
COUNTERS = {}
GEO = {}
PREV = {}                     # values carried between balancing iterations

def reset():
    NODES.clear()
    del NODE_ORDER[:]
    del PARTS[:]
    COUNTERS.clear()
    GEO.clear()
    for key, name in (("BASE", "Plataforma_Base"), ("MOV", "Plataforma_Movel")):
        NODES[key] = dict(name=name, parent=None, origin=(0.0, 0.0, 0.0), rel=(0.0, 0.0, 0.0),
                          slide=(0.0, 0.0, 0.0), rot=0.0, hide=None, attrs=None, var="modo")
        NODE_ORDER.append(key)

def frac(poly, t):
    """Motion law of a node: fraction (0..1) of its full slide / rotation at
    t = modo. None is linear; (a, b) is the cubic a t + b t^2 + (1-a-b) t^3,
    fitted to the true kinematics of the linkage so the animation stays exact."""
    if poly is None:
        return t
    a, b = poly
    return a * t + b * t * t + (1.0 - a - b) * t * t * t

def fit_frac(samples):
    """Least-squares (a, b) for frac() from [(t, fraction)] samples."""
    s11 = s12 = s22 = r1 = r2 = 0.0
    for t, f in samples:
        u, w, y = t - t ** 3, t * t - t ** 3, f - t ** 3
        s11 += u * u
        s12 += u * w
        s22 += w * w
        r1 += u * y
        r2 += w * y
    det = s11 * s22 - s12 * s12
    return ((r1 * s22 - r2 * s12) / det, (s11 * r2 - s12 * r1) / det)

def frac_formula(poly, var):
    if poly is None:
        return var
    a, b = poly
    return "(%.5f*%s+%.5f*%s*%s+%.5f*%s*%s*%s)" % (a, var, b, var, var, 1.0 - a - b, var, var, var)

def node(key, name, parent, origin, slide=(0.0, 0.0, 0.0), rot=0.0, hide=None, var="modo", attrs=None):
    """Register a Dynamic Component node. origin: world point (F1 pose) of its
    local origin (hinge axis for rotating parts). slide (mm, parent frame) and
    rot (deg about local X) are the values at var = 1."""
    NODES[key] = dict(name=name, parent=parent, origin=origin, rel=v_sub(origin, NODES[parent]["origin"]),
                      slide=slide, rot=rot, hide=hide, var=var, attrs=attrs, rpoly=None, spoly=None)
    NODE_ORDER.append(key)
    return key

def new_id(asm, kind):
    k = asm + "-" + kind
    COUNTERS[k] = COUNTERS.get(k, 0) + 1
    return "%s-%s%02d" % (asm, kind, COUNTERS[k])

def part(nd, asm, kind, desc, geom, mat, mass, centroid, prof=None, length=None, line=None,
         spec=None, group=None, cut=None, bom=True):
    pid = new_id(asm, kind) if bom else ""
    PARTS.append(dict(id=pid, node=nd, asm=asm, kind=kind, desc=desc, geom=geom, mat=mat, mass=mass,
                      centroid=centroid, prof=prof, length=length, line=line, spec=spec, group=group,
                      cut=cut, bom=bom))
    return PARTS[-1]

def tube(nd, asm, desc, p0, p1, prof="20x30", up=(0, 0, 1), cut=None, group=None, mat=None):
    """Rectangular tube on the centreline p0 -> p1. Its drawn length is its cut length."""
    L = v_len(v_sub(p1, p0))
    return part(nd, asm, "T", desc, ("tube", p0, p1, prof, up), mat or ("T_" + prof), prof_kgm(prof) * L / 1000.0,
                v_lerp(p0, p1, 0.5), prof=prof, length=L, line=(p0, p1), cut=cut, group=group)

def box(nd, asm, kind, desc, o, xa, ya, za, ext, mat, mass=None, spec=None, group=None, bom=True):
    """Oriented box, ext = (x0, y0, z0, x1, y1, z1) in the frame (o, xa, ya, za)."""
    dx, dy, dz = abs(ext[3] - ext[0]), abs(ext[4] - ext[1]), abs(ext[5] - ext[2])
    c = local_to_world(o, xa, ya, za, (0.5 * (ext[0] + ext[3]), 0.5 * (ext[1] + ext[4]), 0.5 * (ext[2] + ext[5])))
    m = mass if mass is not None else dx * dy * dz * STEEL
    return part(nd, asm, kind, desc, ("box", o, xa, ya, za, ext), mat, m, c, spec=spec, group=group, bom=bom)

def abox(nd, asm, kind, desc, x0, y0, z0, x1, y1, z1, mat, mass=None, spec=None, group=None, bom=True):
    """Axis-aligned box between two corners."""
    return box(nd, asm, kind, desc, (0.0, 0.0, 0.0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
               (min(x0, x1), min(y0, y1), min(z0, z1), max(x0, x1), max(y0, y1), max(z0, z1)),
               mat, mass, spec, group, bom)

def cyl(nd, asm, kind, desc, p0, p1, r, mat, mass=None, spec=None, group=None, n=16, bom=True):
    L = v_len(v_sub(p1, p0))
    m = mass if mass is not None else math.pi * r * r * L * STEEL
    return part(nd, asm, kind, desc, ("cyl", p0, p1, r, n), mat, m, v_lerp(p0, p1, 0.5),
                length=L, line=(p0, p1), spec=spec, group=group, bom=bom)

def ball(nd, asm, kind, desc, c, r, mat, mass=None, spec=None, group=None, bom=True):
    m = mass if mass is not None else 4.0 / 3.0 * math.pi * r ** 3 * STEEL
    return part(nd, asm, kind, desc, ("ball", c, r), mat, m, c, spec=spec, group=group, bom=bom)

def pin(nd, asm, desc, c, half, r=8.0):
    """Hinge / locking pin along X centred on c."""
    if "indexador" in desc:
        # mode-locking pins are bought-in quick-release pins (one per side)
        spec = "Pino de trava rapida (ball lock) Ø%d x %d mm com argola" % (int(2 * r), int(2 * half))
    else:
        spec = "Pino aco 1045 Ø%d x %d mm + 2 travas (anel elastico) + 2 buchas soldadas" % (int(2 * r), int(2 * half))
    return cyl(nd, asm, "PI", desc, (c[0] - half, c[1], c[2]), (c[0] + half, c[1], c[2]), r, "Aco", spec=spec, n=12)

# ----------------------------------------------------------------------------
# Kinematics of the node tree (pure Python, mirrors the DC formulas)
# ----------------------------------------------------------------------------

def node_tf(key, m, door=0.0):
    """World transform of a node at modo = m (door = door opening 0..1):
    returns (angle_deg, tx, ty, tz) so that a point p given in F1 world
    coordinates maps to Rx(angle) * (p - origin) + t."""
    nd = NODES[key]
    if nd["parent"] is None:
        return (0.0, nd["origin"][0], nd["origin"][1], nd["origin"][2])
    pa, ptx, pty, ptz = node_tf(nd["parent"], m, door)
    s = door if nd["var"] == "aberta" else m
    rel = v_add(nd["rel"], v_mul(nd["slide"], frac(nd["spoly"], s)))
    a = math.radians(pa)
    oy = rel[1] * math.cos(a) - rel[2] * math.sin(a)
    oz = rel[1] * math.sin(a) + rel[2] * math.cos(a)
    return (pa + nd["rot"] * frac(nd["rpoly"], s), ptx + rel[0], pty + oy, ptz + oz)

def node_point(key, p, m, door=0.0):
    ang, tx, ty, tz = node_tf(key, m, door)
    o = NODES[key]["origin"]
    q = rot_x(v_sub(p, o), (0.0, 0.0, 0.0), ang)
    return (q[0] + tx, q[1] + ty, q[2] + tz)

def node_hidden(key, m):
    nd = NODES[key]
    while nd is not None:
        if nd["hide"] == "F1" and m > 0.001:
            return True
        if nd["hide"] == "TRK" and m < 0.999:
            return True
        nd = NODES[nd["parent"]] if nd["parent"] else None
    return False

def in_tree(key, root):
    while key is not None:
        if key == root:
            return True
        key = NODES[key]["parent"]
    return False

def mass_props(m, root="MOV"):
    """Mass, centre of gravity and inertia (pitch about X, roll about Y, both
    through PIVOT) of everything under `root` at modo = m, door closed."""
    M = 0.0
    S = [0.0, 0.0, 0.0]
    pts = []
    for p in PARTS:
        if not in_tree(p["node"], root) or node_hidden(p["node"], m):
            continue
        c = node_point(p["node"], p["centroid"], m)
        M += p["mass"]
        S = [S[0] + p["mass"] * c[0], S[1] + p["mass"] * c[1], S[2] + p["mass"] * c[2]]
        own_p = own_r = 0.0
        if p["line"] is not None and p["length"]:
            a = node_point(p["node"], p["line"][0], m)
            b = node_point(p["node"], p["line"][1], m)
            u = v_norm(v_sub(b, a))
            k = p["mass"] * p["length"] ** 2 / 12.0
            own_p, own_r = k * (1 - u[0] ** 2), k * (1 - u[1] ** 2)
        pts.append((p["mass"], c, own_p, own_r))
    cg = (S[0] / M, S[1] / M, S[2] / M)
    ip = sum(mm * ((c[1] - PIVOT[1]) ** 2 + (c[2] - PIVOT[2]) ** 2) + op for mm, c, op, orr in pts) * 1e-6
    ir = sum(mm * (c[0] ** 2 + (c[2] - PIVOT[2]) ** 2) + orr for mm, c, op, orr in pts) * 1e-6
    return dict(mass=M, cg=cg, I_pitch=ip, I_roll=ir)

# ----------------------------------------------------------------------------
# Anthropometry and postures
# ----------------------------------------------------------------------------

THIGH, SHANK, TORSO, UPPER_ARM, FOREARM_TO_GRIP = 420.0, 420.0, 520.0, 300.0, 310.0
HIP_HALF_WIDTH, SHOULDER_HALF_WIDTH = 90.0, 195.0
X = (1.0, 0.0, 0.0)
SUB_DEG = 9.0                 # seat sub-frame angle (the parallelogram keeps it constant)
LINK = 360.0                  # seat four-bar link length (pin to pin)

# Logitech G920 / Driving Force Shifter (published overall sizes, mm). Internal
# proportions (rim section, pad layout, mounting points) are estimates.
G920_RIM_R = 135.0
G920_WHEEL_PLANE = 80.0       # rim plane vs. the base's mounting face (estimate)
G920_PED_W, G920_PED_D, G920_PED_H = 428.5, 311.0, 167.0
G920_PAD_PITCH = 125.0
PEDAL_DEPTH = 130.0
SHIFTER_W, SHIFTER_D, SHIFTER_H = 130.0, 165.0, 167.0

# Driver segment mass fractions (Dempster / Winter).
SEG_FRAC = dict(head=0.081, trunk=0.497, upper_arm=0.028, forearm_hand=0.022, thigh=0.100, shank=0.0465, foot=0.0145)

# Postures before balancing. `hip` is (Y, height above ZP). The balancing loop
# translates each posture along Y (ergonomics unchanged) so that the combined
# centre of gravity of the moving platform sits over the pivot in both modes.
POSE_F1 = dict(tag="F1", hip=(650.0, 290.0), torso=145.0, pan=25.0, thigh=25.0, knee=145.0, ankle=100.0,
               knee_x=100.0, ankle_x={"E": -115.0, "D": 125.0}, head=(-40.0, 213.0),
               wheel_off=(420.0, 60.0), wheel_tilt=80.0, grip=90.0)
POSE_TRK = dict(tag="Caminhao", hip=(700.0, 597.0), torso=105.0, pan=9.0, thigh=5.0, knee=120.0, ankle=90.0,
                knee_x=130.0, ankle_x={"E": -115.0, "D": 125.0}, head=(-10.0, 230.0),
                wheel_off=(460.0, -100.0), wheel_tilt=45.0, grip=60.0)

def posture(c, shift):
    P = {"c": c}
    Hc = (0.0, c["hip"][0] + shift, ZP + c["hip"][1])
    tdir = sag(c["torso"])
    S = v_add(Hc, v_mul(tdir, TORSO))
    legs = {}
    for side, sx in (("E", -1), ("D", 1)):
        hx, kx, ax = sx * HIP_HALF_WIDTH, sx * c["knee_x"], c["ankle_x"][side]
        th = math.radians(c["thigh"])
        ps = math.radians(c["thigh"] - (180 - c["knee"]))
        lt = math.sqrt(THIGH ** 2 - (kx - hx) ** 2)
        ls = math.sqrt(SHANK ** 2 - (ax - kx) ** 2)
        hip = (hx, Hc[1], Hc[2])
        K = (kx, hip[1] + lt * math.cos(th), hip[2] + lt * math.sin(th))
        A = (ax, K[1] + ls * math.cos(ps), K[2] + ls * math.sin(ps))
        phi = math.degrees(ps) + 180 - c["ankle"]
        f = sag(phi)
        s = (0.0, f[2], -f[1])
        heel = v_add(A, v_add(v_mul(s, 70), v_mul(f, -50)))
        legs[side] = dict(hip=hip, K=K, A=A, f=f, s=s, heel=heel, phi=phi)
    tw_ = math.radians(c["wheel_tilt"])
    n = (0.0, -math.sin(tw_), math.cos(tw_))
    eu = (0.0, math.cos(tw_), math.sin(tw_))
    W = v_add(S, (0.0, c["wheel_off"][0], c["wheel_off"][1]))
    R, g = G920_RIM_R, math.radians(c["grip"])
    grips = {"E": v_add(W, v_add(v_mul(X, -R * math.sin(g)), v_mul(eu, R * math.cos(g)))),
             "D": v_add(W, v_add(v_mul(X, R * math.sin(g)), v_mul(eu, R * math.cos(g))))}
    elbows = {}
    for side, sx in (("E", -1), ("D", 1)):
        Psh, T = (sx * SHOULDER_HALF_WIDTH, S[1], S[2]), grips[side]
        dvec = v_sub(T, Psh)
        d = min(v_len(dvec), UPPER_ARM + FOREARM_TO_GRIP - 2)
        u = v_norm(dvec)
        pole = v_norm((sx * 0.35, -0.15, -1.0))
        pole = v_norm(v_sub(pole, v_mul(u, v_dot(pole, u))))
        a = (UPPER_ARM ** 2 - FOREARM_TO_GRIP ** 2 + d ** 2) / (2 * d)
        elbows[side] = (Psh, v_add(Psh, v_add(v_mul(u, a), v_mul(pole, math.sqrt(max(UPPER_ARM ** 2 - a ** 2, 0.0))))), T)
    al, be = math.radians(c["pan"]), math.radians(c["torso"])
    npn = (0.0, -math.sin(al), math.cos(al))
    nb = (0.0, math.sin(be), -math.cos(be))
    r1, r2 = v_dot(npn, Hc) - 95, v_dot(nb, Hc) - 115
    det = npn[1] * nb[2] - npn[2] * nb[1]
    C = (0.0, (r1 * nb[2] - npn[2] * r2) / det, (npn[1] * r2 - r1 * nb[1]) / det)
    Ph = (0.0, 0.5 * (legs["E"]["heel"][1] + legs["D"]["heel"][1]), 0.5 * (legs["E"]["heel"][2] + legs["D"]["heel"][2]))
    P.update(Hc=Hc, S=S, tdir=tdir, legs=legs, W=W, n=n, eu=eu, grips=grips, elbows=elbows, C=C, Ph=Ph,
             head=v_add(S, (0.0, c["head"][0], c["head"][1])))
    return P

def solve_pivot(p1, p2, alpha, pz):
    """Pivot (at height pz) and rail slide dy such that rotating p1 by alpha
    about the pivot and then sliding along Y lands on p2."""
    ca, sa = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    py = p1[1] - (p2[2] - pz - (p1[2] - pz) * ca) / sa
    q = rot_x(p1, (0.0, py, pz), alpha)
    return (0.0, py, pz), p2[1] - q[1]

def seg_dist(p, a, b):
    ab = v_sub(b, a)
    t = max(0.0, min(1.0, v_dot(v_sub(p, a), ab) / v_dot(ab, ab)))
    return v_len(v_sub(p, v_add(a, v_mul(ab, t))))

def sag_seg_dist(y, z, a, b):
    dy, dz = b[1] - a[1], b[2] - a[2]
    t = max(0.0, min(1.0, ((y - a[1]) * dy + (z - a[2]) * dz) / (dy * dy + dz * dz)))
    return math.hypot(y - a[1] - t * dy, z - a[2] - t * dz)

def leg_clearance(P, y, z, half):
    c = 1e9
    for L in P["legs"].values():
        c = min(c, sag_seg_dist(y, z, L["hip"], L["K"]) - 66 - half, sag_seg_dist(y, z, L["K"], L["A"]) - 50 - half)
    return c

def sight_margin(P, y, ztop):
    eye = v_add(P["head"], (0.0, 60.0, 20.0))
    top = v_add(P["W"], v_mul(P["eu"], G920_RIM_R + 14))
    if y <= top[1]:
        return 999.0
    return eye[2] + (top[2] - eye[2]) * (y - eye[1]) / (top[1] - eye[1]) - ztop

# ----------------------------------------------------------------------------
# Build: the whole model for a given pair of posture shifts (pure Python)
# ----------------------------------------------------------------------------

RAIL_TOP = ZP + 30.0          # top of the seat / pedal rails (30x50 laid flat)
CARR_Z = RAIL_TOP + 15.0      # carriage tube centreline
CARR_TOP = RAIL_TOP + 30.0

COARSE = False                # coarse mechanism searches (used while balancing)

# >>> PRESET (written by ferramentas/analisar.py - do not edit by hand)
PRESET = {'kl': 80, 'drag_gy': 350, 'back_cand': 33, 'back_gy': 470, 'pan_cand': 5, 'pan_gy': 460, 'ped_ll': 240, 'ped_ang': 180, 'ped_gz': 680, 'shift': [-17.785, -170.666]}
# <<< PRESET
# The SketchUp sandbox stops a script after 30 s, which is less than the
# mechanism searches take. When EMIT is on and PRESET holds the solution found
# by the last local run (same postures), the searches are restricted to that
# solution; locally (EMIT = False) the full searches always run and the tool
# rewrites PRESET from their result.
USE_PRESET = bool(EMIT and PRESET)

def grid(key, full):
    """Search grid for one parameter: the full range, or the preset value."""
    if USE_PRESET:
        return (PRESET[key],)
    return full

def build(shift_f1, shift_trk):
    reset()
    K = 3 if COARSE else 1
    F1, TRK = posture(POSE_F1, shift_f1), posture(POSE_TRK, shift_trk)
    GEO.update(F1=F1, TRK=TRK, shift=(shift_f1, shift_trk))
    O = (0.0, 0.0, 0.0)
    rep = {}

    # ======================= PLATAFORMA-BASE (fixed) =======================
    B = "BASE"
    Y0, Y1 = -200.0, 1850.0
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        tube(B, "B", "Longarina base " + sd, (s * 330, Y0, 15), (s * 330, Y1, 15), "20x30")
    for y in (-190.0, 40.0, 300.0, 560.0, 840.0, 1100.0, 1700.0, 1840.0):
        tube(B, "B", "Travessa base y%d" % int(y), (-320, y, 15), (320, y, 15), "20x30")
    tube(B, "B", "Diagonal base", (-320, 850, 15), (320, 1690, 15), "10x20", cut="angulo")
    for s in (-1, 1):
        tube(B, "B", "Apoio motor " + ("E" if s < 0 else "D"), (s * 115, -110, 15), (s * 115, 30, 15), "20x30")
    for sx in (-1, 1):
        for (yb, yt) in ((560, 670), (840, 730)):
            tube(B, "B", "Perna torre pivo", (sx * 200, yb, 30), (sx * 45, yt, 213), "20x30",
                 up=(0, 1 if yb < 700 else -1, 0), cut="angulo")
    abox(B, "B", "CH", "Chapa topo torre pivo 160x160x8", -80, 620, 220, 80, 780, 228, "Chapa")
    # cardan (bought-in universal joint): pitch about X, roll about Y
    cj = "Junta cardan (cruzeta com rolamentos de agulha), capacidade estatica >= 20 kN"
    abox(B, "B", "CO", "Garfo inferior cardan", -35, 655, 228, 35, 745, 250, "Cardan", spec=cj, group="Junta_Cardan")
    for sy in (-1, 1):
        abox(B, "B", "CO", "Orelha garfo inferior", -16, 700 + sy * 45 - 7, 250, 16, 700 + sy * 45 + 7, 285,
             "Cardan", spec=cj, group="Junta_Cardan")
    cyl(B, "B", "CO", "Cruzeta eixo Y (rolagem)", (0, 640, PIVOT[2]), (0, 760, PIVOT[2]), 13, "Aco", spec=cj, group="Junta_Cardan")
    actuators = []
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        abox(B, "B", "CH", "Chapa suporte motor 6mm " + sd, s * 289, -120, 30, s * 295, 0, 207, "Chapa")
        cyl(B, "B", "CO", "Redutor planetario " + sd, (s * 285, MY, MZ), (s * 190, MY, MZ), 42, "Aco", mass=3.0,
            spec="Redutor planetario 20:1 para NEMA 34 (flange 86 mm), torque nominal de saida >= 150 N.m")
        abox(B, "B", "CO", "Motor de passo NEMA 34 " + sd, s * 190, MY - 43, MZ - 43, s * 40, MY + 43, MZ + 43,
             "Motor", mass=5.5, spec="Motor de passo NEMA 34 12 N.m em malha fechada (ex.: 86HSE12N) + driver")
        abox(B, "B", "CH", "Calco motor " + sd, s * 105, MY - 30, 30, s * 125, MY + 30, MZ - 43, "Chapa")
        cyl(B, "B", "PI", "Eixo saida redutor " + sd, (s * 295, MY, MZ), (s * 318, MY, MZ), 10, "Aco")
        # crank: 12 mm plate (a 20x30 tube would see ~120 MPa cyclic at the motor's max force)
        abox(B, "B", "CH", "Braco manivela 10mm " + sd, s * ROD_X - 5, MY - 25, MZ - 25, s * ROD_X + 5, ROD_Y + 25,
             MZ + 25, "Aco")
        ball(B, "B", "CO", "Terminal rotular inferior " + sd, (s * ROD_X, ROD_Y, MZ), 13, "Aco",
             spec="Terminal rotular M16 (rod end)")
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        tube(B, "B", "Coluna batente " + sd, (s * 270, 1700, 30), (s * 270, 1700, 140), "20x30", up=(0, 1, 0))
        cyl(B, "B", "CO", "Batente borracha " + sd, (s * 270, 1700, 140), (s * 270, 1700, 175), 24, "Borracha",
            mass=0.1, spec="Batente de borracha Ø48 x 35")

    # =================== PLATAFORMA-MOVEL: chassis ==================
    M = "MOV"
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        tube(M, "M", "Longarina principal " + sd, (s * 270, 0, ZP - 25), (s * 270, 1780, ZP - 25), "30x50")
    CROSS = [(25.0, "30x50"), (300.0, "20x30"), (650.0, "30x50"), (700.0, "30x50"), (750.0, "30x50"), (900.0, "20x30"),
             (1100.0, "20x30"), (1450.0, "20x30"), (1765.0, "20x30")]
    for y, pr in CROSS:
        h = PROFILES[pr][1]
        tube(M, "M", "Travessa chassi y%d" % int(y), (-255, y, ZP - h / 2), (255, y, ZP - h / 2), pr)
    for (ya, yb) in ((25.0, 300.0), (1100.0, 1450.0)):
        tube(M, "M", "Diagonal piso y%d-%d" % (int(ya), int(yb)), (-250, ya + 15, ZP - 10), (250, yb - 15, ZP - 10),
             "10x20", cut="angulo")
    # the fork sits right on the three pivot crosses; the plate is a doubler, not a beam
    abox(M, "M", "CH", "Chapa pivo 300x140x8", -150, 630, ZP - 58, 150, 770, ZP - 50, "Chapa")
    # sill truss under the left main rail: the door gap interrupts the ring truss on
    # that side, so the rear rod force reaches the pivot crosses through this
    # shallow truss (top chord = main rail) instead of the rail alone in bending.
    door_f_est = PREV.get("door_f", 1080.0)
    # node stations (y) and truss depth below the rail centreline: deepest at the
    # pivot, where the rod moment F x 640 peaks, shallow at the ends (base clearance)
    sill = [(ROD_Y, 60.0), (300.0, 130.0), (700.0, 205.0), (door_f_est, 60.0)]
    zr = ZP - 25.0
    for i in range(len(sill) - 1):
        (ya, da), (yb, db) = sill[i], sill[i + 1]
        tube(M, "M", "Banzo inferior soleira E y%d-%d" % (int(ya), int(yb)), (-270, ya, zr - da), (-270, yb, zr - db), "30x50",
             cut="angulo")
    for y, d in sill:
        tube(M, "M", "Montante soleira E y%d" % int(y), (-270, y, zr - d + 25), (-270, y, ZP - 50), "20x30", up=(0, 1, 0))
    for i in range(len(sill) - 1):
        (ya, da), (yb, db) = sill[i], sill[i + 1]
        if i % 2 == 0:
            p0, p1 = (-270, ya + 10, zr - da + 25), (-270, yb - 10, ZP - 50)
        else:
            p0, p1 = (-270, ya + 10, ZP - 50), (-270, yb - 10, zr - db + 25)
        tube(M, "M", "Diagonal soleira E y%d-%d" % (int(ya), int(yb)), p0, p1, "20x30", cut="angulo")
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        abox(M, "M", "CH", "Suporte haste traseira " + sd, s * 285, ROD_Y - 25, ZP - 70, s * 330, ROD_Y + 25, ZP - 50, "Chapa")
        cyl(M, "M", "CO", "Haste ligacao " + sd, (s * ROD_X, ROD_Y, MZ + 13), (s * ROD_X, ROD_Y, ZP - 83), 8, "Aco",
            spec="Barra rosqueada M16 + porcas (entre rotulas)")
        ball(M, "M", "CO", "Terminal rotular superior " + sd, (s * ROD_X, ROD_Y, ZP - 83), 13, "Aco",
             spec="Terminal rotular M16 (rod end)")
    abox(M, "M", "CO", "Garfo superior cardan", -60, 670, 300, 60, 730, ZP - 58, "Cardan", spec=cj, group="Junta_Cardan")
    for sx in (-1, 1):
        abox(M, "M", "CO", "Orelha garfo superior", sx * 48 - 7, 684, 252, sx * 48 + 7, 716, 300, "Cardan", spec=cj,
             group="Junta_Cardan")
    cyl(M, "M", "CO", "Cruzeta eixo X (arfagem)", (-60, 700, PIVOT[2]), (60, 700, PIVOT[2]), 13, "Aco", spec=cj,
        group="Junta_Cardan")
    for s in (-1, 1):
        sd = "E" if s < 0 else "D"
        tube(M, "M", "Trilho banco " + sd, (s * 150, 5, ZP + 15), (s * 150, 1115, ZP + 15), "30x50", up=(1, 0, 0))
        tube(M, "M", "Trilho pedaleira " + sd, (s * 230, 740, ZP + 15), (s * 230, 1775, ZP + 15), "30x50", up=(1, 0, 0))

    # ======================= SEAT (1 DOF) =======================
    def seg_frame(nd, nm, o, d, back, length):
        """Spine of a seat segment: one 20x30 cross tube behind the cushion
        (offset 75 mm along `back`), at 40 % of the segment length, where the
        hinge pin and the lever plates attach. The 15 mm plywood board of the
        cushion (mass counted in the upholstery) carries the rest."""
        off = v_mul(back, 75.0)
        a = v_add(v_add(o, off), v_mul(d, 0.4 * length))
        tube(nd, "BC", "Espinha %s" % nm, v_add(a, (-220.0, 0.0, 0.0)), v_add(a, (220.0, 0.0, 0.0)), "20x30", up=back)
    P_SUB, N_SUB = sag(SUB_DEG), sag(SUB_DEG + 90)
    C1, C2 = F1["C"], TRK["C"]
    B_R1 = v_sub(v_add(C1, v_mul(P_SUB, 70)), v_mul(N_SUB, 95))
    B_F1 = v_sub(v_add(C1, v_mul(P_SUB, 380)), v_mul(N_SUB, 95))
    A_Z = CARR_TOP + 15.0
    TH1 = math.asin((B_R1[2] - A_Z) / LINK)
    TH2 = math.asin(math.sin(TH1) + (C2[2] - C1[2]) / LINK)
    A_R = v_sub(B_R1, v_mul((0.0, math.cos(TH1), math.sin(TH1)), LINK))
    A_F = v_add(A_R, v_sub(B_F1, B_R1))
    SEAT_SLIDE = (C2[1] - C1[1]) - LINK * (math.cos(TH2) - math.cos(TH1))
    LINK_ROT = math.degrees(TH2 - TH1)
    BACK_ROT = POSE_TRK["torso"] - POSE_F1["torso"]
    PAN_ROT = POSE_TRK["pan"] - POSE_F1["pan"]
    HEADREST_DEG = POSE_TRK["torso"]
    CARRO_O = (0.0, A_R[1], CARR_Z)
    node("CARRO", "Carro_Banco", "MOV", CARRO_O, slide=(0.0, SEAT_SLIDE, 0.0))
    ys, ye = A_R[1] - 60, A_F[1] + 60
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube("CARRO", "BC", "Longarina carro banco " + sd, (sx * 150, ys, CARR_Z), (sx * 150, ye, CARR_Z), "20x30")
        for nm, a_pt in (("tras", A_R), ("diant", A_F)):
            abox("CARRO", "BC", "CH", "Mancal biela %s %s" % (nm, sd), sx * 140, a_pt[1] - 20, CARR_TOP,
                 sx * 160, a_pt[1] + 20, a_pt[2] + 12, "Chapa")
            pin("CARRO", "BC", "Pino mancal biela %s %s" % (nm, sd), (sx * 167, a_pt[1], a_pt[2]), 15)
        pin("CARRO", "BC", "Pino indexador trilho banco " + sd, (sx * 150, ye - 25, RAIL_TOP - 5), 30, 6)
    tube("CARRO", "BC", "Travessa carro banco tras", (-140, ys + 15, CARR_Z), (140, ys + 15, CARR_Z), "20x30")
    tube("CARRO", "BC", "Travessa carro banco diant", (-140, ye - 15, CARR_Z), (140, ye - 15, CARR_Z), "20x30")

    BANCO_O = None
    for nm, a_pt in (("Tras", A_R), ("Diant", A_F)):
        for sx in (-1, 1):
            sd = "E" if sx < 0 else "D"
            b_pt = v_add(a_pt, v_sub(B_R1, A_R))
            a3, b3 = (sx * 180, a_pt[1], a_pt[2]), (sx * 180, b_pt[1], b_pt[2])
            key = "BIELA_%s_%s" % (sd, nm)
            node(key, "Biela_Banco_%s_%s" % (sd, nm), "CARRO", a3, rot=LINK_ROT)
            u = v_norm(v_sub(b3, a3))
            tube(key, "BC", "Biela banco %s %s" % (sd, nm), v_sub(a3, v_mul(u, 15)), v_add(b3, v_mul(u, 15)), "20x30",
                 up=(0, -math.sin(TH1), math.cos(TH1)), mat="Biela")
            if nm == "Tras" and sd == "E":
                node("BANCO", "Banco", key, b3, rot=-LINK_ROT)
                BANCO_O = b3
    # level sub-frame
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        rb, rf, rc = (sx * 262, B_R1[1], B_R1[2]), (sx * 262, B_F1[1], B_F1[2]), (sx * 262, C1[1], C1[2])
        tube("BANCO", "BC", "Longarina subchassi " + sd, v_sub(rb, v_mul(P_SUB, 25)), v_add(rf, v_mul(P_SUB, 25)),
             "20x30", up=N_SUB)
        tube("BANCO", "BC", "Montante articulacao " + sd, rb, rc, "20x30", up=N_SUB, cut="angulo")
        for b_pt in (B_R1, B_F1):
            pin("BANCO", "BC", "Pino biela subchassi " + sd, (sx * 221, b_pt[1], b_pt[2]), 45)
    pin("BANCO", "BC", "Eixo articulacao assento/encosto", C1, 290, 10)
    # F1 rest: in F1 the sub-frame rails sit on rubber pads on the chassis main
    # rails, so the near-flat links carry (almost) no load in F1.
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        for nm, b_pt in (("tras", B_R1), ("diant", B_F1)):
            yy = b_pt[1] + (30 if nm == "tras" else -30)
            top = b_pt[2] + (yy - b_pt[1]) * math.tan(math.radians(SUB_DEG)) - 15 / math.cos(math.radians(SUB_DEG))
            cyl("MOV", "BC", "CO", "Calco borracha apoio F1 %s %s" % (nm, sd), (sx * 264, yy, ZP), (sx * 264, yy, top), 14,
                "Borracha", mass=0.05, spec="Calco de borracha Ø28 com parafuso (apoio do subchassi em F1)")

    # pan
    P1, N1 = sag(POSE_F1["pan"]), sag(POSE_F1["pan"] + 90)
    node("ASSENTO", "Assento", "BANCO", C1, rot=PAN_ROT)
    box("ASSENTO", "BC", "ES", "Assento estofado 480x440x60", C1, X, P1, N1, (-240, 0, -60, 240, 440, 0), "Estofado",
        mass=3.0, spec="Assento estofado sobre base de compensado/aco")
    seg_frame("ASSENTO", "assento", C1, P1, v_mul(N1, -1), 440.0)
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        box("ASSENTO", "BC", "ES", "Aba assento " + sd, C1, X, P1, N1,
            (min(sx * 200, sx * 250), 30, 0, max(sx * 200, sx * 250), 380, 55), "Aba", mass=0.4)
    # back (lumbar) + upper shoulder/head segment at clavicle height
    back_dir, back_n = sag(POSE_F1["torso"]), v_cross(X, sag(POSE_F1["torso"]))
    D_J = v_dot(v_sub(F1["S"], C1), back_dir)
    J = v_add(C1, v_mul(back_dir, D_J))
    node("ENCOSTO", "Encosto", "BANCO", C1, rot=BACK_ROT)
    box("ENCOSTO", "BC", "ES", "Encosto lombar 480x%dx60" % round(D_J), C1, X, back_dir, back_n,
        (-240, 0, 0, 240, D_J, 60), "Estofado", mass=3.5, spec="Encosto estofado sobre base de compensado/aco")
    seg_frame("ENCOSTO", "encosto", C1, back_dir, back_n, D_J)
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        box("ENCOSTO", "BC", "ES", "Aba encosto " + sd, C1, X, back_dir, back_n,
            (min(sx * 200, sx * 250), 70, -65, max(sx * 200, sx * 250), D_J - 30, 0), "Aba", mass=0.5)
    pin("ENCOSTO", "BC", "Eixo articulacao apoio superior", J, 262, 9)
    hr_dir, hr_n = sag(HEADREST_DEG), v_cross(X, sag(HEADREST_DEG))
    node("APOIO", "Encosto_Superior", "ENCOSTO", J, rot=-BACK_ROT)
    box("APOIO", "BC", "ES", "Apoio ombros/cabeca 480x430x60", J, X, hr_dir, hr_n, (-240, 0, 0, 240, 430, 60),
        "Estofado", mass=2.5, spec="Apoio estofado sobre base de compensado/aco")
    seg_frame("APOIO", "apoio superior", J, hr_dir, hr_n, 430.0)
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        box("APOIO", "BC", "ES", "Aba ombro " + sd, J, X, hr_dir, hr_n,
            (min(sx * 245, sx * 275), 0, -90, max(sx * 245, sx * 275), 160, 0), "Aba", mass=0.4)
        box("APOIO", "BC", "ES", "Aba cabeca " + sd, J, X, hr_dir, hr_n,
            (min(sx * 112, sx * 142), 250, -100, max(sx * 112, sx * 142), 430, 0), "Aba", mass=0.3)
    # parallel tie rods: C' on the sub-frame, J' on the upper segment, offset v
    # from C and J; C-J-J'-C' is a parallelogram, so the upper segment keeps the
    # sub-frame's (constant) orientation: always HEADREST_DEG, no lock needed.
    v_tie = v_mul(sag(0.5 * (POSE_F1["torso"] + POSE_TRK["torso"]) + 90 + 180), 100.0)
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        c_t, j_t = v_add((sx * 268, C1[1], C1[2]), v_tie), v_add((sx * 268, J[1], J[2]), v_tie)
        key = "TIRANTE_" + sd
        node(key, "Tirante_Apoio_" + sd, "BANCO", c_t, rot=BACK_ROT)
        tube(key, "BC", "Tirante paralelo apoio superior " + sd, c_t, j_t, "10x20", up=back_n)
        abox("BANCO", "BC", "CH", "Orelha tirante subchassi " + sd, sx * 262 - 4, c_t[1] - 15, c_t[2] - 15,
             sx * 262 + 4, C1[1] + 15, C1[2] + 15, "Chapa")
        box("APOIO", "BC", "CH", "Orelha tirante apoio " + sd, J, X, hr_dir, hr_n,
            (sx * 250 - 4 if sx > 0 else sx * 250 - 4, -15, 60, sx * 250 + 4 if sx > 0 else sx * 250 + 4, 15, 100), "Chapa")
        pin(key, "BC", "Pino tirante inferior " + sd, (sx * 268, c_t[1], c_t[2]), 12, 6)
        pin(key, "BC", "Pino tirante superior " + sd, (sx * 268, j_t[1], j_t[2]), 12, 6)

    # coupling rods. Each rod grounds on the carriage (pan, back) or on the
    # chassis (drag links). The physical linkage is simulated with the rear
    # link angle as input (carriage slide from the drag link, pan and back
    # angles from their rods) and the ground points / lever geometry are
    # searched so that the worst transmission along the whole F1 <-> truck
    # travel stays far from a dead point. The DC animation interpolates the
    # same end poses linearly (exact at modo = 0 and 1).
    rods = {}
    NS = 36
    MIN_SIN = math.sin(math.radians(25.0))

    def sin2(a, b):
        """|sin| of the sagittal angle between vectors a and b."""
        la, lb = math.hypot(a[1], a[2]), math.hypot(b[1], b[2])
        return abs(a[1] * b[2] - a[2] * b[1]) / (la * lb) if la > 1e-9 and lb > 1e-9 else 0.0

    def drag_solve(g, kl, ld, th, sign):
        dy = A_R[1] + kl * math.cos(th) - g[1]
        dz = A_R[2] + kl * math.sin(th) - g[2]
        q = ld * ld - dz * dz
        if q < 0:
            return None
        return -dy + sign * math.sqrt(q)

    def drag_eval(g, kl):
        k1 = (0.0, A_R[1] + kl * math.cos(TH1), A_R[2] + kl * math.sin(TH1))
        ld = v_len(v_sub(k1, g))
        sign = 1.0 if k1[1] - g[1] >= 0 else -1.0
        worst, samples = 1.0, []
        for i in range(NS + 1):
            th = TH1 + (TH2 - TH1) * i / NS
            s = drag_solve(g, kl, ld, th, sign)
            if s is None:
                return -1.0, None, ld
            a = (0.0, A_R[1] + s, A_R[2])
            k = v_add(a, (0.0, kl * math.cos(th), kl * math.sin(th)))
            d = v_sub(k, g)
            worst = min(worst, abs(d[1]) / ld, sin2(d, (0.0, math.cos(th), math.sin(th))))
            samples.append((th, s))
        if abs(samples[-1][1] - SEAT_SLIDE) > 0.5:
            return -1.0, None, ld
        return worst, samples, ld

    best = None
    for kl in grid("kl", range(80, 300, 20 * K)):
        k1 = (0.0, A_R[1] + kl * math.cos(TH1), A_R[2] + kl * math.sin(TH1))
        k2 = (0.0, A_R[1] + SEAT_SLIDE + kl * math.cos(TH2), A_R[2] + kl * math.sin(TH2))
        for gy in grid("drag_gy", range(60, 745, 10 * K)):
            g = bisector_point(k1, k2, "y", float(gy))
            if not (ZP + 15 <= g[2] <= ZP + 200) or v_len(v_sub(k1, g)) < 60:
                continue
            w, smp, ld = drag_eval(g, float(kl))
            if smp and (best is None or w > best[0]):
                best = (w, float(kl), g, smp, ld)
                GEO["preset_kl"], GEO["preset_drag_gy"] = kl, gy
    DRAG_Q, KL, G_DRAG, SEAT_PATH, L_DRAG = best
    NODES["CARRO"]["spoly"] = fit_frac([(i / NS, SEAT_PATH[i][1] / SEAT_SLIDE) for i in range(NS + 1)])

    def banco_t(s, th):
        b = (0.0, A_R[1] + s + LINK * math.cos(th), A_R[2] + LINK * math.sin(th))
        return v_sub(b, (0.0, B_R1[1], B_R1[2]))

    def rot_solve(lever, g, L, s, th, guess):
        """Rotation (deg, about C) of a lever point so that its distance to the
        carriage ground point g (F1 coords) equals L; root nearest `guess`."""
        t = banco_t(s, th)
        gg = (0.0, g[1] + s, g[2])

        def f(phi):
            e = v_add(rot_x(lever, C1, phi), t)
            return math.hypot(e[1] - gg[1], e[2] - gg[2]) - L
        best_r = None
        for k in range(-60, 61):
            a, b = guess + k * 1.0, guess + (k + 1) * 1.0
            fa, fb = f(a), f(b)
            if fa == 0 or fa * fb < 0:
                for _ in range(40):
                    mid = 0.5 * (a + b)
                    if f(a) * f(mid) <= 0:
                        b = mid
                    else:
                        a = mid
                r = 0.5 * (a + b)
                if best_r is None or abs(r - guess) < abs(best_r - guess):
                    best_r = r
        return best_r

    def lever_eval(lever, g, total):
        e1 = lever
        L = math.hypot(e1[1] - g[1], e1[2] - g[2])
        worst, prev, out, armmin = 1.0, 0.0, [], 1e9
        for i, (th, s) in enumerate(SEAT_PATH):
            guess = prev if i else 0.0
            phi = rot_solve(lever, g, L, s, th, guess + total / NS)
            if phi is None or (i and abs(phi - prev) > 8.0):
                return -1.0, None, L, 0.0
            t = banco_t(s, th)
            e = v_add(rot_x(lever, C1, phi), t)
            c = v_add(C1, t)
            rd, la = v_sub(e, (0.0, g[1] + s, g[2])), v_sub(e, c)
            sn = sin2(rd, la)
            worst = min(worst, sn)
            armmin = min(armmin, math.hypot(la[1], la[2]) * sn)
            out.append(phi)
            prev = phi
        if abs(out[-1] - total) > 0.3:
            return -1.0, None, L, 0.0
        return worst, out, L, armmin

    def place_lever_rod(kind, total, cands, ylo, yhi):
        best_l = None
        if USE_PRESET:
            cands = [cands[PRESET[kind + "_cand"]]]
        for ci, lever in enumerate(cands):
            e1 = lever
            e2 = v_add(rot_x(lever, C1, total), banco_t(SEAT_SLIDE, TH2))
            e2c = v_sub(e2, (0.0, SEAT_SLIDE, 0.0))
            for gy in grid(kind + "_gy", range(int(ylo), int(yhi), 10 * K)):
                g = bisector_point(e1, e2c, "y", float(gy))
                if not (CARR_TOP + 10 <= g[2] <= CARR_TOP + 200):
                    continue
                if math.hypot(e1[1] - g[1], e1[2] - g[2]) < 80:
                    continue
                w, path, L, arm = lever_eval(lever, g, total)
                # objective: largest worst-case effective lever arm (lowest rod
                # force for a given seat moment), with transmission >= 25 deg
                if path and w >= MIN_SIN and (best_l is None or arm > best_l[5]):
                    best_l = (w, lever, g, path, L, arm)
                    GEO["preset_" + kind + "_cand"] = PRESET[kind + "_cand"] if USE_PRESET else ci
                    GEO["preset_" + kind + "_gy"] = gy
        return best_l[:5]

    back_cands = []
    for ln in (90.0, 120.0, 150.0, 180.0, 210.0, 240.0):
        for off in range(-90, 91, 15):
            d = sag(POSE_F1["torso"] + 180 + off)
            back_cands.append(v_add(C1, v_mul(d, ln)))
    BACK_Q, lever_b, gb, BACK_PATH, L_BACK = place_lever_rod("back", BACK_ROT, back_cands, ys + 15, ye - 15)
    pan_cands = []
    for d_al in range(100, 420, 40 * K):
        for d_n in range(40, 150, 20 * K):
            pan_cands.append(v_sub(v_add(C1, v_mul(P1, float(d_al))), v_mul(N1, float(d_n))))
    PAN_Q, lever_p, gp, PAN_PATH, L_PAN = place_lever_rod("pan", PAN_ROT, pan_cands, ys + 15, ye - 15)
    NODES["ASSENTO"]["rpoly"] = fit_frac([(i / NS, PAN_PATH[i] / PAN_ROT) for i in range(NS + 1)])
    bp = fit_frac([(i / NS, BACK_PATH[i] / BACK_ROT) for i in range(NS + 1)])
    for k in ("ENCOSTO", "APOIO", "TIRANTE_E", "TIRANTE_D"):
        NODES[k]["rpoly"] = bp

    def rod(key, name, parent, ground_pt, end_node, end_pt, asm, desc, quality, prof="20x30"):
        e1 = node_point(end_node, end_pt, 0.0)
        e2 = node_point(end_node, end_pt, 1.0)
        g1 = node_point(parent, ground_pt, 0.0)
        g2 = node_point(parent, ground_pt, 1.0)
        a1 = sag_angle(v_sub(e1, g1))
        a2 = sag_angle(v_sub(e2, g2))
        pr1, pr2 = node_tf(parent, 0.0)[0], node_tf(parent, 1.0)[0]
        da = (a2 - a1) - (pr2 - pr1)
        while da > 180:
            da -= 360
        while da < -180:
            da += 360
        node(key, name, parent, ground_pt, rot=da)
        smp, prev = [], a1
        for i in range(NS + 1):
            t = i / NS
            a = sag_angle(v_sub(node_point(end_node, end_pt, t), node_point(parent, ground_pt, t)))
            while a - prev > 180:
                a -= 360
            while a - prev < -180:
                a += 360
            prev = a
            smp.append((t, (a - a1 - (node_tf(parent, t)[0] - pr1)) / da))
        NODES[key]["rpoly"] = fit_frac(smp)
        L1, L2 = v_len(v_sub(e1, g1)), v_len(v_sub(e2, g2))
        tube(key, asm, desc, ground_pt, end_pt, prof, up=(1, 0, 0))
        # DC animation (linear interpolation) length error, for information
        anim = max(abs(v_len(v_sub(node_point(end_node, end_pt, i / 10.0), node_point(parent, ground_pt, i / 10.0))) - L1)
                   for i in range(11))
        rods[key] = dict(length=L1, err=abs(L1 - L2), a1=a1, a2=a2, transmissao_min_graus=math.degrees(math.asin(min(1.0, quality))),
                         desvio_animacao_mm=anim)
        return key

    # back rods: a pair beside the carriage rails (x = +-100), so each carries half
    # of the backrest moment and loads the carriage close to its side members
    bl = v_sub((0.0, lever_b[1], lever_b[2]), C1)
    gb0 = gb
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        lb = (sx * 100.0, lever_b[1], lever_b[2])
        g_ = (sx * 100.0, gb0[1], gb0[2])
        xs = sorted((sx * 117.0, sx * 129.0))
        box("ENCOSTO", "BC", "CH", "Alavanca encosto %s (chapa 12x70)" % sd, C1, X, v_norm(bl), v_cross(X, v_norm(bl)),
            (xs[0], -10, -60, xs[1], v_len(bl) + 15, 10), "Chapa")
        rod("HASTE_ENC_" + sd, "Haste_Encosto_" + sd, "CARRO", g_, "ENCOSTO", lb, "BC", "Haste comando encosto " + sd, BACK_Q)
        abox("CARRO", "BC", "CH", "Suporte haste encosto " + sd, xs[0], g_[1] - 20, CARR_TOP, xs[1], g_[1] + 20, g_[2] + 20,
             "Chapa")
    gb = gb0
    # pan rod
    lever_p = (32.0, lever_p[1], lever_p[2])
    gp = (32.0, gp[1], gp[2])
    pl = v_sub(lever_p, C1)
    lp_along = v_dot(pl, P1)
    box("ASSENTO", "BC", "CH", "Orelha haste assento (chapa 12)", C1, X, P1, N1,
        (51, lp_along - 35, v_dot(pl, N1) - 15, 63, lp_along + 35, -60), "Chapa")
    rod("HASTE_ASS", "Haste_Assento", "CARRO", gp, "ASSENTO", lever_p, "BC", "Haste comando assento", PAN_Q)
    abox("CARRO", "BC", "CH", "Suporte haste assento", 51, gp[1] - 20, CARR_TOP, 63, gp[1] + 20, gp[2] + 20, "Chapa")
    for gy_, nm in ((gb[1], "encosto"), (gp[1], "assento")):
        if min(abs(gy_ - (ys + 15)), abs(gy_ - (ye - 15))) > 25:
            tube("CARRO", "BC", "Travessa carro banco haste " + nm, (-140, gy_, CARR_Z), (140, gy_, CARR_Z), "20x30")
    # drag links: chassis -> rear link, so the carriage slide follows the link angle
    if min(abs(G_DRAG[1] - y) for y, _ in CROSS) > 25:
        tube(M, "M", "Travessa suporte tirante arrasto", (-255, G_DRAG[1], ZP - 15), (255, G_DRAG[1], ZP - 15), "20x30")
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        kp = (sx * 200, A_R[1] + KL * math.cos(TH1), A_R[2] + KL * math.sin(TH1))
        bk = "BIELA_%s_Tras" % sd
        g = (sx * 200, G_DRAG[1], G_DRAG[2])
        rod("ARRASTO_" + sd, "Tirante_Carro_" + sd, "MOV", g, bk, kp, "BC", "Tirante de arrasto do carro " + sd, DRAG_Q)
        abox("MOV", "BC", "CH", "Suporte tirante arrasto " + sd, sx * 190, g[1] - 12, ZP, sx * 210, g[1] + 12, g[2] + 12, "Chapa")
        pin(bk, "BC", "Pino tirante arrasto na biela " + sd, (sx * 190, kp[1], kp[2]), 12, 6)
    GEO.update(SEAT_PATH=SEAT_PATH, PAN_PATH=PAN_PATH, BACK_PATH=BACK_PATH, KL=KL)
    GEO["rods"] = rods

    # ======================= PEDALS (1 DOF) =======================
    PED_ALPHA = TRK["legs"]["D"]["phi"] - F1["legs"]["D"]["phi"]
    PED_P, PED_SLIDE = solve_pivot(F1["Ph"], TRK["Ph"], PED_ALPHA, ZP + 130.0)
    f1, s1 = F1["legs"]["D"]["f"], F1["legs"]["D"]["s"]
    ms1 = v_mul(s1, -1)
    Ph1 = F1["Ph"]
    pc_o = (0.0, PED_P[1], CARR_Z)
    node("PCARRO", "Carro_Pedaleira", "MOV", pc_o, slide=(0.0, PED_SLIDE, 0.0))
    pys, pye = PED_P[1] - 90, PED_P[1] + 140
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube("PCARRO", "PE", "Longarina carro pedaleira " + sd, (sx * 230, pys, CARR_Z), (sx * 230, pye, CARR_Z), "20x30")
        tube("PCARRO", "PE", "Torre pivo pedaleira " + sd, (sx * 230, PED_P[1], CARR_TOP), (sx * 230, PED_P[1], PED_P[2] + 20),
             "20x30", up=(0, 1, 0))
        pin("PCARRO", "PE", "Pino indexador trilho pedaleira " + sd, (sx * 230, pye - 25, RAIL_TOP - 5), 30, 6)
    tube("PCARRO", "PE", "Travessa carro pedaleira", (-220, pye - 15, CARR_Z), (220, pye - 15, CARR_Z), "20x30")
    pin("PCARRO", "PE", "Eixo pivo pedaleira", PED_P, 245, 9)
    node("DECK", "Pedaleira_Basculante", "PCARRO", PED_P, rot=PED_ALPHA)
    DY0 = -60.0
    box("DECK", "PE", "CH", "Chapa fixacao pedais 400x331x2", Ph1, X, f1, ms1,
        (-200, DY0 - 10, -(PEDAL_DEPTH + 2), 200, DY0 + G920_PED_D + 10, -PEDAL_DEPTH), "Chapa")
    xc = 0.5 * (F1["legs"]["E"]["heel"][0] + F1["legs"]["D"]["heel"][0])
    g920p = "Logitech G920 - pedais (428,5 x 311 x 167 mm, 3,1 kg)"
    box("DECK", "PE", "CO", "Pedais G920 (carcaca)", Ph1, X, f1, ms1,
        (xc - G920_PED_W / 2, DY0, -PEDAL_DEPTH, xc + G920_PED_W / 2, DY0 + G920_PED_D, -PEDAL_DEPTH + 55), "Motor",
        mass=2.4, spec=g920p, group="G920_Pedais")
    box("DECK", "PE", "CO", "Pedais G920 (apoio calcanhar)", Ph1, X, f1, ms1,
        (xc - 190, DY0, -PEDAL_DEPTH + 55, xc + 190, DY0 + 70, -PEDAL_DEPTH + 75), "Carbono", mass=0.1, spec=g920p,
        group="G920_Pedais")
    for nm, px in (("embreagem", xc - G920_PAD_PITCH), ("freio", xc), ("acelerador", xc + G920_PAD_PITCH)):
        box("DECK", "PE", "CO", "Pedais G920 (pedal %s)" % nm, Ph1, X, f1, ms1, (px - 37, 85, -24, px + 37, 245, -8),
            "PedalAl", mass=0.2, spec=g920p, group="G920_Pedais")
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        pv = (sx * 190, PED_P[1], PED_P[2])
        lo = local_to_world(Ph1, X, f1, ms1, (sx * 190, DY0 + 20, -(PEDAL_DEPTH + 23)))
        hi = local_to_world(Ph1, X, f1, ms1, (sx * 190, DY0 + G920_PED_D - 10, -(PEDAL_DEPTH + 23)))
        tube("DECK", "PE", "Braco pedaleira inferior " + sd, pv, lo, "20x30", cut="angulo")
        tube("DECK", "PE", "Braco pedaleira superior " + sd, pv, hi, "20x30", cut="angulo")
    for nm, yy in (("inferior", DY0 + 20), ("superior", DY0 + G920_PED_D - 10)):
        tube("DECK", "PE", "Travessa pedaleira " + nm, local_to_world(Ph1, X, f1, ms1, (-200, yy, -(PEDAL_DEPTH + 23))),
             local_to_world(Ph1, X, f1, ms1, (200, yy, -(PEDAL_DEPTH + 23))), "20x30", up=ms1)
    # coupling rod: a lever on the deck's torsion tube is tied to the chassis by
    # one rod on the centreline, so sliding the carriage tilts the deck (one
    # movement, no machined slot). Physical kinematics simulated as for the seat.
    tube("DECK", "PE", "Tubo de torcao pedaleira", (-180, PED_P[1], PED_P[2]), (180, PED_P[1], PED_P[2]), "20x30")

    # lowest points of the deck: frame (between the rails, must clear the chassis
    # floor) and the G920 base edges (overhang the rails, must clear them)
    deck_low = [(local_to_world(Ph1, X, f1, ms1, (xx, yy, zz)), ZP + 8) for xx in (-200.0, 200.0)
                for yy in (DY0 - 10, DY0 + 10, DY0 + 30, DY0 + G920_PED_D + 10)
                for zz in (-(PEDAL_DEPTH + 38), -(PEDAL_DEPTH + 3))]
    deck_low += [(local_to_world(Ph1, X, f1, ms1, (xx, yy, -PEDAL_DEPTH)), RAIL_TOP + 5)
                 for xx in (xc - G920_PED_W / 2, xc + G920_PED_W / 2) for yy in (DY0, DY0 + G920_PED_D)]

    def ped_eval(lever, g):
        L = math.hypot(lever[1] - g[1], lever[2] - g[2])
        worst, prev, out, armmin = 1.0, 0.0, [], 1e9
        for i in range(NS + 1):
            sl = PED_SLIDE * i / NS
            pv = (0.0, PED_P[1] + sl, PED_P[2])

            def f(a):
                e = v_add(rot_x(lever, PED_P, a), (0.0, sl, 0.0))
                return math.hypot(e[1] - g[1], e[2] - g[2]) - L
            guess = prev + (PED_ALPHA / NS if i else 0.0)
            root = None
            for k in range(-40, 40):
                a, b = guess + k * 0.5, guess + (k + 1) * 0.5
                if f(a) == 0 or f(a) * f(b) < 0:
                    for _ in range(40):
                        mid = 0.5 * (a + b)
                        if f(a) * f(mid) <= 0:
                            b = mid
                        else:
                            a = mid
                    r_ = 0.5 * (a + b)
                    if root is None or abs(r_ - guess) < abs(root - guess):
                        root = r_
            if root is None or (i and abs(root - prev) > 6.0):
                return -1.0, L, None
            e = v_add(rot_x(lever, PED_P, root), (0.0, sl, 0.0))
            sn = sin2(v_sub(e, g), v_sub(e, pv))
            worst = min(worst, sn)
            armmin = min(armmin, sn * math.hypot(e[1] - pv[1], e[2] - pv[2]))
            if min(rot_x(q, PED_P, root)[2] - need for q, need in deck_low) < 0:  # no any() in the sandbox
                return -1.0, L, None
            prev = root
            out.append(root)
        if abs(prev - PED_ALPHA) > 0.3:
            return -1.0, L, None
        return (worst, armmin), L, out

    best_pr = None
    for ll in grid("ped_ll", range(60, 260, 20 * K)):
        for ang in grid("ped_ang", range(0, 360, 10 * K)):
            lever = v_add(PED_P, v_mul(sag(float(ang)), float(ll)))
            if lever[2] < ZP + 40:
                continue
            e2 = node_point("DECK", lever, 1.0)
            for gz in grid("ped_gz", range(int(ZP + 20), int(ZP + 301), 10 * K)):
                g = bisector_point(lever, e2, "z", float(gz))
                if not (600 <= g[1] <= 1000 or 1410 <= g[1] <= 1700) or math.hypot(lever[1] - g[1], lever[2] - g[2]) < 80:
                    continue
                w, L, out = ped_eval(lever, g)
                if out and w[0] >= MIN_SIN and (best_pr is None or w[1] > best_pr[4]):
                    best_pr = (w[0], lever, g, out, w[1])
                    GEO["preset_ped_ll"], GEO["preset_ped_ang"], GEO["preset_ped_gz"] = ll, ang, gz
    PED_Q, lev_p, g_p, PED_PATH, _ = best_pr
    NODES["DECK"]["rpoly"] = fit_frac([(i / NS, PED_PATH[i] / PED_ALPHA) for i in range(NS + 1)])
    tube("DECK", "PE", "Alavanca haste pedaleira", (0.0, PED_P[1], PED_P[2]), lev_p, "20x30", up=(1, 0, 0))
    # lever at x = 0 (30 wide), rod beside it at x = 32, ground post at x = 60
    g_p = (32.0, g_p[1], g_p[2])
    rod("HASTE_PED", "Haste_Pedaleira", "MOV", g_p, "DECK", (32.0, lev_p[1], lev_p[2]), "PE", "Haste comando pedaleira", PED_Q)
    pin("DECK", "PE", "Pino haste/alavanca pedaleira", (25.0, lev_p[1], lev_p[2]), 25, 8)
    tube("MOV", "PE", "Poste suporte haste pedaleira", (60.0, g_p[1], ZP), (60.0, g_p[1], g_p[2] + 20), "20x30", up=(0, 1, 0))
    pin("MOV", "PE", "Pino haste pedaleira no poste", (46.0, g_p[1], g_p[2]), 24, 8)
    # brace: post + brace form a triangle, so the rod force is carried axially
    tube("MOV", "PE", "Mao-francesa poste haste pedaleira", (60.0, g_p[1] - 15, g_p[2] + 5),
         (60.0, g_p[1] - 15 - 0.75 * (g_p[2] - ZP), ZP), "20x30", cut="angulo")
    if min(abs(g_p[1] - y) for y, _ in CROSS) > 25:
        tube(M, "M", "Travessa suporte haste pedaleira", (-255, g_p[1], ZP - 15), (255, g_p[1], ZP - 15), "20x30")
    slots = {}
    GEO["slots"] = slots

    # ======================= STEERING (1 DOF) =======================
    WHEEL_ALPHA = POSE_TRK["wheel_tilt"] - POSE_F1["wheel_tilt"]
    best = None
    for qz in range(int(ZP + 600), int(ZP + 950), 10):
        q, dm = solve_pivot(F1["W"], TRK["W"], WHEEL_ALPHA, float(qz))
        c1 = leg_clearance(F1, q[1], qz, 25)
        c2 = leg_clearance(TRK, q[1] + dm, qz, 25)
        s1 = sight_margin(F1, q[1], qz + 25)
        s2 = sight_margin(TRK, q[1] + dm, qz + 25)
        arm_len = v_len(v_sub(F1["W"], q))
        ok = c1 >= 40 and c2 >= 40 and s1 >= 30 and s2 >= 30 and 300 < q[1] < 1650 and 300 < q[1] + dm < 1650
        score = (1 if ok else 0, -arm_len if ok else min(c1, c2, s1, s2))
        if best is None or score > best[0]:
            best = (score, q, dm, c1, c2, min(s1, s2), arm_len)
    _, Q_PIV, MAST_SLIDE, c1, c2, smin, arm_len = best
    W1, n1, eu1 = F1["W"], F1["n"], F1["eu"]
    qy, qz = Q_PIV[1], Q_PIV[2]
    mast_o = (0.0, qy, ZP)
    node("MASTRO", "Mastro_Volante", "MOV", mast_o, slide=(0.0, MAST_SLIDE, 0.0))
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        abox("MASTRO", "VO", "CH", "Patim trilho mastro " + sd, sx * 255, qy - 40, ZP, sx * 285, qy + 40, ZP + 12, "Chapa")
        tube("MASTRO", "VO", "Coluna volante " + sd, (sx * 270, qy, ZP + 12), (sx * 270, qy, qz + 40), "30x50",
             up=(0, 1, 0))
        # guide plates above and below the ring: the mast top slides along the ring
        for nm, z0 in (("superior", RING_Z + 15), ("inferior", RING_Z - 21)):
            abox("MASTRO", "VO", "CH", "Guia %s coluna/anel %s" % (nm, sd), sx * 255, qy - 40, z0, sx * 305, qy + 40, z0 + 6,
                 "Chapa")
        pin("MASTRO", "VO", "Pino indexador mastro " + sd, (sx * 290, qy + 30, ZP + 6), 12, 6)
        cyl("MASTRO", "VO", "CO", "Mancal tubo torcao " + sd, (sx * 255, qy, qz), (sx * 285, qy, qz), 30, "Aco", mass=0.4,
            spec="Mancal flangeado para eixo Ø25")
    node("BRACO", "Braco_Volante", "MASTRO", Q_PIV, rot=WHEEL_ALPHA)
    tube("BRACO", "VO", "Tubo de torcao volante", (-252, qy, qz), (252, qy, qz), "30x50")
    cyl("BRACO", "VO", "PI", "Eixo pivo volante", (-300, qy, qz), (300, qy, qz), 12.5, "Aco")
    tilt0 = POSE_F1["wheel_tilt"] - G920_WHEEL_PLANE
    tY, tZ = sag(tilt0), sag(tilt0 + 90)
    BASE_DROP = 135.0
    mesa_o = v_sub(W1, v_mul(tZ, BASE_DROP))
    box("BRACO", "VO", "CH", "Mesa volante 300x240x3", mesa_o, X, tY, tZ, (-150, 45, -3, 150, 285, 0), "Chapa")
    g920w = "Logitech G920 - volante (270 x 260 x 278 mm, 2,25 kg)"
    box("BRACO", "VO", "CO", "Volante G920 (base)", mesa_o, X, tY, tZ, (-130, 30, 0, 130, 262, 170), "Motor", mass=1.6,
        spec=g920w, group="G920_Volante")
    box("BRACO", "VO", "CO", "Volante G920 (tampa)", mesa_o, X, tY, tZ, (-95, 60, 170, 95, 240, 190), "Carbono",
        mass=0.05, spec=g920w, group="G920_Volante")
    part("BRACO", "VO", "CO", "Volante G920 (aro)", ("torus", W1, X, eu1, n1, G920_RIM_R, 14.0), "Volante", 0.5, W1,
         spec=g920w, group="G920_Volante")
    cyl("BRACO", "VO", "CO", "Volante G920 (cubo)", v_add(W1, v_mul(n1, 18)), local_to_world(mesa_o, X, tY, tZ, (0, 40, BASE_DROP)),
        42, "Carbono", mass=0.1, spec=g920w, group="G920_Volante")
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube("BRACO", "VO", "Braco volante " + sd, local_to_world(mesa_o, X, tY, tZ, (sx * 40, 165, -8)), (sx * 40, qy, qz),
             "30x50", up=(1, 0, 0), cut="angulo")
    lev_v = (0.0, qy - 140.0, qz)
    vpath = [node_point("BRACO", lev_v, i / 20.0) for i in range(21)]
    slots["VO"] = vpath
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube("BRACO", "VO", "Alavanca came volante " + sd, (sx * 295, qy, qz), (sx * 295, lev_v[1] - 12, lev_v[2]),
             "20x30")
        pin("BRACO", "VO", "Pino seguidor came volante " + sd, (sx * 305, lev_v[1], lev_v[2]), 12, 6)
        ymin, ymax = min(p[1] for p in vpath) - 30, max(p[1] for p in vpath) + 30
        zmin, zmax = min(p[2] for p in vpath) - 30, max(p[2] for p in vpath) + 30
        abox("MOV", "VO", "CH", "Chapa came volante 4mm " + sd, sx * 306, ymin, max(zmin, RING_Z + 15), sx * 310, ymax, zmax, "Chapa")

    # ======================= RING (roll frame) + DOOR BAR =======================
    door_r = 200.0
    door_f = min(qy - 90.0, 1300.0)
    right = [25.0, 400.0, 1000.0, 1765.0]
    left_rear = [25.0, door_r]
    left_front = [door_f] + [y for y in (1450.0, 1765.0) if y > door_f + 150]
    GEO.update(door_r=door_r, door_f=door_f)
    PREV["door_f"] = door_f
    for sx, stations in ((1, right), (-1, left_rear + left_front)):
        sd = "E" if sx < 0 else "D"
        for y in stations:
            tube("MOV", "A", "Coluna anel %s y%d" % (sd, int(y)), (sx * RING_X, y, ZP - 50), (sx * RING_X, y, RING_Z - 15),
                 "20x30", up=(0, 1, 0))
    tube("MOV", "A", "Anel lado D", (RING_X, 15, RING_Z), (RING_X, 1775, RING_Z), "20x30")
    tube("MOV", "A", "Travessa traseira rebaixada", (-RING_X + 10, 25, RING_REAR_Z), (RING_X - 10, 25, RING_REAR_Z), "20x30")
    tube("MOV", "A", "Anel dianteiro", (-RING_X + 10, 1765, RING_Z), (RING_X - 10, 1765, RING_Z), "20x30")
    tube("MOV", "A", "Anel lado E traseiro", (-RING_X, 15, RING_Z), (-RING_X, door_r + 10, RING_Z), "20x30")
    tube("MOV", "A", "Anel lado E dianteiro", (-RING_X, door_f - 10, RING_Z), (-RING_X, 1775, RING_Z), "20x30")
    # diagonals only where the ring carries the rod moment into the pivot (the
    # two right-hand bays around y = 700) and in the end bays that guide the mast;
    # elsewhere the ring is a guide rail, not a truss (v3-leve)
    bays = [(1, right[0], right[1], True), (1, right[1], right[2], False), (-1, left_rear[0], left_rear[1], True)]
    if len(left_front) > 1:
        bays.append((-1, left_front[0], left_front[1], True))
    for sx, ya, yb, up_fwd in bays:
        sd = "E" if sx < 0 else "D"
        za, zb = (ZP - 25, RING_Z - 15) if up_fwd else (RING_Z - 15, ZP - 25)
        tube("MOV", "A", "Diagonal anel %s y%d-%d" % (sd, int(ya), int(yb)), (sx * RING_X, ya + 10, za),
             (sx * RING_X, yb - 10, zb), "20x30", cut="angulo")
    tube("MOV", "A", "Diagonal quadro traseiro", (-RING_X + 10, 25, ZP - 25), (RING_X - 10, 25, RING_REAR_Z - 15), "20x30",
         cut="angulo")
    tube("MOV", "A", "Diagonal quadro dianteiro", (RING_X - 10, 1765, ZP - 25), (-RING_X + 10, 1765, RING_Z - 15), "20x30",
         cut="angulo")
    # shifter fixed on the right ring, truck position (F1 uses the G920 paddles)
    y_sh = TRK["Hc"][1] + 200.0
    abox("MOV", "A", "CH", "Chapa suporte cambio 190x140x6", RING_X - 10, y_sh - 95, RING_Z + 15, 435, y_sh + 95,
         RING_Z + 21, "Chapa")
    tube("MOV", "A", "Mao-francesa suporte cambio", (RING_X + 10, y_sh, RING_Z - 15), (425, y_sh, RING_Z + 12), "20x30",
         up=(0, 1, 0), cut="angulo")
    shs = "Logitech Driving Force Shifter (130 x 165 x 167 mm, estimado)"
    abox("MOV", "A", "CO", "Cambio (carcaca)", 360 - SHIFTER_W / 2, y_sh - SHIFTER_D / 2, RING_Z + 21, 360 + SHIFTER_W / 2,
         y_sh + SHIFTER_D / 2, RING_Z + 91, "Motor", mass=0.7, spec=shs, group="G920_Cambio")
    cyl("MOV", "A", "CO", "Cambio (alavanca)", (360.0, y_sh, RING_Z + 91), (360.0, y_sh, RING_Z + 21 + SHIFTER_H - 25), 7,
        "Aco", mass=0.1, spec=shs, group="G920_Cambio")
    ball("MOV", "A", "CO", "Cambio (manopla)", (360.0, y_sh, RING_Z + 21 + SHIFTER_H - 22), 22, "Carbono", mass=0.1,
         spec=shs, group="G920_Cambio")
    GEO["y_shifter"] = y_sh
    # door bar: hinged on the rear door post, opens upwards (rotation about X)
    hinge = (-RING_X, door_r, RING_Z)
    node("PORTA", "Porta_Barra", "MOV", hinge, rot=85.0, var="aberta")
    tube("PORTA", "PO", "Barra da porta", (-RING_X, door_r + 15, RING_Z), (-RING_X, door_f - 15, RING_Z), "30x50")
    pin("PORTA", "PO", "Eixo dobradica porta", hinge, 25, 8)
    abox("MOV", "PO", "CH", "Orelhas dobradica porta", -RING_X - 22, door_r - 15, RING_Z - 15, -RING_X + 22, door_r + 15,
         RING_Z + 25, "Chapa")
    # latch: open fork (two cheeks + seat) on the front door post; the bar drops in
    # from above and a ball-lock pin crosses cheeks and bar
    for sx, nm in ((-1, "externa"), (1, "interna")):
        abox("MOV", "PO", "CH", "Fecho da porta - orelha %s" % nm, -RING_X + sx * 10, door_f - 40, RING_Z - 15,
             -RING_X + sx * 16, door_f - 5, RING_Z + 25, "Chapa")
    abox("MOV", "PO", "CH", "Fecho da porta - apoio", -RING_X - 16, door_f - 40, RING_Z - 21, -RING_X + 16, door_f - 5,
         RING_Z - 15, "Chapa")
    cyl("PORTA", "PO", "PI", "Trava rapida da porta", (-RING_X - 24, door_f - 22, RING_Z), (-RING_X + 24, door_f - 22, RING_Z),
        5, "Aco", mass=0.05, spec="Pino de trava rapida (ball lock) Ø10 x 50")

    # ======================= CENTRING SPRINGS (power-off safety) =======================
    # Extension springs between the base and the chassis. Each pair is symmetric
    # about one axis: the pitch pair on the centreline (y = 300 and 1100), the
    # roll pair under the main rails on the pivot line (y = 700). The preload
    # only pulls the chassis down (the cardan stays in compression); rotating
    # the platform stretches one spring and relaxes the other, so the pair
    # gives a restoring moment larger than the inverted-pendulum moment m.g.h
    # (sizing in docs/CALCULOS.md). With the motors off, the platform returns
    # to level instead of falling to one side.
    for nm, sp, ztop in (("arfagem tras", (0.0, 300.0), ZP - 30), ("arfagem diant", (0.0, 1100.0), ZP - 30),
                         ("rolagem E", (-200.0, 650.0), ZP - 50), ("rolagem D", (200.0, 650.0), ZP - 50)):
        abox(B, "MO", "CH", "Olhal base mola %s" % nm, sp[0] - 4, sp[1] - 15, 30, sp[0] + 4, sp[1] + 15, 60, "Chapa")
        abox("MOV", "MO", "CH", "Olhal chassi mola %s" % nm, sp[0] - 4, sp[1] - 15, ztop - 30, sp[0] + 4, sp[1] + 15, ztop,
             "Chapa")
        cyl(B, "MO", "CO", "Mola de tracao %s" % nm, (sp[0], sp[1], 50), (sp[0], sp[1], ztop - 20), 20, "Mola", mass=0.6,
            spec="Mola de tracao com olhais (rigidez/pre-carga em CALCULOS.md)", n=12)
        GEO.setdefault("springs", []).append((nm, sp, 50.0, ztop - 20))

    # ======================= MANNEQUINS (mass of the driver) =======================
    for P, hide in ((F1, "F1"), (TRK, "TRK")):
        key = "MQ_" + P["c"]["tag"]
        node(key, "Manequim_" + P["c"]["tag"], "MOV", O, hide=hide)
        mq = DRIVER_KG
        Hc, S, tdir = P["Hc"], P["S"], P["tdir"]
        part(key, "MQ", "MQ", "Tronco", ("bone", "Manequim_Tronco", v_sub(Hc, v_mul(tdir, 75)), v_add(S, v_mul(tdir, 10)), 1.35, 0.75),
             "Osso", mq * SEG_FRAC["trunk"], v_lerp(Hc, S, 0.5), bom=False)
        part(key, "MQ", "MQ", "Pescoco", ("bone", "Manequim_Pescoco", S, v_add(S, v_mul(v_sub(P["head"], S), 0.62)), 1.0, 1.0),
             "Osso", 0.0, S, bom=False)
        part(key, "MQ", "MQ", "Cabeca", ("joint", "Manequim_Cabeca", P["head"], 98, 1.18), "Osso", mq * SEG_FRAC["head"],
             P["head"], bom=False)
        for side in ("E", "D"):
            L = P["legs"][side]
            part(key, "MQ", "MQ", "Quadril " + side, ("joint", "Manequim_Articulacao", L["hip"], 58, 1.0), "Junta", 0.0, L["hip"], bom=False)
            part(key, "MQ", "MQ", "Coxa " + side, ("bone", "Manequim_Coxa", L["hip"], L["K"], 1.0, 1.0), "Osso",
                 mq * SEG_FRAC["thigh"], v_lerp(L["hip"], L["K"], 0.433), bom=False)
            part(key, "MQ", "MQ", "Joelho " + side, ("joint", "Manequim_Articulacao", L["K"], 48, 1.0), "Junta", 0.0, L["K"], bom=False)
            part(key, "MQ", "MQ", "Canela " + side, ("bone", "Manequim_Canela", L["K"], L["A"], 1.0, 1.0), "Osso",
                 mq * SEG_FRAC["shank"], v_lerp(L["K"], L["A"], 0.433), bom=False)
            part(key, "MQ", "MQ", "Tornozelo " + side, ("joint", "Manequim_Articulacao", L["A"], 37, 1.0), "Junta", 0.0, L["A"], bom=False)
            box(key, "MQ", "MQ", "Pe " + side, L["heel"], X, L["f"], v_mul(L["s"], -1), (-45, -10, 0, 45, 255, 80), "Osso",
                mass=mq * SEG_FRAC["foot"], bom=False)
            Psh, E, T = P["elbows"][side]
            part(key, "MQ", "MQ", "Ombro " + side, ("joint", "Manequim_Articulacao", Psh, 46, 1.0), "Junta", 0.0, Psh, bom=False)
            part(key, "MQ", "MQ", "Braco " + side, ("bone", "Manequim_Braco", Psh, E, 1.0, 1.0), "Osso",
                 mq * SEG_FRAC["upper_arm"], v_lerp(Psh, E, 0.436), bom=False)
            part(key, "MQ", "MQ", "Cotovelo " + side, ("joint", "Manequim_Articulacao", E, 37, 1.0), "Junta", 0.0, E, bom=False)
            part(key, "MQ", "MQ", "Antebraco " + side, ("bone", "Manequim_Antebraco", E, T, 1.0, 1.0), "Osso",
                 mq * SEG_FRAC["forearm_hand"], v_lerp(E, T, 0.6), bom=False)
            part(key, "MQ", "MQ", "Mao " + side, ("joint", "Manequim_Articulacao", T, 40, 1.0), "Junta", 0.0, T, bom=False)

    GEO.update(C1=C1, C2=C2, J=J, D_J=D_J, A_R=A_R, A_F=A_F, B_R1=B_R1, B_F1=B_F1, TH1=TH1, TH2=TH2,
               SEAT_SLIDE=SEAT_SLIDE, LINK_ROT=LINK_ROT, BACK_ROT=BACK_ROT, PAN_ROT=PAN_ROT, ys=ys, ye=ye,
               PED_P=PED_P, PED_SLIDE=PED_SLIDE, PED_ALPHA=PED_ALPHA, pys=pys, pye=pye, Ph1=Ph1,
               Q=Q_PIV, MAST_SLIDE=MAST_SLIDE, WHEEL_ALPHA=WHEEL_ALPHA, arm_len=arm_len, mast_clear=(c1, c2),
               sight=smin, P1=P1, N1=N1, mesa_o=mesa_o, tY=tY, tZ=tZ, hr_n=hr_n, back_dir=back_dir)
    return GEO

# ----------------------------------------------------------------------------
# Balancing: translate both postures so the moving CG sits over the pivot
# ----------------------------------------------------------------------------

SHIFT = [-110.0, -250.0]
BAL_LOG = []
COARSE = True
if USE_PRESET:
    SHIFT = list(PRESET["shift"])
    COARSE = False
for it in range(10):
    build(SHIFT[0], SHIFT[1])
    mp0, mp1 = mass_props(0.0), mass_props(1.0)
    e0, e1 = mp0["cg"][1] - PIVOT[1], mp1["cg"][1] - PIVOT[1]
    BAL_LOG.append((round(SHIFT[0], 1), round(SHIFT[1], 1), round(e0, 1), round(e1, 1), "grossa" if COARSE else "fina"))
    if abs(e0) < 1.0 and abs(e1) < 1.0:
        if not COARSE:
            break
        COARSE = False
        continue
    mov0 = sum(p["mass"] for p in PARTS if p["node"] not in ("MOV", "BASE", "PORTA") and in_tree(p["node"], "MOV")
               and not node_hidden(p["node"], 0.0))
    mov1 = sum(p["mass"] for p in PARTS if p["node"] not in ("MOV", "BASE", "PORTA") and in_tree(p["node"], "MOV")
               and not node_hidden(p["node"], 1.0))
    SHIFT[0] -= e0 * mp0["mass"] / mov0
    SHIFT[1] -= e1 * mp1["mass"] / mov1
if COARSE:
    COARSE = False
    build(SHIFT[0], SHIFT[1])

MP = {"F1": mass_props(0.0), "Caminhao": mass_props(1.0), "Base": mass_props(0.0, "BASE")}

# ----------------------------------------------------------------------------
# Checks (clearances that the review asked for)
# ----------------------------------------------------------------------------

def checks():
    F1, TRK = GEO["F1"], GEO["TRK"]
    out = {}
    # F1 elbows vs ring
    out["cotovelo_F1_abaixo_anel_mm"] = round(RING_Z - 15 - max(F1["elbows"][s][1][2] + 37 for s in ("E", "D")))
    out["cotovelo_F1_dentro_anel_mm"] = round(RING_X - 10 - max(abs(F1["elbows"][s][1][0]) + 37 for s in ("E", "D")))
    # F1 headrest vs rear ring (rear face of the upper segment at ring height)
    out["apoio_F1_y_min_mm"] = round(min(local_to_world(GEO["J"], X, sag(POSE_TRK["torso"]), GEO["hr_n"], (0, t, 60))[1]
                                         for t in range(0, 431, 10)))
    out["anel_traseiro_y_face_mm"] = 40
    # seat pan (F1) vs rear link
    C1, N1 = GEO["C1"], GEO["N1"]
    corner = v_sub(C1, v_mul(N1, 60))
    A_R, B_R1 = GEO["A_R"], GEO["B_R1"]
    lz = A_R[2] + (B_R1[2] - A_R[2]) * (corner[1] - A_R[1]) / (B_R1[1] - A_R[1])
    out["folga_assento_biela_F1_mm"] = round(corner[2] - (lz + 15))
    out["folga_cabeca_apoio_F1_mm"] = round(v_dot(v_sub(F1["head"], GEO["J"]), v_mul(GEO["hr_n"], -1)) - 98)
    out["folga_tronco_apoio_F1_mm"] = round(v_dot(v_sub(F1["S"], GEO["J"]), v_mul(GEO["hr_n"], -1)) - 62)
    # carriages inside rails
    out["carro_banco_y"] = [round(min(GEO["ys"], GEO["ys"] + GEO["SEAT_SLIDE"])), round(max(GEO["ye"], GEO["ye"] + GEO["SEAT_SLIDE"]))]
    out["carro_pedal_y"] = [round(min(GEO["pys"], GEO["pys"] + GEO["PED_SLIDE"])), round(max(GEO["pye"], GEO["pye"] + GEO["PED_SLIDE"]))]
    out["mastro_y"] = [round(GEO["Q"][1]), round(GEO["Q"][1] + GEO["MAST_SLIDE"])]
    out["porta_y"] = [GEO["door_r"], GEO["door_f"]]
    out["hastes_erro_comprimento_mm"] = {k: round(v["err"], 2) for k, v in GEO["rods"].items()}
    out["hastes_comprimento_mm"] = {k: round(v["length"]) for k, v in GEO["rods"].items()}
    # synthesis exactness
    out["erro_calcanhar_mm"] = round(v_len(v_sub(node_point("DECK", GEO["Ph1"], 1.0), TRK["Ph"])), 2)
    out["erro_volante_mm"] = round(v_len(v_sub(node_point("BRACO", F1["W"], 1.0), TRK["W"])), 2)
    out["erro_canto_banco_mm"] = round(v_len(v_sub(node_point("BANCO", GEO["C1"], 1.0), TRK["C"])), 2)
    out["topo_cabeca_mm"] = {"F1": round(F1["head"][2] + 116), "Caminhao": round(TRK["head"][2] + 116)}
    out["cames_y_z"] = {k: [[round(p[1]), round(p[2] - ZP)] for p in (v[0], v[-1])] for k, v in GEO["slots"].items()}
    return out

CHECKS = checks()

def bom_rows():
    rows = []
    for p in PARTS:
        if not p["bom"]:
            continue
        rows.append(dict(id=p["id"], asm=p["asm"], kind=p["kind"], desc=p["desc"], prof=p["prof"],
                         length=round(p["length"]) if (p["kind"] == "T" and p["length"]) else None,
                         mass=round(p["mass"], 3), spec=p["spec"], cut=p["cut"]))
    return rows

RESULT = dict(
    balanco=dict(iteracoes=BAL_LOG, deslocamento_F1_mm=round(SHIFT[0], 1), deslocamento_caminhao_mm=round(SHIFT[1], 1)),
    massa={k: dict(massa_kg=round(v["mass"], 1), cg_mm=[round(c) for c in v["cg"]], I_arfagem_kgm2=round(v["I_pitch"], 2),
                   I_rolagem_kgm2=round(v["I_roll"], 2)) for k, v in MP.items()},
    mecanismos=dict(banco=dict(biela_graus=[round(math.degrees(GEO["TH1"]), 1), round(math.degrees(GEO["TH2"]), 1)],
                               curso_carro_mm=round(GEO["SEAT_SLIDE"], 1), giro_assento=GEO["PAN_ROT"], giro_encosto=GEO["BACK_ROT"]),
                    pedaleira=dict(pivo_y_z=[round(GEO["PED_P"][1]), round(GEO["PED_P"][2] - ZP)], giro=round(GEO["PED_ALPHA"], 1),
                                   curso_mm=round(GEO["PED_SLIDE"], 1)),
                    volante=dict(pivo_y_z=[round(GEO["Q"][1]), round(GEO["Q"][2] - ZP)], giro=GEO["WHEEL_ALPHA"],
                                 curso_mm=round(GEO["MAST_SLIDE"], 1), braco_mm=round(GEO["arm_len"]),
                                 folga_pernas_mm=[round(c) for c in GEO["mast_clear"]], margem_visao_mm=round(GEO["sight"]))),
    verificacoes=CHECKS,
)

# =============================================================================
# EMISSION (SketchUp)
# =============================================================================

COLORS = {
    "T_30x50": (36, 70, 110), "T_20x30": (52, 57, 66), "T_10x20": (232, 118, 28), "Biela": (40, 110, 190),
    "Aco": (175, 180, 188), "Motor": (28, 29, 32), "Chapa": (150, 156, 164), "Borracha": (20, 20, 20),
    "Cardan": (196, 160, 40), "Estofado": (26, 26, 30), "Aba": (150, 28, 32), "PedalAl": (190, 194, 200),
    "Volante": (34, 34, 38), "Carbono": (60, 62, 70), "Osso": (208, 211, 218), "Junta": (70, 92, 124),
    "Mola": (200, 170, 40),
}

if EMIT:
    MATS = {}

    def get_mat(name):
        if name in MATS:
            return MATS[name]
        for m in model.get_materials():
            if m.get_name() == name:
                MATS[name] = m
                return m
        r, g, b = COLORS.get(name, (128, 128, 128))
        m = Material()
        m.set_name(name)
        m.set_color(SUColor(r, g, b, 255))
        model.add_materials([m])
        MATS[name] = m
        return m

    def paint(ents, m):
        for f in ents.get_faces():
            f.set_front_material(m)
            f.set_back_material(m)

    def frame_tf(o, xa, ya, za, sx=1.0, sy=1.0, sz=1.0):
        return SUTransformation([xa[0] * sx, xa[1] * sx, xa[2] * sx, 0, ya[0] * sy, ya[1] * sy, ya[2] * sy, 0,
                                 za[0] * sz, za[1] * sz, za[2] * sz, 0, o[0] * MM, o[1] * MM, o[2] * MM, 1])

    def move_tf(o):
        return frame_tf(o, (1, 0, 0), (0, 1, 0), (0, 0, 1))

    def add_group(parent_ents, name):
        g = Group()
        parent_ents.add_group(g)
        g.set_name(name)
        return g

    def box_geom(x0, y0, z0, x1, y1, z1):
        g = GeometryInput()
        g.set_vertices([SUPoint3D(x * MM, y * MM, z * MM) for (x, y, z) in [
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]])
        for fv in [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [4, 5, 6, 7], [0, 3, 2, 1]]:
            lp = LoopInput()
            for i in fv:
                lp.add_vertex_index(i)
            _, g = g.add_face(lp)
        return g

    def _tri(geom, a, b, c):
        lp = LoopInput()
        lp.add_vertex_index(a)
        lp.add_vertex_index(b)
        lp.add_vertex_index(c)
        _, geom = geom.add_face(lp)
        return geom

    def lofted(profile, n=16):
        geom = GeometryInput()
        verts, rings = [], []
        last = len(profile) - 1
        for i, (h, r) in enumerate(profile):
            if r == 0 and i in (0, last):
                verts.append(SUPoint3D(0, 0, h * MM))
                rings.append(("p", len(verts) - 1))
            else:
                s0 = len(verts)
                verts.extend([SUPoint3D(r * MM * math.cos(2 * math.pi * k / n), r * MM * math.sin(2 * math.pi * k / n), h * MM)
                              for k in range(n)])
                rings.append(("r", list(range(s0, s0 + n))))
        geom.set_vertices(verts)
        for i in range(len(rings) - 1):
            (ak, ap), (bk, bp) = rings[i], rings[i + 1]
            for j in range(n):
                j2 = (j + 1) % n
                if ak == "r" and bk == "r":
                    geom = _tri(geom, ap[j], ap[j2], bp[j2])
                    geom = _tri(geom, ap[j], bp[j2], bp[j])
                elif ak == "p" and bk == "r":
                    geom = _tri(geom, bp[j], ap, bp[j2])
                elif ak == "r" and bk == "p":
                    geom = _tri(geom, ap[j], ap[j2], bp)
        for idx, rev in ((0, True), (-1, False)):
            kind, ring = rings[idx]
            if kind == "r":
                lp = LoopInput()
                for k in (list(reversed(ring)) if rev else ring):
                    lp.add_vertex_index(k)
                _, geom = geom.add_face(lp)
        return geom

    def sphere_profile(r, k=8):
        return [(r - r * math.cos(math.pi * i / k), r * math.sin(math.pi * i / k) if 0 < i < k else 0) for i in range(k + 1)]

    def soften(ents, angle=40.0):
        c = math.cos(math.radians(angle))
        for e in ents.get_edges():
            fs = e.get_faces()
            if len(fs) == 2:
                a, b = fs[0].get_normal(), fs[1].get_normal()
                if a.x * b.x + a.y * b.y + a.z * b.z > c:
                    e.set_soft(True)
                    e.set_smooth(True)

    def torus_geom(R, r, nR=36, nr=10):
        geom = GeometryInput()
        verts = []
        for i in range(nR):
            t = 2 * math.pi * i / nR
            for j in range(nr):
                p = 2 * math.pi * j / nr
                rr = R + r * math.cos(p)
                verts.append(SUPoint3D(rr * math.cos(t) * MM, rr * math.sin(t) * MM, r * math.sin(p) * MM))
        geom.set_vertices(verts)
        for i in range(nR):
            i2 = (i + 1) % nR
            for j in range(nr):
                j2 = (j + 1) % nr
                a, b, c, d = i * nr + j, i2 * nr + j, i2 * nr + j2, i * nr + j2
                geom = _tri(geom, a, b, c)
                geom = _tri(geom, a, c, d)
        return geom

    # mannequin segment definitions (nominal length 1000 mm, stretched per instance)
    NOM = 1000.0
    MQ_DEFS = {}

    def mq_def(name):
        if name in MQ_DEFS:
            return MQ_DEFS[name]
        prof = {"Manequim_Coxa": [(0, 60), (150, 66), (700, 52), (1000, 46)],
                "Manequim_Canela": [(0, 45), (250, 48), (1000, 32)],
                "Manequim_Braco": [(0, 40), (1000, 32)],
                "Manequim_Antebraco": [(0, 32), (750, 26), (1000, 22)],
                "Manequim_Tronco": [(0, 110), (120, 126), (400, 104), (650, 116), (880, 124), (1000, 82)],
                "Manequim_Pescoco": [(0, 38), (1000, 36)],
                "Manequim_Cabeca": [(h - 100, r) for (h, r) in sphere_profile(100, 10)],
                "Manequim_Articulacao": [(h - 50, r) for (h, r) in sphere_profile(50)]}[name]
        cd = ComponentDefinition()
        cd.set_name(name)
        model.add_component_definitions([cd])
        cd.get_entities().fill(lofted(prof, 20 if name in ("Manequim_Tronco", "Manequim_Cabeca") else 16), weld_vertices=True)
        soften(cd.get_entities())
        paint(cd.get_entities(), get_mat("Junta" if name == "Manequim_Articulacao" else "Osso"))
        MQ_DEFS[name] = cd
        return cd

    def emit_part(ents, p):
        g = p["geom"]
        nm = (p["id"] + " " if p["id"] else "") + p["desc"]
        mat = get_mat(p["mat"])
        t = g[0]
        if t == "bone":
            _, dn, a, b, sx, sy = g
            d = v_sub(b, a)
            if v_len(d) < 1:
                return
            za = v_norm(d)
            xa, ya = axis_frame(za)
            inst = mq_def(dn).create_instance()
            inst.set_name(nm)
            inst.set_transform(frame_tf(a, xa, ya, za, sx, sy, v_len(d) / NOM))
            ents.add_instance(inst)
            return
        if t == "joint":
            _, dn, c, r, sz = g
            s = r / (50.0 if dn == "Manequim_Articulacao" else 100.0)
            inst = mq_def(dn).create_instance()
            inst.set_name(nm)
            inst.set_transform(frame_tf(c, (1, 0, 0), (0, 1, 0), (0, 0, 1), s, s, s * sz))
            ents.add_instance(inst)
            return
        grp = add_group(ents, nm)
        ge = grp.get_entities()
        if t == "tube":
            _, p0, p1, prof, up = g
            w, h, _t = PROFILES[prof]
            d = v_sub(p1, p0)
            xa, ya, za = axes_from(d, up)
            ge.fill(box_geom(0, -w / 2, -h / 2, v_len(d), w / 2, h / 2), weld_vertices=True)
            grp.set_transform(frame_tf(p0, xa, ya, za))
        elif t == "box":
            _, o, xa, ya, za, ext = g
            ge.fill(box_geom(*ext), weld_vertices=True)
            grp.set_transform(frame_tf(o, xa, ya, za))
        elif t == "cyl":
            _, p0, p1, r, n = g
            d = v_sub(p1, p0)
            za = v_norm(d)
            xa, ya = axis_frame(za)
            ge.fill(lofted([(0, r), (v_len(d), r)], n), weld_vertices=True)
            soften(ge)
            grp.set_transform(frame_tf(p0, xa, ya, za))
        elif t == "ball":
            _, c, r = g
            ge.fill(lofted(sphere_profile(r), 16), weld_vertices=True)
            soften(ge)
            grp.set_transform(move_tf((c[0], c[1], c[2] - r)))
        elif t == "torus":
            _, o, xa, ya, za, R, r = g
            ge.fill(torus_geom(R, r), weld_vertices=True)
            soften(ge, 50)
            grp.set_transform(frame_tf(o, xa, ya, za))
        paint(ge, mat)

    def tv(v):
        if isinstance(v, str):
            return TypedValue(string_value=v)
        return TypedValue(double_value=float(v))

    def dc_write(cd, attrs):
        ad = cd.get_attribute_dictionary("dynamic_attributes")
        for k, v in attrs.items():
            ad[k] = tv(v)

    def node_attrs(key):
        nd = NODES[key]
        rel, slide, var = nd["rel"], nd["slide"], nd["var"]
        a = {"_name": nd["name"], "_lengthunits": "CENTIMETERS", "_formatversion": 1.0,
             "modo": 0.0, "_modo_label": "modo", "_modo_formula": "Parent!modo", "_modo_access": "NONE",
             "sentido": 1.0, "_sentido_label": "sentido", "_sentido_formula": "Parent!sentido", "_sentido_access": "NONE"}
        if var == "aberta":
            a.update({"aberta": 0.0, "_aberta_label": "Aberta", "_aberta_access": "LIST",
                      "_aberta_options": "&Fechada=0&Aberta=1&", "_aberta_formlabel": "Barra da porta",
                      "_aberta_units": "FLOAT", "onclick": 'ANIMATE("aberta",0,1)', "_onclick_label": "onClick",
                      "_onclick_formula": 'ANIMATE("aberta",0,1)'})
        for ax, i in (("x", 0), ("y", 1), ("z", 2)):
            f = "%.2f" % (rel[i] / 10.0)
            if abs(slide[i]) > 1e-6:
                f += "+%s*(%.2f)" % (frac_formula(nd["spoly"], var), slide[i] / 10.0)
            a[ax] = rel[i] * MM
            a["_%s_label" % ax] = ax.upper()
            a["_%s_formula" % ax] = f
        if nd["rot"]:
            a.update({"rotx": 0.0, "_rotx_label": "RotX", "_rotx_formula": "%s*sentido*(%.3f)" % (frac_formula(nd["rpoly"], var), nd["rot"])})
        if nd["hide"]:
            rule = {"F1": "IF(modo>0.001,1,0)", "TRK": "IF(modo<0.999,1,0)"}[nd["hide"]]
            a.update({"hidden": 0.0 if nd["hide"] == "F1" else 1.0, "_hidden_label": "Hidden", "_hidden_formula": rule})
        return a

    # nodes -> components (each with a wrapper group translated by -origin)
    NODE_CD, NODE_WRAP, NODE_INST = {}, {}, {}
    base_grp = add_group(model.get_entities(), "Plataforma_Base")
    NODE_WRAP["BASE"] = base_grp
    for key in NODE_ORDER:
        if key == "BASE":
            continue
        nd = NODES[key]
        cd = ComponentDefinition()
        cd.set_name(nd["name"])
        model.add_component_definitions([cd])
        wrap = add_group(cd.get_entities(), nd["name"] + "_geometria")
        wrap.set_transform(move_tf(v_mul(nd["origin"], -1)))
        inst = cd.create_instance()
        inst.set_name(nd["name"])
        if nd["parent"] is None:
            model.get_entities().add_instance(inst)
        else:
            inst.set_transform(move_tf(nd["rel"]))
            NODE_CD[nd["parent"]].get_entities().add_instance(inst)
        if nd["hide"] == "TRK":
            inst.set_hidden(True)
        NODE_CD[key], NODE_WRAP[key], NODE_INST[key] = cd, wrap, inst
        if key == "MOV":
            dc_write(cd, {
                "_name": "Plataforma_Movel", "_lengthunits": "CENTIMETERS", "_formatversion": 1.0,
                "modo": 0.0, "_modo_label": "Modo", "_modo_access": "LIST", "_modo_options": "&F1=0&Caminhao=1&",
                "_modo_formlabel": "Posicao de pilotagem (0 = F1, 1 = Caminhao)", "_modo_units": "FLOAT",
                "sentido": 1.0, "_sentido_label": "Sentido", "_sentido_access": "LIST",
                "_sentido_options": "&Padrao=1&Invertido=-1&",
                "_sentido_formlabel": "Sentido das rotacoes (troque se a animacao girar ao contrario)", "_sentido_units": "FLOAT",
                "onclick": 'ANIMATE("modo",0,1)', "_onclick_label": "onClick", "_onclick_formula": 'ANIMATE("modo",0,1)',
                "description": "Clique com a ferramenta Interagir para alternar entre F1 e Caminhao. Clique na barra da porta para abri-la.",
                "_description_label": "Description", "_description_access": "VIEW"})
        else:
            dc_write(cd, node_attrs(key))

    # parts -> geometry (optionally inside named sub-components)
    SUBCOMP = {}
    for p in PARTS:
        wrap = NODE_WRAP[p["node"]]
        ents_owner = wrap
        if p["group"]:
            k = (p["node"], p["group"])
            if k not in SUBCOMP:
                cd = ComponentDefinition()
                cd.set_name(p["group"])
                model.add_component_definitions([cd])
                inst = cd.create_instance()
                inst.set_name(p["group"])
                wrap.get_entities().add_instance(inst)
                SUBCOMP[k] = cd
            ents_owner = SUBCOMP[k]
        emit_part(ents_owner.get_entities(), p)

    # slot traces on the cam plates (visual)
    for keyname, path, xs in (("VO", GEO["slots"]["VO"], 309.0),):
        for sx in (-1, 1):
            g = add_group(NODE_WRAP["MOV"].get_entities(), "Rasgo came %s %s" % (keyname, "E" if sx < 0 else "D"))
            for i in range(len(path) - 1):
                a, b = path[i], path[i + 1]
                d = v_sub(b, a)
                if v_len(d) < 0.5:
                    continue
                xa, ya, za = axes_from(d, (1, 0, 0))
                sg = add_group(g.get_entities(), "seg")
                sg.get_entities().fill(box_geom(0, -6, -4, v_len(d), 6, 4), weld_vertices=True)
                sg.set_transform(frame_tf((sx * (xs + 4), a[1], a[2]), xa, ya, za))
                paint(sg.get_entities(), get_mat("Motor"))

    # tags, style, scenes, camera
    def mk_layer(name):
        for l in model.get_layers():
            if l.get_name() == name:
                return l
        l = Layer()
        l.set_name(name)
        model.add_layers([l])
        return l

    L_BASE, L_MOV, L_MAN = mk_layer("Plataforma_Base"), mk_layer("Plataforma_Movel"), mk_layer("Manequim")
    base_grp.set_layer(L_BASE)
    NODE_INST["MOV"].set_layer(L_MOV)
    NODE_INST["MQ_F1"].set_layer(L_MAN)
    NODE_INST["MQ_Caminhao"].set_layer(L_MAN)
    ro, si = model.get_rendering_options(), model.get_shadow_info()
    for k, v in {"EDGE_DISPLAY_MODE": TypedValue(int_value=1), "RENDER_MODE": TypedValue(int_value=2),
                 "MODEL_TRANSPARENCY": TypedValue(bool_value=False), "DRAW_SILHOUETTES": TypedValue(bool_value=True),
                 "SILHOUETTE_WIDTH": TypedValue(int_value=2), "DRAW_DEPTH_QUE": TypedValue(bool_value=False),
                 "DRAW_HORIZON": TypedValue(bool_value=False), "DRAW_GROUND": TypedValue(bool_value=False),
                 "DISPLAY_SKETCH_AXES": TypedValue(bool_value=False),
                 "HIGHLIGHT_COLOR": TypedValue(color_value=SUColor(0, 1, 255, 255)),
                 "LOCKED_COLOR": TypedValue(color_value=SUColor(255, 0, 0, 255)),
                 "BACKGROUND_COLOR": TypedValue(color_value=SUColor(214, 216, 218, 255)),
                 "FOREGROUND_COLOR": TypedValue(color_value=SUColor(50, 48, 45, 255)),
                 "AMBIENT_OCCLUSION": TypedValue(bool_value=True)}.items():
        ro[RenderingOptionKey[k]] = v
    si[ShadowInfoKey["DISPLAY_SHADOWS"]] = TypedValue(bool_value=True)
    model.get_styles().get_active_style().set_name("Estudio - Cockpit")

    def cam(eye, tgt, fov):
        c = Camera()
        c.set_orientation(SUPoint3D(*[v * MM for v in eye]), SUPoint3D(*[v * MM for v in tgt]), SUVector3D(0, 0, 1))
        c.enable_perspective()
        c.set_perspective_frustum_fov(fov)
        return c

    V_SIDE = ((-4400, 820, 900), (0, 820, 800), 32)
    V_34 = ((-2400, 3300, 1850), (0, 780, 650), 35)
    V_MECH = ((2000, -2200, 1150), (0, 450, 250), 35)
    scene_defs = [("Vista 3/4", V_34, []), ("Vista Lateral", V_SIDE, []),
                  ("Mecanismos (sem manequim)", V_SIDE, [L_MAN]), ("Base e Atuadores", V_MECH, [L_MOV, L_MAN])]
    scenes = []
    for name, _, _ in scene_defs:
        sc = Scene()
        sc.set_name(name)
        scenes.append(sc)
    model.add_scenes(scenes)
    for sc, (name, view, hidden) in zip(scenes, scene_defs):
        sc.set_use_camera(True)
        sc.set_camera(cam(*view))
        sc.set_use_hidden_layers(True)
        for l in hidden:
            sc.add_layer(l)
    model.set_active_scene(scenes[0])
    model.set_camera(cam(*V_34))

    def apply_state(m, door=0.0, sentido=1.0):
        """Pose every DC part as its formulas would (preview helper)."""
        for key, inst in NODE_INST.items():
            nd = NODES[key]
            if nd["parent"] is None:
                continue
            s = door if nd["var"] == "aberta" else m
            o = v_add(nd["rel"], v_mul(nd["slide"], frac(nd["spoly"], s)))
            a = math.radians(nd["rot"] * frac(nd["rpoly"], s) * sentido)
            c, sn = math.cos(a), math.sin(a)
            inst.set_transform(frame_tf(o, (1, 0, 0), (0, c, sn), (0, -sn, c)))
            if nd["hide"]:
                inst.set_hidden(node_hidden(key, m))

    session_state["apply_state"] = apply_state
    result = RESULT
