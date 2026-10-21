# =============================================================================
# 2-DOF motion simulator cockpit - parametric SketchUp model generator
#
# Runs inside the SketchUp MCP `build_model` sandbox (Trimble SketchUp
# connector). The sandbox pre-loads `model`, `math`, SUPoint3D, SUVector3D,
# SUTransformation, SUColor, GeometryInput, LoopInput, Group,
# ComponentDefinition, Material, Layer, Scene, Camera, RenderingOptionKey,
# ShadowInfoKey and TypedValue, and forbids `import`. It does NOT run as-is in
# the SketchUp desktop Ruby console.
#
# Usage: send the whole file as the `code` of one `build_model` call with
# `clean: true`, then call `save_model`.
#
# Conventions
#   - All dimensions below are in millimetres; SketchUp's internal unit is the
#     inch, so every coordinate goes through MM (= 1/25.4) at creation time.
#   - Axes: X = lateral (driver's right is +X), Y = longitudinal (driver faces
#     +Y, rear of the platform is Y = 0), Z = up (floor is Z = 0).
#   - "H" tubes are 20x30 mm (high stress / weight), "L" tubes are 10x20 mm
#     (low stress). The larger side always points along the tube's `up` hint.
#
# Model tree
#   Cockpit_Simulador
#     Plataforma_Base   (fixed: frame, pivot tower, cardan joint, actuators)
#     Plataforma_Movel  (moving frame + one group per seating configuration)
#       Chassi_Movel
#       Config_F1        (seat, pedals, wheel, mast, mannequin) - tag Config_F1
#       Config_Caminhao  (same parts, truck posture)            - tag Config_Caminhao
# =============================================================================

MM = 1.0 / 25.4
ZP = 380.0  # top of the moving-platform main rails (mm above floor)
MZ, MY = 160.0, -40.0  # stepper output shaft height / longitudinal position

# ----------------------------------------------------------------------------
# Vector helpers (plain 3-tuples, mm)
# ----------------------------------------------------------------------------

def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def v_dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def v_cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def v_len(a): return math.sqrt(v_dot(a, a))

def v_norm(a):
    l = v_len(a)
    return (a[0] / l, a[1] / l, a[2] / l)

def sag(deg):
    """Unit vector in the sagittal (Y-Z) plane, angle measured from +Y towards +Z."""
    a = math.radians(deg)
    return (0.0, math.cos(a), math.sin(a))

def local_to_world(o, xa, ya, za, p):
    """Map a point given in the frame (o, xa, ya, za) to world coordinates."""
    return v_add(o, v_add(v_mul(xa, p[0]), v_add(v_mul(ya, p[1]), v_mul(za, p[2]))))

# ----------------------------------------------------------------------------
# Materials, groups, transforms
# ----------------------------------------------------------------------------

def get_mat(name, r, g, b, a=255):
    """Return the material called `name`, creating it once (avoids duplicate-name failures)."""
    for m in model.get_materials():
        if m.get_name() == name:
            return m
    m = Material()
    m.set_name(name)
    m.set_color(SUColor(r, g, b, a))
    model.add_materials([m])
    return m

def paint(ents, m):
    """Paint every face of an entities collection on both sides."""
    for f in ents.get_faces():
        f.set_front_material(m)
        f.set_back_material(m)

def new_group(parent, name):
    """Create a named group inside `parent` (or at model root when parent is None)."""
    g = Group()
    if parent is None:
        model.get_entities().add_group(g)
    else:
        parent.get_entities().add_group(g)
    g.set_name(name)
    return g

def frame_tf(o, xa, ya, za, sx=1.0, sy=1.0, sz=1.0):
    """Column-major transform from a local frame (unit axes, optional scale) and an origin in mm."""
    return SUTransformation([xa[0] * sx, xa[1] * sx, xa[2] * sx, 0,
                             ya[0] * sy, ya[1] * sy, ya[2] * sy, 0,
                             za[0] * sz, za[1] * sz, za[2] * sz, 0,
                             o[0] * MM, o[1] * MM, o[2] * MM, 1])

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
    """Two unit axes perpendicular to za (for solids of revolution)."""
    ref = (1, 0, 0) if abs(za[0]) < 0.9 else (0, 1, 0)
    xa = v_norm(v_sub(ref, v_mul(za, v_dot(ref, za))))
    return xa, v_cross(za, xa)

# ----------------------------------------------------------------------------
# Geometry builders
# ----------------------------------------------------------------------------

def box_geom(x0, y0, z0, x1, y1, z1):
    """Six-quad box between two corners given in mm."""
    g = GeometryInput()
    g.set_vertices([SUPoint3D(x * MM, y * MM, z * MM) for (x, y, z) in [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]])
    for fv in [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [4, 5, 6, 7], [0, 3, 2, 1]]:
        lp = LoopInput()
        for i in fv:
            lp.add_vertex_index(i)
        _, g = g.add_face(lp)
    return g

def _circle(cz, r, n):
    return [SUPoint3D(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), cz) for i in range(n)]

def _tri(geom, a, b, c):
    lp = LoopInput()
    lp.add_vertex_index(a)
    lp.add_vertex_index(b)
    lp.add_vertex_index(c)
    _, geom = geom.add_face(lp)
    return geom

def lofted(profile, n=16):
    """Solid of revolution about local Z. profile = [(h_mm, r_mm), ...]; r = 0 at an end makes a pole."""
    geom = GeometryInput()
    verts, rings = [], []
    last = len(profile) - 1
    for i, (h, r) in enumerate(profile):
        if r == 0 and i in (0, last):
            verts.append(SUPoint3D(0, 0, h * MM))
            rings.append(("p", len(verts) - 1))
        else:
            s = len(verts)
            verts.extend(_circle(h * MM, r * MM, n))
            rings.append(("r", list(range(s, s + n))))
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
    """Profile of a sphere of radius r sitting on z = 0."""
    return [(r - r * math.cos(math.pi * i / k), r * math.sin(math.pi * i / k) if 0 < i < k else 0) for i in range(k + 1)]

def soften(ents, angle=40.0):
    """Soften/smooth edges whose adjacent faces meet at less than `angle` degrees."""
    c = math.cos(math.radians(angle))
    for e in ents.get_edges():
        fs = e.get_faces()
        if len(fs) == 2:
            a, b = fs[0].get_normal(), fs[1].get_normal()
            if a.x * b.x + a.y * b.y + a.z * b.z > c:
                e.set_soft(True)
                e.set_smooth(True)

# ----------------------------------------------------------------------------
# Part builders (all coordinates in mm)
# ----------------------------------------------------------------------------

TUBE = {"H": (20.0, 30.0), "L": (10.0, 20.0)}

def tube(parent, name, p0, p1, kind="H", up=(0, 0, 1), size=None, m=None):
    """Rectangular tube along the centreline p0 -> p1. kind 'H' = 20x30, 'L' = 10x20."""
    w, h = size if size else TUBE[kind]
    d = v_sub(p1, p0)
    xa, ya, za = axes_from(d, up)
    g = new_group(parent, name)
    g.get_entities().fill(box_geom(0, -w / 2, -h / 2, v_len(d), w / 2, h / 2), weld_vertices=True)
    g.set_transform(frame_tf(p0, xa, ya, za))
    paint(g.get_entities(), m if m else (M_H if kind == "H" else M_L))
    return g

def block(parent, name, x0, y0, z0, x1, y1, z1, m):
    """Axis-aligned box (plates, brackets, motor bodies)."""
    g = new_group(parent, name)
    g.get_entities().fill(box_geom(x0, y0, z0, x1, y1, z1), weld_vertices=True)
    paint(g.get_entities(), m)
    return g

def oblock(parent, name, o, xa, ya, za, ext, m):
    """Box with local extents ext = (x0, y0, z0, x1, y1, z1) in the frame (o, xa, ya, za)."""
    g = new_group(parent, name)
    g.get_entities().fill(box_geom(*ext), weld_vertices=True)
    g.set_transform(frame_tf(o, xa, ya, za))
    paint(g.get_entities(), m)
    return g

def cyl(parent, name, p0, p1, r, m, n=16):
    """Cylinder whose axis runs p0 -> p1."""
    d = v_sub(p1, p0)
    za = v_norm(d)
    xa, ya = axis_frame(za)
    g = new_group(parent, name)
    g.get_entities().fill(lofted([(0, r), (v_len(d), r)], n), weld_vertices=True)
    soften(g.get_entities())
    g.set_transform(frame_tf(p0, xa, ya, za))
    paint(g.get_entities(), m)
    return g

def ball(parent, name, c, r, m, n=16):
    """Sphere centred on c (rod-end ball joints)."""
    g = new_group(parent, name)
    g.get_entities().fill(lofted(sphere_profile(r), n), weld_vertices=True)
    soften(g.get_entities())
    g.set_transform(frame_tf((c[0], c[1], c[2] - r), (1, 0, 0), (0, 1, 0), (0, 0, 1)))
    paint(g.get_entities(), m)
    return g

def torus(parent, name, o, xa, ya, za, R, r, m, nR=36, nr=10):
    """Torus in the local XY plane (steering-wheel rim)."""
    g = new_group(parent, name)
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
    g.get_entities().fill(geom, weld_vertices=True)
    soften(g.get_entities(), 50)
    g.set_transform(frame_tf(o, xa, ya, za))
    paint(g.get_entities(), m)
    return g

# ----------------------------------------------------------------------------
# Materials
# ----------------------------------------------------------------------------

M_H = get_mat("Tubo_20x30_grafite", 52, 57, 66)
M_L = get_mat("Tubo_10x20_laranja", 232, 118, 28)
M_STEEL = get_mat("Aco_usinado", 175, 180, 188)
M_MOTOR = get_mat("Motor_preto", 28, 29, 32)
M_PLATE = get_mat("Chapa_aluminio", 150, 156, 164)
M_RUBBER = get_mat("Borracha", 20, 20, 20)
M_PIVOT = get_mat("Junta_cardan", 196, 160, 40)
M_SEAT = get_mat("Banco_estofado", 26, 26, 30)
M_BOLST = get_mat("Banco_abas", 150, 28, 32)
M_PED = get_mat("Pedal_aluminio", 190, 194, 200)
M_WHEEL = get_mat("Volante_preto", 34, 34, 38)
M_CARB = get_mat("Volante_carbono", 60, 62, 70)
M_BONE = get_mat("Manequim_segmento", 208, 211, 218)
M_JOINT = get_mat("Manequim_articulacao", 70, 92, 124)

root = new_group(None, "Cockpit_Simulador")

# ============================================================================
# PLATAFORMA-BASE (fixed to the floor)
# ============================================================================

base = new_group(root, "Plataforma_Base")
Y0, Y1 = -200.0, 1850.0
for s in (-1, 1):
    tube(base, "Base_Longarina_" + ("E" if s < 0 else "D"), (s * 330, Y0, 15), (s * 330, Y1, 15), "H")
for y in (-120.0, 40.0, 560.0, 840.0, 1700.0, Y0 + 10, Y1 - 10):
    tube(base, "Base_Travessa_y%d" % int(y), (-320, y, 15), (320, y, 15), "H")
tube(base, "Base_Diagonal", (-320, 850, 10), (320, 1690, 10), "L")
for s in (-1, 1):
    tube(base, "Base_Suporte_Motor_" + ("E" if s < 0 else "D"), (s * 115, -110, 15), (s * 115, 30, 15), "H")

# Pivot tower: carries the whole moving mass, so it is all 20x30.
tw = new_group(base, "Torre_Pivo")
for sx in (-1, 1):
    for (yb, yt) in ((560, 670), (840, 730)):
        tube(tw, "Torre_Perna", (sx * 200, yb, 30), (sx * 45, yt, 215), "H", up=(0, 1 if yb < 700 else -1, 0))
block(tw, "Torre_Chapa_Topo", -80, 620, 215, 80, 780, 228, M_PLATE)

# Cardan (universal) joint: allows roll (sideways) and pitch (fore/aft); yaw is
# blocked by the two rear rods.
cj = new_group(base, "Junta_Universal_Cardan")
block(cj, "Garfo_Inferior_Base", -35, 655, 228, 35, 745, 255, M_PIVOT)
for sy in (-1, 1):
    block(cj, "Garfo_Inferior_Orelha", -16, 700 + sy * 45 - 7, 255, 16, 700 + sy * 45 + 7, 300, M_PIVOT)
cyl(cj, "Cruzeta_Eixo_Y", (0, 640, 285), (0, 760, 285), 13, M_STEEL)
cyl(cj, "Cruzeta_Eixo_X", (-60, 700, 285), (60, 700, 285), 13, M_STEEL)
for sx in (-1, 1):
    block(cj, "Garfo_Superior_Orelha", sx * 48 - 7, 684, 270, sx * 48 + 7, 716, 318, M_PIVOT)
block(cj, "Garfo_Superior_Base", -60, 670, 318, 60, 730, 340, M_PIVOT)

# Actuators: NEMA 34 stepper + planetary gearbox + 100 mm crank, one per side.
act = new_group(base, "Atuadores_Traseiros")
for s in (-1, 1):
    side = "E" if s < 0 else "D"
    a = new_group(act, "Atuador_" + side)
    block(a, "Chapa_Suporte_Motor", min(s * 285, s * 295), -120, 30, max(s * 285, s * 295), 40, 230, M_PLATE)
    cyl(a, "Redutor_Planetario", (s * 285, MY, MZ), (s * 190, MY, MZ), 42, M_STEEL)
    block(a, "Motor_Passo_NEMA34", min(s * 190, s * 40), MY - 43, MZ - 43, max(s * 190, s * 40), MY + 43, MZ + 43, M_MOTOR)
    block(a, "Calco_Motor", min(s * 105, s * 125), MY - 30, 30, max(s * 105, s * 125), MY + 30, MZ - 43, M_PLATE)
    cyl(a, "Eixo_Saida", (s * 295, MY, MZ), (s * 318, MY, MZ), 10, M_STEEL)
    tube(a, "Braco_Manivela", (s * 310, MY - 15, MZ), (s * 310, 60 + 15, MZ), "H", size=(14.0, 34.0), m=M_STEEL)
    ball(a, "Terminal_Rotula_Inf", (s * 310, 60, MZ), 13, M_STEEL)

# Front bump stops: limit how far the platform nose can drop.
for s in (-1, 1):
    tube(base, "Batente_Coluna_" + ("E" if s < 0 else "D"), (s * 270, 1700, 30), (s * 270, 1700, 140), "H", up=(0, 1, 0))
    cyl(base, "Batente_Borracha_" + ("E" if s < 0 else "D"), (s * 270, 1700, 140), (s * 270, 1700, 175), 24, M_RUBBER)

# ============================================================================
# PLATAFORMA-MOVEL (frame shared by both configurations)
# ============================================================================

mov = new_group(root, "Plataforma_Movel")
fr = new_group(mov, "Chassi_Movel")
for s in (-1, 1):
    tube(fr, "Longarina_Principal_" + ("E" if s < 0 else "D"), (s * 270, 0, 365), (s * 270, 1780, 365), "H")
for y in (15.0, 700.0, 1765.0):
    tube(fr, "Travessa_Principal_y%d" % int(y), (-260, y, 365), (260, y, 365), "H")
for y in (350.0, 1100.0, 1450.0):
    tube(fr, "Travessa_Leve_y%d" % int(y), (-260, y, 370), (260, y, 370), "L")
for s in (-1, 1):
    tube(fr, "Trilho_Banco_" + ("E" if s < 0 else "D"), (s * 150, 5, 390), (s * 150, 1110, 390), "L")
    tube(fr, "Trilho_Pedaleira_" + ("E" if s < 0 else "D"), (s * 230, 1095, 390), (s * 230, 1775, 390), "L")
block(fr, "Chapa_Pivo", -80, 620, 340, 80, 780, 350, M_PLATE)
for s in (-1, 1):
    side = "E" if s < 0 else "D"
    block(fr, "Suporte_Haste_" + side, min(s * 280, s * 330), 35, 330, max(s * 280, s * 330), 85, 350, M_PLATE)
    cyl(fr, "Haste_Ligacao_" + side, (s * 310, 60, MZ + 13), (s * 310, 60, 317), 8, M_STEEL)
    ball(fr, "Terminal_Rotula_Sup_" + side, (s * 310, 60, 317), 13, M_STEEL)

# ============================================================================
# Mannequin (articulated "rigged" figure, ~173 cm standing)
# Segments are component definitions with a nominal length of 1000 mm; each
# instance is stretched along its local Z to the real bone length.
# ============================================================================

NOM = 1000.0

def get_def(name):
    for d in model.get_component_definitions():
        if d.get_name() == name:
            return d, False
    cd = ComponentDefinition()
    cd.set_name(name)
    model.add_component_definitions([cd])
    return cd, True

def make_def(name, profile, m, n=16):
    cd, new = get_def(name)
    if new:
        cd.get_entities().fill(lofted(profile, n), weld_vertices=True)
        soften(cd.get_entities())
        paint(cd.get_entities(), m)
    return cd

D_THIGH = make_def("Manequim_Coxa", [(0, 60), (150, 66), (700, 52), (1000, 46)], M_BONE)
D_SHANK = make_def("Manequim_Canela", [(0, 45), (250, 48), (1000, 32)], M_BONE)
D_UARM = make_def("Manequim_Braco", [(0, 40), (1000, 32)], M_BONE)
D_FARM = make_def("Manequim_Antebraco", [(0, 32), (750, 26), (1000, 22)], M_BONE)
D_TORSO = make_def("Manequim_Tronco", [(0, 110), (120, 126), (400, 104), (650, 116), (880, 124), (1000, 82)], M_BONE, 20)
D_NECK = make_def("Manequim_Pescoco", [(0, 38), (1000, 36)], M_BONE)
D_HEAD = make_def("Manequim_Cabeca", [(h - 100, r) for (h, r) in sphere_profile(100, 10)], M_BONE, 20)
D_JOINT = make_def("Manequim_Articulacao", [(h - 50, r) for (h, r) in sphere_profile(50)], M_JOINT)

# Anthropometry (mm) for a 170-175 cm adult.
THIGH, SHANK, TORSO, UPPER_ARM, FOREARM_TO_GRIP = 420.0, 420.0, 520.0, 300.0, 310.0
HIP_HALF_WIDTH, SHOULDER_HALF_WIDTH = 90.0, 195.0

def bone(parent, cd, name, a, b, sx=1.0, sy=1.0):
    """Instance of a segment definition stretched from joint a to joint b."""
    d = v_sub(b, a)
    za = v_norm(d)
    xa, ya = axis_frame(za)
    inst = cd.create_instance()
    inst.set_name(name)
    inst.set_transform(frame_tf(a, xa, ya, za, sx, sy, v_len(d) / NOM))
    parent.get_entities().add_instance(inst)

def joint(parent, name, c, r, cd=None, sz=1.0):
    """Sphere instance (joint or head) centred on c."""
    cd = cd or D_JOINT
    s = r / (50.0 if cd is D_JOINT else 100.0)
    inst = cd.create_instance()
    inst.set_name(name)
    inst.set_transform(frame_tf(c, (1, 0, 0), (0, 1, 0), (0, 0, 1), s, s, s * sz))
    parent.get_entities().add_instance(inst)

# ============================================================================
# Seating configuration builder
# Every adjustable part (seat, pedal box, steering mast) is derived from the
# mannequin's posture, so changing an angle below moves the hardware with it.
# ============================================================================

def build_config(c):
    rep = {}
    X = (1.0, 0.0, 0.0)
    grp = new_group(mov, c["name"])
    Hc = (0.0, c["hip"][0], ZP + c["hip"][1])  # hip-joint centre

    # --- posture (forward kinematics in the sagittal plane) ---
    tdir = sag(c["torso"])
    S = v_add(Hc, v_mul(tdir, TORSO))  # shoulder line centre
    legs = {}
    for side, sx in (("E", -1), ("D", 1)):
        hx, kx, ax = sx * HIP_HALF_WIDTH, sx * c["knee_x"], c["ankle_x"][side]
        th = math.radians(c["thigh"])
        ps = math.radians(c["thigh"] - (180 - c["knee"]))
        lt = math.sqrt(THIGH ** 2 - (kx - hx) ** 2)  # in-plane length keeps the 3D length exact
        ls = math.sqrt(SHANK ** 2 - (ax - kx) ** 2)
        hip = (hx, Hc[1], Hc[2])
        K = (kx, hip[1] + lt * math.cos(th), hip[2] + lt * math.sin(th))
        A = (ax, K[1] + ls * math.cos(ps), K[2] + ls * math.sin(ps))
        f = sag(math.degrees(ps) + 180 - c["ankle"])  # heel -> toe direction
        s = (0.0, f[2], -f[1])                          # sole normal (towards the pedal)
        heel = v_add(A, v_add(v_mul(s, 70), v_mul(f, -50)))
        legs[side] = dict(hip=hip, K=K, A=A, f=f, s=s, heel=heel)

    tw_ = math.radians(c["wheel_tilt"])  # wheel plane angle from horizontal
    n = (0.0, -math.sin(tw_), math.cos(tw_))  # wheel axis, pointing at the driver
    eu = (0.0, math.cos(tw_), math.sin(tw_))  # "12 o'clock" direction in the wheel plane
    W = v_add(S, (0.0, c["wheel_off"][0], c["wheel_off"][1]))
    R, g = c["wheel_r"], math.radians(c["grip"])
    grips = {"E": v_add(W, v_add(v_mul(X, -R * math.sin(g)), v_mul(eu, R * math.cos(g)))),
             "D": v_add(W, v_add(v_mul(X, R * math.sin(g)), v_mul(eu, R * math.cos(g))))}

    # --- mannequin ---
    man = new_group(grp, "Manequim_" + c["tag"] + "_172cm")
    bone(man, D_TORSO, "Tronco", v_sub(Hc, v_mul(tdir, 75)), v_add(S, v_mul(tdir, 10)), 1.35, 0.75)
    hc = v_add(S, (0.0, c["head"][0], c["head"][1]))
    bone(man, D_NECK, "Pescoco", S, v_add(S, v_mul(v_sub(hc, S), 0.62)))
    joint(man, "Cabeca", hc, 98, D_HEAD, 1.18)
    for side, sx in (("E", -1), ("D", 1)):
        L = legs[side]
        joint(man, "Quadril_" + side, L["hip"], 58)
        bone(man, D_THIGH, "Coxa_" + side, L["hip"], L["K"])
        joint(man, "Joelho_" + side, L["K"], 48)
        bone(man, D_SHANK, "Canela_" + side, L["K"], L["A"])
        joint(man, "Tornozelo_" + side, L["A"], 37)
        oblock(man, "Pe_" + side, L["heel"], X, L["f"], v_mul(L["s"], -1), (-45, -10, 0, 45, 255, 80), M_BONE)
        # Two-bone arm IK: shoulder -> hand on the wheel rim, elbow down and out.
        P, T = (sx * SHOULDER_HALF_WIDTH, S[1], S[2]), grips[side]
        dvec = v_sub(T, P)
        d = min(v_len(dvec), UPPER_ARM + FOREARM_TO_GRIP - 2)
        u = v_norm(dvec)
        pole = v_norm((sx * 0.6, -0.15, -1.0))
        pole = v_norm(v_sub(pole, v_mul(u, v_dot(pole, u))))
        a = (UPPER_ARM ** 2 - FOREARM_TO_GRIP ** 2 + d ** 2) / (2 * d)
        E = v_add(P, v_add(v_mul(u, a), v_mul(pole, math.sqrt(max(UPPER_ARM ** 2 - a ** 2, 0.0)))))
        joint(man, "Ombro_" + side, P, 46)
        bone(man, D_UARM, "Braco_" + side, P, E)
        joint(man, "Cotovelo_" + side, E, 37)
        bone(man, D_FARM, "Antebraco_" + side, E, T)
        joint(man, "Mao_" + side, T, 40)
        rep["alcance_" + side] = round(v_len(dvec), 1)

    # --- seat: pan + reclining back, on a carriage that slides on the seat rails ---
    al, be = math.radians(c["pan"]), math.radians(c["torso"])
    p, npn = (0.0, math.cos(al), math.sin(al)), (0.0, -math.sin(al), math.cos(al))
    b, nb = (0.0, math.cos(be), math.sin(be)), (0.0, math.sin(be), -math.cos(be))
    # Corner C: hip joint sits 95 mm above the pan and 115 mm in front of the back.
    r1, r2 = v_dot(npn, Hc) - 95, v_dot(nb, Hc) - 115
    det = npn[1] * nb[2] - npn[2] * nb[1]
    C = (0.0, (r1 * nb[2] - npn[2] * r2) / det, (npn[1] * r2 - r1 * nb[1]) / det)
    seat = new_group(grp, "Banco_" + c["tag"])
    bz = v_cross(X, b)
    oblock(seat, "Assento", C, X, p, npn, (-240, 0, -60, 240, 440, 0), M_SEAT)
    oblock(seat, "Encosto", C, X, b, bz, (-240, 0, 0, 240, 590, 60), M_SEAT)
    for sx in (-1, 1):
        oblock(seat, "Aba_Assento", C, X, p, npn, (min(sx * 200, sx * 250), 30, 0, max(sx * 200, sx * 250), 380, 55), M_BOLST)
        oblock(seat, "Aba_Encosto", C, X, b, bz, (min(sx * 200, sx * 250), 70, -65, max(sx * 200, sx * 250), 470, 0), M_BOLST)
    PF = v_sub(v_add(C, v_mul(p, 400)), v_mul(npn, 60))
    PR = v_sub(v_add(C, v_mul(p, 40)), v_mul(npn, 60))
    BM = v_add(v_add(C, v_mul(b, 320)), v_mul(bz, 60))
    ys, ye = min(PR[1], BM[1]) - 40, PF[1] + 40
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube(seat, "Carro_Banco_" + sd, (sx * 150, ys, 415), (sx * 150, ye, 415), "H")
        for nm, pt in (("Coluna_Frontal", PF), ("Coluna_Traseira", PR), ("Coluna_Encosto", BM)):
            if pt[2] - 430 > 12:
                tube(seat, nm + "_" + sd, (sx * 150, pt[1], 430), (sx * 150, pt[1], pt[2]), "H", up=(0, 1, 0))
        tube(seat, "Travamento_Diagonal_" + sd, (sx * 150, ye - 20, 430),
             (sx * 150, BM[1] + 0.5 * (PR[1] - BM[1]), max(PR[2], BM[2]) - 15), "L", up=(1, 0, 0))
    rep["folga_assento_carro_mm"] = round(PR[2] - 430, 1)

    # --- pedal box: tilting base plate on height posts and a sliding carriage ---
    ped = new_group(grp, "Pedaleira_" + c["tag"])
    f, s = legs["D"]["f"], legs["D"]["s"]
    ms = v_mul(s, -1)
    Ph = (0.0, 0.5 * (legs["E"]["heel"][1] + legs["D"]["heel"][1]), 0.5 * (legs["E"]["heel"][2] + legs["D"]["heel"][2]))
    off = c["pedal_off"]
    oblock(ped, "Base_Pedais_Inclinavel", Ph, X, f, ms, (-235, -45, -(off + 10), 235, 285, -off), M_PLATE)
    lx, rx = legs["E"]["heel"][0], legs["D"]["heel"][0]
    for nm, px in (("Embreagem", lx), ("Freio", rx - 115), ("Acelerador", rx)):
        oblock(ped, "Pedal_" + nm, Ph, X, f, ms, (px - 38, 70, -24, px + 38, 235, -8), M_PED)
        tube(ped, "Braco_Pedal_" + nm, local_to_world(Ph, X, f, ms, (px, 140, -24)),
             local_to_world(Ph, X, f, ms, (px, 15, -off)), "L", up=f, m=M_STEEL)
        if nm == "Freio":
            cyl(ped, "Celula_Carga_Freio", local_to_world(Ph, X, f, ms, (px, 200, -24)),
                local_to_world(Ph, X, f, ms, (px, 250, -off)), 14, M_STEEL)
    B0 = local_to_world(Ph, X, f, ms, (0, -40, -(off + 5)))
    B1 = local_to_world(Ph, X, f, ms, (0, 280, -(off + 5)))
    ys, ye = min(B0[1], B1[1]) - 60, min(max(B0[1], B1[1]) + 40, 1775)
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        tube(ped, "Carro_Pedaleira_" + sd, (sx * 230, ys, 415), (sx * 230, ye, 415), "H")
        for nm, pt in (("Coluna_Inferior", B0), ("Coluna_Superior_Regulavel", B1)):
            if pt[2] - 430 > 12:
                tube(ped, nm + "_" + sd, (sx * 245, pt[1], 430), (sx * 245, pt[1], pt[2]), "H", up=(0, 1, 0))

    # --- steering: wheel + direct-drive motor on a tilting plate, sliding mast ---
    st = new_group(grp, "Volante_" + c["tag"])
    vol = new_group(st, "Volante")
    torus(vol, "Aro", W, X, eu, n, R, c["rim_r"], M_WHEEL)
    cyl(vol, "Cubo", v_sub(W, v_mul(n, 10)), v_add(W, v_mul(n, 22)), 45, M_CARB)
    for ang in c["spokes"]:
        a = math.radians(ang)
        d2 = v_add(v_mul(X, math.cos(a)), v_mul(eu, math.sin(a)))
        tube(vol, "Raio", v_add(v_mul(d2, 35), W), v_add(v_mul(d2, R - 5), W), "H", up=n, size=(26.0, 14.0), m=M_CARB)
    if c["tag"] == "F1":
        oblock(vol, "Painel_Volante_F1", W, X, eu, n, (-115, -70, 5, 115, 45, 30), M_CARB)
    M0, M1 = v_sub(W, v_mul(n, 22)), v_sub(W, v_mul(n, 225))
    cyl(st, "Motor_Direct_Drive", M0, M1, 55, M_MOTOR)
    Mm, mdir = v_mul(v_add(M0, M1), 0.5), v_mul(n, -1)
    oblock(st, "Chapa_Fixacao_Motor_Inclinavel", Mm, X, mdir, eu, (-65, -95, -68, 65, 95, -56), M_PLATE)
    if c["tag"] == "F1":
        yc, zc = M1[1] + 40, M1[2] - 85
    else:  # truck: crossbar ahead of the knees, motor passes between them
        yc, zc = max(legs["E"]["K"][1], legs["D"]["K"][1]) + 190, M1[2] - 70
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        block(st, "Abracadeira_Trilho_" + sd, min(sx * 255, sx * 285), yc - 35, 380, max(sx * 255, sx * 285), yc + 35, 396, M_PLATE)
        tube(st, "Coluna_Volante_" + sd, (sx * 270, yc, 396), (sx * 270, yc, zc + 15), "H", up=(0, 1, 0))
        tube(st, "Suporte_Motor_" + sd, local_to_world(Mm, X, mdir, eu, (sx * 40, 0, -68)), (sx * 40, yc, zc + 15), "H", up=(1, 0, 0))
    tube(st, "Travessa_Volante", (-260, yc, zc), (260, yc, zc), "H")

    rep["quadril_y_z"] = [round(Hc[1]), round(Hc[2] - ZP)]
    rep["ombro_y_z"] = [round(S[1]), round(S[2] - ZP)]
    rep["volante_y_z"] = [round(W[1]), round(W[2] - ZP)]
    rep["calcanhar_y_z"] = [round(Ph[1]), round(Ph[2] - ZP)]
    rep["topo_cabeca_mm"] = round(hc[2] + 98 * 1.18)
    return grp, rep

# Angles in degrees (sagittal plane, from +Y towards +Z); positions in mm,
# `hip` is (Y, height above ZP).
CONFIG_F1 = dict(name="Config_F1", tag="F1", hip=(650.0, 235.0), torso=145.0, thigh=25.0, knee=145.0, ankle=100.0,
                 knee_x=100.0, ankle_x={"E": -115.0, "D": 125.0}, pan=12.0, head=(-25.0, 220.0),
                 wheel_off=(420.0, 60.0), wheel_tilt=80.0, wheel_r=140.0, rim_r=17.0, grip=90.0,
                 spokes=(0, 180, 270), pedal_off=115.0)
CONFIG_CAMINHAO = dict(name="Config_Caminhao", tag="Caminhao", hip=(700.0, 537.0), torso=105.0, thigh=5.0, knee=120.0,
                       ankle=90.0, knee_x=130.0, ankle_x={"E": -130.0, "D": 135.0}, pan=6.0, head=(-10.0, 230.0),
                       wheel_off=(415.0, -232.0), wheel_tilt=35.0, wheel_r=200.0, rim_r=15.0, grip=60.0,
                       spokes=(0, 180, 270), pedal_off=65.0)

g_f1, rep_f1 = build_config(CONFIG_F1)
g_trk, rep_trk = build_config(CONFIG_CAMINHAO)

# ============================================================================
# Tags (layers), style, scenes and opening camera
# ============================================================================

def mk_layer(name):
    for l in model.get_layers():
        if l.get_name() == name:
            return l
    l = Layer()
    l.set_name(name)
    model.add_layers([l])
    return l

L_EST, L_F1, L_TRK = mk_layer("Estrutura"), mk_layer("Config_F1"), mk_layer("Config_Caminhao")
base.set_layer(L_EST)
fr.set_layer(L_EST)
g_f1.set_layer(L_F1)
g_trk.set_layer(L_TRK)

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

V_SIDE = ((-4200, 820, 850), (0, 820, 760), 32)
V_34 = ((-2300, 3200, 1750), (0, 780, 600), 35)
V_MECH = ((2000, -2200, 1150), (0, 450, 250), 35)
scene_defs = [("F1 - 3/4", V_34, [L_TRK]), ("F1 - Lateral", V_SIDE, [L_TRK]),
              ("Caminhao - 3/4", V_34, [L_F1]), ("Caminhao - Lateral", V_SIDE, [L_F1]),
              ("Base e Atuadores", V_MECH, [L_F1, L_TRK])]
scenes = []
for name, _, _ in scene_defs:
    sc = Scene()
    sc.set_name(name)
    scenes.append(sc)
model.add_scenes(scenes)  # scenes must be in the model before they are configured
for sc, (name, view, hidden) in zip(scenes, scene_defs):
    sc.set_use_camera(True)
    sc.set_camera(cam(*view))
    sc.set_use_hidden_layers(True)
    for l in hidden:
        sc.add_layer(l)  # add_layer HIDES the tag in that scene
model.set_active_scene(scenes[0])
L_TRK.set_visibility(False)  # opening view shows only the F1 configuration
model.set_camera(cam(*V_34))

result = {"F1": rep_f1, "Caminhao": rep_trk}
