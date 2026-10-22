# =============================================================================
# 2-DOF motion simulator cockpit - parametric SketchUp model generator (v2)
#
# Runs inside the SketchUp MCP `build_model` sandbox (Trimble SketchUp
# connector). The sandbox pre-loads `model`, `math`, SUPoint3D, SUVector3D,
# SUTransformation, SUColor, GeometryInput, LoopInput, Group,
# ComponentDefinition, Material, Layer, Scene, Camera, RenderingOptionKey,
# ShadowInfoKey and TypedValue, and forbids `import`. It does NOT run as-is in
# the SketchUp desktop Ruby console.
#
# Usage: send the whole file as the `code` of one `build_model` call with
# `clean: true`, then call `save_model`. Optionally call `apply_state(m)`
# (stored in `session_state`) in a later call to preview an intermediate
# mechanism position (0 = F1, 1 = truck) before saving.
#
# v2: ONE adjustable cockpit instead of two overlaid configurations. The seat
# (four-bar linkage + reclining back), the pedal box (tilting deck) and the
# steering (pivoting arm) move between the F1 and truck postures only through
# hinges, rails and a wheel quick-release. Each moving part is a SketchUp
# Dynamic Component driven by one attribute, `modo` (0 = F1, 1 = truck), on
# the `Plataforma_Movel` component; its OnClick runs ANIMATE("modo",0,1), so
# clicking the platform with the Interact tool animates the change-over.
#
# Conventions
#   - All dimensions below are in millimetres; SketchUp's internal unit is the
#     inch, so every coordinate goes through MM (= 1/25.4) at creation time.
#     Dynamic Component formulas use centimetres (`_lengthunits`).
#   - Axes: X = lateral (driver's right is +X), Y = longitudinal (driver faces
#     +Y, rear of the platform is Y = 0), Z = up (floor is Z = 0).
#   - Angles in the sagittal (Y-Z) plane are measured from +Y towards +Z, i.e.
#     a positive rotation about +X (right-hand rule).
#   - "H" tubes are 20x30 mm (high stress / weight), "L" tubes are 10x20 mm
#     (low stress). The larger side always points along the tube's `up` hint.
#
# Model tree (model root)
#   Plataforma_Base (group)       fixed frame, pivot tower, cardan, actuators
#   Plataforma_Movel (DC)         chassis + adjustable parts; holds `modo`
#     Chassi_Movel (group)
#     Carro_Banco (DC, slides)
#       Biela_Banco_* (DC, rotate)      four-bar links
#         Banco (DC, counter-rotates)   level sub-frame, only under Biela_Banco_E_Tras
#           Assento (DC, tilts about C)        pan: follows the thigh in F1
#           Encosto (DC, reclines about C)
#             Encosto_Superior (DC)            shoulder/head shell, constant angle
#     Carro_Pedaleira (DC, slides)
#       Pedaleira_Basculante (DC, tilts)
#     Mastro_Volante (DC, slides)
#       Braco_Volante (DC, tilts)      carries the G920 wheel base
#     Manequim_F1 / Manequim_Caminhao (DC, shown only at modo = 0 / 1)
#
# Peripherals: Logitech G920 wheel + pedals and the Driving Force Shifter,
# modelled as named components (G920_Volante, G920_Pedais, G920_Cambio) from
# the manufacturer's published overall dimensions, so they can be swapped for
# a detailed model in SketchUp desktop (right click > Reload).
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

def rot_x(p, centre, deg):
    """Rotate point p about the X-parallel axis through `centre` (right-hand rule)."""
    a = math.radians(deg)
    dy, dz = p[1] - centre[1], p[2] - centre[2]
    return (p[0], centre[1] + dy * math.cos(a) - dz * math.sin(a), centre[2] + dy * math.sin(a) + dz * math.cos(a))

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
    """Create a named group inside `parent` (group or definition; model root when None)."""
    g = Group()
    if parent is None:
        model.get_entities().add_group(g)
    else:
        parent.get_entities().add_group(g)
    g.set_name(name)
    return g

def named_component(parent, name):
    """Component (not group) placed at identity inside `parent`, so it can be
    swapped in SketchUp desktop via right click > Reload."""
    cd = ComponentDefinition()
    cd.set_name(name)
    model.add_component_definitions([cd])
    inst = cd.create_instance()
    inst.set_name(name)
    parent.get_entities().add_instance(inst)
    return cd

def frame_tf(o, xa, ya, za, sx=1.0, sy=1.0, sz=1.0):
    """Column-major transform from a local frame (unit axes, optional scale) and an origin in mm."""
    return SUTransformation([xa[0] * sx, xa[1] * sx, xa[2] * sx, 0,
                             ya[0] * sy, ya[1] * sy, ya[2] * sy, 0,
                             za[0] * sz, za[1] * sz, za[2] * sz, 0,
                             o[0] * MM, o[1] * MM, o[2] * MM, 1])

def move_tf(o):
    """Pure translation (mm)."""
    return frame_tf(o, (1, 0, 0), (0, 1, 0), (0, 0, 1))

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
    g.get_entities().fill(box_geom(min(x0, x1), min(y0, y1), min(z0, z1), max(x0, x1), max(y0, y1), max(z0, z1)),
                          weld_vertices=True)
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

def pin(parent, name, c, half, r=8):
    """Locking/hinge pin along X centred on c."""
    return cyl(parent, name, (c[0] - half, c[1], c[2]), (c[0] + half, c[1], c[2]), r, M_STEEL, 12)

# ----------------------------------------------------------------------------
# Dynamic Components
# Every moving part is a component whose geometry sits in a wrapper group
# translated by -origin, so the component's own origin is its hinge axis (or
# any reference point for sliders) and everything can be modelled in world
# coordinates of the F1 pose. In the F1 pose (modo = 0) every DC frame is an
# un-rotated translation of the world frame.
# ----------------------------------------------------------------------------

DC_STATE = []  # (instance, rel_origin_mm, slide_mm_per_unit, rot_deg_per_unit, hide_rule)

def _tv(v):
    if isinstance(v, str):
        return TypedValue(string_value=v)
    return TypedValue(double_value=float(v))

def dc_write(cd, attrs):
    """Write Dynamic Component attributes on a definition (read by SketchUp's DC extension)."""
    ad = cd.get_attribute_dictionary("dynamic_attributes")
    for k, v in attrs.items():
        ad[k] = _tv(v)

def dc_child(parent_cd, name, origin, parent_origin, slide=(0.0, 0.0, 0.0), rot=0.0, hide=None):
    """Create a dynamic child component.

    origin/parent_origin: world points (F1 pose) of this component's and its
    parent's origins. slide: displacement (mm) at modo = 1, in the parent's
    frame. rot: rotation about local X (deg) at modo = 1, multiplied by
    `sentido`. hide: None, "F1" (visible only at modo = 0) or "TRK" (visible
    only at modo = 1).
    Returns (definition, instance, wrapper group for geometry).
    """
    cd = ComponentDefinition()
    cd.set_name(name)
    model.add_component_definitions([cd])
    wrap = new_group(cd, name + "_geometria")
    wrap.set_transform(move_tf(v_mul(origin, -1)))
    inst = cd.create_instance()
    inst.set_name(name)
    rel = v_sub(origin, parent_origin)
    inst.set_transform(move_tf(rel))
    parent_cd.get_entities().add_instance(inst)
    a = {"_name": name, "_lengthunits": "CENTIMETERS", "_formatversion": 1.0,
         "modo": 0.0, "_modo_label": "modo", "_modo_formula": "Parent!modo", "_modo_access": "NONE",
         "sentido": 1.0, "_sentido_label": "sentido", "_sentido_formula": "Parent!sentido", "_sentido_access": "NONE"}
    for ax, i in (("x", 0), ("y", 1), ("z", 2)):
        f = "%.2f" % (rel[i] / 10.0)
        if abs(slide[i]) > 1e-6:
            f += "+modo*(%.2f)" % (slide[i] / 10.0)
        a[ax] = rel[i] * MM
        a["_%s_label" % ax] = ax.upper()
        a["_%s_formula" % ax] = f
    if rot:
        a.update({"rotx": 0.0, "_rotx_label": "RotX", "_rotx_formula": "modo*sentido*(%.3f)" % rot})
    if hide:
        rule = {"F1": "IF(modo>0.001,1,0)", "TRK": "IF(modo<0.999,1,0)"}[hide]
        a.update({"hidden": 0.0 if hide == "F1" else 1.0, "_hidden_label": "Hidden", "_hidden_formula": rule})
        if hide == "TRK":
            inst.set_hidden(True)
    dc_write(cd, a)
    DC_STATE.append((inst, rel, slide, rot, hide))
    return cd, inst, wrap

def apply_state(m, sentido=1.0):
    """Pose every dynamic part exactly as its DC formulas would at modo = m (preview helper)."""
    for inst, rel, slide, rot, hide in DC_STATE:
        o = v_add(rel, v_mul(slide, m))
        a = math.radians(rot * m * sentido)
        c, s = math.cos(a), math.sin(a)
        inst.set_transform(frame_tf(o, (1, 0, 0), (0, c, s), (0, -s, c)))
        if hide:
            hidden = m > 0.001 if hide == "F1" else m < 0.999
            inst.set_hidden(hidden)

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
M_LINK = get_mat("Articulacao_azul", 40, 110, 190)

# ============================================================================
# PLATAFORMA-BASE (fixed to the floor) - unchanged from v1
# ============================================================================

base = new_group(None, "Plataforma_Base")
Y0, Y1 = -200.0, 1850.0
for s in (-1, 1):
    tube(base, "Base_Longarina_" + ("E" if s < 0 else "D"), (s * 330, Y0, 15), (s * 330, Y1, 15), "H")
for y in (-120.0, 40.0, 560.0, 840.0, 1700.0, Y0 + 10, Y1 - 10):
    tube(base, "Base_Travessa_y%d" % int(y), (-320, y, 15), (320, y, 15), "H")
tube(base, "Base_Diagonal", (-320, 850, 10), (320, 1690, 10), "L")
for s in (-1, 1):
    tube(base, "Base_Suporte_Motor_" + ("E" if s < 0 else "D"), (s * 115, -110, 15), (s * 115, 30, 15), "H")

tw = new_group(base, "Torre_Pivo")
for sx in (-1, 1):
    for (yb, yt) in ((560, 670), (840, 730)):
        tube(tw, "Torre_Perna", (sx * 200, yb, 30), (sx * 45, yt, 215), "H", up=(0, 1 if yb < 700 else -1, 0))
block(tw, "Torre_Chapa_Topo", -80, 620, 215, 80, 780, 228, M_PLATE)

cj = new_group(base, "Junta_Universal_Cardan")
block(cj, "Garfo_Inferior_Base", -35, 655, 228, 35, 745, 255, M_PIVOT)
for sy in (-1, 1):
    block(cj, "Garfo_Inferior_Orelha", -16, 700 + sy * 45 - 7, 255, 16, 700 + sy * 45 + 7, 300, M_PIVOT)
cyl(cj, "Cruzeta_Eixo_Y", (0, 640, 285), (0, 760, 285), 13, M_STEEL)
cyl(cj, "Cruzeta_Eixo_X", (-60, 700, 285), (60, 700, 285), 13, M_STEEL)
for sx in (-1, 1):
    block(cj, "Garfo_Superior_Orelha", sx * 48 - 7, 684, 270, sx * 48 + 7, 716, 318, M_PIVOT)
block(cj, "Garfo_Superior_Base", -60, 670, 318, 60, 730, 340, M_PIVOT)

act = new_group(base, "Atuadores_Traseiros")
for s in (-1, 1):
    side = "E" if s < 0 else "D"
    a = new_group(act, "Atuador_" + side)
    block(a, "Chapa_Suporte_Motor", s * 285, -120, 30, s * 295, 40, 230, M_PLATE)
    cyl(a, "Redutor_Planetario", (s * 285, MY, MZ), (s * 190, MY, MZ), 42, M_STEEL)
    block(a, "Motor_Passo_NEMA34", s * 190, MY - 43, MZ - 43, s * 40, MY + 43, MZ + 43, M_MOTOR)
    block(a, "Calco_Motor", s * 105, MY - 30, 30, s * 125, MY + 30, MZ - 43, M_PLATE)
    cyl(a, "Eixo_Saida", (s * 295, MY, MZ), (s * 318, MY, MZ), 10, M_STEEL)
    tube(a, "Braco_Manivela", (s * 310, MY - 15, MZ), (s * 310, 60 + 15, MZ), "H", size=(14.0, 34.0), m=M_STEEL)
    ball(a, "Terminal_Rotula_Inf", (s * 310, 60, MZ), 13, M_STEEL)

for s in (-1, 1):
    tube(base, "Batente_Coluna_" + ("E" if s < 0 else "D"), (s * 270, 1700, 30), (s * 270, 1700, 140), "H", up=(0, 1, 0))
    cyl(base, "Batente_Borracha_" + ("E" if s < 0 else "D"), (s * 270, 1700, 140), (s * 270, 1700, 175), 24, M_RUBBER)

# ============================================================================
# Postures (mannequin kinematics). Everything adjustable is derived from these.
# ============================================================================

THIGH, SHANK, TORSO, UPPER_ARM, FOREARM_TO_GRIP = 420.0, 420.0, 520.0, 300.0, 310.0
HIP_HALF_WIDTH, SHOULDER_HALF_WIDTH = 90.0, 195.0
SUB_DEG = 9.0          # seat sub-frame angle (the parallelogram keeps it constant)
X = (1.0, 0.0, 0.0)

# Logitech G920 / Driving Force Shifter (published overall sizes, mm). Internal
# proportions (rim section, pedal pad layout, mounting points) are estimates.
G920_RIM_R = 135.0                  # wheel: 270 H x 260 W x 278 D
G920_WHEEL_PLANE = 80.0             # rim plane vs. the base's mounting face (estimate)
G920_PED_W, G920_PED_D, G920_PED_H = 428.5, 311.0, 167.0
G920_PAD_PITCH = 125.0              # clutch / brake / throttle spacing (estimate)
PEDAL_DEPTH = 130.0                 # sole (pad face) -> pedal-unit mounting face
SHIFTER_W, SHIFTER_D, SHIFTER_H = 130.0, 165.0, 167.0

# Angles in degrees (sagittal plane); `hip` is (Y, height above ZP).
# `pan` is the seat-pan angle; in F1 it matches the thigh so the pan carries it.
# The F1 head rests on the upper back segment (see Encosto_Superior).
POSE_F1 = dict(tag="F1", hip=(650.0, 290.0), torso=145.0, pan=25.0, thigh=25.0, knee=145.0, ankle=100.0,
               knee_x=100.0, ankle_x={"E": -115.0, "D": 125.0}, head=(-40.0, 213.0),
               wheel_off=(420.0, 60.0), wheel_tilt=80.0, wheel_r=G920_RIM_R, grip=90.0)
POSE_TRK = dict(tag="Caminhao", hip=(700.0, 597.0), torso=105.0, pan=9.0, thigh=5.0, knee=120.0, ankle=90.0,
                knee_x=130.0, ankle_x={"E": -115.0, "D": 125.0}, head=(-10.0, 230.0),
                wheel_off=(460.0, -100.0), wheel_tilt=45.0, wheel_r=G920_RIM_R, grip=60.0)

def posture(c):
    """Joint positions, seat corner, heel point and wheel pose for one posture."""
    P = {"c": c}
    Hc = (0.0, c["hip"][0], ZP + c["hip"][1])
    tdir = sag(c["torso"])
    S = v_add(Hc, v_mul(tdir, TORSO))
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
        phi = math.degrees(ps) + 180 - c["ankle"]  # heel -> toe direction angle
        f = sag(phi)
        s = (0.0, f[2], -f[1])  # sole normal (towards the pedal)
        heel = v_add(A, v_add(v_mul(s, 70), v_mul(f, -50)))
        legs[side] = dict(hip=hip, K=K, A=A, f=f, s=s, heel=heel, phi=phi)
    tw_ = math.radians(c["wheel_tilt"])
    n = (0.0, -math.sin(tw_), math.cos(tw_))  # wheel axis, pointing at the driver
    eu = (0.0, math.cos(tw_), math.sin(tw_))  # 12 o'clock direction in the wheel plane
    W = v_add(S, (0.0, c["wheel_off"][0], c["wheel_off"][1]))
    R, g = c["wheel_r"], math.radians(c["grip"])
    grips = {"E": v_add(W, v_add(v_mul(X, -R * math.sin(g)), v_mul(eu, R * math.cos(g)))),
             "D": v_add(W, v_add(v_mul(X, R * math.sin(g)), v_mul(eu, R * math.cos(g))))}
    # seat corner C: hip 95 mm above the pan and 115 mm in front of the back
    al, be = math.radians(c["pan"]), math.radians(c["torso"])
    npn = (0.0, -math.sin(al), math.cos(al))
    nb = (0.0, math.sin(be), -math.cos(be))
    r1, r2 = v_dot(npn, Hc) - 95, v_dot(nb, Hc) - 115
    det = npn[1] * nb[2] - npn[2] * nb[1]
    C = (0.0, (r1 * nb[2] - npn[2] * r2) / det, (npn[1] * r2 - r1 * nb[1]) / det)
    Ph = (0.0, 0.5 * (legs["E"]["heel"][1] + legs["D"]["heel"][1]), 0.5 * (legs["E"]["heel"][2] + legs["D"]["heel"][2]))
    P.update(Hc=Hc, S=S, tdir=tdir, legs=legs, W=W, n=n, eu=eu, R=R, grips=grips, C=C, Ph=Ph,
             head=v_add(S, (0.0, c["head"][0], c["head"][1])))
    return P

F1, TRK = posture(POSE_F1), posture(POSE_TRK)
REPORT = {}

# ============================================================================
# Mechanism synthesis: solve the hinge positions so that ONE rotation plus ONE
# rail slide takes each part exactly from its F1 pose to its truck pose.
# ============================================================================

def solve_pivot(p1, p2, alpha, pz):
    """Pivot (y at fixed height pz) and rail slide dy such that rotating p1 by
    alpha about the pivot, then sliding by dy along Y, lands on p2."""
    ca, sa = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    py = p1[1] - (p2[2] - pz - (p1[2] - pz) * ca) / sa
    q = rot_x(p1, (0.0, py, pz), alpha)
    return (0.0, py, pz), p2[1] - q[1]

# --- seat: parallelogram four-bar (level sub-frame) + carriage slide; the pan
# and the back hinge on the sub-frame at C, so their angles are free ---
P_SUB, N_SUB = sag(SUB_DEG), sag(SUB_DEG + 90)
LINK = 360.0
B_R1 = v_sub(v_add(F1["C"], v_mul(P_SUB, 70)), v_mul(N_SUB, 85))   # link top pivots on the sub-frame
B_F1 = v_sub(v_add(F1["C"], v_mul(P_SUB, 380)), v_mul(N_SUB, 85))
TH1 = math.asin((B_R1[2] - (ZP + 65)) / LINK)                     # rear carriage pivot at ZP + 65
DZ = TRK["C"][2] - F1["C"][2]
TH2 = math.asin(math.sin(TH1) + DZ / LINK)
A_R = v_sub(B_R1, v_mul((0.0, math.cos(TH1), math.sin(TH1)), LINK))
A_F = v_add(A_R, v_sub(B_F1, B_R1))
SEAT_SLIDE = (TRK["C"][1] - F1["C"][1]) - LINK * (math.cos(TH2) - math.cos(TH1))
LINK_ROT = math.degrees(TH2 - TH1)
BACK_ROT = POSE_TRK["torso"] - POSE_F1["torso"]
PAN_ROT = POSE_TRK["pan"] - POSE_F1["pan"]
HEADREST_DEG = POSE_TRK["torso"]  # upper back segment keeps the truck back angle in both postures
REPORT["banco"] = {"biela_graus_F1_cam": [round(math.degrees(TH1), 1), round(math.degrees(TH2), 1)],
                   "curso_carro_mm": round(SEAT_SLIDE, 1), "reclinacao_graus": BACK_ROT,
                   "giro_assento_graus": PAN_ROT}

# --- pedal box: deck tilts about P on a carriage that slides ---
PED_ALPHA = TRK["legs"]["D"]["phi"] - F1["legs"]["D"]["phi"]
PED_P, PED_SLIDE = solve_pivot(F1["Ph"], TRK["Ph"], PED_ALPHA, ZP + 130.0)
REPORT["pedaleira"] = {"pivo_y_z": [round(PED_P[1]), round(PED_P[2] - ZP)], "giro_graus": round(PED_ALPHA, 1),
                       "curso_carro_mm": round(PED_SLIDE, 1)}

# --- steering: arm tilts about Q on a mast that slides. Among crossbar
# heights that clear both postures' legs (thighs, knees, shins) by >= 40 mm
# and stay below the eye -> rim-top sight line by >= 30 mm, take the one with
# the shortest arm (pivot -> wheel centre). ---
WHEEL_ALPHA = POSE_TRK["wheel_tilt"] - POSE_F1["wheel_tilt"]

def leg_clearance(P, y, z, half):
    """Sagittal clearance from a full-width bar at (y, z) to both legs."""
    def sd(a, b):
        dy, dz = b[1] - a[1], b[2] - a[2]
        t = max(0.0, min(1.0, ((y - a[1]) * dy + (z - a[2]) * dz) / (dy * dy + dz * dz)))
        return math.hypot(y - a[1] - t * dy, z - a[2] - t * dz)
    c = 1e9
    for L in P["legs"].values():
        c = min(c, sd(L["hip"], L["K"]) - 66 - half, sd(L["K"], L["A"]) - 50 - half)
    return c

def sight_margin(P, y, ztop):
    """Height of the eye -> rim-top line above ztop at station y (only ahead of the rim)."""
    eye = v_add(P["head"], (0.0, 60.0, 20.0))
    top = v_add(P["W"], v_mul(P["eu"], P["R"] + 14))
    if y <= top[1]:
        return 999.0
    return eye[2] + (top[2] - eye[2]) * (y - eye[1]) / (top[1] - eye[1]) - ztop

Q_PIV, best = None, None
for qz in range(int(ZP + 450), int(ZP + 950), 10):
    q, dm = solve_pivot(F1["W"], TRK["W"], WHEEL_ALPHA, float(qz))
    c1, c2 = leg_clearance(F1, q[1], qz, 15), leg_clearance(TRK, q[1] + dm, qz, 15)
    s1, s2 = sight_margin(F1, q[1], qz + 15), sight_margin(TRK, q[1] + dm, qz + 15)
    arm_len = v_len(v_sub(F1["W"], q))
    if c1 >= 40 and c2 >= 40 and s1 >= 30 and s2 >= 30 and 150 < q[1] < 1650 and 150 < q[1] + dm < 1650:
        if best is None or arm_len < best[0]:
            best = (arm_len, q, dm, c1, c2, min(s1, s2))
_, Q_PIV, MAST_SLIDE, c1, c2, smin = best
REPORT["volante"] = {"pivo_y_z": [round(Q_PIV[1]), round(Q_PIV[2] - ZP)], "giro_graus": WHEEL_ALPHA,
                     "curso_mastro_mm": round(MAST_SLIDE, 1), "braco_mm": round(best[0]),
                     "folga_pernas_F1_cam_mm": [round(c1), round(c2)], "margem_visao_mm": round(smin)}

# ============================================================================
# PLATAFORMA-MOVEL (root Dynamic Component)
# ============================================================================

O = (0.0, 0.0, 0.0)
mov_cd = ComponentDefinition()
mov_cd.set_name("Plataforma_Movel")
model.add_component_definitions([mov_cd])
dc_write(mov_cd, {
    "_name": "Plataforma_Movel", "_lengthunits": "CENTIMETERS", "_formatversion": 1.0,
    "modo": 0.0, "_modo_label": "Modo", "_modo_access": "LIST", "_modo_options": "&F1=0&Caminhao=1&",
    "_modo_formlabel": "Posicao de pilotagem (0 = F1, 1 = Caminhao)", "_modo_units": "FLOAT",
    "sentido": 1.0, "_sentido_label": "Sentido", "_sentido_access": "LIST", "_sentido_options": "&Padrao=1&Invertido=-1&",
    "_sentido_formlabel": "Sentido das rotacoes (troque se a animacao girar ao contrario)", "_sentido_units": "FLOAT",
    "onclick": 'ANIMATE("modo",0,1)', "_onclick_label": "onClick", "_onclick_formula": 'ANIMATE("modo",0,1)',
    "description": "Clique com a ferramenta Interagir para alternar entre as posicoes F1 e Caminhao.",
    "_description_label": "Description", "_description_access": "VIEW",
})
mov_inst = mov_cd.create_instance()
mov_inst.set_name("Plataforma_Movel")
model.get_entities().add_instance(mov_inst)

fr = new_group(mov_cd, "Chassi_Movel")
for s in (-1, 1):
    tube(fr, "Longarina_Principal_" + ("E" if s < 0 else "D"), (s * 270, 0, 365), (s * 270, 1780, 365), "H")
for y in (15.0, 700.0, 1765.0):
    tube(fr, "Travessa_Principal_y%d" % int(y), (-260, y, 365), (260, y, 365), "H")
for y in (350.0, 1100.0, 1450.0):
    tube(fr, "Travessa_Leve_y%d" % int(y), (-260, y, 370), (260, y, 370), "L")
for s in (-1, 1):
    tube(fr, "Trilho_Banco_" + ("E" if s < 0 else "D"), (s * 150, 5, 390), (s * 150, 1110, 390), "L")
    tube(fr, "Trilho_Pedaleira_" + ("E" if s < 0 else "D"), (s * 230, 880, 390), (s * 230, 1775, 390), "L")
block(fr, "Chapa_Pivo", -80, 620, 340, 80, 780, 350, M_PLATE)
for s in (-1, 1):
    side = "E" if s < 0 else "D"
    block(fr, "Suporte_Haste_" + side, s * 280, 35, 330, s * 330, 85, 350, M_PLATE)
    cyl(fr, "Haste_Ligacao_" + side, (s * 310, 60, MZ + 13), (s * 310, 60, 317), 8, M_STEEL)
    ball(fr, "Terminal_Rotula_Sup_" + side, (s * 310, 60, 317), 13, M_STEEL)

# ----------------------------------------------------------------------------
# Seat: carriage (rail slide) -> four links (rotate) -> pan (counter-rotates,
# stays level) -> back (reclines about the pan/back hinge C).
# ----------------------------------------------------------------------------

CARRO_O = (0.0, A_R[1], ZP + 35)
carro_cd, _, carro = dc_child(mov_cd, "Carro_Banco", CARRO_O, O, slide=(0.0, SEAT_SLIDE, 0.0))
ys, ye = A_R[1] - 60, A_F[1] + 60
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    tube(carro, "Carro_Banco_" + sd, (sx * 150, ys, ZP + 35), (sx * 150, ye, ZP + 35), "H")
    for nm, a_pt in (("Tras", A_R), ("Diant", A_F)):
        block(carro, "Mancal_Biela_%s_%s" % (nm, sd), sx * 140, a_pt[1] - 20, ZP + 50, sx * 160, a_pt[1] + 20, a_pt[2] + 12, M_PLATE)
        pin(carro, "Pino_Mancal_%s_%s" % (nm, sd), (sx * 165, a_pt[1], a_pt[2]), 18)
    pin(carro, "Pino_Trava_Trilho_" + sd, (sx * 150, ye - 25, ZP + 20), 22, 6)
tube(carro, "Travessa_Carro_Banco", (-140, ys + 15, ZP + 35), (140, ys + 15, ZP + 35), "H")
tube(carro, "Travessa_Carro_Banco_Diant", (-140, ye - 15, ZP + 35), (140, ye - 15, ZP + 35), "H")

banco_wrap = None
for nm, a_pt in (("Tras", A_R), ("Diant", A_F)):
    for sx in (-1, 1):
        sd = "E" if sx < 0 else "D"
        b_pt = v_add(a_pt, v_sub(B_R1, A_R))
        a3, b3 = (sx * 180, a_pt[1], a_pt[2]), (sx * 180, b_pt[1], b_pt[2])
        bcd, _, bw = dc_child(carro_cd, "Biela_Banco_%s_%s" % (sd, nm), a3, CARRO_O, rot=LINK_ROT)
        tube(bw, "Biela", v_sub(a3, v_mul(v_norm(v_sub(b3, a3)), 15)), v_add(b3, v_mul(v_norm(v_sub(b3, a3)), 15)), "H",
             up=(0, -math.sin(TH1), math.cos(TH1)), m=M_LINK)
        if nm == "Tras" and sd == "E":
            # The pan hangs from this link's top pivot and counter-rotates, so it
            # translates exactly as the parallelogram dictates.
            banco_cd, _, banco_wrap = dc_child(bcd, "Banco", b3, a3, rot=-LINK_ROT)
            BANCO_O = b3

C1 = F1["C"]
seat = banco_wrap  # level sub-frame: rails, hinge posts, lock sectors and the shifter
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    rb, rf, rc = (sx * 262, B_R1[1], B_R1[2]), (sx * 262, B_F1[1], B_F1[2]), (sx * 262, C1[1], C1[2])
    tube(seat, "Longarina_Subchassi_" + sd, v_sub(rb, v_mul(P_SUB, 25)), v_add(rf, v_mul(P_SUB, 25)), "H", up=N_SUB)
    tube(seat, "Montante_Articulacao_" + sd, rb, rc, "H", up=N_SUB)
    for b_pt in (B_R1, B_F1):
        pin(seat, "Pino_Biela_" + sd, (sx * 221, b_pt[1], b_pt[2]), 45)
    cyl(seat, "Setor_Trava_Assento_" + sd, (sx * 274, C1[1], C1[2]), (sx * 278, C1[1], C1[2]), 70, M_PLATE, 20)
    cyl(seat, "Setor_Trava_Encosto_" + sd, (sx * 280, C1[1], C1[2]), (sx * 284, C1[1], C1[2]), 80, M_PLATE, 20)
pin(seat, "Eixo_Articulacao_Banco", C1, 290, 10)

# Driving Force Shifter on a bracket bolted to the right sub-frame rail, so it
# rides with the seat and stays level in both postures.
sh_o = v_add(v_sub(v_add(C1, v_mul(P_SUB, 150)), v_mul(N_SUB, 40)), (360.0, 0.0, 0.0))
tube(seat, "Suporte_Cambio", (272.0, sh_o[1], sh_o[2] - 15), (295.0, sh_o[1], sh_o[2] - 15), "H")
block(seat, "Chapa_Cambio", 295, sh_o[1] - 95, sh_o[2] - 30, 435, sh_o[1] + 95, sh_o[2], M_PLATE)
shf = named_component(seat, "G920_Cambio")
block(shf, "Carcaca", 360 - SHIFTER_W / 2, sh_o[1] - SHIFTER_D / 2, sh_o[2], 360 + SHIFTER_W / 2, sh_o[1] + SHIFTER_D / 2, sh_o[2] + 70, M_MOTOR)
cyl(shf, "Alavanca", (360.0, sh_o[1], sh_o[2] + 70), (360.0, sh_o[1], sh_o[2] + SHIFTER_H - 25), 7, M_STEEL)
ball(shf, "Manopla", (360.0, sh_o[1], sh_o[2] + SHIFTER_H - 22), 22, M_CARB)

# Pan: hinged at C, follows the thigh in F1 (25 deg) and returns to 9 deg for the truck.
P1, N1 = sag(POSE_F1["pan"]), sag(POSE_F1["pan"] + 90)
_, _, assento = dc_child(banco_cd, "Assento", C1, BANCO_O, rot=PAN_ROT)
oblock(assento, "Assento_Estofado", C1, X, P1, N1, (-240, 0, -60, 240, 440, 0), M_SEAT)
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    oblock(assento, "Aba_Assento_" + sd, C1, X, P1, N1, (min(sx * 200, sx * 250), 30, 0, max(sx * 200, sx * 250), 380, 55), M_BOLST)
    pin(assento, "Pino_Trava_Assento_" + sd, v_add((sx * 268, C1[1], C1[2]), v_mul(P1, 50)), 8, 6)

# Back: lower (lumbar) segment reclines about C; the upper segment hinges at
# clavicle height (J, the F1 shoulder line projected on the back) and counter-
# rotates, so it always sits at HEADREST_DEG: a racing-shell shoulder/head
# support in F1, a straight high back in the truck.
back_dir, back_n = sag(POSE_F1["torso"]), v_cross(X, sag(POSE_F1["torso"]))  # back_n points rearwards
D_J = v_dot(v_sub(F1["S"], C1), back_dir)
J = v_add(C1, v_mul(back_dir, D_J))
enc_cd, _, encosto = dc_child(banco_cd, "Encosto", C1, BANCO_O, rot=BACK_ROT)
oblock(encosto, "Encosto_Lombar", C1, X, back_dir, back_n, (-240, 0, 0, 240, D_J, 60), M_SEAT)
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    oblock(encosto, "Aba_Encosto_" + sd, C1, X, back_dir, back_n, (min(sx * 200, sx * 250), 70, -65, max(sx * 200, sx * 250), D_J - 30, 0), M_BOLST)
    pin(encosto, "Pino_Trava_Encosto_" + sd, v_add((sx * 287, C1[1], C1[2]), v_mul(back_dir, 60)), 6, 6)
    cyl(encosto, "Setor_Trava_Apoio_Superior_" + sd, (sx * 252, J[1], J[2]), (sx * 258, J[1], J[2]), 60, M_PLATE, 20)
pin(encosto, "Eixo_Articulacao_Apoio_Superior", J, 262, 9)

hr_dir, hr_n = sag(HEADREST_DEG), v_cross(X, sag(HEADREST_DEG))
_, _, apoio = dc_child(enc_cd, "Encosto_Superior", J, C1, rot=-BACK_ROT)
oblock(apoio, "Apoio_Ombros_Cabeca", J, X, hr_dir, hr_n, (-240, 0, 0, 240, 430, 60), M_SEAT)
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    oblock(apoio, "Aba_Ombro_" + sd, J, X, hr_dir, hr_n, (min(sx * 245, sx * 275), 0, -90, max(sx * 245, sx * 275), 160, 0), M_BOLST)
    oblock(apoio, "Aba_Cabeca_" + sd, J, X, hr_dir, hr_n, (min(sx * 112, sx * 142), 250, -100, max(sx * 112, sx * 142), 430, 0), M_BOLST)
    pin(apoio, "Pino_Trava_Apoio_Superior_" + sd, v_add((sx * 255, J[1], J[2]), v_mul(hr_dir, 50)), 8, 6)

f1, s1 = F1["legs"]["D"]["f"], F1["legs"]["D"]["s"]
ms1 = v_mul(s1, -1)
Ph1 = F1["Ph"]
pc_o = (0.0, PED_P[1], ZP + 35)
pcarro_cd, _, pcarro = dc_child(mov_cd, "Carro_Pedaleira", pc_o, O, slide=(0.0, PED_SLIDE, 0.0))
pys, pye = PED_P[1] - 90, PED_P[1] + 140
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    tube(pcarro, "Carro_Pedaleira_" + sd, (sx * 230, pys, ZP + 35), (sx * 230, pye, ZP + 35), "H")
    tube(pcarro, "Torre_Pivo_Pedaleira_" + sd, (sx * 230, PED_P[1], ZP + 50), (sx * 230, PED_P[1], PED_P[2] + 20), "H", up=(0, 1, 0))
    cyl(pcarro, "Setor_Trava_Pedaleira_" + sd, (sx * 241, PED_P[1], PED_P[2]), (sx * 247, PED_P[1], PED_P[2]), 60, M_PLATE, 20)
    pin(pcarro, "Pino_Trava_Trilho_Ped_" + sd, (sx * 230, pye - 25, ZP + 20), 22, 6)
tube(pcarro, "Travessa_Carro_Pedaleira", (-220, pys + 15, ZP + 35), (220, pys + 15, ZP + 35), "H")
pin(pcarro, "Eixo_Pivo_Pedaleira", PED_P, 250, 9)

_, _, deck = dc_child(pcarro_cd, "Pedaleira_Basculante", PED_P, pc_o, rot=PED_ALPHA)
# Deck frame: origin at the heel point, ya = heel -> toe, za = from the pedal
# unit towards the driver's sole. The mounting plate takes the G920 pedal unit
# (bolted through its base); the pad faces sit at the sole plane (za ~ -8).
DY0 = -60.0  # heel-rest edge of the pedal unit, along the sole
oblock(deck, "Chapa_Fixacao_Pedais", Ph1, X, f1, ms1, (-218, DY0 - 10, -(PEDAL_DEPTH + 8), 218, DY0 + G920_PED_D + 10, -PEDAL_DEPTH), M_PLATE)
ped = named_component(deck, "G920_Pedais")
xc = 0.5 * (F1["legs"]["E"]["heel"][0] + F1["legs"]["D"]["heel"][0])
oblock(ped, "Carcaca", Ph1, X, f1, ms1, (xc - G920_PED_W / 2, DY0, -PEDAL_DEPTH, xc + G920_PED_W / 2, DY0 + G920_PED_D, -PEDAL_DEPTH + 55), M_MOTOR)
oblock(ped, "Apoio_Calcanhar", Ph1, X, f1, ms1, (xc - 190, DY0, -PEDAL_DEPTH + 55, xc + 190, DY0 + 70, -PEDAL_DEPTH + 75), M_CARB)
for nm, px in (("Embreagem", xc - G920_PAD_PITCH), ("Freio", xc), ("Acelerador", xc + G920_PAD_PITCH)):
    oblock(ped, "Pedal_" + nm, Ph1, X, f1, ms1, (px - 37, 85, -24, px + 37, 245, -8), M_PED)
    tube(ped, "Haste_Pedal_" + nm, local_to_world(Ph1, X, f1, ms1, (px, 165, -24)),
         local_to_world(Ph1, X, f1, ms1, (px, 225, -PEDAL_DEPTH + 55)), "H", up=f1, size=(18.0, 12.0), m=M_STEEL)
# deck arms hang from the pivot, outboard of the towers, and carry the plate
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    pv = (sx * 250, PED_P[1], PED_P[2])
    lo = local_to_world(Ph1, X, f1, ms1, (sx * 250, DY0 + 20, -(PEDAL_DEPTH + 23)))
    hi = local_to_world(Ph1, X, f1, ms1, (sx * 250, DY0 + G920_PED_D - 10, -(PEDAL_DEPTH + 23)))
    tube(deck, "Braco_Pedaleira_Inf_" + sd, pv, lo, "H")
    tube(deck, "Braco_Pedaleira_Sup_" + sd, pv, hi, "H")
for nm, yy in (("Inf", DY0 + 20), ("Sup", DY0 + G920_PED_D - 10)):
    tube(deck, "Travessa_Pedaleira_" + nm, local_to_world(Ph1, X, f1, ms1, (-260, yy, -(PEDAL_DEPTH + 23))),
         local_to_world(Ph1, X, f1, ms1, (260, yy, -(PEDAL_DEPTH + 23))), "H", up=ms1)

# ----------------------------------------------------------------------------
# Steering: mast (rail slide) -> arm carrying the G920 table (rotates about
# Q_PIV). One rim serves both postures; only its angle and position change.
# ----------------------------------------------------------------------------

W1, n1, eu1 = F1["W"], F1["n"], F1["eu"]
mast_o = (0.0, Q_PIV[1], ZP)
mast_cd, _, mast = dc_child(mov_cd, "Mastro_Volante", mast_o, O, slide=(0.0, MAST_SLIDE, 0.0))
qy, qz = Q_PIV[1], Q_PIV[2]
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    block(mast, "Abracadeira_Trilho_" + sd, sx * 255, qy - 35, 380, sx * 285, qy + 35, 396, M_PLATE)
    tube(mast, "Coluna_Volante_" + sd, (sx * 270, qy, 396), (sx * 270, qy, qz + 15), "H", up=(0, 1, 0))
    pin(mast, "Pino_Trava_Trilho_Vol_" + sd, (sx * 290, qy + 20, 388), 12, 6)
    block(mast, "Mancal_Braco_" + sd, sx * 52, qy - 22, qz - 15, sx * 64, qy + 22, qz + 25, M_PLATE)
    cyl(mast, "Setor_Trava_Volante_" + sd, (sx * 66, qy, qz), (sx * 72, qy, qz), 55, M_PLATE, 20)
tube(mast, "Travessa_Volante", (-260, qy, qz), (260, qy, qz), "H")
pin(mast, "Eixo_Pivo_Volante", Q_PIV, 75, 9)

arm_cd, _, arm = dc_child(mast_cd, "Braco_Volante", Q_PIV, mast_o, rot=WHEEL_ALPHA)
# The G920 base bolts flat on a table; the rim plane is fixed relative to the
# base, so tilting the table tilts the rim. F1 table frame = world axes, rotated
# so that the rim plane sits at the F1 angle.
tilt0 = POSE_F1["wheel_tilt"] - G920_WHEEL_PLANE
tY, tZ = sag(tilt0), sag(tilt0 + 90)
BASE_DROP = 135.0  # wheel centre -> underside of the G920 base
mesa_o = v_sub(W1, v_mul(tZ, BASE_DROP))
oblock(arm, "Mesa_Volante", mesa_o, X, tY, tZ, (-150, 45, -8, 150, 285, 0), M_PLATE)
g920 = named_component(arm, "G920_Volante")
oblock(g920, "Base_Motor", mesa_o, X, tY, tZ, (-130, 30, 0, 130, 262, 170), M_MOTOR)
oblock(g920, "Tampa", mesa_o, X, tY, tZ, (-95, 60, 170, 95, 240, 190), M_CARB)
for sx in (-1, 1):
    cyl(g920, "Grampo_Mesa", local_to_world(mesa_o, X, tY, tZ, (sx * 105, 200, -8)),
        local_to_world(mesa_o, X, tY, tZ, (sx * 105, 200, -60)), 12, M_CARB, 12)
torus(g920, "Aro", W1, X, eu1, n1, G920_RIM_R, 14.0, M_WHEEL)
cyl(g920, "Cubo", v_add(W1, v_mul(n1, 18)), local_to_world(mesa_o, X, tY, tZ, (0, 40, BASE_DROP)), 42, M_CARB)
for ang in (0, 180, 270):
    a = math.radians(ang)
    d2 = v_add(v_mul(X, math.cos(a)), v_mul(eu1, math.sin(a)))
    tube(g920, "Raio", v_add(v_mul(d2, 35), W1), v_add(v_mul(d2, G920_RIM_R - 5), W1), "H", up=n1, size=(30.0, 14.0), m=M_CARB)
for sx in (-1, 1):
    oblock(g920, "Borboleta_" + ("E" if sx < 0 else "D"), v_sub(W1, v_mul(n1, 25)), X, eu1, n1,
           (sx * 60, -10, -6, sx * 115, 45, 0), M_STEEL)
for sx in (-1, 1):
    sd = "E" if sx < 0 else "D"
    for k, ly in (("Diant", 70), ("Tras", 260)):
        tube(arm, "Braco_Volante_%s_%s" % (k, sd), local_to_world(mesa_o, X, tY, tZ, (sx * 40, ly, -8)), (sx * 40, qy, qz), "H", up=(1, 0, 0))

# ============================================================================
# Mannequins (articulated "rigged" figure, ~173 cm standing). One per posture,
# shown only when the mechanism is fully in that posture.
# ============================================================================

NOM = 1000.0

def make_def(name, profile, m, n=16):
    cd = ComponentDefinition()
    cd.set_name(name)
    model.add_component_definitions([cd])
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

def build_mannequin(P, hide):
    c = P["c"]
    _, inst, man = dc_child(mov_cd, "Manequim_" + c["tag"], O, O, hide=hide)
    Hc, S, tdir = P["Hc"], P["S"], P["tdir"]
    bone(man, D_TORSO, "Tronco", v_sub(Hc, v_mul(tdir, 75)), v_add(S, v_mul(tdir, 10)), 1.35, 0.75)
    bone(man, D_NECK, "Pescoco", S, v_add(S, v_mul(v_sub(P["head"], S), 0.62)))
    joint(man, "Cabeca", P["head"], 98, D_HEAD, 1.18)
    reach = {}
    for side, sx in (("E", -1), ("D", 1)):
        L = P["legs"][side]
        joint(man, "Quadril_" + side, L["hip"], 58)
        bone(man, D_THIGH, "Coxa_" + side, L["hip"], L["K"])
        joint(man, "Joelho_" + side, L["K"], 48)
        bone(man, D_SHANK, "Canela_" + side, L["K"], L["A"])
        joint(man, "Tornozelo_" + side, L["A"], 37)
        oblock(man, "Pe_" + side, L["heel"], X, L["f"], v_mul(L["s"], -1), (-45, -10, 0, 45, 255, 80), M_BONE)
        # two-bone arm IK: shoulder -> hand on the wheel rim, elbow down and out
        Psh, T = (sx * SHOULDER_HALF_WIDTH, S[1], S[2]), P["grips"][side]
        dvec = v_sub(T, Psh)
        d = min(v_len(dvec), UPPER_ARM + FOREARM_TO_GRIP - 2)
        u = v_norm(dvec)
        pole = v_norm((sx * 0.6, -0.15, -1.0))
        pole = v_norm(v_sub(pole, v_mul(u, v_dot(pole, u))))
        a = (UPPER_ARM ** 2 - FOREARM_TO_GRIP ** 2 + d ** 2) / (2 * d)
        E = v_add(Psh, v_add(v_mul(u, a), v_mul(pole, math.sqrt(max(UPPER_ARM ** 2 - a ** 2, 0.0)))))
        joint(man, "Ombro_" + side, Psh, 46)
        bone(man, D_UARM, "Braco_" + side, Psh, E)
        joint(man, "Cotovelo_" + side, E, 37)
        bone(man, D_FARM, "Antebraco_" + side, E, T)
        joint(man, "Mao_" + side, T, 40)
        reach[side] = round(v_len(dvec))
    return inst, reach

man_f1, reach_f1 = build_mannequin(F1, "F1")
man_trk, reach_trk = build_mannequin(TRK, "TRK")
REPORT["alcance_mm"] = {"F1": reach_f1, "Caminhao": reach_trk}

# ============================================================================
# Cross-checks of the synthesis (truck pose reached by the mechanisms).
# ============================================================================

def ped_world(p, m):
    q = rot_x(p, PED_P, PED_ALPHA * m)
    return (q[0], q[1] + PED_SLIDE * m, q[2])

def wheel_world(p, m):
    q = rot_x(p, Q_PIV, WHEEL_ALPHA * m)
    return (q[0], q[1] + MAST_SLIDE * m, q[2])

deck_corners = [local_to_world(Ph1, X, f1, ms1, (0, yy, zz)) for yy in (DY0 - 10, DY0 + G920_PED_D + 10)
                for zz in (-(PEDAL_DEPTH + 38), 0)]
g920_pts = [local_to_world(mesa_o, X, tY, tZ, (xx, yy, zz)) for xx in (-130, -65, 0, 65, 130)
            for yy in range(30, 263, 29) for zz in range(-8, 191, 25)]
def seg_dist(p, a, b):
    ab = v_sub(b, a)
    t = max(0.0, min(1.0, v_dot(v_sub(p, a), ab) / v_dot(ab, ab)))
    return v_len(v_sub(p, v_add(a, v_mul(ab, t))))

rim_trk = [v_add(TRK["W"], v_add(v_mul(X, G920_RIM_R * math.cos(t)), v_mul(TRK["eu"], G920_RIM_R * math.sin(t))))
           for t in [i * math.pi / 12 for i in range(24)]]
min_knee = 1e9  # G920 base / rim vs truck thighs (r ~62) and knees (r 48)
for L in TRK["legs"].values():
    for p in g920_pts:
        q = wheel_world(p, 1)
        min_knee = min(min_knee, seg_dist(q, L["hip"], L["K"]) - 62, v_len(v_sub(q, L["K"])) - 48)
    for q in rim_trk:
        min_knee = min(min_knee, seg_dist(q, L["hip"], L["K"]) - 62 - 14)
hr_front = v_mul(hr_n, -1)
link_z = A_R[2] + (B_R1[2] - A_R[2]) * ((C1[1] + v_mul(N1, -60)[1]) - A_R[1]) / (B_R1[1] - A_R[1])
REPORT["banco_F1"] = {
    "articulacao_clavicula_mm_do_eixo": round(D_J),
    "folga_cabeca_apoio_mm": round(v_dot(v_sub(F1["head"], J), hr_front) - 98),
    "folga_tronco_apoio_mm": round(v_dot(v_sub(F1["S"], J), hr_front) - 62),
    "folga_assento_biela_mm": round((C1[2] - 60 * N1[2]) - (link_z + 15)),
    "folga_coxa_assento_mm": round(95 - 66),
}
REPORT["verificacao"] = {
    "erro_calcanhar_cam_mm": round(v_len(v_sub(ped_world(Ph1, 1), TRK["Ph"])), 2),
    "erro_volante_cam_mm": round(v_len(v_sub(wheel_world(W1, 1), TRK["W"])), 2),
    "folga_G920_pernas_cam_mm": round(min_knee),
    "base_pedais_z_min_cam_mm_acima_chassi": round(min(ped_world(p, 1)[2] for p in deck_corners) - ZP),
    "carro_banco_y_min_max": [round(min(ys, ys + SEAT_SLIDE)), round(max(ye, ye + SEAT_SLIDE))],
    "carro_pedal_y_min_max": [round(min(pys, pys + PED_SLIDE)), round(max(pye, pye + PED_SLIDE))],
    "mastro_y_F1_cam": [round(qy), round(qy + MAST_SLIDE)],
    "topo_cabeca_mm": {"F1": round(F1["head"][2] + 116), "Caminhao": round(TRK["head"][2] + 116)},
}

# ============================================================================
# Tags, style, scenes and opening camera
# ============================================================================

def mk_layer(name):
    for l in model.get_layers():
        if l.get_name() == name:
            return l
    l = Layer()
    l.set_name(name)
    model.add_layers([l])
    return l

L_BASE, L_MOV, L_MAN = mk_layer("Plataforma_Base"), mk_layer("Plataforma_Movel"), mk_layer("Manequim")
base.set_layer(L_BASE)
mov_inst.set_layer(L_MOV)
man_f1.set_layer(L_MAN)
man_trk.set_layer(L_MAN)

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
scene_defs = [("Vista 3/4", V_34, []), ("Vista Lateral", V_SIDE, []),
              ("Mecanismos (sem manequim)", V_SIDE, [L_MAN]), ("Base e Atuadores", V_MECH, [L_MOV, L_MAN])]
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
model.set_camera(cam(*V_34))

apply_state(0.0)
session_state["apply_state"] = apply_state
result = REPORT
