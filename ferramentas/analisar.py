"""Engineering calculations, bill of materials and cut plan (runs locally).

Loads the parametric model (modelo/gerar_modelo.py, EMIT = False) and writes:
  docs/CALCULOS.md            mass properties, motion envelope, springs,
                              actuators, stresses, pins, frequencies and the
                              geometric verification summary
  docs/LISTA_DE_MATERIAIS.md  bill of materials (+ docs/lista_de_materiais.csv)
  docs/PLANO_DE_CORTE.md      tube cut plan on 6 m bars (+ docs/plano_de_corte.csv)

Every number in those documents comes from this file or from the model, so
re-running it after a model change keeps the documents consistent:
    python3 ferramentas/analisar.py
Assumptions are explicit constants below; change them here, not in the docs.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verificar as V  # noqa: E402

ROOT = V.ROOT
DOCS = os.path.join(ROOT, "docs")
G = 9.81

# ----------------------------------------------------------------- assumptions
E_STEEL = 200000.0        # MPa
SY = 250.0                # MPa, yield of cold-formed carbon steel tube (SAE 1010/1020, NBR 6591) - conservative
SIG_ADM = SY / 2.0        # MPa, static allowable (safety factor 2 on yield) for the design (peak) loads
DSIG_FAT = 50.0           # MPa, allowable stress RANGE at welded joints for the cyclic loads (~EN 1993-1-9 cat. 50)
ETA_GEAR = 0.90           # planetary gearbox efficiency
J_MOTOR = 2.7e-4          # kg.m2, NEMA 34 12 N.m rotor inertia (catalogue order of magnitude)
CRANK_PHI_MAX = 60.0      # deg, software limit of the crank angle from horizontal (keeps force multiplication <= 2)
DYN = 1.5                 # dynamic factor on driver loads
K_SPRING_FACTOR = 1.10    # spring rotational stiffness = 1.10 x m.g.h of the truck mode (stable in both modes)
F_BRAKE = 500.0           # N, hard push on the G920 brake pedal
F_WHEEL = 300.0           # N, driver pulling / pushing the wheel rim
F_DOOR = 600.0            # N, person leaning on the door bar (entry / exit)
STOCK = 6000.0            # mm, commercial bar length
KERF = 3.0                # mm, saw kerf
TRIM = 10.0               # mm, squaring cut at each bar end

# Motor torque curves (motor shaft N.m vs rpm). [Provavel] typical catalogue
# shapes; replace with the datasheet of the motor actually bought.
MOTORS = {
    "NEMA 34 malha fechada 12 N.m (80 VCC)": [(0, 12.0), (60, 11.5), (120, 10.8), (300, 9.0), (600, 6.5), (900, 4.8),
                                              (1200, 3.6), (1500, 2.8), (2000, 1.8), (3000, 0.5)],
    "Servo CA 750 W (pico 7,1 N.m ate 3000 rpm)": [(0, 7.1), (3000, 7.1), (3500, 4.0), (4000, 0.5)],
}
RATIOS = (10, 15, 20, 25, 30)
CHOSEN_MOTOR = "NEMA 34 malha fechada 12 N.m (80 VCC)"
CHOSEN_RATIO = 20


def curve(pts, rpm):
    rpm = abs(rpm)
    for (a, ta), (b, tb) in zip(pts, pts[1:]):
        if rpm <= b:
            return ta + (tb - ta) * (rpm - a) / (b - a)
    return pts[-1][1]


def rect(w, h, t):
    A = w * h - (w - 2 * t) * (h - 2 * t)
    Is = (w * h ** 3 - (w - 2 * t) * (h - 2 * t) ** 3) / 12.0
    Iw = (h * w ** 3 - (h - 2 * t) * (w - 2 * t) ** 3) / 12.0
    Am = (w - t) * (h - t)
    J = 4 * Am * Am * t / (2 * ((w - t) + (h - t)))
    return dict(A=A, Is=Is, Ws=Is / (h / 2), Iw=Iw, Ww=Iw / (w / 2), J=J, Wt=2 * Am * t)


def fmt(x, nd=0):
    if x is None:
        return "-"
    s = ("%." + str(nd) + "f") % x
    return s.replace(".", ",")


# =============================================================================
class Calc:
    def __init__(self):
        self.ns = V.load_model()
        ns = self.ns
        self.R = ns["RESULT"]
        self.MP = ns["MP"]
        self.GEO = ns["GEO"]
        self.P = ns["PROFILES"]
        self.sec = {k: rect(*v) for k, v in self.P.items()}
        self.pivot = ns["PIVOT"]
        self.L_pitch = (self.pivot[1] - ns["ROD_Y"]) / 1000.0
        self.L_roll = ns["ROD_X"] / 1000.0
        self.r = ns["CRANK_R"] / 1000.0
        self.rows = []          # stress table rows
        self.notes = []

    # --------------------------------------------------------------- masses
    def mode(self, key):
        mp = self.MP[key]
        h = (mp["cg"][2] - self.pivot[2]) / 1000.0
        return dict(m=mp["mass"], h=h, mgh=mp["mass"] * G * h, Ip=mp["I_pitch"], Ir=mp["I_roll"], cg=mp["cg"])

    # --------------------------------------------------------------- springs
    def springs(self):
        trk = self.mode("Caminhao")
        K = K_SPRING_FACTOR * trk["mgh"]
        sp = {}
        for nm, sp_xy, z0, z1 in self.GEO["springs"]:
            axis = "arfagem" if nm.startswith("arfagem") else "rolagem"
            lever = abs(sp_xy[1] - self.pivot[1]) / 1000.0 if axis == "arfagem" else abs(sp_xy[0]) / 1000.0
            sp.setdefault(axis, dict(lever=lever, n=0, L=(z1 - z0)))
            sp[axis]["n"] += 1
        lim = self.limits()
        for axis, d in sp.items():
            k = K / (d["n"] * d["lever"] ** 2)          # N/m, n springs at the same lever
            amax = lim["arfagem_pura"] if axis == "arfagem" else lim["rolagem_pura"]
            stroke = d["lever"] * math.sin(math.radians(amax)) * 1000.0
            e0 = stroke + 10.0
            d.update(K=K, k=k / 1000.0, stroke=stroke, e0=e0, F0=k / 1000.0 * e0, Fmax=k / 1000.0 * (e0 + stroke))
        self.sp = sp
        self.K = K
        self.preload = sum(d["F0"] * d["n"] for d in sp.values())
        return sp

    # --------------------------------------------------------------- envelope
    def limits(self):
        ns = self.ns
        dmax = ns["CRANK_R"] * math.sin(math.radians(CRANK_PHI_MAX))
        lp = math.degrees(math.asin(dmax / (self.L_pitch * 1000.0)))
        lr = math.degrees(math.asin(dmax / (self.L_roll * 1000.0)))
        geo = V.pitch_roll_limits(ns, 0.0)
        return dict(dmax=dmax, arfagem_pura=lp, rolagem_pura=lr, arfagem_geo=geo["arfagem_max_graus"],
                    rolagem_geo=geo["rolagem_max_graus"])

    # --------------------------------------------------------------- dynamics
    def cue_demand(self, key, axis, A_deg, f, motor, N, cap=None, n=240):
        """Max ratio (required / available motor torque) along one cycle of a
        sinusoidal cue, plus peak rod force, motor rpm and torque."""
        md = self.mode(key)
        I = md["Ip"] if axis == "p" else md["Ir"]
        L = self.L_pitch if axis == "p" else self.L_roll
        kspring = self.K
        A = math.radians(A_deg)
        w = 2 * math.pi * f
        dt = 1.0 / (f * n)
        worst = dict(ratio=0.0, F=0.0, rpm=0.0, T=0.0)
        phis = []
        for i in range(n + 3):
            t = i * dt
            th = A * math.sin(w * t)
            d = L * math.sin(th)
            phis.append(math.asin(max(-1.0, min(1.0, d / self.r))))
        for i in range(1, n + 1):
            t = i * dt
            th = A * math.sin(w * t)
            thdd = -A * w * w * math.sin(w * t)
            M = I * thdd + (kspring - md["mgh"]) * th        # N.m about the axis (motor must supply)
            M *= 1.05                                          # friction / misc
            nrod = 2.0
            F = M / (nrod * L * math.cos(th))                 # per rod
            phi = phis[i]
            phid = (phis[i + 1] - phis[i - 1]) / (2 * dt)
            phidd = (phis[i + 1] - 2 * phis[i] + phis[i - 1]) / (dt * dt)
            Tc = F * self.r * math.cos(phi)
            Tm = abs(Tc) / (N * ETA_GEAR) + J_MOTOR * abs(phidd) * N
            rpm = abs(phid) * N * 60 / (2 * math.pi)
            avail = curve(MOTORS[motor], rpm) * 0.8          # 20 % margin on the catalogue curve
            if cap is not None:
                avail = min(avail, cap)
            ratio = Tm / avail if avail > 1e-6 else 99.0
            if ratio > worst["ratio"]:
                worst.update(ratio=ratio)
            worst["F"] = max(worst["F"], abs(F))
            worst["rpm"] = max(worst["rpm"], rpm)
            worst["T"] = max(worst["T"], Tm)
        return worst

    def fmax(self, key, axis, A_deg, motor, N, cap=None):
        lo, hi = 0.02, 6.0
        if self.cue_demand(key, axis, A_deg, lo, motor, N, cap)["ratio"] > 1:
            return 0.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self.cue_demand(key, axis, A_deg, mid, motor, N, cap)["ratio"] <= 1:
                lo = mid
            else:
                hi = mid
        return lo

    # --------------------------------------------------------------- report
    def add(self, member, case, value, unit, limit, note=""):
        ok = value <= limit
        self.rows.append((member, case, value, unit, limit, "OK" if ok else "NAO ATENDE", note))
        return ok


# =============================================================================

def cut_plan(tubes):
    """First-fit decreasing on STOCK bars, per profile."""
    plans = {}
    for prof in sorted(set(t["prof"] for t in tubes)):
        pieces = sorted([t for t in tubes if t["prof"] == prof], key=lambda t: -t["length"])
        bars = []
        usable = STOCK - 2 * TRIM
        for pc in pieces:
            L = math.ceil(pc["length"])
            if L > usable:
                raise ValueError("piece longer than a bar: %s %s" % (pc["id"], L))
            for b in bars:
                if b["used"] + L + (KERF if b["pieces"] else 0) <= usable:
                    b["used"] += L + (KERF if b["pieces"] else 0)
                    b["pieces"].append((pc["id"], L))
                    break
            else:
                bars.append(dict(used=L, pieces=[(pc["id"], L)]))
        plans[prof] = bars
    return plans


def write_preset(ns):
    """Store the mechanism search result in the generator's PRESET block, so the
    SketchUp run (30 s limit) evaluates this solution instead of searching."""
    G_ = ns["GEO"]
    keys = ("kl", "drag_gy", "back_cand", "back_gy", "pan_cand", "pan_gy", "ped_ll", "ped_ang", "ped_gz")
    pre = {k: G_["preset_" + k] for k in keys}
    pre["shift"] = [round(v, 3) for v in ns["SHIFT"]]
    path = os.path.join(ROOT, "modelo", "gerar_modelo.py")
    with open(path) as f:
        src = f.read()
    a = src.index("# >>> PRESET")
    b = src.index("# <<< PRESET")
    line = src[a:src.index("\n", a) + 1]
    block = line + "PRESET = %r\n" % (pre,)
    new = src[:a] + block + src[b:]
    if new != src:
        with open(path, "w") as f:
            f.write(new)
    return pre


def main():
    c = Calc()
    ns = c.ns
    write_preset(ns)
    GEO = c.GEO
    R = c.R
    PARTS = ns["PARTS"]
    out = []
    w = out.append
    sp = c.springs()
    lim = c.limits()
    f1, trk = c.mode("F1"), c.mode("Caminhao")
    secs = c.sec

    # ------------------------------------------------ actuator selection table
    cues = [("F1", "p", round(lim["arfagem_pura"], 1), "F1 arfagem"), ("F1", "r", round(lim["rolagem_pura"], 1), "F1 rolagem"),
            ("F1", "r", 8.0, "F1 rolagem"), ("F1", "p", 4.0, "F1 arfagem"),
            ("Caminhao", "p", round(lim["arfagem_pura"], 1), "Caminhao arfagem"), ("Caminhao", "r", 6.0, "Caminhao rolagem")]
    # torque cap: limit the rod force at the software crank-angle limit
    F_DESIGN = 2500.0
    cap = {}
    for N in RATIOS:
        cap[N] = F_DESIGN * c.r * math.cos(math.radians(CRANK_PHI_MAX)) / (N * ETA_GEAR)
    table = {}
    for motor in MOTORS:
        for N in RATIOS:
            capN = min(cap[N], MOTORS[motor][0][1])
            table[(motor, N)] = [c.fmax(k, a, A, motor, N, capN) for k, a, A, _ in cues]

    # representative cyclic cue for fatigue: F1 roll +-8 deg at the chosen config's 80 % of fmax
    fm = table[(CHOSEN_MOTOR, CHOSEN_RATIO)]
    cyc_r = c.cue_demand("F1", "r", 8.0, 0.8 * fm[2], CHOSEN_MOTOR, CHOSEN_RATIO)
    cyc_p = c.cue_demand("F1", "p", 4.0, 0.8 * fm[3], CHOSEN_MOTOR, CHOSEN_RATIO)
    F_CYC = cyc_r["F"] + cyc_p["F"]          # rods see roll and pitch together (conservative sum)

    # ------------------------------------------------ static loads
    W = f1["m"] * G
    P_static = W + c.preload
    P_design = W * DYN + c.preload + 2 * F_DESIGN

    # ---------------- structural checks
    s3050, s2030, s1020 = secs["30x50"], secs["20x30"], secs["20x30"]
    # 1. rear rod force into the pivot: side trusses
    M_side = F_DESIGN * c.L_pitch                          # N.m at the pivot section, per side
    d_sill = 0.205                                         # sill truss depth at the pivot
    d_ring = (ns["RING_Z"] - (ns["ZP"] - 25.0)) / 1000.0
    C_sill = M_side / d_sill
    C_ring = M_side / d_ring
    c.add("Trelica soleira E - banzos (longarina 30x50 + banzo inferior 30x50)", "pico: forca maxima da haste (%.0f N)" % F_DESIGN,
          C_sill / s3050["A"], "MPa", SIG_ADM, "forca no banzo %.1f kN" % (C_sill / 1000))
    c.add("Trelica soleira E - banzos", "fadiga: faixa 2 x forca ciclica (%.0f N)" % F_CYC,
          2 * F_CYC * c.L_pitch / d_sill / s3050["A"], "MPa", DSIG_FAT, "amplitude representativa F1")
    Lb = math.hypot(400.0, 75.0)
    Pcr = math.pi ** 2 * E_STEEL * s3050["Iw"] / Lb ** 2
    c.add("Trelica soleira E - banzo comprimido (flambagem, L=400)", "pico", C_sill, "N", Pcr / 2.0, "Pcr/2 = %.0f kN" % (Pcr / 2000))
    c.add("Anel lado D (trelica, banzos 20x30)", "pico", C_ring / s2030["A"], "MPa", SIG_ADM, "altura util %.0f mm" % (d_ring * 1000))
    c.add("Longarina sem trelica (referencia - o que a soleira evita)", "pico", M_side * 1000 / s3050["Ws"], "MPa", SIG_ADM,
          "mostra por que a soleira E e obrigatoria")
    # 2. pivot crosses
    plate_half = 150.0
    arm = (270.0 - plate_half) / 1000.0
    M_cross_peak = (P_design / 2) * arm / 3
    M_cross_cyc = (2 * F_CYC / 2) * arm / 3
    c.add("Travessas do pivo (3 x 30x50)", "pico (peso x %.1f + molas + 2 hastes)" % DYN, M_cross_peak * 1000 / s3050["Ws"], "MPa",
          SIG_ADM, "P = %.1f kN" % (P_design / 1000))
    c.add("Travessas do pivo (3 x 30x50)", "fadiga", 2 * M_cross_cyc * 1000 / s3050["Ws"], "MPa", DSIG_FAT, "")
    Wp = 140.0 * 16 ** 2 / 6
    c.add("Chapa do pivo 300x140x16", "pico", (P_design / 2) * 0.090 * 1000 / Wp, "MPa", SIG_ADM, "balanco 90 mm ate o garfo")
    # 3. base tower legs
    leg_sin = 183.0 / math.sqrt(155.0 ** 2 + 110.0 ** 2 + 183.0 ** 2)
    N_leg = P_design / (4 * leg_sin)
    Pcr_leg = math.pi ** 2 * E_STEEL * secs["20x30"]["Iw"] / 263.0 ** 2
    c.add("Pernas da torre do pivo (4 x 20x30)", "pico, compressao", N_leg / secs["20x30"]["A"], "MPa", SIG_ADM, "")
    c.add("Pernas da torre do pivo (flambagem)", "pico", N_leg, "N", Pcr_leg / 2, "Pcr/2 = %.0f kN" % (Pcr_leg / 2000))
    # 4. seat
    seg = ns["SEG_FRAC"]
    m_on_seat = ns["DRIVER_KG"] * (seg["trunk"] + seg["head"] + 2 * (seg["upper_arm"] + seg["forearm_hand"]) + 2 * seg["thigh"])
    m_seat = sum(p["mass"] for p in PARTS if p["node"] in ("BANCO", "ASSENTO", "ENCOSTO", "APOIO", "TIRANTE_E", "TIRANTE_D"))
    W_seat = (m_on_seat + m_seat) * G * DYN
    th2 = GEO["TH2"]
    N_link = W_seat / (4 * math.sin(th2))
    Pcr_link = math.pi ** 2 * E_STEEL * secs["20x30"]["Iw"] / ns["LINK"] ** 2
    c.add("Bielas do banco (caminhao, %.0f graus)" % math.degrees(th2), "compressao", N_link, "N", Pcr_link / 3,
          "Pcr/3 = %.0f kN" % (Pcr_link / 3000))
    rods = GEO["rods"]
    a_drag = math.radians(rods["ARRASTO_E"]["a2"])
    H = W_seat / math.tan(th2)
    F_drag = H / (2 * abs(math.cos(a_drag)))
    c.add("Tirantes de arrasto (caminhao)", "tracao/compressao", F_drag / secs["20x30"]["A"], "MPa", SIG_ADM,
          "F = %.0f N" % F_drag)

    # moment arms of the coupling rods (F1 pose)
    def rod_arm(desc, centre):
        p = [q for q in PARTS if q["desc"] == desc][0]
        a, b = p["line"]
        u = V._sub(b, a)
        l = math.hypot(u[1], u[2])
        r = V._sub(b, centre)
        return abs(r[1] * u[2] - r[2] * u[1]) / l / 1000.0, p["length"]

    C1 = GEO["C1"]
    D_J = GEO["D_J"] / 1000.0
    m_upper = ns["DRIVER_KG"] * (seg["trunk"] + seg["head"] + 2 * (seg["upper_arm"] + seg["forearm_hand"]))
    beta = math.radians(180 - ns["POSE_F1"]["torso"])
    # brake reaction: the legs push the pelvis / lumbar region into the seat (35 % of the back height)
    M_back = (m_upper * G * math.cos(beta) * 0.6 * D_J + F_BRAKE * 0.35 * D_J) * DYN
    arm_b, Lb_rod = rod_arm("Haste comando encosto E", C1)
    F_back = M_back / arm_b / 2                            # two rods
    Pcr_b = math.pi ** 2 * E_STEEL * secs["20x30"]["Iw"] / Lb_rod ** 2
    c.add("Hastes comando encosto (par, F1)", "compressao (cada)", F_back, "N", Pcr_b / 3, "M = %.0f N.m, braco %.0f mm" % (M_back, arm_b * 1000))
    c.add("Travessas do carro do banco (20x30) - apoio da haste do encosto", "flexao", F_back * 0.035 * 1000 / secs["20x30"]["Ws"],
          "MPa", SIG_ADM, "suporte a 35 mm da longarina do carro")
    W_lev = 12.0 * 70.0 ** 2 / 6
    c.add("Alavancas encosto (chapa 12x70)", "flexao na raiz", F_back * arm_b * 1000 / W_lev * 0.8, "MPa", SIG_ADM,
          "raiz a ~80% do braco")
    M_pan = 2 * seg["thigh"] * ns["DRIVER_KG"] * G * 0.25 * DYN + 0.3 * F_BRAKE * 0.1
    arm_p, Lp_rod = rod_arm("Haste comando assento", C1)
    F_pan = M_pan / arm_p
    c.add("Haste comando assento (F1)", "compressao", F_pan, "N", math.pi ** 2 * E_STEEL * secs["20x30"]["Iw"] / Lp_rod ** 2 / 3, "")
    # pedal
    PED_P = GEO["PED_P"]
    pad = ns["local_to_world"](GEO["Ph1"], (1, 0, 0), GEO["F1"]["legs"]["D"]["f"], V._sub((0, 0, 0), GEO["F1"]["legs"]["D"]["s"]),
                               (0, 200, 0))
    r_pad = math.hypot(pad[1] - PED_P[1], pad[2] - PED_P[2]) / 1000.0
    M_ped = F_BRAKE * r_pad * DYN
    arm_pd, Lpd = rod_arm("Haste comando pedaleira", PED_P)
    F_ped = M_ped / arm_pd
    c.add("Haste comando pedaleira (frenagem)", "axial", F_ped, "N", math.pi ** 2 * E_STEEL * secs["20x30"]["Iw"] / Lpd ** 2 / 3,
          "M = %.0f N.m" % M_ped)
    post = [q for q in PARTS if q["desc"] == "Poste suporte haste pedaleira"][0]
    hpost = post["length"]
    # post + brace triangle (brace at atan(1/0.75) from the floor): member force <= F / sin(angle between them)
    ang = math.atan2(1.0, 0.75)
    F_tri = F_ped / math.sin(ang)
    c.add("Poste + mao-francesa da haste da pedaleira (20x30)", "axial (triangulado)", F_tri / secs["20x30"]["A"], "MPa", SIG_ADM,
          "F_haste = %.0f N" % F_ped)
    # steering
    arm_len = GEO["arm_len"] / 1000.0
    M_w = F_WHEEL * arm_len * DYN
    c.add("Bracos do volante (2 x 30x50, eixo fraco)", "flexao", M_w / 2 * 1000 / secs["30x50"]["Ww"], "MPa", SIG_ADM, "")
    F_follow = M_w / 0.140 / 2
    c.add("Pino seguidor came do volante (Ø12, chapa 6)", "esmagamento", F_follow / (12 * 6), "MPa", SIG_ADM, "")
    hq = (GEO["Q"][2] - ns["ZP"]) / 1000.0
    c.add("Colunas do mastro (2 x 30x50, sem contar a guia no anel)", "flexao no pe", F_WHEEL * DYN * hq / 2 * 1000 / secs["30x50"]["Ws"],
          "MPa", SIG_ADM, "a guia no anel reduz muito este valor")
    k_arm = 2 * 3 * E_STEEL * secs["30x50"]["Iw"] / (arm_len * 1000) ** 3 * 1000  # N/m
    m_tip = 2.25 + 0.4 * 7.85e-6 * 300 * 240 * 4 * 1000 / 1000 + 0.5
    f_arm = math.sqrt(k_arm / m_tip) / (2 * math.pi)
    # crank and rods
    c.add("Braco da manivela (chapa 12x50)", "flexao", F_DESIGN * 100 / (12 * 50 ** 2 / 6), "MPa", SIG_ADM, "")
    c.add("Haste M16 (barra roscada)", "tracao", F_DESIGN / 157.0, "MPa", SIG_ADM, "area resistente 157 mm2")
    # door bar and ring
    span = (GEO["door_f"] - GEO["door_r"]) / 1000.0
    c.add("Barra da porta (20x30x2)", "pessoa apoiada no meio do vao (%.0f N)" % F_DOOR, F_DOOR * span / 4 * 1000 / secs["20x30"]["Ws"],
          "MPa", SIG_ADM, "vao %.0f mm" % (span * 1000))
    # seat rails
    span_r = 0.350
    c.add("Trilhos do banco (30x50 deitado)", "flexao, vao %.0f mm" % (span_r * 1000), W_seat / 2 * span_r / 4 * 1000 / secs["30x50"]["Ww"],
          "MPa", SIG_ADM, "")

    # pins (double shear, bearing on 3 mm welded sleeves vs bare walls)
    pins = [("Bielas do banco (Ø16)", 16, N_link), ("Tirante de arrasto (Ø12)", 12, F_drag),
            ("Hastes do encosto (Ø16)", 16, F_back), ("Haste do assento (Ø16)", 16, F_pan),
            ("Haste da pedaleira (Ø16)", 16, F_ped), ("Eixo do volante (Ø25)", 25, M_w / 0.140),
            ("Eixo assento/encosto (Ø20)", 20, 2 * F_back + W_seat / 2)]
    pin_rows = []
    for nm, d, F in pins:
        tau = F / (2 * math.pi * d * d / 4)
        b12 = F / (2 * d * 1.2)
        b3 = F / (2 * d * 3.0)
        pin_rows.append((nm, d, F, tau, b12, b3))

    # ------------------------------------------------------------- document
    w("# Cálculos — revisão R3\n")
    w("> Gerado por `ferramentas/analisar.py` a partir do modelo paramétrico (`modelo/gerar_modelo.py`).")
    w("> Não edite os números à mão: altere as premissas no script e rode de novo.\n")
    w("Marcação de confiança: **[Certo]** sai direto da geometria/massas do modelo; **[Provável]** é cálculo de")
    w("engenharia com premissa explícita; **[Chutando]** é valor de catálogo típico, que precisa ser trocado pelo")
    w("valor do componente comprado.\n")
    w("## 1. Premissas\n")
    w("| Item | Valor | Confiança |\n|---|---|---|")
    w("| Aço dos tubos | SAE 1010/1020 conformado a frio, σe ≥ %s MPa | [Provável] |" % fmt(SY))
    w("| Tensão admissível (cargas de pico) | σe/2 = %s MPa | [Provável] |" % fmt(SIG_ADM))
    w("| Faixa admissível em solda (cargas cíclicas) | Δσ ≤ %s MPa (≈ categoria 50 da EN 1993-1-9) | [Provável] |" % fmt(DSIG_FAT))
    w("| Fator dinâmico em cargas do piloto | %s | [Provável] |" % fmt(DYN, 1))
    w("| Piloto | %s kg, 170–175 cm, frações de massa de Dempster | [Provável] |" % fmt(ns["DRIVER_KG"]))
    w("| Força no freio / no volante / na barra da porta | %s / %s / %s N | [Provável] |" % (fmt(F_BRAKE), fmt(F_WHEEL), fmt(F_DOOR)))
    w("| Rendimento do redutor | %s | [Chutando] |" % fmt(ETA_GEAR, 2))
    w("| Curvas de torque dos motores | formas típicas de catálogo (tabela no script) | [Chutando] |")
    w("| Limite de software do ângulo da manivela | ±%s° a partir da horizontal | decisão de projeto |" % fmt(CRANK_PHI_MAX))
    w("| Monitores | nenhum (óculos de RV) — não entram em massas nem estrutura | decisão do usuário |")
    w("")

    w("## 2. Massas, centro de gravidade e inércias [Certo]\n")
    w("Somadas peça a peça do modelo (tubos pelo perfil × comprimento, chapas pelo volume, comprados pela massa de catálogo,")
    w("piloto pelos segmentos). O pivô (cruzeta do cardan) fica em y = %s mm, z = %s mm.\n" % (fmt(c.pivot[1]), fmt(c.pivot[2])))
    w("| Conjunto | Massa (kg) | CG x / y / z (mm) | Altura do CG sobre o pivô (mm) | I arfagem (kg·m²) | I rolagem (kg·m²) |")
    w("|---|---|---|---|---|---|")
    for k, lab in (("F1", "Plataforma móvel + piloto — F1"), ("Caminhao", "Plataforma móvel + piloto — caminhão"),
                   ("Base", "Plataforma-base (fixa)")):
        mp = c.MP[k]
        w("| %s | %s | %s / %s / %s | %s | %s | %s |" % (lab, fmt(mp["mass"], 1), fmt(mp["cg"][0]), fmt(mp["cg"][1]), fmt(mp["cg"][2]),
                                                       fmt(mp["cg"][2] - c.pivot[2]) if k != "Base" else "-",
                                                       fmt(mp["I_pitch"], 1), fmt(mp["I_roll"], 1)))
    bal = R["balanco"]
    w("\n**Balanceamento automático.** O gerador desloca, ao longo de y, o conjunto banco + pedaleira + volante de cada")
    w("posição (sem mudar a ergonomia) até o CG longitudinal cair sobre o pivô nos dois modos. Resultado: F1 recuado")
    w("%s mm e caminhão recuado %s mm em relação às posturas da v2. Erro final: %s mm (F1) e %s mm (caminhão)." % (
        fmt(-bal["deslocamento_F1_mm"]), fmt(-bal["deslocamento_caminhao_mm"]), fmt(bal["iteracoes"][-1][2], 1),
        fmt(bal["iteracoes"][-1][3], 1)))
    w("Com o CG sobre o pivô, o torque estático dos motores com a plataforma nivelada é praticamente nulo nos dois modos.")
    w("Recuar o conjunto do caminhão **não** piora o F1: cada modo é balanceado de forma independente. [Certo]\n")
    w("O CG fica **acima** do pivô (pêndulo invertido): %s mm no F1 e %s mm no caminhão. Inclinada, a plataforma tende" % (
        fmt(f1["h"] * 1000), fmt(trk["h"] * 1000)))
    w("a cair para o lado da inclinação com rigidez negativa m·g·h = %s N·m/rad (F1) e %s N·m/rad (caminhão). As molas" % (
        fmt(f1["mgh"]), fmt(trk["mgh"])))
    w("de centragem (seção 4) compensam isso.\n")

    w("## 3. Envoltória de movimento [Certo]\n")
    w("Hastes a %s mm do pivô (braço de arfagem) e a ±%s mm do eixo (braço de rolagem); manivela de %s mm." % (
        fmt(c.L_pitch * 1000), fmt(c.L_roll * 1000), fmt(c.r * 1000)))
    w("Com o ângulo da manivela limitado a ±%s° (deslocamento máximo da haste %s mm), os limites de software ficam:\n" % (
        fmt(CRANK_PHI_MAX), fmt(lim["dmax"], 1)))
    w("| Movimento | Limite de software | Limite geométrico (manivela no ponto morto) |\n|---|---|---|")
    w("| Arfagem pura | ±%s° | ±%s° |" % (fmt(lim["arfagem_pura"], 1), fmt(lim["arfagem_geo"], 1)))
    w("| Rolagem pura — F1 | ±%s° | ±%s° |" % (fmt(lim["rolagem_pura"], 1), fmt(lim["rolagem_geo"], 1)))
    w("| Rolagem — caminhão (limite lógico) | ±6° | — |")
    w("\nArfagem e rolagem dividem o mesmo curso: |%s·sen(arfagem)| + |%s·sen(rolagem)| ≤ %s mm em cada haste." % (
        fmt(c.L_pitch * 1000), fmt(c.L_roll * 1000), fmt(lim["dmax"], 1)))
    w("Na prática, o software (SimTools/FlyPT Mover) faz essa mistura e prioriza um eixo.")
    w("A rolagem do F1 foi **ampliada**: a R2 recomendava limitar o software a ±8–10° (geométrico ±17,9°); a R3 libera")
    w("±%s° no F1, com a manivela limitada a ±60° para a força na haste não disparar. Um curso maior pediria manivela maior," % fmt(
        lim["rolagem_pura"], 1))
    w("o que aumenta o torque (proporcional ao raio) e passa do batente dianteiro na arfagem. Por isso não foi adotado.\n")
    w("Os **batentes de borracha** dianteiros tocam a longarina em ≈ −8,4° de arfagem. Isso fica além do limite de")
    w("software (−%s°): eles são proteção mecânica, não fim de curso de trabalho.\n" % fmt(lim["arfagem_pura"], 1))

    w("## 4. Molas de centragem — o \"freio\" econômico [Provável]\n")
    w("Em caso de queda de energia ou de emergência (drivers desabilitados), a plataforma deve voltar ao nível sozinha.")
    w("A solução mais barata que garante isso é um **par de molas de tração por eixo**, com olhais entre a base e o chassi:\n")
    w("- **arfagem:** duas molas na linha de centro, em y = 300 e y = 1100;")
    w("- **rolagem:** duas molas em x = ±200, na linha do pivô.\n")
    w("Molas de tração (e não de compressão) só puxam o chassi para baixo, então o cardan fica sempre comprimido. A rigidez")
    w("rotacional de cada par é %s × m·g·h do modo caminhão = **%s N·m/rad**. Assim a plataforma é estável nos dois" % (
        fmt(K_SPRING_FACTOR, 2), fmt(c.K)))
    w("modos, com os motores desligados.\n")
    w("| Par | Braço (mm) | k de cada mola (N/mm) | Curso ± (mm) | Pré-tensão: alongamento / força | Força máxima (N) | Comprimento instalado (mm) |")
    w("|---|---|---|---|---|---|---|")
    for axis, d in sp.items():
        w("| %s | %s | %s | %s | %s mm / %s N | %s | %s |" % (axis, fmt(d["lever"] * 1000), fmt(d["k"], 1), fmt(d["stroke"]),
                                                       fmt(d["e0"]), fmt(d["F0"]), fmt(d["Fmax"]), fmt(d["L"])))
    w("\nPré-carga total das molas sobre o pivô (constante, não cíclica): %s N." % fmt(c.preload))
    hold_f1 = (c.K - f1["mgh"]) * math.radians(lim["rolagem_pura"])
    hold_trk = (c.K - trk["mgh"]) * math.radians(6.0)
    w("Torque estático que os motores fazem contra as molas no fim de curso:\n")
    w("- F1, rolagem ±%s°: %s N·m na plataforma. Sem molas seriam %s N·m a favor da queda, então as molas reduzem a carga" % (
        fmt(lim["rolagem_pura"], 1), fmt(hold_f1), fmt(f1["mgh"] * math.sin(math.radians(lim["rolagem_pura"])))))
    w("  e tornam a plataforma segura.")
    w("- Caminhão, rolagem ±6°: %s N·m.\n" % fmt(hold_trk))
    w("Complementos sem custo:\n")
    w("- configurar os drivers para curto-circuitar as fases ao desabilitar, o que dá frenagem dinâmica no motor de passo;")
    w("- ligar um botão de emergência que corta o *enable* dos drivers.\n")
    w("Freio eletromagnético no motor fica como opcional, caso o protótipo mostre necessidade.\n")

    w("## 5. Motores e redução [Provável / curvas Chutando]\n")
    w("Modelo: plataforma rígida girando no cardan.\n")
    w("- Momento que o motor fornece: M = I·θ'' + (K_molas − m·g·h)·θ, mais 5 % de atrito.")
    w("- Força em cada haste: M / (2·L·cos θ).")
    w("- Torque na manivela: F·r·cos φ.")
    w("- Torque no motor: T/(N·η) + J_motor·N·φ''.")
    w("- Rotação do motor: N·φ'.\n")
    w("O sinal é senoidal, de amplitude A. O motor atende quando o torque necessário fica abaixo de 80 % da curva em todo")
    w("o ciclo. O torque do driver é limitado para que a força na haste nunca passe de **%s N**, valor usado no" % fmt(F_DESIGN))
    w("dimensionamento estrutural (seção 6). A tabela mostra a **frequência máxima (Hz)** possível para cada movimento.\n")
    hdr = "| Motor | Redução | " + " | ".join("%s ±%s°" % (lab, fmt(A, 1)) for _, _, A, lab in cues) + " |"
    w(hdr)
    w("|" + "---|" * (len(cues) + 2))
    for motor in MOTORS:
        for N in RATIOS:
            vals = table[(motor, N)]
            w("| %s | %d:1 | " % (motor, N) + " | ".join(fmt(v, 2) for v in vals) + " |")
    w("\nVelocidade de pico correspondente: 2π·f·A. Por exemplo, a rolagem F1 de ±8° a %s Hz chega a %s °/s. [Provável]\n" % (
        fmt(fm[2], 2), fmt(2 * math.pi * fm[2] * 8.0)))
    w("**Escolha:** %s com redutor planetário **%d:1** em cada lado." % (CHOSEN_MOTOR, CHOSEN_RATIO))
    w("Dentro do conjunto que mantém os motores de passo do pedido original, é o que dá o melhor equilíbrio entre torque")
    w("(arfagem/rolagem grandes e lentas) e rotação (pequenas e rápidas).\n")
    w("Limite de torque no driver: **%s N·m no motor**, o que corresponde a %s N na haste com a manivela a %s°." % (
        fmt(min(cap[CHOSEN_RATIO], MOTORS[CHOSEN_MOTOR][0][1]), 1), fmt(F_DESIGN), fmt(CRANK_PHI_MAX)))
    w("Se o protótipo pedir respostas mais rápidas, o caminho é o servo CA de 750 W com a mesma redução (linhas da tabela).")
    w("A troca não muda a estrutura, que já está dimensionada para a força limitada.\n")
    w("O modo caminhão é mais pesado em inércia (I rolagem %s contra %s kg·m² no F1, pelo CG mais alto), mas pede" % (
        fmt(trk["Ir"], 0), fmt(f1["Ir"], 0)))
    w("movimentos menores e mais lentos. A rolagem limitada a ±6° pelo software é coerente com isso. [Provável]\n")

    w("## 6. Esforços e tensões [Provável]\n")
    w("Duas famílias de carga:\n")
    w("- **Pico:** força máxima da haste (%s N, limitada no driver), peso × %s, pré-carga das molas e cargas do piloto." % (
        fmt(F_DESIGN), fmt(DYN, 1)))
    w("  Comparada com σe/2.")
    w("- **Cíclica:** força nas hastes no sinal representativo (F1, rolagem ±8° mais arfagem ±4°, a 80 % da frequência")
    w("  máxima) = %s N por haste. A faixa 2×F é comparada com a admissível em solda.\n" % fmt(F_CYC))
    w("| Elemento | Caso | Valor | Unid. | Limite | Situação | Observação |\n|---|---|---|---|---|---|---|")
    for m_, cs, v, u, l, st, nt in c.rows:
        w("| %s | %s | %s | %s | %s | %s | %s |" % (m_, cs, fmt(v, 0 if v > 100 else 1), u, fmt(l, 0), st, nt))
    w("\nA linha \"longarina sem treliça\" **não atende de propósito**. Ela mostra por que a v3 ganhou a **treliça de")
    w("soleira** sob a longarina esquerda: a abertura da porta interrompe ali a treliça do anel. A força da haste traseira")
    w("precisa chegar às travessas do pivô por uma estrutura alta, e não pela longarina sozinha em flexão. Na direita,")
    w("essa função é do próprio anel treliçado. [Provável]\n")
    w("### 6.1 Pinos e furos [Provável]\n")
    w("Pino em corte duplo. Esmagamento comparado entre a parede do tubo (1,2 mm) e a bucha soldada (parede 3 mm).\n")
    w("| Articulação | Ø pino (mm) | Força (N) | τ no pino (MPa) | Esmagamento em parede 1,2 mm (MPa) | Esmagamento em bucha 3 mm (MPa) |")
    w("|---|---|---|---|---|---|")
    for nm, d, F, tau, b12, b3 in pin_rows:
        w("| %s | %s | %s | %s | %s | %s |" % (nm, fmt(d), fmt(F), fmt(tau, 1), fmt(b12), fmt(b3)))
    w("\n**Regra de fabricação:** nenhum pino trabalha direto na parede do tubo. Todo furo de articulação leva **bucha de")
    w("aço soldada** (parede ≥ 3 mm, folga H8/f7) ou orelha de chapa ≥ 5 mm. Na parede de 1,2 mm o esmagamento é")
    w("aceitável no pico, mas ovaliza com o uso cíclico e gera folga, que se sente no volante e no banco. [Provável]\n")
    w("### 6.2 Frequência natural do braço do volante [Provável]\n")
    w("Dois braços 30x50 em balanço de %s mm (eixo fraco), com volante G920 + mesa (≈ %s kg) na ponta: f ≈ **%s Hz**." % (
        fmt(arm_len * 1000), fmt(m_tip, 1), fmt(f_arm)))
    w("Isso fica bem acima das frequências de movimento (< 3 Hz). Interação com o force feedback do G920 é improvável.\n")
    w("## 7. Verificação geométrica automática [Certo]\n")
    w("`ferramentas/verificar.py` representa cada peça por uma caixa orientada e procura interferências:\n")
    w("- entre peças de componentes diferentes, em 5 pontos da transição F1 → caminhão;")
    w("- com a barra da porta aberta e a meio curso, nos dois modos;")
    w("- entre o manequim e a estrutura, nos dois modos;")
    w("- entre a plataforma móvel e a base, em todas as %s poses de arfagem × rolagem alcançáveis pela manivela.\n" % "340")
    w("Contatos previstos (pinos, articulações e carros sobre trilhos) são filtrados por pontos de junta.")
    rep = V.run_all(ns)
    n_int = sum(len(v) for k, v in rep.items() if k.startswith(("transicao", "porta", "manequim")))
    w("Resultado: **%d interferências** na transição, porta e manequim." % n_int)
    env0 = rep["envoltoria"][0.0][0]
    pos = [r for r in env0 if r[0] > 0]
    neg = [r for r in env0 if r[0] <= 0]
    w("Na envoltória de movimento, os únicos contatos são os batentes dianteiros (%s), como previsto. A menor folga" % (
        "; ".join("%s × %s, %s mm a %s°" % (r[2], r[4], fmt(r[0], 1), fmt(r[5])) for r in pos[:1]) if pos else "nenhum"))
    w("restante é de %s mm (%s × %s).\n" % (fmt(-neg[0][0], 1), neg[0][2], neg[0][4]) if neg else "")
    w("Mecanismos de 1 movimento (síntese e simulação cinemática real, não interpolação):\n")
    w("| Haste | Comprimento (mm) | Menor ângulo de transmissão no curso | Erro de comprimento F1/caminhão (mm) | Desvio da animação DC (mm) |")
    w("|---|---|---|---|---|")
    for k, d in rods.items():
        w("| %s | %s | %s° | %s | %s |" % (k, fmt(d["length"]), fmt(d["transmissao_min_graus"]), fmt(d["err"], 2),
                                          fmt(d["desvio_animacao_mm"], 1)))
    tmin = min(d["transmissao_min_graus"] for d in rods.values())
    w("\nCritério de projeto: ângulo de transmissão ≥ 25° em todo o curso. O menor valor encontrado é %s°, então nenhuma" % fmt(tmin))
    w("haste passa perto de ponto morto. A lei de")
    w("movimento de cada componente dinâmico (deslizamento e giro em função de `modo`) é um polinômio ajustado à")
    w("cinemática real. Por isso a animação no SketchUp mostra as hastes sem esticar.\n")

    with open(os.path.join(DOCS, "CALCULOS.md"), "w") as f:
        f.write("\n".join(out) + "\n")

    # ================================================= BOM
    bom = [p for p in PARTS if p["bom"]]
    asm_names = {"B": "Plataforma-base", "M": "Chassi móvel", "A": "Anel estrutural", "PO": "Barra da porta", "BC": "Banco",
                 "PE": "Pedaleira", "VO": "Volante", "MO": "Molas de centragem"}
    kind_names = {"T": "Tubo", "CH": "Chapa/peça usinada", "PI": "Pino", "CO": "Comprado", "ES": "Estofado"}

    def dims(p):
        g = p["geom"]
        if g[0] == "box":
            e = g[5]
            d = sorted([abs(e[3] - e[0]), abs(e[4] - e[1]), abs(e[5] - e[2])])
            return "%s x %s x %s" % (fmt(d[2]), fmt(d[1]), fmt(d[0]))
        if g[0] == "cyl":
            return "Ø%s x %s" % (fmt(2 * g[3]), fmt(p["length"]))
        if g[0] == "tube":
            return "%s x %s" % (p["prof"], fmt(math.ceil(p["length"])))
        return ""

    lines = []
    L = lines.append
    L("# Lista de materiais — revisão R3\n")
    L("> Gerada por `ferramentas/analisar.py` a partir do modelo. Cada código abaixo é o nome da peça no SketchUp")
    L("> (janela *Outliner*: `<código> <descrição>`), então qualquer peça do desenho é localizada pelo código.\n")
    L("**Códigos:** `<conjunto>-<tipo><nº>`.\n")
    L("- Conjuntos: B base, M chassi móvel, A anel, PO barra da porta, BC banco, PE pedaleira, VO volante, MO molas.")
    L("- Tipos: T tubo, CH chapa/peça, PI pino, CO comprado, ES estofado.\n")
    tot = {}
    for p in bom:
        tot[p["kind"]] = tot.get(p["kind"], 0) + p["mass"]
    L("## Resumo\n")
    L("| Tipo | Itens | Massa (kg) |\n|---|---|---|")
    for k in ("T", "CH", "PI", "CO", "ES"):
        L("| %s | %d | %s |" % (kind_names[k], sum(1 for p in bom if p["kind"] == k), fmt(tot.get(k, 0), 1)))
    L("| **Total (sem piloto)** | %d | %s |\n" % (len(bom), fmt(sum(tot.values()), 1)))
    tl = {}
    for p in bom:
        if p["kind"] == "T":
            tl[p["prof"]] = tl.get(p["prof"], 0) + p["length"]
    L("| Perfil | Comprimento total (m) | kg/m | Massa (kg) |\n|---|---|---|---|")
    for prof in sorted(tl):
        w_, h_, t_ = c.P[prof]
        L("| Tubo %s x %s x %s mm | %s | %s | %s |" % (fmt(w_), fmt(h_), fmt(t_, 1), fmt(tl[prof] / 1000, 2),
                                                    fmt(ns["prof_kgm"](prof), 3), fmt(ns["prof_kgm"](prof) * tl[prof] / 1000, 1)))
    L("")
    for k in ("T", "CH", "PI", "ES"):
        L("## %s\n" % {"T": "Tubos (segmentos)", "CH": "Chapas e peças usinadas", "PI": "Pinos e eixos", "ES": "Estofados"}[k])
        L("| Código | Conjunto | Descrição | Dimensões (mm) | Corte | Massa (kg) |\n|---|---|---|---|---|---|")
        for p in bom:
            if p["kind"] != k:
                continue
            cut = ""
            if k == "T":
                a, b = p["line"]
                d = V._sub(b, a)
                ln = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
                ang = [math.degrees(math.acos(min(1.0, abs(c) / ln))) for c in d]
                cut = ("ângulo (eixo a %s° / %s° / %s° de X / Y / Z)" % tuple(fmt(a_) for a_ in ang)
                       if p["cut"] == "angulo" else "reto")
            L("| %s | %s | %s | %s | %s | %s |" % (p["id"], asm_names.get(p["asm"], p["asm"]), p["desc"], dims(p), cut, fmt(p["mass"], 2)))
        L("")
    L("## Itens comprados (agrupados)\n")
    L("| Especificação | Qtd. | Códigos no desenho |\n|---|---|---|")
    groups = {}
    for p in bom:
        if p["kind"] == "CO":
            key = p["spec"] or p["desc"]
            groups.setdefault(key, []).append(p["id"])
    for p in bom:
        if p["kind"] == "PI" and p["spec"] and "ball lock" in p["spec"]:
            groups.setdefault(p["spec"], []).append(p["id"])
    for spec, ids in groups.items():
        qty = len(ids)
        if spec.startswith("Junta cardan"):
            qty = 1
        if spec.startswith("Logitech"):
            qty = "1 (já possui)"
        L("| %s | %s | %s |" % (spec, qty, ", ".join(ids)))
    L("")
    L("## Itens não desenhados (necessários)\n")
    L("| Item | Qtd. | Observação |\n|---|---|---|")
    extras = [
        ("Driver para motor de passo em malha fechada (ex.: HBS86H)", "2", "compatível com o motor escolhido em CALCULOS.md"),
        ("Fonte 70–80 VCC, ≥ 600 W", "1", "dimensionar pelo driver"),
        ("Controladora USB (Arduino + firmware de simulador, ou placa dedicada)", "1", "interface com SimTools / FlyPT Mover / SimHub"),
        ("Botão de emergência (corta o enable dos drivers)", "1", "segurança, ver CALCULOS.md §4"),
        ("Sensores de fim de curso / homing das manivelas", "2", "indutivos ou micro-switch"),
        ("Buchas de aço para articulações (parede ≥ 3 mm)", "2 por pino", "ver CALCULOS.md §6.1"),
        ("Anéis elásticos / travas para pinos", "2 por pino", ""),
        ("Parafusos, porcas e arruelas M6–M16 (kit)", "1", "fixação de chapas, G920, câmbio e estofados"),
        ("Compensado naval 15 mm para bases dos estofados", "≈ 0,6 m²", "assento, encosto e apoio"),
        ("Arame/eletrodo de solda, discos de corte e lixa", "-", "consumíveis"),
        ("Fundo anticorrosivo + tinta", "≈ 1,5 L", "área ≈ %s m² de tubo" % fmt(sum(2 * (c.P[p['prof']][0] + c.P[p['prof']][1]) * p["length"]
                                                                                   for p in bom if p["kind"] == "T") / 1e6, 1)),
        ("Pés niveladores com borracha", "6", "sob a base"),
    ]
    for it in extras:
        L("| %s | %s | %s |" % it)
    with open(os.path.join(DOCS, "LISTA_DE_MATERIAIS.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(DOCS, "lista_de_materiais.csv"), "w") as f:
        f.write("codigo;conjunto;tipo;descricao;perfil;dimensoes_mm;comprimento_corte_mm;corte;massa_kg;especificacao\n")
        for p in bom:
            f.write("%s;%s;%s;%s;%s;%s;%s;%s;%s;%s\n" % (
                p["id"], asm_names.get(p["asm"], p["asm"]), kind_names[p["kind"]], p["desc"], p["prof"] or "", dims(p),
                math.ceil(p["length"]) if p["kind"] == "T" else "", p["cut"] or ("reto" if p["kind"] == "T" else ""),
                ("%.3f" % p["mass"]).replace(".", ","), p["spec"] or ""))

    # ================================================= cut plan
    tubes = [p for p in bom if p["kind"] == "T"]
    plans = cut_plan(tubes)
    by_id = {p["id"]: p for p in tubes}
    lines = []
    L = lines.append
    L("# Plano de corte dos tubos — revisão R3\n")
    L("> Gerado por `ferramentas/analisar.py`. Barras comerciais de %s m, perda de serra de %s mm por corte e %s mm de" % (
        fmt(STOCK / 1000), fmt(KERF), fmt(TRIM)))
    L("> esquadrejamento em cada ponta da barra. Distribuição pelo método *first-fit decreasing*.\n")
    L("**Como ler:** cada linha é uma barra. Corte as peças na ordem listada e marque cada peça com o código, que é o")
    L("mesmo do desenho e da lista de materiais.\n")
    L("Os comprimentos são os do eixo da peça no modelo, de face a face das peças que ela encontra. Para peças com corte")
    L("em ângulo (diagonais, pernas e bielas), o comprimento é medido no eixo e o ângulo está na lista de materiais.")
    L("Recomenda-se cortar com +2 mm e ajustar na montagem (gabarito).\n")
    for prof, bars in plans.items():
        used = sum(b["used"] for b in bars)
        need = sum(math.ceil(by_id[i]["length"]) for b in bars for i, _ in b["pieces"])
        L("## Tubo %s\n" % prof)
        L("Barras de 6 m: **%d** · peças: %d · comprimento líquido: %s m · aproveitamento: %s %%\n" % (
            len(bars), sum(len(b["pieces"]) for b in bars), fmt(need / 1000, 2), fmt(100 * need / (len(bars) * STOCK))))
        L("| Barra | Peças (código: comprimento mm) | Usado (mm) | Sobra (mm) |\n|---|---|---|---|")
        for i, b in enumerate(bars):
            L("| %d | %s | %s | %s |" % (i + 1, ", ".join("%s: %s" % (pid, fmt(l)) for pid, l in b["pieces"]), fmt(b["used"]),
                                       fmt(STOCK - 2 * TRIM - b["used"])))
        L("")
    L("## Compra de tubos\n")
    L("| Perfil | Barras de 6 m | Observação |\n|---|---|---|")
    for prof, bars in plans.items():
        L("| %s | %d (+1 de reserva recomendada) | sobras ≥ 300 mm servem para gabaritos e calços |" % (prof, len(bars)))
    with open(os.path.join(DOCS, "PLANO_DE_CORTE.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(DOCS, "plano_de_corte.csv"), "w") as f:
        f.write("perfil;barra;ordem;codigo;descricao;comprimento_mm\n")
        for prof, bars in plans.items():
            for i, b in enumerate(bars):
                for j, (pid, l) in enumerate(b["pieces"]):
                    f.write("%s;%d;%d;%s;%s;%d\n" % (prof, i + 1, j + 1, pid, by_id[pid]["desc"], l))

    summary = dict(stress=[r for r in c.rows if r[5] != "OK"], fmax=table[(CHOSEN_MOTOR, CHOSEN_RATIO)], F_cyc=F_CYC,
                   bars={k: len(v) for k, v in plans.items()}, springs=sp, interferences=n_int, mass=tot, limits=lim)
    return summary


if __name__ == "__main__":
    s = main()
    for k, v in s.items():
        print(k, v)
