# Design Report v2 — Motion Simulator Cockpit with a 2-Degree-of-Freedom Platform

> **Language note.** The ASD-STE100 standard (Simplified Technical English) exists only in English. For this
> reason, this version is in English. The Portuguese report `MEMORIAL-DESCRITIVO.md` (R3-leve) did not change.
> This version follows ASD-STE100 for about 80 % of the text: short sentences, active voice, one idea in each
> sentence, and the same word for the same thing. Some technical names and tables do not follow the standard.

| Item | Description |
|---|---|
| Project | Cockpit for racing games. It changes between **Formula 1 (F1)** and **truck** driving positions |
| Target user | Height 170 – 175 cm. Mass 90 kg (design assumption) |
| Peripherals | Logitech **G920** (wheel and pedals), **Driving Force Shifter**, and a **VR headset** (no monitors) |
| Motion | 2 degrees of freedom (pitch and roll). Two NEMA 34 stepper motors with reduction gears |
| Structure | Steel rectangular tube, wall 1.2 mm: **30×50** (highest loads), **20×30** (general structure), **10×20** (light bracing) |
| Revision | **R3-leve-J**. The design is checked. It is ready for an independent review |

## Revision history

| Rev. | Change |
|---|---|
| R0 | Concept model. The two configurations overlap. They use different parts |
| R1 | One set of parts. Seat, pedal deck, and wheel arm move on rails and hinges. G920 peripherals |
| R2 | Level seat sub-frame. Tilting seat. Two-part backrest. Shoulder and head rest hinged at the collarbone |
| R3 | Structural ring. Door bar. Sill truss. One-movement conversion. Automatic balance of the center of gravity (CG). Part identification |
| R3-leve | Mass cut from 179 kg to 136 kg. Moving mass cut from 130 kg to 98 kg |
| **R3-leve-J** | **Tube joints are now correct** (section 7). Report v2 in simplified English. STEP files and a macro for SolidWorks (section 14) |

### What changed in R3-leve-J

- **Joints.** Before R3-leve-J, the drawing cut each tube square at the end of its center line. This caused
  three faults: corner caps that stick out, V joints with no mitre, and tubes that cross each other.
- **Now.** The generator cuts each tube end to fit the tube that it meets (section 7).
- **Check.** A solid model check finds 0 overlaps between tubes of the same welded assembly.
- **Layout changes** that this work needed:
  - The sill truss has two uprights and one diagonal. It had four uprights and three diagonals.
  - The ring diagonals and the end-frame diagonals now end on the side faces of the columns.
  - The floor diagonals now end on the cross tubes, not on the rails.
  - The base has one more cross tube (y = 650) and two more motor-plate rails.
  - The pedal-post brace now stands on a cross tube of the chassis.

## 1. Purpose and status

The cockpit takes the F1 position and the truck position by hinges, rails, and pins only. The platform moves in
pitch and roll.

> **Status.** Calculations and an automatic geometry check verify the design (section 12). No third party has
> reviewed it. No prototype exists. Catalog values marked **[Chutando]** in `CALCULOS.md` are estimates. Replace
> them with the data of the parts that you buy.

## 2. Project documents

| Document | Content |
|---|---|
| `docs/MEMORIAL-DESCRITIVO.md` | Portuguese report (R3-leve). Not changed |
| `docs/MEMORIAL-DESCRITIVO-v2.md` | This report |
| `docs/CALCULOS.md` | Masses, CG, inertia, envelope, springs, motors, stresses, pins, frequencies |
| `docs/LISTA_DE_MATERIAIS.md` (+ CSV) | All parts with code, size, cut, and mass |
| `docs/PLANO_DE_CORTE.md` (+ CSV) | Tube parts on 6 m bars |
| `solidworks/` | STEP files, part list, positions, joint list, and a SolidWorks macro (section 14) |
| `modelo/gerar_modelo.py` | Generator. It is the only source of geometry and numbers |
| `ferramentas/verificar.py` | Interference and motion-envelope checks |
| `ferramentas/cad.py` | Solid model of the tubes and the joint check |
| `ferramentas/exportar_cad.py` | Writes the STEP files |
| `ferramentas/analisar.py` | Calculations, bill of materials, cut plan, and the `PRESET` block |

Tools write the numbers in the documents. Nobody types them by hand.

## 3. Coordinates, conventions, and part codes

- Unit: millimeter. **X** is lateral (the right side of the driver is +X). **Y** is longitudinal (the driver
  looks toward +Y. The rear of the moving platform is at Y = 0). **Z** is vertical (floor at Z = 0).
- The top of the main rails of the moving chassis is at **ZP = 380 mm**.
- Angles in the side view start at +Y and turn toward +Z. "E" is the left side of the driver. "D" is the right side.
- **Part codes:** `<assembly>-<type><number>`, for example `M-T01`, `BC-PI11`, or `VO-CO05`.
  - Assemblies: **B** base, **M** moving chassis, **A** ring, **PO** door bar, **BC** seat, **PE** pedals,
    **VO** wheel, **MO** springs.
  - Types: **T** tube, **CH** plate or machined part, **PI** pin, **CO** bought part, **ES** upholstery.
- The code is the part name in SketchUp, in the bill of materials, and in the cut plan.

## 4. Assumptions

| Body segment | Length (mm) |
|---|---|
| Thigh / lower leg | 420 / 420 |
| Torso (hip to shoulders) | 520 |
| Upper arm / forearm and hand | 300 / 310 |
| Half width of hips / shoulders | 90 / 195 |

- The resulting height is about **1730 mm**. The driver mass is **90 kg**.
- The VR headset replaces the monitors. The model has no monitor and no monitor support.
- The G920 outer sizes come from the maker. The inner shapes are estimates.

## 5. General description

| Dimension | Value |
|---|---|
| Base | 720 × 2050 mm (Y −200 to 1850) |
| Moving chassis | 570 × 1780 mm |
| Ring | ±295 mm, z = 930 mm, four columns on the right and four on the left |
| Head top | 1297 mm (F1) / 1825 mm (truck) |
| Structure mass, no driver | **135.5 kg**: about 96 kg moving and 39.5 kg base |
| Moving mass with driver | 186 kg |

## 6. Base (fixed platform)

- **Frame:** two 20×30 rails at X = ±330. Nine cross tubes. One 10×20 diagonal.
- **Pivot tower:** four sloped 20×30 legs and a top plate 160×160×8. A bought **universal joint** has its center at
  Y = 700, Z = 268. The roll axis (Y) is in the lower fork. The pitch axis (X) is in the upper fork.
- **Actuators** (one on each side): a closed-loop NEMA 34 motor, a 20:1 planetary gearbox, and a **10 mm steel
  plate crank** (radius 100 mm). Rod ends and a threaded rod M16 connect the crank to the platform at Y = 60.
- **Motor plate rails.** One 20×30 rail at X = ±292 carries each motor plate.
- **Rubber stops** at Y = 1700 limit the motion if the software fails.
- **Centering springs:** four extension springs (section 11).

## 7. Tube joints

This section is new in R3-leve-J.

### 7.1 Rule

All tubes of one moving group are one rigid welded assembly. Each tube end gets one of three cuts:

| Cut | When | Result |
|---|---|---|
| **Mitre** | Two tubes of the same section meet at a corner or a V | Both tubes are cut on the bisector plane. The faces fit |
| **Butt** | A tube ends on a larger or longer tube | The tube is cut on the plane of the host face |
| **Square** | The end is free, or it touches a face flat | No cut |

When a tube meets two tubes, the generator tries a mitre first. If a mitre is not possible, the tube butts onto the
tube that has the larger section. The generator reports each end that it cannot resolve.

### 7.2 Layout rules that make the rule work

- Each tube end must lie on a node or on a face of the host tube.
- An upright or a diagonal must not stand on a V node. It stands on one flat face, past the mitre.
- A diagonal ends on the side face of an upright, with a margin that clears the chord.
- A rail ends flush with the face of the cross tube that it meets.
- Frame diagonals stay above the chassis rails.

### 7.3 Check

`ferramentas/cad.py` builds a solid for each tube and finds overlaps. The result:

| Check | Result |
|---|---|
| Overlap between tubes of the same welded assembly | **0** |
| Tube ends with no contact inside the welded chassis and base | 0 welded joints. The only free end is the top of the pedal post, which carries a pin |
| Tube ends with no contact in the moving groups (links, rods, posts, mast columns) | 25. Each end carries a pin or a bearing. These are not welds |

The tube solid has a hollow section and rounded outer corners (radius 1.5 × wall). The SketchUp drawing shows each
tube as a solid shape with the same mitre and butt cuts. The STEP files have the true hollow tubes.

## 8. Moving platform — structure

### 8.1 Chassis

- **Main rails:** 30×50 at X = ±270, Y 10 to 1780.
- **Cross tubes:** 30×50 at Y = 25. Three 30×50 cross tubes at the pivot (Y = 650, 700, 750). A pivot plate
  300×140×8 sits between them and the upper fork. The other cross tubes are 20×30. The floor diagonals are 10×20.
- **Seat and pedal rails:** 30×50, laid flat.
- **Sill truss (left side).** The door opening interrupts the ring truss on the left side. Without a truss, the rear
  rod force reaches the pivot only through the rail in bending. This gives about 450 MPa at the maximum force.
  - Top chord: the left main rail. Bottom chord: 30×50 in three straight pieces.
  - The end nodes of the bottom chord touch the rail. The inner nodes are 130 mm and 205 mm below the rail axis.
    The truss is deepest at the pivot, where the moment is highest.
  - Two 20×30 uprights stand on the bottom chord. One 20×30 diagonal closes the quadrilateral bay.

### 8.2 Structural ring

- **Position:** z = 930 mm. It is below the driver shoulders in F1.
- **Sides:** four columns on each side. Diagonals only where the ring carries the rod moment to the pivot, and in the
  end bays that guide the mast.
- **Rear frame:** a cross tube at z = 760 mm and a diagonal. The cross tube is lower than the ring. The F1 head rest
  extends behind the rear of the chassis. A cross tube at ring height would hit it.
- **Front frame:** a cross tube and a diagonal.
- **Mast:** the mast columns slide along the ring between two guide plates.

### 8.3 Door bar

- **Construction:** one 30×50 tube. Length about 930 mm. The hinge is at Y = 200. The latch is at Y ≈ 1130.
- **Opening:** the bar turns **85° upward** about a transverse axis. It does not take lateral space.
- **Latch:** a fork with two ears and a support. A 10 mm ball-lock pin goes through the three parts.
- **Animation:** the dynamic component has the attribute `aberta`. A click runs `ANIMATE("aberta",0,1)`.
- **Strength:** the bar carries a 600 N person at mid-span (54 MPa). It is not a step.

### 8.4 Shifter

The shifter is on the right side of the ring. It sits on a 190×140×6 plate with a bracket at Y ≈ 700. In F1, the
driver uses the paddles of the G920. The shifter does not touch the arms.

## 9. One-movement conversion

Each module (seat, pedals, wheel) has one degree of freedom. When the carriage slides, rods or a cam move all other
parts of the module into the correct position. Quick-release pins lock each module on its rails. The driver does
not set any angle by hand.

This is **option A**. It is ready for **option B**: one actuator that moves the three carriages. The carriage strokes
have a fixed ratio: seat +58.5 mm, pedals −366 mm, mast +158 mm.

### 9.1 Seat

| Element | Function |
|---|---|
| Seat carriage | Slides on the 30×50 rails (stroke +58.5 mm) |
| Parallelogram | Four links of 360 mm. They turn from 2.1° (F1) to 66.0° (truck). The sub-frame stays at 9° |
| Drag links (pair, 77 mm) | Join the chassis to the rear link. The carriage stroke sets the link angle |
| Seat rod (291 mm) | Turns the seat from 25° (F1) to 9° (truck) |
| Backrest rods (pair, 275 mm) | Turn the backrest from 35° to 75° |
| Parallel link of the upper rest (pair, 10×20) | Keeps the shoulder and head rest at **75°** in all positions |
| Rubber pads in F1 (four) | In F1 the links are almost flat. The pads carry the weight, not the links |
| Steel spines | A 20×30 tube under each 15 mm plywood base carries the axes and levers |

### 9.2 Pedals

- The carriage slides on the 30×50 rails. The stroke is −366 mm.
- The pedal deck turns −35° about an axis on the carriage.
- A 207 mm rod connects a lever on the deck torsion tube to a braced post on the chassis.

### 9.3 Wheel

- Two 30×50 mast columns slide on the chassis (stroke +158 mm). The ring guides them.
- The wheel arm turns −35° (rim plane from 80° to 45°).
- A follower pin on each side runs in a curved slot in a plate. The slot has the exact path of the pin.

### 9.4 Synthesis

A search that simulates the real kinematics finds the fixed points of the rods. The criteria are the longest
effective arm and a transmission angle of at least 25° over the full stroke. The minimum transmission angle is 28°.
The position error at the end poses is 0.0 mm.

## 10. Postures and balance

| Parameter | F1 | Truck |
|---|---|---|
| Backrest (from horizontal) | 35° | 75° |
| Seat | 25° | 9° |
| Upper rest | 75° | 75° |
| Thigh / knee / ankle | +25° / 145° / 100° | +5° / 120° / 90° |
| Wheel plane | 80° | 45° |
| Shift for balance | 17 mm | 171 mm |

The generator moves each group along Y until the CG is over the pivot. The final error is less than 1 mm in both modes.

## 11. Dynamic components and animation

| Attribute | Where | Effect |
|---|---|---|
| `modo` (0 = F1, 1 = truck) | `Plataforma_Movel` | A click runs `ANIMATE("modo",0,1)`. All children read it |
| `sentido` | `Plataforma_Movel` | Reverses the rotations if SketchUp turns them the wrong way |
| `aberta` | `Porta_Barra` | A click runs `ANIMATE("aberta",0,1)` |

- Each sliding or turning component follows a cubic law in `modo`. The law fits the real kinematics.
  The rods keep their length within 2.3 mm.
- The F1 mannequin shows only at `modo = 0`. The truck mannequin shows only at `modo = 1`.
- The `aberta` terms and the cubic laws are **not yet tested** in the desktop version of SketchUp.

## 12. Motion, motors, and springs

| Item | Value |
|---|---|
| Moving mass with driver | 186.0 kg. CG 469 mm (F1) and 681 mm (truck) above the pivot |
| Software limits | Pitch ±7.8°. F1 roll ±16.2°. Truck roll ±6° |
| Motor and reduction | NEMA 34 closed loop, 12 N·m, 20:1. The driver limits the torque to 5.0 N·m |
| Estimated performance | F1 roll ±16° up to about 1.1 Hz. ±8° up to about 1.4 Hz |
| Design force at the rod | 1.8 kN. The driver limits it. The cyclic need is 1.1 kN |
| Centering springs | Pitch pair 4.3 N/mm. Roll pair 17.1 N/mm. Stiffness 1.1 × m·g·h of the truck |

**Low-cost brake:**

- Extension springs keep the platform stable with the motors off. They only pull down, so the joint never has tension.
- The motor drivers use dynamic braking.
- An emergency button cuts the enable signal.
- Rubber stops limit the stroke.
- An electromagnetic brake is optional. Decide after the prototype tests.

## 13. Checks

`ferramentas/verificar.py` and `ferramentas/cad.py`:

| Check | Result |
|---|---|
| Interference between parts of different groups, at five points of the transition | 0 |
| Door bar open and half open, in both modes | 0 |
| Mannequin and structure, in both modes | 0 (hands on the rim are not counted) |
| Platform and base, 340 reachable poses | One contact: the front stops at about −8.4° of pitch (planned). Smallest other clearance: about 14 mm |
| Overlap between tubes of one welded assembly | 0 |
| Stress, buckling, fatigue, pins | All pass. See `CALCULOS.md`. The only exception is the reference case "rail without truss", which fails on purpose |
| Mass of hollow tubes in the STEP files against the generator | −1.9 % (the round corners explain the difference) |

## 14. SolidWorks files

> **Limit.** Native `.SLDPRT` and `.SLDASM` files are a closed binary format. Only SolidWorks can write them. This
> project cannot write them. The folder `solidworks/` has STEP files and a macro. The macro makes the native files.

| File | Content |
|---|---|
| `solidworks/pecas/*.step` | One file for each different part: 73 files |
| `solidworks/Cockpit_2DOF_F1.step` | The assembly in the F1 pose. Sub-assemblies follow the moving groups |
| `solidworks/manifesto_pecas.csv` | Code, file, name, quantity, instances, mass |
| `solidworks/posicoes.csv` | Position matrix of each instance |
| `solidworks/juntas.csv` | Joint, parent, axis, and stroke of each moving group |
| `solidworks/converter_para_solidworks.bas` | Macro. STEP to `.SLDPRT` and `.SLDASM` |
| `solidworks/LEIA-ME.md` | How to use these files (Portuguese) |

**Rules for the parts:**

- Welded tubes and the plates welded to them are **one part**. The generator fuses them into one solid.
- Each tube is a **hollow tube**. The wall is 1.2 mm. The outer corner radius is 1.8 mm.
- Bought items that belong together are one part (for example, the G920 pedals).
- Identical parts share one file. The quantity is in the manifest.
- The macro was not run in SolidWorks. Test it on one file first.

## 15. Open points (check before you build)

1. **Motor curves and gearbox efficiency** are typical catalog values. Use the data sheet of the part you buy.
2. **Universal joint.** Specify the cross by the design load (about 11 kN at peak). Static capacity must be 20 kN or more.
3. **Springs.** Confirm 17 N/mm (roll pair) with about 120 mm of extension with the maker.
4. **Welds.** The fatigue check uses a stress range of 50 MPa or less. Use continuous welds, with no undercut, at the
   truss nodes and at the pivot cross tubes.
5. **Bushings** are necessary in all hinge holes. A 1.2 mm tube does not last with a bare pin.
6. **Animation.** Test `modo` and `aberta` in desktop SketchUp.
7. **Cut lengths.** The plan gives the length of the longest edge of each tube. Cut 2 mm longer and fit on a jig. Check
   each mitre angle during assembly.
8. **Upholstery.** Thickness and side wings are proposals. Test the seat and the upper rest with the real user.
9. **Option B** (one actuator) needs links between the three carriages. The strokes are fixed.
10. **Plates with no welded support.** In the model, these plates do not touch a tube. Add a bracket or a bolt
    pattern for each one:
    - the two wheel-cam plates (4 mm) on the ring;
    - the seat-lever plates and the seat-rod ear (bolted to the plywood base);
    - the head-rest link ears.
11. **Plates inside tubes.** The ears of the sub-frame links sit inside the hollow section of a tube. Slot the tube
    and weld the ear, or move the ear to the outer face.
12. **SolidWorks mates** are not in the STEP file. Add them from `juntas.csv`.
13. **The SketchUp model must be generated again** to include the three base tubes that were added after the last
    export. The generator is correct.

## 16. 3D model layout

```
Plataforma_Base (group)            frame, tower, joint, motors, cranks, springs, stops
Plataforma_Movel (DC: modo, sentido)
├─ chassis, ring, sill, shifter, cam plates, F1 pads
├─ Carro_Banco → Biela_Banco_* → Banco → Assento / Encosto → Encosto_Superior
│                                      → Tirante_Apoio_E/D
│  Haste_Encosto_E/D, Haste_Assento, Tirante_Carro_E/D
├─ Carro_Pedaleira → Pedaleira_Basculante ; Haste_Pedaleira
├─ Mastro_Volante → Braco_Volante
├─ Porta_Barra (DC: aberta)
└─ Manequim_F1 / Manequim_Caminhao
```

## 17. How to make the files again

1. On your computer, run these commands:
   - `python3 ferramentas/verificar.py` (interference checks);
   - `python3 ferramentas/cad.py` (tube joint check);
   - `python3 ferramentas/analisar.py` (documents and the `PRESET` block);
   - `python3 ferramentas/exportar_cad.py` (STEP files).
2. In SketchUp, send `modelo/gerar_modelo.py` as the `code` of `build_model` (Trimble SketchUp MCP, `clean: true`).
   Then call `save_model`.
3. The SketchUp connector stops scripts after 30 s. It has no `any()` or `all()`. The `PRESET` block lets the
   generator skip the searches.
4. In SolidWorks, run `solidworks/converter_para_solidworks.bas`.
