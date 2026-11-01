# Lecture 33: Sheet metal and plastic parts

<div class="goals">

**After this lecture you can**

- derive the bend allowance from the neutral surface, and compute a flat length from flange lengths by hand;
- explain the K-factor, what it models and what it leaves out, and exactly how Geop uses it;
- describe why Geop keeps a sheet-metal part as flats joined by bends rather than as a general solid, and why that makes its flat pattern exact;
- describe what ribs, lips, grooves and drafts do to a solid, and which earlier machinery each reuses;
- say what each of these operations refuses, and why a refusal is part of its design.

</div>

So far, a part has been a solid and little else. Manufacturing adds meaning. A bracket cut from flat stock and folded in a press brake has to *unfold*, which no general solid does: a designer asks not only "what shape is it" but "what blank do I cut". A moulded housing has to come out of its mould, which constrains the angle of every wall. These are the first operations in this course whose subject is how a part is made.

They also make a design point well. Each of the crates in this lecture is a small program on top of machinery you already know: the sheet's solid is built whole (`BodySpec`, [Lecture 17](./17-brep.md)), a cut across a bend is a 2-D boolean of curves, a rib is an "up to next" boolean ([Lecture 22](./22-boolean.md)), a draft is the rebuild of [Lecture 30](./30-one-rebuild.md).

## What happens in a bend

Take a strip of metal of thickness `$T$` and bend it round a punch of radius `$r$`, through a turn of `$\theta$` radians (`$\theta=\pi/2$` for a right angle). The inside of the bend has radius `$r$`; the outside has radius `$r+T$`. So a fibre on the inside has length `$\theta r$` over the bend and a fibre on the outside has length `$\theta(r+T)$`.

The original strip had one length. So the inside has been compressed and the outside stretched, and somewhere between them lies a surface that has neither: the **neutral surface**. Its length before bending and after bending are the same. *That* is the length of flat material the bend consumes. Let the neutral surface lie a fraction `$K$` of the thickness out from the inside. Then

`$$\boxed{\ \text{BA}=\theta\,(r+K\,T)\ }$$`

is the **bend allowance**: the developed length of the bend. `$K$` is the **K-factor**.

If bending were pure bending of an elastic beam, the neutral surface would lie at mid-thickness, `$K=\tfrac12$`. In practice metal in a tight bend shifts it towards the inside; shop tables typically give values between 0.3 and 0.5, depending on material, tooling and the ratio `$r/T$`. Geop treats `$K$` as data given by the user. It has one rule per body (`k_factor` in `SheetMetalRules`, default `0.44`), it does not depend on the radius, and no bend tables or per-bend K-factors are implemented.

<figure class="demo" data-demo="bend-allowance">
<figcaption>A bend in section, and beneath it the flat strip to the same scale. Orange is the neutral surface, in both views: it has the same length folded and unfolded. Watch the flat strip change as you move K: only the strip's middle changes. The formulas are the kernel's, <code>Sheet::developed_length</code> and the setbacks of <code>EdgeFlange</code>, for one bend alone.</figcaption>
</figure>

## From flanges to flat length

Designers rarely dimension a flange from the bend's tangent line. They dimension it from where the outsides of two flanges *would* meet if the bend were sharp: the **outer sharp corner** (`LengthReference::OuterSharp`). Work out the gap. The two tangent points on the outside of the bend, and the sharp corner, form a kite with half-angle `$\theta/2$` at the centre:

`$$\text{OSSB}=(r+T)\tan\frac{\theta}{2},\qquad \text{ISSB}=r\tan\frac{\theta}{2}.$$`

These are the **outside setback** and inside setback. A flange of outer-sharp length `$A$` therefore has a flat part `$A-\text{OSSB}$`. Laid out flat, two flanges joined by a bend take

`$$L_{\text{flat}}=(A-\text{OSSB})+\text{BA}+(B-\text{OSSB})=A+B-\text{BD},\qquad \text{BD}=2\,\text{OSSB}-\text{BA}.$$`

`$\text{BD}$` is the **bend deduction**. Since the sharp corner overstates the path, the flat is shorter than the sum of the flanges.

### Worked example

Take `$T=1$`, `$r=2$`, `$K=0.44$`, `$\theta=90°$`.

- `$\text{BA}=\tfrac{\pi}{2}(2+0.44)=3.8327$`
- `$\text{OSSB}=3\tan45°=3$`
- `$\text{BD}=6-3.8327=2.1673$`

Flanges of 100 and 50 give a flat of `$150-2.1673=147.8327$`.

### Checked against the kernel

The example program `sheet_metal_bracket` (`ops/geop-ops-sheetmetal/src/examples.rs`) is a plate `$2\times1.2$` of thickness `$T=0.08$`, inner radius `$r=0.08$`, default `$K=0.44$`, with a 0.6 flange on one long edge and a 0.4 flange on the other, both at 90°, `OuterSharp` and `MaterialInside`, so the plate's old outline is where the flanges' outsides stand. With `$\text{OSSB}=0.16$` and `$\text{BA}=\tfrac\pi2\cdot0.1152=0.18096$`:

`$$L=0.6+1.2+0.4-2\,(0.32-0.18096)=1.92191.$$`

We ran it. Adding a `flat_pattern` step to the saved program, and asking `geop compile` for an STL capped as AGENTS.md asks, gave a blank whose extent across the bends is `1.921911` (the STL's `f32` corners) and along them `2.0`. The kernel agrees with the hand calculation in all the digits the format keeps.

<figure class="demo" data-demo="model-viewer" data-args='{"model":"sheet_metal_bracket","example":"sheet_metal_bracket"}'>
<figcaption>The bracket as the kernel meshes it. Note the rectangular reliefs beside the front flange, which stops short of its edge's ends. Open it in the editor and add a flat pattern step to see the blank.</figcaption>
</figure>

## The sheet is the model

Here is the design decision that everything else follows from. How would you write a flat pattern for a general solid? You would have to recognize which faces are cylinders of the right radius, which are the flat parts, and whether the part happens to unfold at all, each from a B-rep that may have come from anywhere. That is guessing, from geometry, a thing someone knew when building the part.

So Geop does not guess. Each sheet-metal operation records a `Sheet` (`ops/geop-ops-sheetmetal/src/sheet.rs`) for the solid it builds, in the part's target `sheets`: `flats`, and `bends` joining them into a tree. The solid is *derived* from the sheet, in one go.

- **Sheet coordinates.** Every flat is drawn in one plane of coordinates `$(x,y)$` and placed in space by a `Placement` of its own. A `Bend` joins an edge of a parent flat to an edge of a child flat, and the child's placement is the parent's turned about the bend's axis. So in sheet coordinates the child's edge *is* the parent's edge.
- **Flat pattern.** Laid flat, every flat keeps its shape exactly and moves away from its parent, across the bend's first edge, by the bend's developed length. `Sheet::flat_shifts` is about ten lines: for each bend, the child's shift is the parent's plus `$\text{out}\cdot\text{BA}$`. The bend itself becomes a strip as wide as `Sheet::developed_length`.
- **Sides.** The A side is the one drawn. The B side lies a thickness along each flat's normal. A plane moved along its normal, and a cylinder of radius `$r\pm T$` about the same axis, are exact, so the solid's two sides are given together and nothing is approximated (`thicken` in `ops/geop-ops-sheetmetal/src/thicken.rs`).

`FlatPattern` follows from this and is correspondingly short. Unfolding is topological and it is exact. The module `ops/geop-ops-sheetmetal/src/layout.rs` puts it plainly: "unfolding moves its curves, never approximates them".

The price is a refusal: only a body whose bends were recorded as it was built unfolds, and only while it is still made of just the faces they built. A body some other step has changed is refused, naming the faces that are not sheet metal. A fillet on a bracket is not something this unfolds. That is a fair trade: a false flat pattern is worse than none.

### Cuts across a bend

A hole cut through a bend is where the design pays off. In the folded solid it would be a boolean with a curved surface. Instead (`Layout::subtract`), the cut is subtracted in the flat layout, where a bend is no different from a flat and every curve is still a line or an arc; then each face is folded back into its place (`Sheet::folded`).

How does a point in the strip go back onto the bend? By arc length on the neutral surface: at across-distance `$a$`, the angle round the axis is `$a/(r+KT)$`, the formula you would expect (see `BendFrame` and its use in `ops/geop-ops-sheetmetal/src/fold.rs`).

One thing cannot stay exact. A cut's curve that crosses a bend at any angle other than straight along or across it wraps round the cylinder, and no NURBS is exactly a straight line wrapped round a cylinder (a helix is not a rational curve). `fold.rs` encloses it: a spline through points of it, widened to take in the rest by `NurbCurve3D::enclosing_pad`. Its edges along the axis are lines and its edges across are arcs, so those stay exact. Only the wrapped curve carries width, and its width says so honestly.

### Flanges, hems and reliefs

`EdgeFlange` bends a flange up from a straight edge of a flat. Three choices define it.

**Length reference.** `LengthReference::OuterSharp`, `LengthReference::InnerSharp`, or `LengthReference::Tangent` for the flat part only. The form subtracts `$(r+T)\tan(\theta/2)$`, `$r\tan(\theta/2)$` or nothing from the length, and refuses a flange with nothing flat left after its bend.

**Position.** `FlangePosition::MaterialInside` puts the flange's outside where the edge was, so the sheet is set back by the outer setback. `FlangePosition::MaterialOutside` puts its inside there, set back by the inner setback. `FlangePosition::BendOutside` starts the bend at the edge, adding everything outside.

**Reliefs and corners.** Where a flange is narrower than its edge, it would tear the sheet beside it. `SheetMetalRules::relief_size` is `relief_ratio` times the thickness, the width of a rectangular notch either side (`Relief::Rectangular`), and for `Relief::Tear` none. And a tear with a set-back bend beside uncut sheet is refused: the two would touch. The form's error says what to do instead. `Corner` decides what a flange does where another already stands on the edge beside it, and `corner_gap` keeps the two apart.

A **hem** (`ops/geop-ops-sheetmetal/src/hem.rs`) is a flange through half a turn. `HemKind::Closed` uses the body's own bend radius, and its comment explains why it cannot go to radius zero: "A fold of no radius at all, the hem pressed flat onto the sheet, would leave a solid touching itself, which is not one". `HemKind::Open` uses half the gap as inner radius. Geometry decides what the kernel will call a solid, whatever real presses do.

<div class="kernel">

**In the kernel.** `ops/geop-ops-sheetmetal` has `BaseFlange`, `EdgeFlange`, `Hem`, `SheetCut`, `FlatPattern`, and the exporter `FlatPatternDxf` (`ops/geop-ops-sheetmetal/src/cutting.rs`), which writes a sheet with the outline on a `CUT` layer and the bend lines on a `BEND` layer, each with how it bends. `Sheet::bend_lines` gives each bend's line down the middle of its strip, also available as the flat pattern's sketch. A sheet of paper for that is `lib/geop-lib-sheet`, which also serves drawings ([Lecture 35](./35-step-and-drawings.md)).

</div>

## Features for plastic parts

An injection-moulded housing has a different vocabulary. `ops/geop-ops-plastic` offers `Rib`, `Lip`, `Groove` and `Draft`. Note that these are marked experimental in the code: `OperationInfo::experimental`, with the comment "rarely used, and not yet robust enough to offer by default". Read this section as a description of what the code does, not as a claim of production readiness.

### Draft

A mould pulls apart along a direction, the **pull direction** `$p$`. A wall exactly parallel to it rubs along the whole stroke, so walls are tilted slightly, by a **draft angle**. `draft` (`ops/geop-ops-plastic/src/draft.rs`) tilts planar faces about a **neutral plane** whose normal is `$p$`. Each face turns about the **hinge**, the line where its plane meets the neutral plane, so that its outward normal leans towards `$p$`. Past the neutral plane, along `$p$`, the solid gets narrower.

For a wall of height `$h$` above the neutral plane and angle `$\alpha$`, the top moves in by `$h\tan\alpha$`. A wall 20 high at 2° moves in by `$20\tan2°\approx0.698$`, and every neighbour has to follow.

The file's comment explains why it does not use a boolean with a wedge per face. The wedge's tilted face would have to pass *exactly through* the face's edge on the neutral plane, a cut along an edge that already exists: the degenerate configuration from "Never place a cut on a corner that already exists" in AGENTS.md. And a wedge beside an inward corner would cut into the neighbour. Instead a draft "only changes the planes some faces lie in". The topology is unchanged, so it calls `replace_faces` from `lib/geop-lib-shell/src/rebuild.rs` and gets vertices re-met and edges re-traced, with every name kept. The tilted surface itself comes from turning the plane's control points in homogeneous form, with no division by the weight (`Tilt::turn`).

Stated limits, from the module comment: not faces that are not planar, not faces parallel to the neutral plane (no hinge), not faces meeting a neighbour tangentially (`Smooth::Refuse`), and an angle so large that an edge turns around is refused.

### Ribs

A **rib** is a thin stiffening wall grown from an open sketch profile until it meets the solid (`rib` in `ops/geop-ops-plastic/src/rib.rs`). Two ideas from earlier lectures appear in it.

Each end of the profile runs on along its tangent, in the sketch's plane, into the wall it points at, to the *middle* of that wall: between where the line enters the solid and where it leaves. "A place the geometry defines, so the rib's end face lies on no face of the solid." Stopping exactly at the wall would put a tool's face on a solid's face, a coincidence no boolean should be asked to resolve. A long-standing rule, here met by choosing a position inside material.

The growth itself is the "up to next" of an extrude: the sweep reaches well past the solid, and only what grows from the profile until it runs into the solid is joined (a `Combine` with a `Tool` going up to the next face). A profile not stopped by the solid all along is refused.

The profile is thickened by offsetting (`ops/geop-ops-plastic/src/band.rs`): a line moves sideways, an arc grows or shrinks around its centre, and both stay exact. Any other curve's offset is no NURBS, so a cubic is fitted through points of it until it comes within `FIT_TOLERANCE` of the distance everywhere, proven by `NurbCurve2D::offset_miss`. Where two lines meet at a corner their offsets are mitred to where they cross. A corner involving any other piece is not supported, and says so.

### Lips and grooves

Two halves of an enclosure meet at a **lip** on one and a **groove** on the other (`ops/geop-ops-plastic/src/lip.rs`). Both are a band along a chain of edges of a planar rim, swept square off the rim. The lip is `width` wide and `height` high. The groove is `clearance` wider and deeper. Its cut reaches past the rim and past the wall into the air, `width` both ways, "so that none of its faces lies on a face of the solid". The same rule again. By now you should be expecting it: when a tool is built, you choose where its surfaces go, and you choose them to cross.

## What these have in common

Compare this lecture's designs against a naive version of each.

| | naive | what Geop does | why |
|---|---|---|---|
| Flat pattern | recognize cylinders in a solid | keep a `Sheet`, derive the solid from it | a recorded fact beats a guessed one |
| Cut across a bend | boolean on the folded solid | subtract in the layout, then fold | where the problem is linear, solve it there |
| Draft | subtract a wedge per face | replace the surface, rebuild | never cut along an existing edge |
| Rib end | stop at the wall | run to the middle of the wall | a tool's face must not lie on a solid's |
| Groove | cut exactly to the rim | reach past into the air | same |

None of these is a sheet-metal or plastics idea. They are the kernel's rules, seen through the question of how a part is made.

<div class="recap">

## Recap

- The neutral surface keeps its length. `$\text{BA}=\theta(r+KT)$`, with `$K$` a per-body number Geop takes as given. `$\text{BD}=2(r+T)\tan(\theta/2)-\text{BA}$`.
- A sheet-metal body is a recorded `Sheet` of flats and bends. The solid is derived, and the flat pattern exact. A body changed by other steps is refused.
- Cuts are subtracted in the flat layout and folded back. A curve running obliquely across a bend is the one thing that must be enclosed, not exact.
- A draft replaces planes and rebuilds; ribs and lips are bands swept and combined. Their tools end where geometry defines, never on a face of the solid.
- The plastic features are experimental in the code.

</div>

## Exercises

**1.** For `$T=2$`, `$r=3$`, `$K=0.4$`, a bend of `$60°$`: compute BA, the outside setback, and the bend deduction. Check with the demo. Why does BD get smaller at small angles, and what is it at `$\theta\to0$`?

<details class="solution"><summary>Solution</summary>

`$\theta=1.0472$`. `$\text{BA}=1.0472\,(3+0.8)=3.9794$`. `$\text{OSSB}=5\tan30°=2.8868$`. `$\text{BD}=5.7735-3.9794=1.7941$`. As `$\theta\to0$`, `$\text{BD}\approx(r+T)\theta-\theta(r+KT)=\theta\,T(1-K)\to0$`: the flat strip with no bend needs no correction.

</details>

**2.** Show that the flat length does not depend on `$K$` if flanges are given by their flat parts only (`LengthReference::Tangent`) and the *bend's strip* is excluded. Then show it does depend on `$K$` if they are given outer-sharp. Which dimension in a real part is therefore sensitive to `$K$`?

<details class="solution"><summary>Solution</summary>

Tangent lengths add directly; only BA contains `$K$`, so excluding the strip removes it. With outer-sharp lengths, `$L=A+B-2\text{OSSB}+\text{BA}$`, which grows as `$\theta TK$`. So a change `$\Delta K$` shifts the position of everything past the bend by `$\theta T\Delta K$`. For `$T=2$`, `$\theta=\pi/2$`, `$\Delta K=0.1$`: `$0.31$`. Hole positions across a bend inherit this.

</details>

**3.** A bracket has flats joined in a chain: flat 0 — bend a — flat 1 — bend b — flat 2. Write the shift of flat 2's coordinates in terms of the bends' developed lengths and out-directions, as `Sheet::flat_shifts` computes it. Is the shift of a child dependent on the order the bends are listed? What invariant must the list satisfy, and what happens if it does not?

<details class="solution"><summary>Solution</summary>

`$s_2=s_1+\text{out}_b\text{BA}_b=\text{out}_a\text{BA}_a+\text{out}_b\text{BA}_b$`. It reads each parent's shift as it goes, so each bend's parent must come before it: the doc of `Sheet` says "each child built after its parent". `flat_shifts` returns an error, "hangs off a flat that is not laid out before it", if one does not, and another if a flat is joined by no bend.

</details>

**4.** A straight cut in the flat strip of a bend runs at 45° across it. Parametrize its image on the bend's neutral cylinder. Is it a NURBS? Which well-known curve is it, and in what way does the kernel handle it?

<details class="solution"><summary>Solution</summary>

With `$a$` the across-distance and `$b$` the along-distance, the line `$b=a$` maps to `$(R\cos(a/R),R\sin(a/R),a)$` with `$R=r+KT$`: a circular helix of pitch `$2\pi R$`. A helix is not rational, so no NURBS is exactly it. The kernel encloses it in a spline through points of it widened by `NurbCurve3D::enclosing_pad`, in `ops/geop-ops-sheetmetal/src/fold.rs`.

</details>

**5.** A draft's hinge is `plane.intersect_plane(neutral)`. Explain why a face parallel to the neutral plane must be refused, what the error says, and how you would change the design if someone really wanted a parallel face offset. Which operation of [Lecture 30](./30-one-rebuild.md) does that already?

<details class="solution"><summary>Hint</summary>

Two parallel planes have no line of intersection, so there is nothing to turn about. A parallel face that moves is an offset, a different operation from a tilt: `Smooth` and `replace_faces` serve both.

</details>

**6.** Why does a rib's end run on to the middle of the wall, instead of a fixed small distance into it? Give a configuration where a fixed distance fails, and connect your answer to "Never place a cut on a corner that already exists" in AGENTS.md.

<details class="solution"><summary>Solution</summary>

A fixed distance is a number unrelated to the geometry. If the wall is thinner than it, the rib's end pokes out the far side; if it equals the wall's thickness, the rib's end face lies on the far face, the coincidence the rule forbids. Since a design dimension is often a round number, such coincidences are not rare. AGENTS.md notes the same trap: in *A tool's own curves must not lie in the solid's faces*, "a fraction of one design dimension lands on another". The middle between entry and exit is defined by the geometry, and lies strictly inside the wall for any thickness.

</details>
