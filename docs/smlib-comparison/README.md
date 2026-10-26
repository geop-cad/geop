# geop vs SMLib: feature comparison

SMLib (`usd-brep/source/SMLib`, ~870k lines of C++) is a mature NURBS B-rep
kernel built on tolerances. geop is younger and built on interval enclosures.
This comparison was made in October 2026 to find what geop still lacks; each
area has a detailed file with a feature table, the special cases to port
(SMLib and geop file:line, a port sketch in geop's style) and the missing
whole features:

| area | file | headline gaps |
|---|---|---|
| Booleans and SSI | [booleans.md](booleans.md) | closed intersection loops no edge pierces; separate lumps and voids; post-process merge of faces on one surface; imprint, XOR, section curves, N-ary, sheet/wire booleans |
| Fillets and chamfers | [fillets.md](fillets.md) | corners with mixed convexity, different radii, 4+ edges; rollover over creases; vanishing fillets; setbacks and bevel corners; defillet; face-to-face fillets; conic/two-radius/constant-width sections |
| Offsets and shells | [offsets.md](offsets.md) | edges neither straight nor circular; vanishing faces; inward-corner openings; free-form offsets; self-intersection detection; per-face/outward shell; spline sketch offsets |
| Direct modelling and healing | [direct.md](direct.md) | push-pull / move and offset face; delete face and heal; merge faces on one surface; stitch with T-junctions; defeature; scale; bend/twist/taper; mesh to B-rep |
| Geometry, construction and queries | [geometry.md](geometry.md) | revolve with profile touching the axis; tapered extrude; sweep mitres at curved corners; loft to a point and closed lofts; crack-free adaptive tessellation; knot removal and degree reduction; G1 surfacing; tapered helix; projection; IGES |

## Where geop is already better

- Honest enclosures instead of capture distances, vertex averaging and gap
  squeezing; named refusals instead of silent tolerance growth.
- Geometry far from the origin, poles and cone apexes in intersection.
- Mass properties (exact quadrature, enclosed error), proven global closest
  points, exact sweep frames along lines and arcs, hidden-line removal,
  exact helix.
- STEP healing moves vertices and edges onto where surfaces meet; SMLib only
  grows tolerances.
- Persistent naming through every operation; direct shell construction that
  keeps topology and exact surfaces (SMLib offsets everything and runs a
  boolean).
- Validation after every operation.

## Not to port

SMLib dispatches on surface type throughout (analytic plane/quadric
intersections, offsets of extrusions and revolutions through their
profiles, exact torus fillets, analytic STEP entities). geop keeps every
curve and surface a NURBS and ports only the configurations SMLib handles,
never the type dispatch (see AGENTS.md, "Everything is a NURBS curve or
surface"). The detailed files still list those analytic items; they are not
part of the port.


SMLib has stubs and bugs that must not be copied (each file lists them):
`SmBrep::ExtendSurfaces` is a no-op (its type check is always true),
`NSidedPatchRebuild` is marked as not working, five heal steps are stubs,
`Fix_BadLoops` is disabled, twist clamps left-hand twists, `SpaceBend` picks
zone faces by hard-coded indices, the fillet self-intersection check is a
stub, and gap repair by tolerance (`FixGaps`, vertex squeezing, 0.95·d face
removal) contradicts geop's enclosure rules.

## Porting

Each area was ported on its own branch off `sm-lib-port` by one agent, in the
order of its file's priority list:

| area | owns |
|---|---|
| booleans | `lib/geop-lib-booleans`, SSI in `core/geop-core-geometry`, `ops/geop-ops-boolean` |
| fillets | `ops/geop-ops-fillet` |
| offsets | `lib/geop-lib-shell`, `ops/geop-ops-shell`, surface and sketch offsets |
| direct modelling | `ops/geop-ops-edit`, merge of faces on one surface (core topology), healing, scale, deformation, mesh to B-rep |
| geometry | sweeps, lofts, revolve, tapered extrude, surfacing, tessellation, fitting, projection, IGES |
