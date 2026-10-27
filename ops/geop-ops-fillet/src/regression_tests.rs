//! Programs from bug reports, rebuilt with the coordinates they were
//! reported with: a "clean" equivalent may not carry the same case at all.
//! Each is the smallest program found that still fails.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{
    NamedModel,
    tangent::tangent_chain,
    testing::{Built, assert_valid, edges_with_middle_where},
};

use geop_core_math::primitives::FrameAxis;
use geop_core_program::element::ElementReference;
use geop_lib_fixtures::GivenDatumArgs;
use geop_ops_extrude_revolve::{
    Extents, LoftArgs, Output,
    testing::{extrude, extrusion},
};
use geop_ops_model::{
    Combine,
    datums::datums,
    sketch::{
        Sketch, SketchPlane,
        figures::{n, on_origin},
        whole_sketch,
    },
};
use geop_ops_sketch::AddSketchArgs;

use geop_core_topology::contains::shell::PointClassification;

use crate::{Fillet, FilletArgs};

/// The operations these programs are written in: given solids, blended.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Fillet)
}

/// Builds `program` and checks the part it builds is valid.
fn assert_builds_valid(program: &Program) {
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The edges of `part` whose middle is at `at`.
fn edges_at(part: &NamedModel, at: [f64; 3]) -> Vec<String> {
    edges_with_middle_where(part, |m| (0..3).all(|k| near(m[k], at[k])))
}

/// A prism over `corners`, 30 high, with its upright edge at `corner`
/// rounded by `radius`.
fn prism_with_rounded_upright(corners: &[[f64; 2]], corner: [f64; 2], radius: f64) -> Program {
    let mut program = Program::new();
    program.push("p", PrimitiveArgs::prism(corners, 0.0, 30.0));
    let before = program.build_model(registry).unwrap();
    let edge = edges_at(&before, [corner[0], corner[1], 15.0]);
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("round", FilletArgs::constant(edge, radius));
    program
}

/// A phone stand's side profile (Fusion 360 beginner tutorial, "Phone
/// stand": the profile sketched on a side plane, extruded, every side edge
/// filleted) has a back rest leaning 4 over 20 from a vertical back: the
/// two faces meet at 169°. Rounding that edge alone fails.
///
/// Rebuilt here as a prism with the same two faces at the corner `(0, 40)`:
/// the vertical side from `(0, 0)` and the side leaning to `(4, 60)`. The
/// fillet's boolean fails tracing the blend tool across the leaning face:
///
/// > the corrector did not bring the two surfaces onto a common point
/// > within 20 iterations: face_a lands at (u=0.0004, v=0.669) and face_b
/// > at (u=0.916, v=1e-16), which cannot be the same point
///
/// at `predictor_corrector_step` from the vertex at `(0, 39.85, 30)` on the
/// prism's top. Ruled out: the size (the same prism at a tenth fails the
/// same way), the radius (0.15 on the small one fails). A chamfer of the
/// same edge builds. Up to 155° the fillet builds; at 160° it fails
/// differently (see [`fillet_of_an_upright_edge_between_faces_at_160_degrees`]).
/// That was the extruded tool; rolled, as every edge now is, it builds.
#[test]
fn fillet_of_an_upright_edge_between_faces_at_169_degrees() {
    assert_builds_valid(&prism_with_rounded_upright(
        &[
            [0.0, 0.0],
            [70.0, 0.0],
            [12.0, 60.0],
            [4.0, 60.0],
            [0.0, 40.0],
        ],
        [0.0, 40.0],
        1.5,
    ));
}

/// The same prism as [`fillet_of_an_upright_edge_between_faces_at_169_degrees`]
/// with the back rest leaning 7 over 20 (the faces at 160.7°). The fillet
/// failed classifying a face of the blend tool:
///
/// > face_interior_point: no point strictly inside face FaceId(58) was
/// > found, stepping inward from the midpoint of each of its 3 outer
/// > coedges; that loop spans u=[-2e-9, 0.0012], v=[0.662, 0.671]
///
/// The face is a piece of the prism's leaning side cut off by the tool, a
/// thin triangle, 0.0012 across in the side's parameters: thinner than the
/// margin from its boundary `face_interior_point` asked of a point, the
/// boolean's resolution in `(u, v)`. That margin is now made finer until a
/// point is found.
#[test]
fn fillet_of_an_upright_edge_between_faces_at_160_degrees() {
    assert_builds_valid(&prism_with_rounded_upright(
        &[
            [0.0, 0.0],
            [70.0, 0.0],
            [40.0, 60.0],
            [7.0, 60.0],
            [0.0, 40.0],
        ],
        [0.0, 40.0],
        1.5,
    ));
}

/// A drawer handle (Fusion 360 tutorial "Drawer pull": two round posts, a
/// bar across their tops, the joints rounded): a post of radius 3 standing
/// under a bar 8 wide, the circle where they meet. The program, and that
/// circle's arcs.
fn post_under_bar() -> (Program, Vec<String>) {
    let mut program = Program::new();
    program.push(
        "post",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 3.0, 20.0),
    );
    program.push(
        "bar",
        PrimitiveArgs::block([-10.0, -4.0, 20.0], [10.0, 4.0, 28.0]).join("cylinder(post)"),
    );
    let before = program.build_model(registry).unwrap();
    let arcs = edges_with_middle_where(&before, |m| {
        near(m[2], 20.0) && (m[0].powi(2) + m[1].powi(2)).sqrt() < 3.5
    });
    assert_eq!(arcs.len(), 4, "{arcs:?}");
    (program, arcs)
}

/// The drawer handle's joint rounded by 1.5: the fillet's contact on the
/// bar's underside is a circle of radius 4.5, which runs off the bar
/// (half-width 4) on both sides, over its convex side edges: a cliff. Its
/// contact checked only halfway along each arc, it built an invalid solid
/// (edges at `y = 4.11`, outside the bar). Checked whole, the contact
/// crosses the bar's side edges, and the fill is cut off flush with the
/// bar's sides, carried on below them: filled under the bar, not past it.
/// (Merging the fill's edge in the side's plane, a U from one crossing to
/// the other, with the bar's straight edge between the same two vertices
/// put the U's coedges on the line: two edges sharing their vertices are
/// now one only where one's middle lies on the other.)
#[test]
fn fillet_of_a_post_joint_whose_contact_runs_off_the_bar() {
    let (mut program, arcs) = post_under_bar();
    program.push("round", FilletArgs::constant(arcs, 1.5));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let at = |p| where_in(&part, "fillet(round)", p);
    // Under the bar, between the post and the contact circle: filled.
    assert_eq!(at([0.0, 3.6, 19.8]), PointClassification::Inside);
    assert_eq!(at([3.6, 0.0, 19.8]), PointClassification::Inside);
    // Past the bar's side, where the fill would have run on: not.
    assert_eq!(at([0.0, 4.05, 19.99]), PointClassification::Outside);
    assert_eq!(at([0.0, -4.05, 19.99]), PointClassification::Outside);
}

/// A block with a rib standing on its top at one end, set back from the
/// front edge: the front top edge rounded so large that its contact on the
/// top runs under the rib, across its foot — a wall, the edge between them
/// concave: the round crashes into it. It is cut off flush with the rib's
/// side, so that it cuts nothing of the rib.
#[test]
fn fillet_whose_contact_runs_into_a_rib() {
    let mut program = Program::new();
    program.push(
        "base",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 10.0, 5.0]),
    );
    program.push(
        "rib",
        PrimitiveArgs::block([8.0, 0.5, 5.0], [10.0, 10.0, 8.0]).join("block(base)"),
    );
    let part = rounded_where(program, |m| near(m[1], 0.0) && near(m[2], 5.0), 1.0, true);
    let at = |p| where_in(&part, "fillet(round)", p);
    assert_eq!(at([4.0, 0.05, 4.95]), PointClassification::Outside);
    assert_eq!(at([4.0, 0.5, 4.5]), PointClassification::Inside);
    // In the rib, just above its foot by the edge: not cut into.
    assert_eq!(at([9.0, 0.7, 5.1]), PointClassification::Inside);
}

/// A block with a rib standing on its top, set back from the front edge
/// and running across it: the front top edge rounded so large that its
/// contact on the top runs under the rib, across its foot and on past it
/// back onto the top. Cut off by the rib's sides, the round would be cut
/// away on the rib's far side too: refused, naming the rib's side.
#[test]
fn fillet_whose_contact_runs_under_a_rib_and_out_again_is_refused() {
    let mut program = Program::new();
    program.push(
        "base",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 10.0, 5.0]),
    );
    program.push(
        "rib",
        PrimitiveArgs::block([4.0, 0.5, 5.0], [5.0, 10.0, 8.0]).join("block(base)"),
    );
    let before = program.build_model(registry).unwrap();
    let edge = edges_with_middle(&before, |m| near(m[1], 0.0) && near(m[2], 5.0));
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("round", FilletArgs::constant(edge, 1.0));
    let Err(error) = program.build_model(registry) else {
        panic!("a round running under a rib and out again is not refused");
    };
    let error = format!("{error:?}");
    assert!(
        error.contains("a blend is cut off only by a face its contact leaves for good"),
        "{error}"
    );
}

/// A thin plate with a hole (Fusion 360 "Bottle opener" and sheet-like
/// tutorials: a large plate, a hole, its rim rounded by a small radius): a
/// plate 100 x 100, a hole of radius 5 through it, the rim rounded by 0.04.
/// The fillet failed classifying the piece of the top cut off by the tool:
///
/// > face_interior_point: no point strictly inside face FaceId(351) was
/// > found, stepping inward from the midpoint of each of its 4 outer
/// > coedges; that loop spans u=[0.4496, 0.5504], v=[0.4496, 0.5504]
///
/// The loop is the fillet's contact circle (radius 5.04) on the plate's
/// top, 0.0504 of the surface's domain across; what it bounds is the
/// annulus between it and the hole's rim, 0.0004 wide in `(u, v)`: thinner
/// than the margin from its boundary `face_interior_point` asked of a
/// point, the boolean's resolution in `(u, v)`. That margin is now made
/// finer until a point is found. A plate 20 x 20 with the same hole and
/// fillet built: the failure came with the fillet's width relative to the
/// face it cuts.
#[test]
fn fillet_of_a_hole_rim_tiny_against_its_face() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [100.0, 100.0, 1.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([50.0, 50.0, -1.0], 5.0, 3.0).cut("block(plate)"),
    );
    let before = program.build_model(registry).unwrap();
    let rim = edges_with_middle_where(&before, |m| {
        near(m[2], 1.0) && (m[0] - 50.0).abs() < 5.01 && (m[1] - 50.0).abs() < 5.01
    });
    assert_eq!(rim.len(), 4, "{rim:?}");
    program.push("round", FilletArgs::constant(rim, 0.04));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The rim moved out to 5.04 on the top.
    let contact = part.topology().vertices.values().any(|v| {
        let (dx, dy) = (
            v.point[0].sub(Scalar::from_f64(50.0)),
            v.point[1].sub(Scalar::from_f64(50.0)),
        );
        v.point[2].could_be_equal(Scalar::from_f64(1.0))
            && dx
                .mul(dx)
                .add(dy.mul(dy))
                .could_be_equal(Scalar::from_f64(5.04 * 5.04))
    });
    assert!(contact);
}

/// The edges of `part` at its vertex named `vertex`, by name.
fn edges_at_vertex(part: &NamedModel, vertex: &str) -> Vec<String> {
    let vertex = part.vertex_id(vertex).unwrap();
    let mut edges: Vec<String> = (part.topology().edges.iter())
        .filter(|(_, e)| e.start_vertex == vertex || e.end_vertex == vertex)
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    edges.sort();
    edges
}

/// A cylinder with a flat cut down its side (the stress sweep's
/// `cylinder_with_flat`), the three edges at a top corner of the flat
/// rounded together: the corner rounded by a ball, which joins the tools
/// of the top rim and the flat's top edge into one. Both run on to the
/// flat's other top corner, and leave the solid there — the rim through the
/// flat, the flat's edge through the cylinder — so in that one tool they
/// would overlap: refused, naming the two edges and the vertex. (Built, the
/// tool was a solid overlapping itself and the boolean left the cylinder's
/// top face inside out.) Round that corner too, and both corners are
/// rounded by balls.
#[test]
fn rounding_a_corner_of_a_cylinders_flat() {
    let (part, _) =
        geop_lib_fixtures::bodies::one_solid(&geop_lib_fixtures::bodies::cylinder_with_flat());
    let corner = "combine(flat,block(flat,c3,end),cylinder(body,c0,end),0,1)";
    let other = "combine(flat,block(flat,c3,end),cylinder(body,c3,end),0,1)";
    let edges = edges_at_vertex(&part, corner);
    assert_eq!(edges.len(), 3, "{edges:?}");
    let refused = geop_ops_model::testing::apply_to_model::<Fillet>(
        part.clone(),
        "round",
        &FilletArgs::constant(edges.clone(), 0.1),
    );
    let message = format!("{:?}", geop_ops_model::testing::error(refused));
    assert!(
        message.contains("both end on their own at vertex"),
        "{message}"
    );
    let mut both = edges;
    both.extend(edges_at_vertex(&part, other));
    both.sort();
    both.dedup();
    let rounded = geop_ops_model::testing::apply_to_model::<Fillet>(
        part,
        "round",
        &FilletArgs::constant(both, 0.1),
    )
    .unwrap();
    assert_valid(&rounded);
}

/// The operations the turbine example is written in, and fillets.
fn turbine_registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(geop_ops_reference_geometry::AddDatum)?
        .with(geop_ops_pattern::CircularPattern)?
        .with(Fillet)
}

/// The edges of the turbine example (`geop_ops_pattern::examples::
/// turbine_blade`: a revolved hub, a blade lofted through three sketches,
/// patterned six times round the hub and joined to it) whose names start
/// with `prefix`.
fn turbine_edges(part: &NamedModel, prefix: &str) -> Vec<String> {
    let mut names: Vec<String> = (part.topology().edges.keys())
        .filter_map(|&e| part.name_of(e))
        .filter(|n| n.starts_with(prefix))
        .map(|n| n.to_string())
        .collect();
    names.sort();
    names
}

/// `program`, its solid `solid` with the edges `edges` rounded by `r` as
/// the step `round`, as a user rounds them in the editor: built, valid, and
/// its volume changed the way the blend goes — less, cut, where `cuts`;
/// more, filled, else — by what a round's corner holds along the chains
/// blended: between `r^2 / 10` and `10 r^2` per unit of their length (a
/// right angle's corner holds `0.21 r^2`, the blade's trailing edge, of 25
/// degrees, `3.2 r^2`).
fn assert_rounds(
    mut program: Program,
    registry: fn() -> GeopResult<OperationRegistry>,
    solid: &str,
    edges: Vec<String>,
    r: f64,
    cuts: bool,
) -> NamedModel {
    let before = program.build_model(registry).unwrap();
    let mut chained: Vec<geop_core_topology::EdgeId> = Vec::new();
    for edge in &edges {
        let chain = tangent_chain(before.topology(), before.edge_id(edge).unwrap()).unwrap();
        chained.extend(chain.edges());
    }
    chained.sort_by_key(|e| e.0);
    chained.dedup();
    let length: f64 = (chained.iter())
        .map(|&e| {
            let curve = &before.topology().get_edge(e).unwrap().curve;
            curve.length().unwrap().0.to_f64()
        })
        .sum();
    program.push("round", FilletArgs::constant(edges, r));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // Integrated, not meshed: a mesh's volume is off by more than a round
    // this small changes it by.
    let volume = |part: &NamedModel, solid: &str| {
        let solid = part.solid_id(solid).unwrap();
        let mass = part.topology().mass_properties(solid).unwrap();
        mass.volume.midpoint().to_f64()
    };
    let (a, b) = (volume(&before, solid), volume(&part, "fillet(round)"));
    let change = if cuts { a - b } else { b - a };
    // The faces the blend made, by name: what tells a blend that was not
    // applied from one applied wrong.
    let mut made: Vec<&str> = (part.topology().faces.keys())
        .filter_map(|&f| part.name_of(f))
        .filter(|n| n.starts_with("fillet(round"))
        .collect();
    made.sort();
    assert!(
        change > 0.1 * r * r * length && change < 10.0 * r * r * length,
        "the volume went from {a} to {b}, along {length} of edges; the blend made faces {made:?}"
    );
    part
}

/// The turbine example, the edges named by `prefixes` rounded by 0.1 (see
/// [`assert_rounds`]).
fn assert_turbine_rounds(prefixes: &[&str], cuts: bool) {
    let program = geop_ops_pattern::examples::turbine_blade();
    let before = program.build_model(turbine_registry).unwrap();
    let mut edges = Vec::new();
    for prefix in prefixes {
        let found = turbine_edges(&before, prefix);
        assert_eq!(found.len(), 1, "{prefix}: {found:?}");
        edges.extend(found);
    }
    let solid = "circular_pattern(circular_pattern1)";
    assert_rounds(program, turbine_registry, solid, edges, 0.1, cuts);
}

/// The operations the blade's programs below are written in: sketches,
/// extrusions and lofts, given solids and datums, and fillets.
fn blade_registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(geop_lib_fixtures::Primitive)?
        .with(geop_lib_fixtures::GivenDatum)?
        .with(Fillet)
}

/// The turbine example's first blade section, a tenth its size: its
/// points `p5`, `p8` and `p6` (see [`blade_section`]).
const SECTION: [[f64; 2]; 3] = [[-1.008, -1.593], [1.728, 1.138], [1.008, 1.593]];

/// The blade section [`SECTION`] `scale` times its size, sketched: two
/// quadratic splines, `p5 p8 p6` and `p6 p10 p5`, meeting tangentially at
/// `p6` — `p10` as far on past `p6` from `p8` as `p8` is before it — and at
/// a corner of 25 degrees at `p5`, the trailing edge.
fn blade_section(scale: f64) -> Sketch {
    let [p5, p8, p6] = SECTION.map(|[x, y]| [scale * x, scale * y]);
    let p10 = [2.0 * p6[0] - p8[0], 2.0 * p6[1] - p8[1]];
    let mut sketch = Sketch::new();
    let [p5, p8, p6, p10] = [p5, p8, p6, p10].map(|[x, y]| sketch.add_point(n(x), n(y)));
    sketch.add_spline(vec![p5, p8, p6]);
    sketch.add_spline(vec![p6, p10, p5]);
    sketch
}

/// The edges of `part` whose middles are where `keep` says, by name.
fn edges_with_middle(part: &NamedModel, keep: impl Fn([f64; 3]) -> bool) -> Vec<String> {
    let mut edges = edges_with_middle_where(part, keep);
    edges.sort();
    edges
}

/// The blade section's trailing edge `p5`, in its sketch's coordinates,
/// and the unit direction halving its corner, into the section.
fn trailing_corner() -> ([f64; 2], [f64; 2]) {
    let [p5, p8, p6] = SECTION;
    let p10 = [2.0 * p6[0] - p8[0], 2.0 * p6[1] - p8[1]];
    let unit = |q: [f64; 2]| {
        let d = [q[0] - p5[0], q[1] - p5[1]];
        let l = d[0].hypot(d[1]);
        [d[0] / l, d[1] / l]
    };
    let (a, b) = (unit(p8), unit(p10));
    let h = [a[0] + b[0], a[1] + b[1]];
    let l = h[0].hypot(h[1]);
    (p5, [h[0] / l, h[1] / l])
}

/// Where the point `p` is in the solid `solid` of `part`.
fn where_in(part: &NamedModel, solid: &str, p: [f64; 3]) -> PointClassification {
    geop_ops_model::testing::classify(part, solid, p)
}

/// The turbine's first blade section extruded 2 up, the edge round its top
/// rounded by 0.1: the tangent chain runs round both splines' top edges,
/// past the tangent corner, and starts and ends at the trailing edge's top,
/// a corner each end leaves the solid at through the other spline's face,
/// curved. Each end runs on past it, the ball rolled on the faces carried
/// on, and the tool is built in halves, which would overlap there as one.
/// The corner's tip is cut away, the section's middle kept. (The volume is
/// not integrated here: on the rolled faces that takes ten seconds; the
/// turbine's own tests do, `rounding_a_turbine_blades_tip`.)
#[test]
fn rounding_the_top_of_a_blade_section() {
    let mut program = Program::new();
    program.push(
        "section",
        AddSketchArgs::on(on_origin(FrameAxis::Z), blade_section(1.0)),
    );
    program.push("blade", extrude("section", 2.0, false));
    let before = program.build_model(blade_registry).unwrap();
    let top = edges_with_middle(&before, |m| near(m[2], 2.0));
    assert_eq!(top.len(), 2, "{top:?}");
    program.push("round", FilletArgs::constant(vec![top[0].clone()], 0.1));
    let part = program.build_model(blade_registry).unwrap();
    assert_valid(&part);
    let (p5, into) = trailing_corner();
    let tip = [p5[0] + 0.05 * into[0], p5[1] + 0.05 * into[1], 1.99];
    assert_eq!(
        where_in(&before, "extrude(blade)", tip),
        PointClassification::Inside
    );
    assert_eq!(
        where_in(&part, "fillet(round)", tip),
        PointClassification::Outside
    );
    assert_eq!(
        where_in(&part, "fillet(round)", [0.0, 0.0, 1.0]),
        PointClassification::Inside
    );
}

/// The turbine's first blade section lofted into itself twice its size
/// 7.07 above it and on into itself half its size 18.2 above that, as the
/// example spaces and sizes its sections, roughly — scaled exactly, so
/// that the loft matches their curves whole — and the trailing edge, two
/// edges, one per span of the loft, rounded by 0.1.
///
/// Scaled exactly, the sections cross the trailing edge so nearly square
/// to it that the ball reaches the middle section on both faces within a
/// millionth of the way along the edge, and at the edge's vertex there.
/// The stations there are now one — the crossings closer than the boolean
/// resolves taken as one, the vertex's station dropped by them — and the
/// rolled tool is built, but the boolean leaves the first span's face
/// through `c4` straddling the tool (`classify_face`: an intersection curve
/// not imprinted). With that boolean fixed, the one station kept for the
/// two crossings still left one side's contact `7.6e-7` short of the
/// section edge it crosses, and validation refused the tool's edge between
/// the two: the station now takes each side's contact on its own edge. The
/// turbine's own sections, not scaled alike, cross it apart, and its
/// trailing edge is rounded (`rounding_a_turbine_blades_trailing_edge`).
#[test]
#[ignore = "slow: 40 s, a lofted blade's trailing edge rounded — run with `cargo test -- --ignored`"]
fn rounding_the_trailing_edge_of_a_lofted_blade() {
    let mut program = Program::new();
    let heights = [0.0, 7.07, 25.27];
    for (k, (scale, z)) in [1.0, 2.0, 0.5].into_iter().zip(heights).enumerate() {
        let plane = format!("plane{k}");
        program.push(
            &plane,
            GivenDatumArgs::plane([0.0, 0.0, z], [0.0, 0.0, 1.0]),
        );
        program.push(
            format!("section{k}"),
            AddSketchArgs::on(
                SketchPlane::Datum(ElementReference::new(datums(), &plane)),
                blade_section(scale),
            ),
        );
    }
    program.push(
        "blade",
        LoftArgs {
            profiles: (0..3)
                .map(|k| whole_sketch(&format!("section{k}")))
                .collect(),
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let before = program.build_model(blade_registry).unwrap();
    assert_valid(&before);
    // The lateral edges by the trailing edge, below the middle section and
    // above it.
    let trailing = edges_with_middle(&before, |m| {
        heights.iter().all(|&z| !near(m[2], z)) && m[0] < 0.0 && m[1] < 0.0
    });
    assert_eq!(trailing.len(), 2, "{trailing:?}");
    assert_rounds(program, blade_registry, "loft(blade)", trailing, 0.1, true);
}

/// The turbine's first blade section extruded 6 along `x` from the
/// `yz` plane and joined to a cylinder of radius 4 along `z` around the
/// origin, as a blade stands on a hub, and its trailing edge rounded by 0.1:
/// it runs into the cylinder at an angle — the trailing edge is no radius of
/// it — so the blend runs on into it, rolled, and is cut off by it, a
/// sheet on the cylinder carried on splitting the tool. By the trailing
/// edge, the blade is cut away down to the hub, and the hub not into.
#[test]
fn rounding_the_trailing_edge_of_a_blade_section_on_a_hub() {
    let mut program = Program::new();
    program.push("hub", PrimitiveArgs::cylinder_z([0.0, 0.0, -4.0], 4.0, 8.0));
    program.push(
        "section",
        AddSketchArgs::on(on_origin(FrameAxis::X), blade_section(1.0)),
    );
    program.push(
        "blade",
        extrusion(
            "section",
            Extents::blind(6.0),
            Combine::join("cylinder(hub)"),
        ),
    );
    let before = program.build_model(blade_registry).unwrap();
    // The trailing edge, on the side of the blade below the line `y = -z`,
    // standing out of the hub: of the lines along `x`, the one past it.
    let trailing: Vec<String> =
        edges_with_middle(&before, |m| m[0] > 4.0 && m[0] < 6.0 && m[1] + m[2] < 0.0)
            .into_iter()
            .filter(|e| !e.starts_with("cylinder(hub"))
            .collect();
    assert_eq!(trailing.len(), 1, "{trailing:?}");
    // Where the trailing edge is, in `(y, z)`: the sketch's own coordinates,
    // or the other way round.
    let edge = before
        .topology()
        .get_edge(before.edge_id(&trailing[0]).unwrap())
        .unwrap();
    let m = geop_ops_model::testing::middle(edge).to_f64_array();
    let (p5, into) = trailing_corner();
    let swap = !near(m[1], p5[0]);
    let at = |x: f64| {
        let (u, v) = (p5[0] + 0.05 * into[0], p5[1] + 0.05 * into[1]);
        if swap { [x, v, u] } else { [x, u, v] }
    };
    let y = if swap { p5[1] } else { p5[0] };
    let foot = (16.0 - y * y).sqrt();
    let solid = "extrude(blade)";
    for x in [5.0, foot + 0.05] {
        assert_eq!(where_in(&before, solid, at(x)), PointClassification::Inside);
    }
    let rounded = {
        let mut program = program.clone();
        program.push("round", FilletArgs::constant(trailing.clone(), 0.1));
        program.build_model(blade_registry).unwrap()
    };
    assert_valid(&rounded);
    for x in [5.0, foot + 0.05] {
        assert_eq!(
            where_in(&rounded, "fillet(round)", at(x)),
            PointClassification::Outside,
            "{x}"
        );
    }
    assert_eq!(
        where_in(&rounded, "fillet(round)", at(foot - 0.02)),
        PointClassification::Inside
    );
    assert_rounds(program, blade_registry, solid, trailing, 0.1, true);
}

/// The blade's trailing edge, a sharp edge of 25 degrees between the
/// loft's faces through the sketches' curves `c9` and `c11`, rounded by
/// 0.1 (a user report on the turbine example): both of its pieces, from
/// the hub, which the blade is joined to, to the blade's tip.
///
/// It ends at the tip, through the loft's end cap, a plane its faces do
/// not stand square to, and at the hub, a cylinder it runs into.
#[test]
#[ignore = "slow: 25 s, the turbine's volume integrated twice — run with `cargo test -- --ignored`; `rounding_the_trailing_edge_of_a_blade_section_on_a_hub` is the fast test of its end at the hub"]
fn rounding_a_turbine_blades_trailing_edge() {
    assert_turbine_rounds(
        &[
            "circular_pattern(circular_pattern1,1,loft(loft1,sketch1,p5,sketch2>sketch3))",
            "circular_pattern(circular_pattern1,1,loft(loft1,sketch1,p5,sketch1>sketch2))",
        ],
        true,
    );
}

/// The edge round the blade's tip, between the loft's end cap and its face
/// through `c9`, rounded by 0.1 (a user report on the turbine example).
/// The tangent chain runs on round the tip's leading edge, where the faces
/// through `c9` and `c11` meet smoothly, back to the trailing edge: it
/// starts and ends at one corner, where each end leaves the solid through
/// the other end's face, curved.
#[test]
#[ignore = "slow: 20 s, the turbine's volume integrated twice — run with `cargo test -- --ignored`; `rounding_the_top_of_a_blade_section` is the fast test of the tool in halves"]
fn rounding_a_turbine_blades_tip() {
    assert_turbine_rounds(
        &["circular_pattern(circular_pattern1,1,loft(loft1,sketch1,c9,sketch3))"],
        true,
    );
}

/// The blade's root on the hub, between the loft's face through `c9` and
/// the revolved hub, rounded by 0.1 (a user report on the turbine
/// example): a concave edge, filled in. Its chain runs round the root to
/// the blade's trailing edge, a sharp edge of 25 degrees, where it starts
/// and ends: an outward corner, which each end leaves the solid at, through
/// the blade's other face there.
///
/// Refused, naming the faces at the corner. Filled up to the other face —
/// the root filled on along the face through `c9` until it meets the face
/// through `c11` carried on past the trailing edge, and alike from the
/// other side — the two ends make a notch whose edge is the trailing edge
/// itself, below the fill's top: an end of the fill on an edge that exists,
/// which no boolean can cut (it failed splicing the second end's edge into
/// the first's, at a corner `6e-5` across in `(u, v)`). What a designer
/// expects there is the fill rolled round the trailing edge, a ball
/// touching the hub and the edge itself — rolling over a crease, which the
/// blend does not do (see `crate::rolling`).
#[test]
fn rounding_a_turbine_blades_root_on_the_hub_is_refused_at_the_trailing_edge() {
    let mut program = geop_ops_pattern::examples::turbine_blade();
    let before = program.build_model(turbine_registry).unwrap();
    let root = turbine_edges(
        &before,
        "combine(circular_pattern1,1,circular_pattern(circular_pattern1,1,loft(loft1,sketch1,c9,sketch1>sketch2)),revolve(revolve1,sketch4,c10,q0),",
    );
    assert_eq!(root.len(), 1, "{root:?}");
    program.push("round", FilletArgs::constant(root, 0.1));
    let Err(error) = program.build_model(turbine_registry) else {
        panic!("the root filled round to the trailing edge is not refused");
    };
    let error = format!("{error:?}");
    assert!(
        error.contains("the edge leaves the solid there, an outward corner"),
        "{error}"
    );
    // The faces at the corner, by name: the hub and both of the blade's.
    for face in [
        "revolve(revolve1,sketch4,c10,q0) (curved)",
        "circular_pattern(circular_pattern1,1,loft(loft1,sketch1,c9,sketch1>sketch2)) (curved)",
    ] {
        assert!(error.contains(face), "{error}");
    }
}

/// `program` with the one edge whose middle is where `keep` says rounded
/// by `r` as the step `round`, as [`assert_rounds`] checks it — cut where
/// `cuts`, filled else: the part built.
fn rounded_where(
    program: Program,
    keep: impl Fn([f64; 3]) -> bool,
    r: f64,
    cuts: bool,
) -> NamedModel {
    let before = program.build_model(registry).unwrap();
    let edges = edges_with_middle(&before, keep);
    assert_eq!(edges.len(), 1, "{edges:?}");
    let [solid] = before.solid_names().try_into().unwrap();
    assert_rounds(program, registry, &solid, edges, r, cuts)
}

/// A plate standing against a round post (a bracket's arm into its hub):
/// the plate's top edge, straight between two planes, runs into the post's
/// side, a cylinder. Rounded, the blend runs on into the post, rolled, and
/// is cut off by it: the plate's corner is rounded away, the post not cut
/// into. (Swept, a straight edge between planes ended flush only with a
/// plane square to it, and this was refused.)
#[test]
fn fillet_of_a_plate_edge_running_into_a_post() {
    let mut program = Program::new();
    program.push(
        "post",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 5.0, 10.0),
    );
    program.push(
        "plate",
        PrimitiveArgs::block([-12.0, -2.0, 0.0], [0.0, 2.0, 4.0]).join("cylinder(post)"),
    );
    let part = rounded_where(
        program,
        |m| near(m[1], 2.0) && near(m[2], 4.0) && m[0] < 0.0,
        1.0,
        true,
    );
    let solid = "fillet(round)";
    let at = |p| where_in(&part, solid, p);
    assert_eq!(at([-8.0, 1.95, 3.95]), PointClassification::Outside);
    assert_eq!(at([-8.0, 1.0, 3.0]), PointClassification::Inside);
    // In the post, above the plate's corner: not cut into.
    assert_eq!(at([-4.2, 2.4, 4.3]), PointClassification::Inside);
}

/// A rib running into a wall that stands at an angle to it (a gusset into
/// a drafted side): the rib's top edge, straight between two planes, runs
/// into a plane not square to it. Rounded, rolled on into the wall and cut
/// off by it; the wall not cut into.
#[test]
fn fillet_of_a_rib_edge_running_into_an_oblique_wall() {
    let mut program = Program::new();
    program.push(
        "wall",
        PrimitiveArgs::prism(
            &[[10.0, -5.0], [20.0, -5.0], [20.0, 15.0], [14.0, 15.0]],
            0.0,
            10.0,
        ),
    );
    program.push(
        "rib",
        PrimitiveArgs::block([0.0, 4.0, 0.0], [13.0, 6.0, 3.0]).join("prism(wall)"),
    );
    let part = rounded_where(
        program,
        |m| near(m[1], 6.0) && near(m[2], 3.0) && m[0] < 12.0,
        0.8,
        true,
    );
    let solid = "fillet(round)";
    let at = |p| where_in(&part, solid, p);
    assert_eq!(at([5.0, 5.95, 2.95]), PointClassification::Outside);
    assert_eq!(at([5.0, 5.0, 2.0]), PointClassification::Inside);
    // In the wall, just past where the rib's corner meets it.
    assert_eq!(at([12.5, 6.0, 3.0]), PointClassification::Inside);
}

/// A rib on a base plate, both running into a round post: the concave
/// edge between the rib's side and the plate's top, straight between two
/// planes, runs into the post's side. Filled in, rolled on into the post
/// and cut off by it, so that the fill does not stick out of the post.
#[test]
fn fillet_of_a_rib_root_running_into_a_post() {
    let mut program = Program::new();
    program.push(
        "post",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 5.0, 10.0),
    );
    program.push(
        "base",
        PrimitiveArgs::block([-12.0, -6.0, 0.0], [0.0, 6.0, 2.0]).join("cylinder(post)"),
    );
    program.push(
        "rib",
        PrimitiveArgs::block([-12.0, -1.0, 2.0], [0.0, 1.0, 5.0]).join("block(base)"),
    );
    let part = rounded_where(
        program,
        |m| near(m[1], 1.0) && near(m[2], 2.0) && m[0] < 0.0,
        0.8,
        false,
    );
    let solid = "fillet(round)";
    let at = |p| where_in(&part, solid, p);
    assert_eq!(at([-8.0, 1.05, 2.05]), PointClassification::Inside);
    assert_eq!(at([-8.0, 1.7, 2.7]), PointClassification::Outside);
}

/// A thin plate, 0.15 thick, every edge rounded by 0.1 (a user rounding
/// "everything" on a sheet-metal-like part): the balls rounding the two
/// corners at either end of an upright edge touch it 0.1 from each end,
/// past each other. Refused, naming the corner, rather than built with
/// two corner patches overlapping. (What would build it is one patch for
/// both corners — four faces round the short edge — which is missing.)
#[test]
fn fillet_of_every_edge_of_a_plate_thinner_than_two_radii_is_refused() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [1.0, 1.0, 0.15]),
    );
    let before = program.build_model(registry).unwrap();
    let edges = edges_with_middle(&before, |_| true);
    assert_eq!(edges.len(), 12, "{edges:?}");
    program.push("round", FilletArgs::constant(edges, 0.1));
    let Err(error) = program.build_model(registry) else {
        panic!("the plate's edges rounded past each other are not refused");
    };
    let error = format!("{error:?}");
    assert!(error.contains("too large for the edge"), "{error}");
}

/// The floor of a blind hole of radius 0.4 rounded by 0.5: the ball would
/// have to touch the hole's wall from inside a circle narrower than it.
/// Refused, naming the wall and how tightly it bends, rather than failing
/// somewhere in the rolling or the boolean.
#[test]
fn fillet_wider_than_a_hole_it_rounds_the_floor_of_is_refused() {
    let mut program = geop_lib_fixtures::drilled_box();
    let before = program.build_model(registry).unwrap();
    let edges = edges_with_middle(&before, |m| {
        near(m[2], 0.5) && ((m[0] - 1.0).powi(2) + (m[1] - 1.0).powi(2)).sqrt() < 0.45
    });
    assert!(!edges.is_empty(), "no edge round the hole's floor");
    program.push("round", FilletArgs::constant(edges, 0.5));
    let Err(error) = program.build_model(registry) else {
        panic!("a fillet wider than the hole is not refused");
    };
    let error = format!("{error:?}");
    assert!(error.contains("bends more tightly"), "{error}");
}

/// A pipe of radius 0.5 along `x` with a branch of the same radius standing
/// on it along `z`: the two meet in two half ellipses, in the planes
/// `x = z` and `x = -z`, which meet where the pipes touch, at
/// `(0, ±0.5, 0)`, where their normals are one. The pipes' seams cut each
/// in two quarters at its top, which run on into each other tangentially.
/// The program, and the half ellipse in `x = z`: its two quarters.
fn equal_tee() -> (Program, Vec<String>) {
    let mut program = Program::new();
    program.push(
        "pipe",
        PrimitiveArgs::cylinder([-1.0, 0.0, 0.0], FrameAxis::X, 0.5, 2.0),
    );
    program.push(
        "branch",
        PrimitiveArgs::cylinder_z([0.0; 3], 0.5, 1.0).join("cylinder(pipe)"),
    );
    let before = program.build_model(registry).unwrap();
    assert_valid(&before);
    let ellipse = edges_with_middle(&before, |m| {
        m[0] > 0.1 && near(m[0], m[2]) && m[1].abs() > 0.1
    });
    assert_eq!(ellipse.len(), 2, "{ellipse:?}");
    (program, ellipse)
}

/// The equal tee's joint rounded along one half ellipse: the faces either
/// side become tangent at its ends, where the pipes touch, so the fillet
/// vanishes there, its width going to nothing as the ball's center runs
/// into the edge (SMLib's degenerate corner).
#[test]
fn fillet_of_an_equal_tee_vanishes_where_the_pipes_touch() {
    let (mut program, ellipse) = equal_tee();
    program.push("round", FilletArgs::constant(ellipse, 0.1));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let at = |p| where_in(&part, "fillet(round)", p);
    // At the top, where the pipes meet square, the fill reaches
    // `0.1 (sqrt 2 - 1)` out of the corner along the way halving it.
    let d = 0.02 / 2f64.sqrt();
    assert_eq!(at([0.5 + d, 0.0, 0.5 + d]), PointClassification::Inside);
    let d = 0.06 / 2f64.sqrt();
    assert_eq!(at([0.5 + d, 0.0, 0.5 + d]), PointClassification::Outside);
    // Beside where the pipes touch, nothing is filled.
    assert_eq!(at([0.0, 0.51, 0.0]), PointClassification::Outside);
    assert_eq!(at([0.0, -0.51, 0.0]), PointClassification::Outside);
}

/// The equal tee's whole joint rounded: both half ellipses, whose fills
/// vanish at the same two vertices, each into a point there.
#[test]
fn fillet_of_an_equal_tees_whole_joint() {
    let (mut program, _) = equal_tee();
    let before = program.build_model(registry).unwrap();
    let joint = edges_with_middle(&before, |m| {
        near(m[0].abs(), m[2]) && m[1].abs() > 0.1 && m[2] > 0.1
    });
    assert_eq!(joint.len(), 4, "{joint:?}");
    program.push("round", FilletArgs::constant(joint, 0.1));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let at = |p| where_in(&part, "fillet(round)", p);
    let d = 0.02 / 2f64.sqrt();
    assert_eq!(at([-0.5 - d, 0.0, 0.5 + d]), PointClassification::Inside);
    assert_eq!(at([0.5 + d, 0.0, 0.5 + d]), PointClassification::Inside);
}

/// Two equal pipes crossing square, kept where they overlap (a Steinmetz
/// solid): its edges are four half ellipses, convex, meeting four at a
/// time where the pipes touch, at `(0, ±0.5, 0)`. One of them rounded:
/// the cut vanishes at both its ends, the other edges there left sharp.
#[test]
fn fillet_of_a_bicylinder_edge_vanishes_where_the_pipes_touch() {
    let mut program = Program::new();
    program.push(
        "pipe",
        PrimitiveArgs::cylinder([-1.0, 0.0, 0.0], FrameAxis::X, 0.5, 2.0),
    );
    program.push(
        "branch",
        PrimitiveArgs::cylinder_z([0.0, 0.0, -1.0], 0.5, 2.0).combined(Combine::With {
            op: geop_ops_model::BooleanOp::Intersection,
            target: geop_ops_model::solid("cylinder(pipe)"),
        }),
    );
    let before = program.build_model(registry).unwrap();
    assert_valid(&before);
    let edge = edges_with_middle(&before, |m| {
        m[0] > 0.1 && near(m[0], m[2]) && m[1].abs() > 0.1
    });
    assert_eq!(edge.len(), 2, "{edge:?}");
    program.push("round", FilletArgs::constant(edge, 0.1));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let at = |p| where_in(&part, "fillet(round)", p);
    // Cut away out of the top corner, `0.1 (sqrt 2 - 1)` deep along the
    // way halving it.
    let d = 0.02 / 2f64.sqrt();
    assert_eq!(at([0.5 - d, 0.0, 0.5 - d]), PointClassification::Outside);
    let d = 0.06 / 2f64.sqrt();
    assert_eq!(at([0.5 - d, 0.0, 0.5 - d]), PointClassification::Inside);
}

/// A pyramid 2 across and 1.5 high, as two roofs crossed — a triangle
/// extruded along `x` and one along `y`, kept where they overlap: its four
/// faces planes, its apex where the ridges cross a corner of four faces,
/// each of its four slanted edges leaving the solid past it through the
/// two faces across. The program, and the slanted edges.
fn pyramid() -> (Program, Vec<String>) {
    let mut program = Program::new();
    // In the sketch's own coordinates: `(y, z)` on the plane square to
    // `x`, `(x, -z)` on the one square to `y`.
    let roofs = [
        (
            "along_x",
            FrameAxis::X,
            [[-1.0, 0.0], [1.0, 0.0], [0.0, 1.5]],
        ),
        (
            "along_y",
            FrameAxis::Y,
            [[-1.0, 0.0], [1.0, 0.0], [0.0, -1.5]],
        ),
    ];
    for (k, (name, axis, roof)) in roofs.into_iter().enumerate() {
        let section = format!("{name}_section");
        program.push(
            &section,
            AddSketchArgs::on(
                on_origin(axis),
                geop_ops_model::sketch::figures::outline(&roof),
            ),
        );
        let combine = if k == 0 {
            Combine::NewBody
        } else {
            Combine::With {
                op: geop_ops_model::BooleanOp::Intersection,
                target: geop_ops_model::solid("extrude(along_x)"),
            }
        };
        let mut extruded = extrude(&section, 3.0, true);
        extruded.output = Output::Solid(combine);
        program.push(name, extruded);
    }
    let before = program.build_model(blade_registry).unwrap();
    assert_valid(&before);
    let slanted = edges_with_middle(&before, |m| near(m[2], 0.75));
    assert_eq!(slanted.len(), 4, "{slanted:?}");
    (program, slanted)
}

/// A pyramid: a square 2 across on the ground lofted to a point 1.5 above
/// its middle. Its apex is a corner of four faces, each of its four
/// slanted edges leaving the solid past it through the two faces across.
/// The program, and the slanted edges.
fn lofted_pyramid() -> (Program, Vec<String>) {
    let mut program = Program::new();
    program.push("base", GivenDatumArgs::plane([0.0; 3], [0.0, 0.0, 1.0]));
    program.push(
        "square",
        AddSketchArgs::on(
            SketchPlane::Datum(ElementReference::new(datums(), "base")),
            geop_ops_model::sketch::figures::rectangle_outline(-1.0, -1.0, 1.0, 1.0),
        ),
    );
    program.push(
        "top",
        GivenDatumArgs::plane([0.0, 0.0, 1.5], [0.0, 0.0, 1.0]),
    );
    let mut tip = Sketch::new();
    tip.add_point(n(0.0), n(0.0));
    program.push(
        "tip",
        AddSketchArgs::on(
            SketchPlane::Datum(ElementReference::new(datums(), "top")),
            tip,
        ),
    );
    program.push(
        "spire",
        LoftArgs {
            profiles: vec![whole_sketch("square"), whole_sketch("tip")],
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let before = program.build_model(blade_registry).unwrap();
    assert_valid(&before);
    let slanted = edges_with_middle(&before, |m| near(m[2], 0.75));
    assert_eq!(slanted.len(), 4, "{slanted:?}");
    (program, slanted)
}

/// Two opposite slanted edges of the pyramid rounded: at the apex, each
/// fillet leaves the solid through the two faces across from it, and runs
/// out past it there, as at a corner of three faces.
#[test]
fn fillet_of_two_opposite_edges_into_a_pyramids_apex() {
    let (mut program, _) = pyramid();
    let before = program.build_model(blade_registry).unwrap();
    let opposite = edges_with_middle(&before, |m| near(m[2], 0.75) && m[0] * m[1] > 0.0);
    assert_eq!(opposite.len(), 2, "{opposite:?}");
    program.push("round", FilletArgs::constant(opposite, 0.1));
    let part = program.build_model(blade_registry).unwrap();
    assert_valid(&part);
    assert_rounded_along_the_middle(&part);
}

/// Checks `part`, the pyramid with its slanted edge through `(1, 1, 0)`
/// rounded by 0.1, is cut away halfway along it as deep as the round: its
/// faces, of normals along `(1.5, 0, 1)` and `(0, 1.5, 1)`, meet at
/// 107.9 degrees, so the ball's arc lies `0.1 (1 / sin 53.96 - 1) = 0.0237`
/// in from the edge along the way halving them.
fn assert_rounded_along_the_middle(part: &NamedModel) {
    let inward = {
        let n: [f64; 3] = [1.5, 1.5, 2.0];
        let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        n.map(|x| -x / length)
    };
    let at = |depth: f64| {
        let p = [0.5, 0.5, 0.75];
        where_in(
            part,
            "fillet(round)",
            [0, 1, 2].map(|k| p[k] + depth * inward[k]),
        )
    };
    assert_eq!(at(0.01), PointClassification::Outside);
    assert_eq!(at(0.05), PointClassification::Inside);
}

/// Every slanted edge of the pyramid rounded: no one ball touches the four
/// faces at its apex, and the corner is bevelled, each fillet running out
/// past it and the boolean meeting them.
#[test]
fn fillet_of_every_edge_into_a_pyramids_apex() {
    let (mut program, slanted) = pyramid();
    program.push("round", FilletArgs::constant(slanted, 0.1));
    let part = program.build_model(blade_registry).unwrap();
    assert_valid(&part);
    assert_rounded_along_the_middle(&part);
}

/// The lofted pyramid's slanted edges rounded, as the crossed roofs' are
/// in [`fillet_of_every_edge_into_a_pyramids_apex`]. Each fillet runs out
/// past the apex rolled on its faces carried on past it, and a lofted
/// face's apex is a pole of its parametrization: carried on, the pole row
/// is no longer at the end of the domain, where `NurbSurface::normal`
/// finds poles, but inside it, at the knot the continuation is joined on
/// at. The ball rolled on from the station at the apex is seeded there
/// (`u = 1 + 1.6e-15`), its faces' normals come out `±0.75` wide, and
/// `roll` divides by `1 + n0·n1`, which could be zero. Past the pole the
/// patch also runs inside out (`S_v` changes sign), so seeding elsewhere
/// only moves the trouble. Needed: a pole row at a knot of full
/// multiplicity found as one at the domain's end is, and the ball rolled
/// on past it with the faces' normals continued through it, as the
/// offset carries a surface through an apex (`NurbSurface3D::offset`).
#[test]
#[ignore = "a fillet run out past an apex that is a pole of its faces' parametrization: the faces carried on through the pole have no usable normal there (see the doc comment)"]
fn fillet_of_every_edge_into_a_lofted_pyramids_apex() {
    let (mut program, slanted) = lofted_pyramid();
    program.push("round", FilletArgs::constant(slanted, 0.1));
    let part = program.build_model(blade_registry).unwrap();
    assert_valid(&part);
}

/// The operations of [`registry`], and the chamfer.
fn chamfer_registry() -> GeopResult<OperationRegistry> {
    registry()?.with(crate::Chamfer)
}

/// Builds `program` in the operations of `registry` and checks the part is
/// valid.
fn assert_builds_valid_with(program: &Program, registry: fn() -> GeopResult<OperationRegistry>) {
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
}

/// A 40 x 20 x 10 block, the front edge of its top (along `x` at `y = 0`)
/// bevelled 2, then the two front upright edges, which now end on the
/// bevel, rounded by 2 (a part finished edge by edge, as a user does).
/// The fillet fails with no hint of why:
///
/// > Cannot normalize zero-length vector
///
/// at `Link::at(edge=.., fraction=0, t=0.1875)` on the curve of an edge at
/// `x = 40`, `y = 0`, `z = 8`: where the upright edge at `x = 40` meets the
/// bevel. Rounding the uprights first and bevelling after fails too, in
/// another way ([`chamfer_after_rounding_the_edges_it_runs_into`]).
#[test]
#[ignore = "bug: rounding the two upright edges of a block that end on a bevelled top edge fails with \"Cannot normalize zero-length vector\" at Link::at, where an upright meets the bevel"]
fn fillet_of_upright_edges_ending_on_a_chamfer() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 20.0, 10.0]),
    );
    let before = program.build_model(chamfer_registry).unwrap();
    let front = edges_at(&before, [20.0, 0.0, 10.0]);
    assert_eq!(front.len(), 1, "{front:?}");
    program.push("bevel", crate::ChamferArgs::even(front, 2.0));
    let bevelled = program.build_model(chamfer_registry).unwrap();
    let uprights = edges_with_middle_where(&bevelled, |m| {
        (near(m[0], 0.0) || near(m[0], 40.0)) && near(m[1], 0.0)
    });
    assert_eq!(uprights.len(), 2, "{uprights:?}");
    program.push("round", FilletArgs::constant(uprights, 2.0));
    assert_builds_valid_with(&program, chamfer_registry);
}
