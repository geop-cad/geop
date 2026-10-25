//! Surfaces built by programs: the walls an extrude or a loft sweeps as
//! faces, capped with boundary surfaces, knit into solids, thickened.

use crate::{
    BoundarySurface, BoundarySurfaceArgs, Knit, KnitArgs, Thicken, ThickenArgs, ThickenSide,
    test_refs::{edge_curve, named},
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use geop_core_sketch::PointId;
use geop_ops_datums::{ORIGIN, component_name, datums};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, LoftArgs, Output};
use geop_ops_model::{
    NamedModel,
    testing::{Built, assert_valid},
};
use geop_ops_reference_geometry::{AddDatum, AddDatumArgs, Construction, Selected};
use geop_ops_sketch::{
    AddSketchArgs, Sketch, SketchPlane,
    figures::{n, on_origin},
    whole_sketch,
};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(AddDatum)?
        .with(BoundarySurface)?
        .with(Knit)?
        .with(Thicken)
}

fn z_plane() -> SketchPlane {
    on_origin(FrameAxis::Z)
}

fn polygon(corners: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<PointId> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

/// The names of the edges of `part`'s sheets whose ends are both at
/// height `z`.
fn sheet_edges_at(part: &NamedModel<Design>, z: f64) -> Vec<String> {
    let model = part.topology();
    let at = |v| model.get_vertex(v).unwrap().point[2].could_be_equal(S::from_f64(z));
    let mut names: Vec<String> = part
        .sheet_face_names()
        .iter()
        .flat_map(|f| model.iterate_face_coedges(part.face_id(f).unwrap()))
        .filter_map(|c| model.get_coedge(c).unwrap().edge().ok())
        .filter(|&e| {
            let edge = model.get_edge(e).unwrap();
            at(edge.start_vertex) && at(edge.end_vertex)
        })
        .map(|e| part.name_of(e).unwrap().to_string())
        .collect();
    names.sort();
    names.dedup();
    names
}

fn fill(program: &mut Program, id: &str, edges: Vec<String>) {
    program.push(
        id,
        BoundarySurfaceArgs {
            edges: edges.into_iter().map(edge_curve).collect(),
            tangent: Vec::new(),
        },
    );
}

/// A rectangle's outline extruded as faces: four walls standing on their
/// own, open at top and bottom.
fn walls() -> Program {
    let mut program = Program::new();
    program.push(
        "outline",
        AddSketchArgs::on(
            z_plane(),
            polygon(&[[0., 0.], [2., 0.], [2., 1.], [0., 1.]]),
        ),
    );
    program.push(
        "walls",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(1.0),
            output: Output::Faces { up_to: None },
        },
    );
    program
}

/// The four extruded walls, capped at the bottom and the top with boundary
/// surfaces, knit into a box: six faces, a solid.
#[test]
fn extruded_faces_knit_into_a_box() {
    let mut program = walls();
    let part = program.build_model(registry).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "bottom", sheet_edges_at(&part, 0.0));
    fill(&mut program, "top", sheet_edges_at(&part, 1.0));
    program.push(
        "box",
        KnitArgs {
            faces: vec![
                named(wall),
                named("boundary(bottom)"),
                named("boundary(top)"),
            ],
            solid: true,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("knit(box)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 6);
    assert!(part.sheet_face_names().is_empty());
}

/// The walls with only a top, knit into an open box — a cover — and
/// thickened into a solid one a tenth thick.
#[test]
fn open_box_thickened_into_a_cover() {
    let mut program = walls();
    let part = program.build_model(registry).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "top", sheet_edges_at(&part, 1.0));
    program.push(
        "cover",
        KnitArgs {
            faces: vec![named(&wall), named("boundary(top)")],
            solid: false,
        },
    );
    program.push(
        "thick",
        ThickenArgs {
            face: named(wall),
            thickness: 0.1,
            side: ThickenSide::Against,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("thicken(thick)").unwrap();
    // Five faces, their inner copies, and a wall along each of the four
    // open edges at the bottom.
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 5 + 5 + 4);
}

/// A square lofted into a circle as faces, capped with a flat square and a
/// flat disc, knit into a closed solid.
#[test]
fn lofted_faces_knit_into_a_solid() {
    let mut program = Program::new();
    program.push(
        "bottom",
        AddSketchArgs::on(
            z_plane(),
            polygon(&[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]),
        ),
    );
    program.push(
        "top_plane",
        AddDatumArgs {
            selection: vec![Selected::Datum(ElementReference::new(
                datums(),
                component_name(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            ))],
            construction: Construction::Offset {
                distance: 2.0.into(),
            },
        },
    );
    let mut circle = Sketch::new();
    let c = circle.add_point(n(0.0), n(0.0));
    circle.add_circle(c, n(0.6));
    program.push(
        "top",
        AddSketchArgs::on(
            SketchPlane::Datum(ElementReference::new(datums(), "top_plane")),
            circle,
        ),
    );
    program.push(
        "transition",
        LoftArgs {
            profiles: vec![whole_sketch("bottom"), whole_sketch("top")],
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::Faces { up_to: None },
        },
    );
    let part = program.build_model(registry).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "base", sheet_edges_at(&part, 0.0));
    fill(&mut program, "lid", sheet_edges_at(&part, 2.0));
    program.push(
        "solid",
        KnitArgs {
            faces: vec![named(wall), named("boundary(base)"), named("boundary(lid)")],
            solid: true,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("knit(solid)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 4 + 2);
}

/// What the examples are written in: sketches, 3-D sketches, a network
/// surface, a block extruded and split along it.
fn example_registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?)
        .with(geop_ops_sketch3d::AddSketch3d)?
        .with(crate::NetworkSurface)?
        .with(geop_ops_boolean::Split)
}

/// The curved panel is a valid face, through the middle of its middle
/// rib and spine, where they cross; it splits the block under it in
/// two, a point above the saddle in one and one below it in the other.
/// Saved and loaded, the example builds what it built.
#[test]
fn curved_panel_round_trips() {
    use geop_core_topology::contains::shell::PointClassification::Inside;
    use geop_ops_model::testing::classify;

    let registry = std::rc::Rc::new(example_registry().unwrap());
    let program = crate::examples::curved_panel();
    let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
    let part = geop_ops_model::testing::model_of(&built);
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 2, "{:?}", part.solid_names());
    for (p, expected) in [
        ([20.0, 10.0, 15.0], [true, false]),
        ([20.0, 2.0, 15.0], [false, true]),
    ] {
        let inside =
            ["split(shaped,0)", "split(shaped,1)"].map(|solid| classify(&part, solid, p) == Inside);
        assert!(
            inside == expected || inside == [expected[1], expected[0]],
            "at {p:?}: {inside:?}"
        );
    }
    let face = part.face_id("network(panel)").unwrap();
    let surface = &part.topology().get_face(face).unwrap().surface;
    let middle = geop_core_math::vector::Vector3::from_array([20.0, 6.0, 15.0].map(S::from_f64));
    assert!(
        geop_core_geometry::contains::surface::surface_could_contain(
            surface,
            &middle,
            10_000,
            S::from_f64(1e-7)
        )
        .unwrap()
        .is_some()
    );
}

/// The example `curved_panel`, edited as a user edits it, builds
/// incrementally what it builds from scratch.
#[test]
fn curved_panel_builds_incrementally() {
    let registry = std::rc::Rc::new(example_registry().unwrap());
    let programs = geop_core_program::testing::edit_sequence(&crate::examples::curved_panel());
    geop_core_program::testing::assert_incremental(registry, None, &programs);
}
