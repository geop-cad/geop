//! Surfaces built by programs: the walls an extrude or a loft sweeps as
//! faces, capped with boundary surfaces, knit into solids, thickened.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_sketch::PointId;
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, LoftArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};
use geop_ops_surface::{BoundarySurfaceArgs, KnitArgs, ThickenArgs, ThickenSide};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
    part.check_names().unwrap();
}

fn z_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z))
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
fn sheet_edges_at(part: &Part<S>, z: f64) -> Vec<String> {
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
            edges,
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
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: polygon(&[[0., 0.], [2., 0.], [2., 1.], [0., 1.]]),
            ..Default::default()
        },
    );
    program.push(
        "walls",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: true,
            combine: Combine::NewBody,
        },
    );
    program
}

/// The four extruded walls, capped at the bottom and the top with boundary
/// surfaces, knit into a box: six faces, a solid.
#[test]
fn extruded_faces_knit_into_a_box() {
    let mut program = walls();
    let part = program.build::<S>(&NoFiles).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "bottom", sheet_edges_at(&part, 0.0));
    fill(&mut program, "top", sheet_edges_at(&part, 1.0));
    program.push(
        "box",
        KnitArgs {
            faces: vec![wall, "boundary(bottom)".into(), "boundary(top)".into()],
            solid: true,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    let part = program.build::<S>(&NoFiles).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "top", sheet_edges_at(&part, 1.0));
    program.push(
        "cover",
        KnitArgs {
            faces: vec![wall.clone(), "boundary(top)".into()],
            solid: false,
        },
    );
    program.push(
        "thick",
        ThickenArgs {
            face: wall,
            thickness: 0.1,
            side: ThickenSide::Against,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: polygon(&[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]),
            ..Default::default()
        },
    );
    program.push(
        "top_plane",
        AddDatumArgs {
            selection: vec![z_plane()],
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
        AddSketchArgs {
            plane: Some(EntityRef::datum("top_plane")),
            sketch: circle,
            ..Default::default()
        },
    );
    program.push(
        "transition",
        LoftArgs {
            profiles: vec!["bottom".into(), "top".into()],
            matches: Vec::new(),
            guides: Vec::new(),
            face: true,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    let wall = part.sheet_face_names()[0].clone();
    fill(&mut program, "base", sheet_edges_at(&part, 0.0));
    fill(&mut program, "lid", sheet_edges_at(&part, 2.0));
    program.push(
        "solid",
        KnitArgs {
            faces: vec![wall, "boundary(base)".into(), "boundary(lid)".into()],
            solid: true,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("knit(solid)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 4 + 2);
}
