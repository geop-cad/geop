//! Extending a flat square and a Coons patch past their edges.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_ops::{NoFiles, Part};
use geop_ops_extrude_revolve::shapes::cube_solid;

use super::*;
use crate::boundary::tests::{
    assert_valid, coons_sheet, edge_between, line, refused, rising_arc, v3, wavy_top,
};
use crate::thicken::tests::square_sheet;

fn extended(part: Part<S>, edge: &str, distance: f64) -> GeopResult<Part<S>> {
    let args = ExtendSurfaceArgs {
        edge: edge.into(),
        distance,
    };
    ExtendSurface.apply(part, "x", &args, &NoFiles)
}

/// A flat square at `z = 1`, the Coons patch of its sides, carried on half
/// a unit past its edge along `y = 1`: a rectangle reaching `y = 1.5`,
/// every name kept.
#[test]
fn extend_a_flat_square() {
    let mut part = Part::<S>::new();
    let c = [[0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    coons_sheet(&mut part, "sq", [0, 1, 2, 3].map(|k| line(c[k], c[(k + 1) % 4])));
    let mut names_before: Vec<String> = part.names().iter().map(|(_, n)| n.to_string()).collect();
    let part = extended(part, "sq(e2)", 0.5).unwrap();
    assert_valid(&part);
    assert_eq!(edge_between(&part, [1., 1.5, 1.], [0., 1.5, 1.]), "sq(e2)");
    assert_eq!(edge_between(&part, [0., 0., 1.], [1., 0., 1.]), "sq(e0)");
    let mut names_after: Vec<String> = part.names().iter().map(|(_, n)| n.to_string()).collect();
    names_before.sort();
    names_after.sort();
    assert_eq!(names_before, names_after);
}

/// A flat boundary surface is trimmed by its loop, a hair inside the box
/// its plane spans: not its patch's four sides, so not extended.
#[test]
fn a_trimmed_face_is_refused() {
    let (part, _) = square_sheet();
    let edge = edge_between(&part, [1., 1., 1.], [0., 1., 1.]);
    let err = refused(extended(part, &edge, 0.5));
    assert!(err.contains("four sides"), "{err}");
}

/// A Coons patch rising between two quarter circles to a wavy top,
/// carried on past one of its circles: it goes on as it curved.
#[test]
fn extend_a_coons_patch() {
    let mut part = Part::<S>::new();
    coons_sheet(
        &mut part,
        "guide",
        [
            line([0., 0., 0.], [1., 0., 0.]),
            rising_arc(1.),
            wavy_top(),
            rising_arc(0.).reverse(),
        ],
    );
    let part = extended(part, "guide(e1)", 0.3).unwrap();
    assert_valid(&part);
    // The side past which it grew is the same quarter circle, moved out
    // along the patch's `u`; the bottom edge reaches further.
    let bottom = part.edge_id("guide(e0)").unwrap();
    let edge = part.topology().get_edge(bottom).unwrap();
    let end = part.topology().get_vertex(edge.end_vertex).unwrap().point;
    assert!(end[0].definitely_greater(S::from_f64(1.15)), "{end:?}");
}

/// A cube's edge bounds two faces: no face standing on its own to extend.
#[test]
fn a_solids_edge_is_refused() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "c", v3(0., 0., 0.), v3(1., 1., 1.)).unwrap();
    let edge = edge_between(&part, [0., 0., 1.], [1., 0., 1.]);
    let err = refused(extended(part, &edge, 0.2));
    assert!(err.contains("bounds 2 faces"), "{err}");
}
