//! Extending a flat square and a Coons patch past their edges.

use geop_core_math::scalars::Scalar;
use geop_lib_sweep::shapes::block_part;
use geop_ops_model::NamedModel;
use geop_ops_model::testing::apply_to_model;

use super::*;
use crate::boundary::tests::{coons_sheet, edge_between, line, refused, rising_arc, wavy_top};
use crate::test_refs::*;
use crate::thicken::tests::square_sheet;
use geop_ops_model::testing::assert_valid_sheets;

fn extended(part: NamedModel, edge: &str, distance: f64) -> GeopResult<NamedModel> {
    let args = ExtendSurfaceArgs {
        edge: named(edge),
        distance,
    };
    apply_to_model::<ExtendSurface>(part, "x", &args)
}

/// A flat square at `z = 1`, the Coons patch of its sides, carried on half
/// a unit past its edge along `y = 1`: a rectangle reaching `y = 1.5`,
/// every name kept.
#[test]
fn extend_a_flat_square() {
    let mut part = NamedModel::new();
    let c = [[0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    coons_sheet(
        &mut part,
        "sq",
        [0, 1, 2, 3].map(|k| line(c[k], c[(k + 1) % 4])),
    );
    let mut names_before: Vec<String> = part.names().iter().map(|(_, n)| n.to_string()).collect();
    let part = extended(part, "sq(e2)", 0.5).unwrap();
    assert_valid_sheets(&part);
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
    let mut part = NamedModel::new();
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
    assert_valid_sheets(&part);
    // The side past which it grew is the same quarter circle, moved out
    // along the patch's `u`, 0.3 on across its middle; the bottom edge
    // reaches further too, as far as the same parameter carries it.
    let bottom = part.edge_id("guide(e0)").unwrap();
    let edge = part.topology().get_edge(bottom).unwrap();
    let end = part.topology().get_vertex(edge.end_vertex).unwrap().point;
    assert!(end[0].definitely_greater(Scalar::from_f64(1.1)), "{end:?}");
}

/// A flat square whose parametrization speeds up across its edge at
/// `x = 1` — its bottom and top run `x = 0.2 u + 0.8 u²` — carried on half
/// a unit past that edge: the edge lands at `x = 1.5`, as far along the
/// surface as asked, not where the speed at the edge would have put it.
#[test]
fn extend_past_an_edge_the_surface_speeds_up_across() {
    use geop_core_math::vector::Vector4;
    let quadratic = |y: f64, reversed: bool| {
        let p = |x: f64| Vector4::from_array([x, y, 0.0, 1.0].map(Scalar::from_f64));
        let (zero, one) = (Scalar::ZERO, Scalar::ONE);
        let curve = geop_core_geometry::nurb_curve::NurbCurve3D::try_new(
            2,
            vec![p(0.0), p(0.1), p(1.0)],
            vec![zero, zero, zero, one, one, one],
        )
        .unwrap();
        if reversed { curve.reverse() } else { curve }
    };
    let mut part = NamedModel::new();
    coons_sheet(
        &mut part,
        "sq",
        [
            quadratic(0.0, false),
            line([1., 0., 0.], [1., 1., 0.]),
            quadratic(1.0, true),
            line([0., 1., 0.], [0., 0., 0.]),
        ],
    );
    let part = extended(part, "sq(e1)", 0.5).unwrap();
    assert_valid_sheets(&part);
    let edge = part.edge_id("sq(e1)").unwrap();
    let edge = part.topology().get_edge(edge).unwrap();
    for v in [edge.start_vertex, edge.end_vertex] {
        let x = part.topology().get_vertex(v).unwrap().point[0].to_f64();
        assert!((x - 1.5).abs() < 1e-12, "{x}");
    }
}

/// A cube's edge bounds two faces: no face standing on its own to extend.
#[test]
fn a_solids_edge_is_refused() {
    let part = block_part("c", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let edge = edge_between(&part, [0., 0., 1.], [1., 0., 1.]);
    let err = refused(extended(part, &edge, 0.2));
    assert!(err.contains("bounds 2 faces"), "{err}");
}
