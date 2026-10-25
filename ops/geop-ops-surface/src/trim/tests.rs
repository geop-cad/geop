//! Trimming a flat square with a face of a box standing across it.

use geop_core_math::scalars::Scalar;
use geop_lib_sweep::shapes::block;
use geop_ops_model::NamedModel;
use geop_ops_model::testing::apply_to_model;

use super::*;
use crate::boundary::tests::refused;
use crate::test_refs::*;
use crate::thicken::tests::square_sheet;
use geop_lib_sweep::shapes::v3;
use geop_ops_model::testing::assert_valid_sheets;

/// The flat square at `z = 1` and a box beside it from `x = 0.5`, whose
/// face there stands across the square, its normal along `-x`; returns
/// that face's name.
fn square_and_wall() -> (NamedModel, String) {
    let (mut part, _) = square_sheet();
    let wall = block(&mut part, "w", [0.5, -1.0, 0.0], [2.0, 2.0, 2.0]);
    let face = part
        .topology()
        .solid_faces(wall)
        .unwrap()
        .into_iter()
        .find(|&f| {
            let surface = &part.topology().get_face(f).unwrap().surface;
            let plane = surface.as_plane().unwrap().unwrap();
            plane
                .signed_distance(&v3(0.5, 0., 0.))
                .could_be_equal(Scalar::ZERO)
        })
        .unwrap();
    let name = part.name_of(face).unwrap().to_string();
    (part, name)
}

fn trimmed(part: NamedModel, tool: &str, keep: TrimKeep) -> GeopResult<NamedModel> {
    let args = TrimSurfaceArgs {
        face: named("boundary(b)"),
        tool: named(tool),
        keep,
    };
    apply_to_model::<TrimSurface>(part, "t", &args)
}

/// The square cut back to either side of the box's face: the half behind
/// it, `x >= 0.5`, or the half in front, `x <= 0.5`; the box stays.
#[test]
fn trim_a_square_with_a_face() {
    for (keep, from, to) in [(TrimKeep::Back, 0.5, 1.0), (TrimKeep::Front, 0.0, 0.5)] {
        let (part, tool) = square_and_wall();
        let part = trimmed(part, &tool, keep).unwrap();
        assert_valid_sheets(&part);
        assert_eq!(part.sheet_face_names().len(), 1, "{keep:?}");
        let face = part.face_id(&part.sheet_face_names()[0]).unwrap();
        for c in part.topology().iterate_face_coedges(face) {
            let x = part.topology().coedge_start_vertex(c).unwrap().point[0];
            assert!(
                !x.definitely_less(Scalar::from_f64(from))
                    && !x.definitely_greater(Scalar::from_f64(to)),
                "{keep:?}: {x:?}"
            );
        }
        assert_eq!(part.solid_names().len(), 1);
    }
}

/// A face that misses the sheet cuts nothing.
#[test]
fn a_face_that_misses_is_refused() {
    let (mut part, _) = square_sheet();
    let far = block(&mut part, "far", [3.0, 3.0, 3.0], [4.0, 4.0, 4.0]);
    let face = part.topology().solid_faces(far).unwrap()[0];
    let face = part.name_of(face).unwrap().to_string();
    let err = refused(trimmed(part, &face, TrimKeep::Front));
    assert!(err.contains("does not cut the sheet"), "{err}");
}
