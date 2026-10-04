//! Trimming a flat square with a face of a box standing across it.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_ops::{NoFiles, Part};
use geop_ops_extrude_revolve::shapes::cube_solid;

use super::*;
use crate::boundary::tests::{assert_valid, refused, v3};
use crate::thicken::tests::square_sheet;

/// The flat square at `z = 1` and a box beside it from `x = 0.5`, whose
/// face there stands across the square, its normal along `-x`; returns
/// that face's name.
fn square_and_wall() -> (Part<S>, String) {
    let (mut part, _) = square_sheet();
    let wall = cube_solid(&mut part, "w", v3(0.5, -1., 0.), v3(2., 2., 2.)).unwrap();
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
                .could_be_equal(S::ZERO)
        })
        .unwrap();
    let name = part.name_of(face).unwrap().to_string();
    (part, name)
}

fn trimmed(part: Part<S>, tool: &str, keep: TrimKeep) -> GeopResult<Part<S>> {
    let args = TrimSurfaceArgs {
        face: "boundary(b)".into(),
        tool: tool.into(),
        keep,
    };
    TrimSurface.apply(part, "t", &args, &NoFiles)
}

/// The square cut back to either side of the box's face: the half behind
/// it, `x >= 0.5`, or the half in front, `x <= 0.5`; the box stays.
#[test]
fn trim_a_square_with_a_face() {
    for (keep, from, to) in [(TrimKeep::Back, 0.5, 1.0), (TrimKeep::Front, 0.0, 0.5)] {
        let (part, tool) = square_and_wall();
        let part = trimmed(part, &tool, keep).unwrap();
        assert_valid(&part);
        assert_eq!(part.sheet_face_names().len(), 1, "{keep:?}");
        let face = part.face_id(&part.sheet_face_names()[0]).unwrap();
        for c in part.topology().iterate_face_coedges(face) {
            let x = part.topology().coedge_start_vertex(c).unwrap().point[0];
            assert!(
                !x.definitely_less(S::from_f64(from)) && !x.definitely_greater(S::from_f64(to)),
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
    let far = cube_solid(&mut part, "far", v3(3., 3., 3.), v3(4., 4., 4.)).unwrap();
    let face = part.topology().solid_faces(far).unwrap()[0];
    let face = part.name_of(face).unwrap().to_string();
    let err = refused(trimmed(part, &face, TrimKeep::Front));
    assert!(err.contains("does not cut the sheet"), "{err}");
}
