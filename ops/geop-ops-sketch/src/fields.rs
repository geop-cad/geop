//! Building blocks of the forms of what is drawn in a sketch, here and in a
//! 3-D sketch: a filled region, how a sketch's constraints stand.

use geop_core_editor::editing::{Shape, Tone};
use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar, vector::Vector2};

/// The area of `plane`'s `u`/`v` plane inside `outer` but outside `holes`,
/// as triangles — to draw a region filled. No triangles for loops that do
/// not triangulate.
pub fn region<S: Scalar>(
    plane: &CoordinateSystem<S>,
    outer: &[Vector2<S>],
    holes: &[Vec<Vector2<S>>],
) -> Shape<S> {
    let plain = |loop_: &[Vector2<S>]| -> Vec<[f64; 2]> {
        loop_
            .iter()
            .map(|p| [p[0].to_f64(), p[1].to_f64()])
            .collect()
    };
    let holes: Vec<Vec<[f64; 2]>> = holes.iter().map(|h| plain(h)).collect();
    let at = |p: [f64; 2]| plane.uv_to_xyz(&Vector2::from_array(p.map(S::from_f64)));
    let triangles = geop_ops_rasterize::triangulate_polygon(&plain(outer), &holes)
        .into_iter()
        .map(|t| t.map(at))
        .collect();
    Shape::Triangles { triangles }
}

/// How a sketch's constraints stand, followed by what it draws, `drawn`:
/// how many conflict, unless all hold (`converged`), and then whether the
/// sketch is fully constrained or how many degrees of freedom it has left.
pub fn constraint_status(
    converged: bool,
    conflicts: usize,
    dof: usize,
    drawn: &str,
) -> (String, Tone) {
    let plural = |n: usize| if n == 1 { "" } else { "s" };
    if !converged {
        let text = format!(
            "{conflicts} constraint{} conflict · {drawn}",
            plural(conflicts)
        );
        (text, Tone::Error)
    } else if dof == 0 {
        (format!("Fully constrained · {drawn}"), Tone::Success)
    } else {
        (
            format!("{dof} degree{} of freedom · {drawn}", plural(dof)),
            Tone::Hint,
        )
    }
}
