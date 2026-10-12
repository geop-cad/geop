//! Building blocks for the forms of operations that this crate's types
//! appear in: a number field of a [`Formula`], a field picking sketches, a
//! select field over a table of options, a filled region, how a sketch's
//! constraints stand.

use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar, vector::Vector2};
use geop_core_program::editing::{Choice, Number, Shape, Tone, Unit};

use crate::{operation::Lookup, parameters::Formula};

/// A number field of `formula`, evaluated with the parameters `lookup`
/// reads, to show what it comes to — or, where it does not evaluate, why.
///
/// A value that follows a formula is neither slid nor dragged: moving it
/// would have to either break the formula or move the parameters it reads,
/// and neither is what a drag means. So such a field has no slider and no
/// handle ([`Number::range`] and [`Number::handle`] keep none); to drag the
/// value again, type a number.
pub fn formula_number<S: Scalar>(
    label: impl Into<String>,
    formula: &Formula,
    lookup: &mut (impl Lookup<S> + ?Sized),
    unit: Unit,
) -> Number<S> {
    let (value, error) = match formula.evaluate(lookup) {
        Ok(value) => (value, None),
        Err(e) => (0.0, Some(e.root_message().to_string())),
    };
    Number {
        text: Some(formula.to_string()),
        error,
        ..Number::new(label, value, unit)
    }
}

/// A select field's table of options, each `(option, value, label)`: the
/// value it gives `chosen`, and the choices it offers.
pub fn choices<T: PartialEq>(options: &[(T, &str, &str)], chosen: &T) -> (String, Vec<Choice>) {
    let value = (options.iter().find(|o| o.0 == *chosen)).map_or("", |o| o.1);
    let choices = (options.iter()).map(|&(_, v, l)| Choice::new(v, l));
    (value.to_string(), choices.collect())
}

/// The option of a select field's table `options` (see [`choices`]) that
/// has the value `value`.
pub fn chosen<T: Copy>(options: &[(T, &str, &str)], value: &str) -> Option<T> {
    options.iter().find(|o| o.1 == value).map(|o| o.0)
}

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
