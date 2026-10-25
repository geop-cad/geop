//! Measurements from bug reports, rebuilt in the given solids of
//! `geop-lib-fixtures`.

use geop_core_editor::view::drawn;
use geop_core_program::{element::PickedElement, program::Program};
use geop_lib_fixtures::{PrimitiveArgs, registry};
use geop_ops_model::{model, testing::Built};

use crate::measure;

/// A pick of the element of the model named `name`, of what is drawn as
/// `kind`.
fn pick(kind: &str, name: &str) -> PickedElement {
    PickedElement {
        target: model().location().clone(),
        kind: kind.to_string(),
        name: name.to_string(),
    }
}

/// A box whose blind hole is drilled by a cylinder standing out of its top:
/// the hole's rim is then traced by the boolean, a fitted curve whose
/// control points are as wide as the trace, not the tool's own arcs. The
/// distance from the top to the bottom is one; the search over the top's
/// edges once split the rim's spans until their domain was a point.
#[test]
fn top_with_a_traced_rim_measures_to_the_bottom() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 1.0, 0.5], 0.4, 1.0).cut("block(box)"),
    );
    let part = program.built(registry).unwrap();
    let top = pick(drawn::FACE, "block(box,end)");
    let bottom = pick(drawn::FACE, "block(box,start)");
    let measured = measure(&part.current(), &[top, bottom]);
    assert!(measured.error.is_none(), "{measured:?}");
    let distance = measured
        .values
        .iter()
        .find(|m| m.label == "Distance")
        .unwrap()
        .value;
    assert!(distance.contains(1.0), "{distance:?}");
}
