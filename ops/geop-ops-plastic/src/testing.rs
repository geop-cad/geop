//! The enclosure the tests of plastic features are made on: a box shelled
//! open at its top, and a sketch to grow a rib from. Only built for tests:
//! with `testing`, or in this crate's own.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::{element::ElementReference, program::Program};
use geop_ops_datums::datums;
use geop_ops_extrude_revolve::{Extents, testing::extrusion};
use geop_ops_model::{Combine, model, solid};
use geop_ops_reference_geometry::{AddDatumArgs, Construction, Selected};
use geop_ops_shell::ShellArgs;
use geop_ops_sketch::{
    AddSketchArgs, Sketch, SketchPlane,
    figures::{n, on_origin, rectangle_outline},
};

/// The 2 x 2 x 1 box `box`: its walls `extrude(box,outline,c4)` (towards
/// `-y`), `c5` (`+x`), `c6` (`+y`) and `c7` (`-x`), its bottom
/// `extrude(box,start)` and top `extrude(box,end)`.
pub fn boxed() -> Program {
    let mut program = Program::new();
    let outline = rectangle_outline(0.0, 0.0, 2.0, 2.0);
    program.push(
        "outline",
        AddSketchArgs::on(on_origin(FrameAxis::Z), outline),
    );
    program.push(
        "box",
        extrusion("outline", Extents::blind(1.0), Combine::NewBody),
    );
    program
}

/// The wall of [`boxed`] the side `c` of its outline makes.
pub fn wall(c: &str) -> String {
    format!("extrude(box,outline,{c})")
}

/// The box, shelled 0.2 thick open at its top: the solid `shell(s)`, its
/// inner walls `shell(s,extrude(box,outline,c4))` and so on, its rim
/// `shell(s,extrude(box,end))`.
pub fn enclosure() -> Program {
    let mut program = boxed();
    program.push(
        "s",
        ShellArgs {
            solid: solid("extrude(box)"),
            faces: vec![ElementReference::new(model(), "extrude(box,end)")],
            thickness: 0.2.into(),
        },
    );
    program
}

/// The enclosure with a reference plane through its middle, `middle`, at
/// `y = 1` — offset from the `-y` wall, whose plane runs `u` along `x` and
/// `v` along `z` — and on it the sketch `rib_sketch` of a line across the
/// cavity at height 0.7, from `x = 0.5` to `x = 1.5`.
pub fn enclosure_with_rib_sketch() -> Program {
    let mut program = enclosure();
    program.push(
        "middle",
        AddDatumArgs {
            selection: vec![Selected::Face(ElementReference::new(model(), wall("c4")))],
            construction: Construction::Offset {
                distance: (-1.0).into(),
            },
        },
    );
    let mut s = Sketch::new();
    let a = s.add_point(n(0.5), n(0.7));
    let b = s.add_point(n(1.5), n(0.7));
    s.add_line(a, b);
    program.push(
        "rib_sketch",
        AddSketchArgs::on(
            SketchPlane::Datum(ElementReference::new(datums(), "middle")),
            s,
        ),
    );
    program
}
