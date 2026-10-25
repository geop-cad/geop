//! Test-only steps that give a program what an operation's tests build on
//! without sketching and extruding it: a solid at given coordinates
//! ([`Primitive`]: a block, a prism, a cylinder or a sphere, kept as a new
//! body or combined with a solid of the part) and a datum at given
//! coordinates ([`GivenDatum`]). Both are serializable, so a program in an
//! editor's timeline can hold them, and a part the front end clicks on can
//! be built from them.
//!
//! An operation's tests take these as a dev-dependency and register them
//! in the registry of their own programs ([`registry`]) or of their editor
//! ([`register`]). No editor offers them: a change to the sketch or the
//! extrude operation then retests those crates and the composition, not
//! every crate whose tests start from a block.
//!
//! [`bodies`] has the programs the stress sweeps run over, and
//! [`drilled_box`] the box the hand-written tests cut, round and inspect.

// Helpers of tests fail as tests do: by panicking, naming what is wrong.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

pub mod bodies;
mod given_datum;
mod primitive;

pub use given_datum::{GivenDatum, GivenDatumArgs};
pub use primitive::{Primitive, PrimitiveArgs, Shape};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_ops_model::Combine;
use geop_ops_model::parameters::{DefineParameters, LookupTable, PartProperties};

/// The operations a test's programs are written in, besides those it
/// tests: the part's parameters and properties, and the given solids and
/// datums.
pub fn registry() -> GeopResult<OperationRegistry> {
    OperationRegistry::default()
        .with(DefineParameters)?
        .with(LookupTable)?
        .with(PartProperties)?
        .with(Primitive)?
        .with(GivenDatum)
}

/// Adds the given solids and datums to `r`, for an editor a test drives
/// over a part built from them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Primitive)?.operation(GivenDatum)
}

/// A 2 x 2 x 1 box with a blind hole of radius 0.4 drilled half way into
/// its top, around its middle: the block `box` (the solid `block(box)`,
/// its top `block(box,end)`) and the cylinder `hole` cut from it, which
/// leaves the solid `cylinder(hole)`. The tool ends flush with the top, as
/// a disc sketched on it and cut back into the box does, so the rim is the
/// tool's own exact arcs.
///
/// The cylinder's top is flush with the box's, as the cap of an extrusion
/// cut from the top face is: the hole's rim is then the cylinder's own
/// arcs. A cylinder standing out of the top would have its rim traced by
/// the boolean instead, a fitted curve no drawing sees as a circle, and
/// whose spans `least_distance` splits until their domain is a point.
pub fn drilled_box() -> Program {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 1.0, 0.5], 0.4, 0.5).combined(Combine::cut("block(box)")),
    );
    program
}

/// A plate 8 x 2 x 0.5 on the origin with a hole of radius 0.3 at (1, 1)
/// through it: the block `plate` (the solid `block(plate)`, its top
/// `block(plate,end)`) and the cylinder `hole` cut through it, which
/// leaves the solid `cylinder(hole)` — the feature `hole` a pattern of
/// features copies.
pub fn drilled_plate() -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [8.0, 2.0, 0.5]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 1.0, -1.0], 0.3, 2.5).cut("block(plate)"),
    );
    program
}

/// The [`drilled_box`] with its hole cut flush from the top: the cylinder
/// `hole` ends in the top face `block(box,end)` instead of piercing it, so
/// the hole's rim on top is the cylinder's own quarter arcs — a circle a
/// mate takes the axis of, a route runs down — where a pierced top's rim
/// is a traced intersection curve, which is no arc. Same steps, solids
/// and geometry as [`drilled_box`].
pub fn drilled_box_cut_flush() -> Program {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 1.0, 0.5], 0.4, 0.5).cut("block(box)"),
    );
    program
}

#[cfg(test)]
mod tests {
    use std::rc::Rc;

    use super::*;
    use geop_core_math::primitives::{DatumKind, FrameAxis};
    use geop_core_math::scalars::Scalar;
    use geop_core_program::testing::assert_round_trips;
    use geop_core_topology::contains::shell::PointClassification;
    use geop_ops_model::datums::read_datums;
    use geop_ops_model::testing::{
        Built, assert_valid, assert_volume, classify, face_towards, model_of,
    };

    /// The drilled box is a valid solid named by its last step, with the
    /// hole's volume missing, and the program reads back as written.
    #[test]
    fn drilled_box_is_a_box_less_a_blind_hole() {
        let program = drilled_box();
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        assert_eq!(part.solid_names(), ["cylinder(hole)"]);
        let hole = std::f64::consts::PI * 0.4 * 0.4 * 0.5;
        assert_volume(&part, "cylinder(hole)", 4.0 - hole, 0.01);
        assert_eq!(
            classify(&part, "cylinder(hole)", [1.0, 1.0, 0.75]),
            PointClassification::Outside
        );
        assert_eq!(
            classify(&part, "cylinder(hole)", [1.0, 1.0, 0.25]),
            PointClassification::Inside
        );
        // The rim is the tool's own four quarter arcs, and the hole's floor
        // a plane: what a disc sketched on the top and cut back gives.
        let model = part.topology();
        let rim = model
            .edges
            .values()
            .filter(|e| {
                e.curve
                    .as_arc()
                    .unwrap()
                    .is_some_and(|arc| arc.circle.center[2].could_be_equal(Scalar::from_f64(1.0)))
            })
            .count();
        assert_eq!(rim, 4);
        let floors: Vec<&str> = model
            .faces
            .iter()
            .filter(|(_, f)| {
                f.surface.as_plane().unwrap().is_some_and(|plane| {
                    plane.normal[2].could_be_equal(Scalar::ONE)
                        && plane.point[2].could_be_equal(Scalar::from_f64(0.5))
                })
            })
            .filter_map(|(&id, _)| part.name_of(id))
            .collect();
        assert_eq!(floors, ["cylinder(hole,start)"]);
        assert_round_trips(Rc::new(registry().unwrap()), None, &program);
    }

    /// Every shape on its own is a valid solid of the expected volume,
    /// whichever axis a cylinder stands along.
    #[test]
    fn every_shape_builds_valid_with_its_volume() {
        let pi = std::f64::consts::PI;
        let shapes = [
            (
                "block",
                PrimitiveArgs::block([-1.0, 0.0, 2.0], [1.0, 3.0, 3.0]),
                6.0,
            ),
            (
                "prism",
                PrimitiveArgs::prism(&[[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]], 0.0, 1.0),
                2.0,
            ),
            (
                "cylinder",
                PrimitiveArgs::cylinder([1.0, 2.0, 3.0], FrameAxis::X, 0.5, 2.0),
                pi * 0.5,
            ),
            (
                "cylinder",
                PrimitiveArgs::cylinder([1.0, 2.0, 3.0], FrameAxis::Y, 0.5, 2.0),
                pi * 0.5,
            ),
            (
                "cylinder",
                PrimitiveArgs::cylinder_z([1.0, 2.0, 3.0], 0.5, 2.0),
                pi * 0.5,
            ),
            (
                "sphere",
                PrimitiveArgs::sphere([1.0, 1.0, 1.0], 1.0),
                4.0 / 3.0 * pi,
            ),
        ];
        for (kind, args, volume) in shapes {
            let mut program = Program::new();
            program.push("s", args.clone());
            let part = program
                .build_model(registry)
                .unwrap_or_else(|e| panic!("{args:?}: {e}"));
            assert_valid(&part);
            assert_eq!(part.solid_names(), [format!("{kind}(s)")], "{args:?}");
            assert_volume(&part, &format!("{kind}(s)"), volume, volume * 0.02);
        }
    }

    /// A block's faces are named as its sweep names them: `start` at the
    /// bottom, `end` on top, `c0` to `c3` round the sides from the one at
    /// `-y`.
    #[test]
    fn block_faces_are_named_by_side() {
        let mut program = Program::new();
        program.push("s", PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]));
        let part = program.build_model(registry).unwrap();
        let solid = part.solid_id("block(s)").unwrap();
        for (normal, name) in [
            ([0.0, 0.0, -1.0], "block(s,start)"),
            ([0.0, 0.0, 1.0], "block(s,end)"),
            ([0.0, -1.0, 0.0], "block(s,c0)"),
            ([1.0, 0.0, 0.0], "block(s,c1)"),
            ([0.0, 1.0, 0.0], "block(s,c2)"),
            ([-1.0, 0.0, 0.0], "block(s,c3)"),
        ] {
            let face = face_towards(&part, solid, normal);
            assert_eq!(part.name_of(face), Some(name), "{normal:?}");
        }
    }

    /// A given datum is in the part's datums under the step's name, with
    /// its kind and frame.
    #[test]
    fn given_datum_is_added_under_its_step() {
        let mut program = Program::new();
        program.push(
            "top",
            GivenDatumArgs::plane([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
        );
        program.push(
            "axle",
            GivenDatumArgs::axis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        );
        let part = program.built(registry).unwrap();
        let datums = read_datums(&part.current()).unwrap();
        let top = datums.get("top").unwrap();
        assert_eq!(top.kind, DatumKind::Plane);
        assert_eq!(
            top.frame.w().to_array().map(|c| c.to_f64()),
            [0.0, 0.0, 1.0]
        );
        let axle = datums.get("axle").unwrap();
        assert_eq!(axle.kind, DatumKind::Axis);
        assert_eq!(
            axle.frame.origin().to_array().map(|c| c.to_f64()),
            [1.0, 0.0, 0.0]
        );
        assert_eq!(
            axle.frame.w().to_array().map(|c| c.to_f64()),
            [0.0, 1.0, 0.0]
        );
        assert!(model_of(&part).solid_names().is_empty());
        assert_round_trips(Rc::new(registry().unwrap()), None, &program);
    }
}
