//! The examples of [`crate::examples`]: each builds a valid part, saves,
//! loads and builds again the very same part, and builds incrementally,
//! edit by edit, what it builds from scratch.

use std::rc::Rc;

use geop_core_math::{
    geop_error::GeopResult,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::{
    operation_registry::OperationRegistry,
    program::Program,
    testing::{args, assert_incremental, assert_round_trips, edit_args, edit_sequence, step},
    values::Design,
};
use geop_core_topology::{
    contains::shell::PointClassification,
    validation::{ValidationParameters, validate_manifold},
};
use geop_ops_model::parameters::{ParameterKind, parameters_of, set_parameters};
use geop_ops_model::sketch::{
    Constraint,
    figures::{disc, n, on_face},
};
use geop_ops_model::{
    NamedModel, PartDescription,
    testing::{Built, assert_valid, check_valid, classify, error, model_of},
};
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs};

use crate::{
    Extents, ExtrudeArgs,
    examples::{
        box_with_drill_hole, bracket, cross_drilled_shaft, edge_part, handle_with_hole, horn,
        luggage_tag, motor_flange, parametric_plate, pipe, pipe_route, revolved_cone_on_box,
    },
};

/// The operations the examples are written in.
fn registry() -> GeopResult<OperationRegistry> {
    crate::testing::registry()?.with(AddSketch3d)
}

/// Builds `program`, requires it valid and to round-trip (see
/// [`assert_round_trips`]), and returns its model.
fn build_and_round_trip(name: &str, program: &Program) -> NamedModel<Design> {
    let part = assert_round_trips(Rc::new(registry().unwrap()), None, program);
    let model = model_of(&part);
    if let Err(report) = check_valid(&model) {
        panic!("{name}: {report}");
    }
    model
}

/// Runs `program` through [`edit_sequence`] and requires every edit to
/// build incrementally what it builds from scratch (see
/// [`assert_incremental`]).
fn assert_edits_incremental(program: &Program) {
    assert_incremental(Rc::new(registry().unwrap()), None, &edit_sequence(program));
}

#[test]
fn box_with_drill_hole_round_trips() {
    let part = build_and_round_trip("box_with_drill_hole", &box_with_drill_hole());
    let description = PartDescription::of(&part).unwrap();

    // One solid, named after the step that cut the hole.
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(hole)"]
    );
    // The box's top keeps its name, and now has the hole in it.
    let top = &description.faces["extrude(box,end)"];
    assert_eq!(top.holes.len(), 1, "{top:?}");
    // The hole's bottom is the hole tool's end cap; its wall is the
    // four quarters swept by the circle.
    assert!(description.faces.contains_key("extrude(hole,end)"));
    let args: AddSketchArgs = args(&box_with_drill_hole().steps[2]);
    let circle_id = *args.sketch.curves.keys().next().unwrap();
    for piece in ["", "#1", "#2", "#3"] {
        let wall = format!("extrude(hole,hole_sketch,{circle_id}{piece})");
        assert!(description.faces.contains_key(&wall), "no face {wall}");
    }

    assert_eq!(
        classify(&part, "extrude(hole)", [0.3, 0.3, 0.5]),
        PointClassification::Inside
    );
    assert_eq!(
        classify(&part, "extrude(hole)", [1.0, 1.0, 0.8]),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&part, "extrude(hole)", [1.0, 1.0, 0.3]),
        PointClassification::Inside
    );
}

#[test]
fn box_with_drill_hole_builds_incrementally() {
    assert_edits_incremental(&box_with_drill_hole());
}

/// The hole goes all the way through: both the top and the bottom face
/// carry it as a hole, it has no bottom of its own, and its axis is
/// outside the solid at every height.
#[test]
fn bracket_round_trips() {
    let part = build_and_round_trip("bracket", &bracket());
    let description = PartDescription::of(&part).unwrap();

    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(hole)"]
    );
    for cap in ["extrude(block,start)", "extrude(block,end)"] {
        let face = &description.faces[cap];
        assert_eq!(face.holes.len(), 1, "{cap}: {face:?}");
    }
    assert!(!description.faces.contains_key("extrude(hole,end)"));
    for z in [1.0, 5.0, 9.0] {
        assert_eq!(
            classify(&part, "extrude(hole)", [20.0, 20.0, z]),
            PointClassification::Outside,
            "z = {z}"
        );
    }
    assert_eq!(
        classify(&part, "extrude(hole)", [5.0, 5.0, 5.0]),
        PointClassification::Inside
    );
}

#[test]
fn bracket_builds_incrementally() {
    assert_edits_incremental(&bracket());
}

#[test]
fn cross_drilled_shaft_round_trips() {
    let part = build_and_round_trip("cross_drilled_shaft", &cross_drilled_shaft());
    let description = PartDescription::of(&part).unwrap();
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(bore)"]
    );
    assert_eq!(
        classify(&part, "extrude(bore)", [0.0, 0.8, 0.5]),
        PointClassification::Inside
    );
    assert_eq!(
        classify(&part, "extrude(bore)", [0.0, 0.0, 2.2]),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&part, "extrude(bore)", [0.0, 0.0, 2.7]),
        PointClassification::Inside
    );
}

#[test]
fn cross_drilled_shaft_builds_incrementally() {
    assert_edits_incremental(&cross_drilled_shaft());
}

/// Reproduces the real bug report described on `handle_with_hole`.
///
/// Used to fail in the "hole" step's `Difference`, with
/// `face_interior_point` finding no interior point on a side wall of the
/// inner spline. The wall's loop carried a spur whose pcurve ran across
/// the whole face: `fit_pcurve` seeded its first Newton projection from
/// the patch's parametric middle, and on this strongly curved wall that
/// converged to a foot point clamped against the far domain bound. The
/// walk is now seeded where the curve actually starts on the surface.
#[test]
fn handle_with_hole_round_trips() {
    build_and_round_trip("handle_with_hole", &handle_with_hole());
}

#[test]
fn handle_with_hole_builds_incrementally() {
    assert_edits_incremental(&handle_with_hole());
}

/// The example drawn in the editor builds a valid part and round-trips, as
/// the ones written in Rust do.
#[test]
fn edge_part_round_trips() {
    build_and_round_trip("edge_part", &edge_part());
}

#[test]
fn edge_part_builds_incrementally() {
    assert_edits_incremental(&edge_part());
}

#[test]
fn luggage_tag_round_trips() {
    build_and_round_trip("luggage_tag", &luggage_tag());
}

#[test]
fn luggage_tag_builds_incrementally() {
    assert_edits_incremental(&luggage_tag());
}

/// Reproduces the real bug report described on `revolved_cone_on_box`.
#[test]
fn revolved_cone_on_box_round_trips() {
    let part = build_and_round_trip("revolved_cone_on_box", &revolved_cone_on_box());
    // The side face is the plane x = 2.2456, with sketch x along world y
    // and sketch y along world z. The triangle's centroid, turned a
    // quarter around the axis, lies 0.654 off that plane on either side.
    for (p, expected) in [
        ([2.9, -0.198, 1.471], PointClassification::Inside),
        ([1.59, -0.198, 1.471], PointClassification::Inside),
        ([2.9, 1.0, 0.3], PointClassification::Outside),
        ([-0.669, 0.049, 1.0], PointClassification::Outside),
        ([1.5, -1.0, 1.0], PointClassification::Inside),
    ] {
        assert_eq!(classify(&part, "revolve(revolve1)", p), expected, "{p:?}");
    }
}

#[test]
fn revolved_cone_on_box_builds_incrementally() {
    assert_edits_incremental(&revolved_cone_on_box());
}

/// The parametric plate is one solid with its hole, and builds again as
/// it built.
#[test]
fn parametric_plate_round_trips() {
    let part = build_and_round_trip("parametric_plate", &parametric_plate());
    assert_eq!(part.solid_names(), ["extrude(hole)"]);
}

/// The parametric plate edited as [`edit_sequence`] edits it, and then
/// its parameters: a number changed, a row of its table changed, another
/// row selected, each put back again.
#[test]
fn parametric_plate_builds_incrementally() {
    let original = parametric_plate();
    let parameters = parameters_of(&original);
    let edited = |edit: &dyn Fn(&mut ParameterKind)| {
        let mut changed = parameters.clone();
        for parameter in &mut changed.values {
            edit(&mut parameter.kind);
        }
        let mut program = original.clone();
        set_parameters(&mut program, changed);
        assert_ne!(program, original);
        program
    };
    let wider = edited(&|kind| {
        if let ParameterKind::Number { expression, .. } = kind
            && expression == "4"
        {
            *expression = "5".into();
        }
    });
    let looser = edited(&|kind| {
        if let ParameterKind::Table { rows, .. } = kind {
            rows[1].values[0] = 0.5;
        }
    });
    let larger = edited(&|kind| {
        if let ParameterKind::Table { selected, .. } = kind {
            *selected = "M6".into();
        }
    });
    let mut programs = edit_sequence(&original);
    for program in [wider, looser, larger] {
        programs.extend([program, original.clone()]);
    }
    assert_incremental(Rc::new(registry().unwrap()), None, &programs);
}

/// The pipe through space is one valid closed solid: four quarters of
/// wall along each of its five curves and two caps — the far one square
/// to its last line, at `z = 4` — hollow nowhere and solid all along.
#[test]
fn pipe_round_trips() {
    let part = build_and_round_trip("pipe", &pipe());
    let validation = ValidationParameters::default();
    if let Err(errors) = validate_manifold(&validation, part.topology()) {
        panic!("{errors:?}");
    }
    let description = PartDescription::of(&part).unwrap();
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["sweep(pipe)"]
    );
    assert_eq!(description.faces.len(), 5 * 4 + 2);
    let model = part.topology();
    let end = part.face_id("sweep(pipe,end)").unwrap();
    assert!(model.iterate_face_coedges(end).all(|c| {
        model.coedge_start_vertex(c).unwrap().point[2].could_be_equal(S::from_f64(4.0))
    }));
    let half = std::f64::consts::FRAC_1_SQRT_2;
    for (p, expected) in [
        ([1.5, 0.0, 0.0], PointClassification::Inside),
        ([1.5, 0.4, 0.0], PointClassification::Outside),
        ([3.0 + half, 1.0 - half, 0.0], PointClassification::Inside),
        ([4.0, 2.0, 0.2], PointClassification::Inside),
        ([4.0, 3.0 + half, 1.0 - half], PointClassification::Inside),
        ([4.0, 4.0, 3.0], PointClassification::Inside),
        ([4.0, 4.0, 4.2], PointClassification::Outside),
    ] {
        assert_eq!(classify(&part, "sweep(pipe)", p), expected, "at {p:?}");
    }
}

#[test]
fn pipe_builds_incrementally() {
    assert_edits_incremental(&pipe());
}

/// The example `pipe`, of radius `r`, along its route bent at radius
/// `bend`.
fn pipe_of(r: f64, bend: f64) -> Program {
    let mut program = pipe();
    edit_args(step(&mut program, "section"), |args: &mut AddSketchArgs| {
        args.sketch = disc([0.0, 0.0], r);
    });
    edit_args(step(&mut program, "route"), |args: &mut AddSketch3dArgs| {
        args.sketch = pipe_route(bend);
    });
    program
}

/// A pipe bent tighter than it is thick is refused, naming the bend —
/// rather than built crossing itself.
#[test]
fn a_bend_tighter_than_the_pipe_is_refused() {
    let error = pipe_of(0.3, 0.25)
        .build_model(registry)
        .expect_err("refused");
    assert!(error.root_message().contains("bend"), "{error:?}");
}

/// Pipes along routes of lines and bends of several radii, from thin to
/// as thick as the tightest bend allows, and beyond: every one builds a
/// valid solid, or is refused for a bend tighter than the pipe.
#[test]
#[ignore = "slow: sweeps along paths of varied curvature — run with `cargo test -- --ignored`"]
fn pipes_bent_at_varied_radii() {
    for bend in [0.4, 1.0, 1.5] {
        for r in [0.1, 0.3, 0.39, 0.5] {
            match pipe_of(r, bend).build_model(registry) {
                Ok(part) => assert_valid(&part),
                Err(e) if r >= bend && e.root_message().contains("bend") => {}
                Err(e) => panic!("bend {bend}, radius {r}: {e:?}"),
            }
        }
    }
}

/// The names of a program's entities don't depend on its numbers: a
/// taller box with a wider hole has the very same names.
#[test]
fn names_survive_a_change_of_dimensions() {
    let names = |program: &Program| {
        let part = program.build_model(registry).unwrap();
        let description = PartDescription::of(&part).unwrap();
        (
            description.faces.keys().cloned().collect::<Vec<_>>(),
            description.edges.keys().cloned().collect::<Vec<_>>(),
            description.vertices.keys().cloned().collect::<Vec<_>>(),
        )
    };
    let original = box_with_drill_hole();
    let mut edited = original.clone();
    for step in &mut edited.steps {
        match step.id.as_str() {
            "box" => edit_args::<ExtrudeArgs>(step, |args| args.extent = Extents::blind(1.5)),
            "hole_sketch" => edit_args::<AddSketchArgs>(step, |args| {
                for c in args.sketch.constraints.values_mut() {
                    if let Constraint::Radius { value, .. } = c {
                        *value = n(0.6);
                    }
                }
                args.sketch.solve().unwrap();
            }),
            _ => {}
        }
    }
    assert_ne!(original, edited);
    assert_eq!(names(&edited), names(&original));
}

/// An unknown name is reported, not guessed at.
#[test]
fn referring_to_a_missing_entity_fails() {
    let mut program = box_with_drill_hole();
    edit_args::<AddSketchArgs>(&mut program.steps[2], |args| {
        args.plane = Some(on_face("extrude(box,side)"));
    });
    let err = error(program.build_model(registry));
    assert!(format!("{err:?}").contains("extrude(box,side)"), "{err:?}");
}

/// Step ids have to be unique: every name a step creates is built from
/// its id.
#[test]
fn duplicate_step_ids_are_rejected() {
    let mut program = box_with_drill_hole();
    program.steps[3].id = "box".into();
    assert!(program.validate_ids().is_err());
    assert!(program.build_model(registry).is_err());
}

/// The horn is one solid, swept along its axis, flaring as its rail does:
/// as wide as the mouth at its start, and as the rail at its end.
#[test]
fn horn_round_trips() {
    let part = build_and_round_trip("horn", &horn());
    assert_eq!(part.solid_names(), ["sweep(horn)"]);
    for (p, expected) in [
        ([0.1, 0.0, 0.0], PointClassification::Inside),
        ([0.1, 0.0, 0.6], PointClassification::Outside),
        ([3.9, 0.0, 1.4], PointClassification::Inside),
        ([3.9, 0.0, 1.8], PointClassification::Outside),
    ] {
        assert_eq!(classify(&part, "sweep(horn)", p), expected, "at {p:?}");
    }
}

#[test]
fn horn_builds_incrementally() {
    assert_edits_incremental(&horn());
}

/// The motor flange is a ring with six bolt holes through it: solid in
/// its wall, open in its bore and in every hole, the copies round the
/// bolt circle as much as the first.
#[test]
fn motor_flange_round_trips() {
    let part = build_and_round_trip("motor_flange", &motor_flange());
    let solid = "extrude(flange)";
    let at = |angle: f64, r: f64| {
        let a = angle.to_radians();
        [r * a.cos(), r * a.sin(), 1.25]
    };
    for (p, expected) in [
        (at(30.0, 11.0), PointClassification::Inside),
        (at(0.0, 7.0), PointClassification::Inside),
        (at(0.0, 0.0), PointClassification::Outside),
        (at(0.0, 11.0), PointClassification::Outside),
        (at(120.0, 11.0), PointClassification::Outside),
        (at(300.0, 11.0), PointClassification::Outside),
        (at(0.0, 16.0), PointClassification::Outside),
    ] {
        assert_eq!(classify(&part, solid, p), expected, "at {p:?}");
    }
}

#[test]
fn motor_flange_builds_incrementally() {
    assert_edits_incremental(&motor_flange());
}
