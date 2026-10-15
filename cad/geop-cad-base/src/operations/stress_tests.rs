//! Fillets, chamfers and shells across boxes, cylinders and spheres, whole
//! and with parts cut off, added or drilled — and bodies with free-form
//! edges, a boss on a sphere and a pipe tee: every edge blended on its own,
//! every corner's edges together, and the solid shelled closed and open at
//! each of its faces in turn.
//!
//! An operation may refuse what it does not support — saying so — but what
//! it builds has to be a valid solid, and it must not fail otherwise. Each
//! body is one test, so they run side by side; each lists every case that
//! went wrong, not only the first.
//!
//! The bodies that take seconds run with every `cargo test`. The rest —
//! mostly those with circular edges, each blended by a revolved tool and a
//! boolean — take up to a minute each and are `#[ignore]`d: the full set
//! runs with `cargo test -- --ignored`, before changing a blend, a shell or
//! the booleans under them.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::{operation::Operation, program::Program, values::Design};
use geop_core_topology::EdgeId;
use geop_ops_booleans::{Combine, solid};
use geop_ops_extrude_revolve::{Extents, Output, RevolveArgs, RevolveAxis};
use geop_ops_fillet::{Chamfer, ChamferArgs, Fillet, FilletArgs};
use geop_ops_model::{NamedModel, element, testing::apply_to_model};
use geop_ops_shell::{Shell, ShellArgs};
use geop_ops_sketch::{Sketch, SketchPlane, sketch_element, whole_sketch};

use crate::{
    examples::n,
    operations::testing::{
        Built, both_ways, check_valid, disc, extrude_sketch, on_face, on_origin, outline,
        rectangle, sketch,
    },
};

/// What one blend or shell came to.
enum Outcome {
    /// A valid solid.
    Built,
    /// Refused as unsupported, saying why.
    Refused,
    /// Anything else: an error that is not a refusal, an invalid result, a
    /// panic — what the case is listed for.
    Wrong(String),
}

/// Whether an error's root message is the operation refusing what it does
/// not support, rather than failing at what it does.
fn is_refusal(message: &str) -> bool {
    [
        "not supported",
        "only ",
        "too large",
        "too thick",
        "has to",
        "is not planar",
        "no corner to blend",
        "cannot end on a smooth surface",
        "does not stand square",
        "could run along the edge",
        "could meet tangentially",
        "reach across the axis",
        "shrinks the meridian",
        "across its axis",
        "exactly on another edge",
    ]
    .iter()
    .any(|refusal| message.contains(refusal))
}

/// Applies a step of `O` with `args` to a copy of `part` and says how it
/// went.
fn outcome<O: Operation>(part: &NamedModel<Design>, args: &O::Args) -> Outcome {
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        apply_to_model::<O>(part.clone(), "op", args)
    }));
    match run {
        Err(panic) => Outcome::Wrong(format!(
            "panicked: {}",
            panic
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_default()
        )),
        Ok(Err(e)) if is_refusal(e.root_message()) => Outcome::Refused,
        Ok(Err(e)) => Outcome::Wrong(format!("failed: {e:?}")),
        Ok(Ok(built)) => match check_valid(&built) {
            Ok(()) => Outcome::Built,
            Err(e) => Outcome::Wrong(format!("invalid: {e}")),
        },
    }
}

/// The top face of the solid `extrude(body)`.
fn top() -> SketchPlane {
    on_face("extrude(body,end)")
}

/// A box of 2 x 2 x 1 on the ground: the solid `extrude(body)`.
fn block() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "body",
        on_origin(FrameAxis::Z),
        rectangle(0.0, 0.0, 2.0, 2.0),
        Extents::blind(1.0),
        Combine::NewBody,
    );
    program
}

/// A cylinder of radius 1 and height 1.5 around the z axis: the solid
/// `extrude(body)`.
fn cylinder() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "body",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 1.0),
        Extents::blind(1.5),
        Combine::NewBody,
    );
    program
}

/// A sphere of radius 1 around the origin, a half disc turned round the y
/// axis: the solid `revolve(body)`.
fn sphere() -> Program {
    let mut s = Sketch::new();
    let top = s.add_point(n(0.0), n(1.0));
    let bottom = s.add_point(n(0.0), n(-1.0));
    s.add_arc(bottom, top, n(std::f64::consts::PI));
    let axis = s.add_line(top, bottom);
    let mut program = Program::new();
    program.push("body_sketch", sketch(on_origin(FrameAxis::Z), s));
    program.push(
        "body",
        RevolveArgs {
            sketch: whole_sketch("body_sketch"),
            axis: Some(RevolveAxis::Sketch(sketch_element("body_sketch", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// The block with a corner cut off.
fn block_corner_cut_off() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "corner",
        on_origin(FrameAxis::Z),
        outline(&[[1.5, 2.5], [2.5, 1.5], [2.5, 2.5]]),
        Extents::blind(1.0),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a pocket 0.4 deep in its top.
fn block_pocketed() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "pocket",
        top(),
        rectangle(0.5, 0.5, 1.5, 1.5),
        Extents::blind(-0.4),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a step cut along one side.
fn block_stepped() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "step",
        on_origin(FrameAxis::Z),
        rectangle(-0.5, 1.5, 1.0, 2.5),
        Extents::blind(0.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The cylinder with a flat cut along it at `x = 0.6`.
fn cylinder_with_flat() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "flat",
        on_origin(FrameAxis::Z),
        rectangle(0.6, -2.0, 2.0, 2.0),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The sphere, its half below `z = 0` cut off.
fn hemisphere() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "half",
        on_origin(FrameAxis::Z),
        rectangle(-2.0, -2.0, 2.0, 2.0),
        Extents::blind(-2.0),
        Combine::cut("revolve(body)"),
    );
    program
}

/// The one solid `program` builds, and its name.
fn build(program: &Program) -> (NamedModel<Design>, String) {
    let part = program.build_model().unwrap();
    let solids = part.solid_names();
    let [solid] = solids.as_slice() else {
        panic!("not one solid: {solids:?}");
    };
    if let Err(e) = check_valid(&part) {
        panic!("the body itself is invalid: {e}");
    }
    let solid = solid.clone();
    (part, solid)
}

/// Every edge of `part` blended on its own, by fillet and by chamfer, and
/// the solid shelled closed and open at each of its faces; panics listing
/// every case gone wrong. `size` is the fillet's radius, the chamfer's
/// distance and the shell's thickness.
fn stress(program: Program, size: f64) {
    let (part, solid_name) = build(&program);
    let model = part.topology();
    let sorted = |mut names: Vec<String>| {
        names.sort();
        names
    };
    let name = |id: geop_ops_model::RefId| part.name_of(id).unwrap().to_string();
    let edges = sorted(model.edges.keys().map(|&id| name(id.into())).collect());
    let faces = sorted(model.faces.keys().map(|&id| name(id.into())).collect());
    let groups = edges.into_iter().map(|e| (e.clone(), vec![e])).collect();
    let shells = (std::iter::once(Vec::new()).chain(faces.into_iter().map(|f| vec![f])))
        .map(|open: Vec<String>| ShellArgs {
            solid: solid(&solid_name),
            faces: open.into_iter().map(element).collect(),
            thickness: size.into(),
        })
        .collect();
    blend_cases(&part, groups, shells, size);
}

/// Each of `groups` — what a case is called, and its edges — blended
/// together, by fillet and by chamfer, and each of `shells` built; panics
/// listing every case gone wrong. `size` is the fillet's radius and the
/// chamfer's distance.
fn blend_cases(
    part: &NamedModel<Design>,
    groups: Vec<(String, Vec<String>)>,
    shells: Vec<ShellArgs>,
    size: f64,
) {
    let mut wrong = Vec::new();
    let mut tally = [0usize; 2];
    let mut record = |case: String, outcome: Outcome| match outcome {
        Outcome::Built => tally[0] += 1,
        Outcome::Refused => tally[1] += 1,
        Outcome::Wrong(why) => wrong.push(format!("{case}: {why}")),
    };
    for (what, edges) in groups {
        let args = FilletArgs::constant(edges.clone(), size);
        record(format!("fillet {what}"), outcome::<Fillet>(part, &args));
        let args = ChamferArgs::even(edges, size);
        record(format!("chamfer {what}"), outcome::<Chamfer>(part, &args));
    }
    for args in shells {
        let open: Vec<&str> = args.faces.iter().map(|f| f.name()).collect();
        let case = format!("shell open at {open:?}");
        record(case, outcome::<Shell>(part, &args));
    }
    let [built, refused] = tally;
    assert!(
        wrong.is_empty(),
        "{} of {} cases went wrong ({built} built, {refused} refused):\n\n{}",
        wrong.len(),
        wrong.len() + built + refused,
        wrong.join("\n\n")
    );
}

/// The edges of `part` around each of its vertices blended together — the
/// corner rounded where every edge there is filleted — and every edge at
/// once, by fillet and by chamfer; panics listing every case gone wrong.
/// `size` is the fillet's radius and the chamfer's distance.
fn stress_corners(program: Program, size: f64) {
    let (part, _) = build(&program);
    let model = part.topology();
    let name = |id: EdgeId| part.name_of(id).unwrap().to_string();
    let mut groups: Vec<(String, Vec<String>)> = Vec::new();
    for &vertex in model.vertices.keys() {
        let mut edges: Vec<String> = model
            .edges
            .iter()
            .filter(|(_, e)| e.start_vertex == vertex || e.end_vertex == vertex)
            .map(|(&id, _)| name(id))
            .collect();
        edges.sort();
        let at = format!("at vertex {}", part.name_of(vertex).unwrap());
        groups.push((at, edges));
    }
    groups.sort();
    let mut all: Vec<String> = model.edges.keys().map(|&id| name(id)).collect();
    all.sort();
    groups.push(("every edge".to_string(), all));
    blend_cases(&part, groups, Vec::new(), size);
}

#[test]
fn stress_block() {
    stress(block(), 0.1);
}

#[test]
fn stress_block_corners() {
    stress_corners(block(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_corner_cut_off_corners() {
    stress_corners(block_corner_cut_off(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed_corners() {
    stress_corners(block_pocketed(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped_corners() {
    stress_corners(block_stepped(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_flat_corners() {
    stress_corners(cylinder_with_flat(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_drilled() {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([1.0, 1.0], 0.4),
        Extents::blind(1.0),
        Combine::cut("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed() {
    stress(block_pocketed(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped() {
    stress(block_stepped(), 0.1);
}

#[test]
fn stress_block_corner_cut_off() {
    stress(block_corner_cut_off(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_with_boss() {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "boss",
        top(),
        disc([1.0, 1.0], 0.4),
        Extents::blind(0.5),
        Combine::join("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder() {
    stress(cylinder(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_drilled() {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_drilled_off_axis() {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.4, 0.0], 0.2),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_cylinder_with_flat() {
    stress(cylinder_with_flat(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_pocketed() {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "pocket",
        top(),
        disc([0.0, 0.0], 0.5),
        Extents::blind(-0.5),
        Combine::cut("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_boss() {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "boss",
        top(),
        disc([0.0, 0.0], 0.4),
        Extents::blind(0.5),
        Combine::join("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_sphere() {
    stress(sphere(), 0.1);
}

#[test]
fn stress_hemisphere() {
    stress(hemisphere(), 0.1);
}

#[test]
fn stress_sphere_drilled() {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        both_ways(2.0),
        Combine::cut("revolve(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_sphere_with_side_cut_off() {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "side",
        on_origin(FrameAxis::Z),
        rectangle(0.5, -2.0, 2.0, 2.0),
        both_ways(2.0),
        Combine::cut("revolve(body)"),
    );
    stress(program, 0.1);
}

/// A sphere with a cylinder standing on it, around its axis: the two meet
/// in a free-form rim — no face of revolution with a straight meridian on
/// one side of it — which a rolling ball rounds.
fn sphere_with_boss() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "boss",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.4),
        Extents::blind(1.5),
        Combine::join("revolve(body)"),
    );
    program
}

/// A pipe along `x` with a narrower branch standing on it: the two meet in
/// a saddle, a free-form edge round which a rolling ball rolls from face to
/// face of both.
fn pipe_tee() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "pipe",
        on_origin(FrameAxis::X),
        disc([0.0, 0.0], 0.5),
        both_ways(1.0),
        Combine::NewBody,
    );
    extrude_sketch(
        &mut program,
        "branch",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        Extents::blind(1.0),
        Combine::join("extrude(pipe)"),
    );
    program
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_sphere_with_boss() {
    stress(sphere_with_boss(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_pipe_tee() {
    stress(pipe_tee(), 0.1);
}

/// Applies `operation` with `args` to the one solid `program` builds, and
/// checks it builds valid, saying why not.
fn assert_builds<O: Operation>(program: Program, args: O::Args) {
    let (part, _) = build(&program);
    if let Outcome::Wrong(why) = outcome::<O>(&part, &args) {
        panic!("{why}");
    }
}

/// The hemisphere's rim — an arc of the circle where the flat meets the
/// sphere — bevelled: the chamfer rolls round the whole circle, its point
/// on the sphere moving across the seams of the sphere's quarters.
#[test]
fn chamfer_hemisphere_rim() {
    let program = hemisphere();
    let args = ChamferArgs::even(vec!["revolve(body,body_sketch,c2,a0)".into()], 0.1);
    assert_builds::<Chamfer>(program, args);
}

/// The cylinder's flat's upright side bevelled: a straight edge between the
/// flat and the cylinder, rolled.
#[test]
fn chamfer_cylinder_flat_side() {
    let program = cylinder_with_flat();
    let args = ChamferArgs::even(vec!["combine(flat,extrude(body,body_sketch,c1),extrude(flat,flat_sketch,c7),combine(flat,extrude(body,body_sketch,c1,end),extrude(flat,flat_sketch,c7,end),0,1),combine(flat,extrude(body,body_sketch,c1,start),extrude(flat,flat_sketch,c7,start),0,1))".into()], 0.1);
    assert_builds::<Chamfer>(program, args);
}
