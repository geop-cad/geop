//! Fillets, chamfers and shells across boxes, cylinders and spheres, whole
//! and with parts cut off, added or drilled — and bodies with free-form
//! edges, a boss on a sphere and a pipe tee: every edge blended on its own,
//! and the solid shelled closed and open at each of its faces in turn.
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

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::ScalInF64 as S;
use geop_core_topology::EdgeId;
use geop_ops::operation::Operation;
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, RevolveArgs};
use geop_ops_fillet::{Chamfer, ChamferArgs, Fillet, FilletArgs};
use geop_ops_shell::{Shell, ShellArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};

use super::regression_tests::check_valid;
use crate::Program;
use crate::examples::n;

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

/// Applies `operation` with `args` to a copy of `part` and says how it went.
fn outcome<O: Operation>(part: &Part<S>, operation: O, args: &O::Args) -> Outcome {
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        operation.apply(part.clone(), "op", args, &NoFiles)
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

fn plane(axis: FrameAxis) -> Option<EntityRef> {
    Some(EntityRef::datum_component(
        ORIGIN,
        DatumComponent::Plane(axis),
    ))
}

fn polygon(corners: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Sketch {
    polygon(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
}

fn circle(cx: f64, cy: f64, r: f64) -> Sketch {
    let mut s = Sketch::new();
    let c = s.add_point(n(cx), n(cy));
    s.add_circle(c, n(r));
    s
}

/// Pushes `sketch` on `on` as `name_sketch`, and its extrude `extent` as
/// `name`, combined by `combine`.
fn extrude(
    program: &mut Program,
    name: &str,
    on: Option<EntityRef>,
    sketch: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(
        &sketch_name,
        AddSketchArgs {
            plane: on,
            sketch,
            ..Default::default()
        },
    );
    program.push(
        name,
        ExtrudeArgs {
            sketch: sketch_name,
            extent,
            face: false,
            combine,
        },
    );
}

/// Both ways from the sketch plane, `half` each.
fn both_ways(half: f64) -> Extents {
    Extents {
        symmetric: true,
        ..Extents::blind(half)
    }
}

fn cut_from(target: &str) -> Combine {
    Combine::Difference {
        target: target.into(),
    }
}

fn join_to(target: &str) -> Combine {
    Combine::Union {
        target: target.into(),
    }
}

/// A box of 2 x 2 x 1 on the ground: the solid `extrude(body)`.
fn block() -> Program {
    let mut program = Program::new();
    extrude(
        &mut program,
        "body",
        plane(FrameAxis::Z),
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
    extrude(
        &mut program,
        "body",
        plane(FrameAxis::Z),
        circle(0.0, 0.0, 1.0),
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
    program.push(
        "body_sketch",
        AddSketchArgs {
            plane: plane(FrameAxis::Z),
            sketch: s,
            ..Default::default()
        },
    );
    program.push(
        "body",
        RevolveArgs {
            sketch: "body_sketch".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "body_sketch".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// The one solid `program` builds, and its name.
fn build(program: &Program) -> (Part<S>, String) {
    let part = program.build::<S>(&NoFiles).unwrap();
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
/// the solid shelled closed and open at each face; panics listing every
/// case gone wrong. `size` is the fillet's radius, the chamfer's distance
/// and the walls' thickness.
fn stress(program: Program, size: f64) {
    let (part, solid) = build(&program);
    let model = part.topology();
    let mut edges: Vec<String> = model
        .edges
        .keys()
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    edges.sort();
    let mut faces: Vec<String> = model
        .faces
        .keys()
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    faces.sort();

    let mut wrong = Vec::new();
    let mut tally = [0usize; 2];
    let mut record = |case: String, outcome: Outcome| match outcome {
        Outcome::Built => tally[0] += 1,
        Outcome::Refused => tally[1] += 1,
        Outcome::Wrong(why) => wrong.push(format!("{case}: {why}")),
    };
    for edge in &edges {
        let args = FilletArgs::constant(vec![edge.clone()], size);
        record(format!("fillet {edge}"), outcome(&part, Fillet, &args));
        let args = ChamferArgs {
            edges: vec![edge.clone()],
            distance: size.into(),
            distance2: None,
        };
        record(format!("chamfer {edge}"), outcome(&part, Chamfer, &args));
    }
    let shells = std::iter::once(Vec::new()).chain(faces.iter().map(|f| vec![f.clone()]));
    for open in shells {
        let args = ShellArgs {
            solid: solid.clone(),
            faces: open.clone(),
            thickness: size.into(),
        };
        record(
            format!("shell open at {open:?}"),
            outcome(&part, Shell, &args),
        );
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
    for (&vertex, _) in &model.vertices {
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

    let mut wrong = Vec::new();
    let mut tally = [0usize; 2];
    let mut record = |case: String, outcome: Outcome| match outcome {
        Outcome::Built => tally[0] += 1,
        Outcome::Refused => tally[1] += 1,
        Outcome::Wrong(why) => wrong.push(format!("{case}: {why}")),
    };
    for (what, edges) in groups {
        let args = FilletArgs::constant(edges.clone(), size);
        record(format!("fillet {what}"), outcome(&part, Fillet, &args));
        let args = ChamferArgs {
            edges,
            distance: size.into(),
            distance2: None,
        };
        record(format!("chamfer {what}"), outcome(&part, Chamfer, &args));
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
    let mut program = block();
    extrude(
        &mut program,
        "corner",
        plane(FrameAxis::Z),
        polygon(&[[1.5, 2.5], [2.5, 1.5], [2.5, 2.5]]),
        Extents::blind(1.0),
        cut_from("extrude(body)"),
    );
    stress_corners(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed_corners() {
    let mut program = block();
    extrude(
        &mut program,
        "pocket",
        Some(EntityRef::Face {
            name: "extrude(body,end)".into(),
        }),
        rectangle(0.5, 0.5, 1.5, 1.5),
        Extents::blind(-0.4),
        cut_from("extrude(body)"),
    );
    stress_corners(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped_corners() {
    let mut program = block();
    extrude(
        &mut program,
        "step",
        plane(FrameAxis::Z),
        rectangle(-0.5, 1.5, 1.0, 2.5),
        Extents::blind(0.5),
        cut_from("extrude(body)"),
    );
    stress_corners(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_flat_corners() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "flat",
        plane(FrameAxis::Z),
        rectangle(0.6, -2.0, 2.0, 2.0),
        Extents::blind(1.5),
        cut_from("extrude(body)"),
    );
    stress_corners(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_drilled() {
    let mut program = block();
    extrude(
        &mut program,
        "hole",
        plane(FrameAxis::Z),
        circle(1.0, 1.0, 0.4),
        Extents::blind(1.0),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed() {
    let mut program = block();
    extrude(
        &mut program,
        "pocket",
        Some(EntityRef::Face {
            name: "extrude(body,end)".into(),
        }),
        rectangle(0.5, 0.5, 1.5, 1.5),
        Extents::blind(-0.4),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped() {
    let mut program = block();
    extrude(
        &mut program,
        "step",
        plane(FrameAxis::Z),
        rectangle(-0.5, 1.5, 1.0, 2.5),
        Extents::blind(0.5),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_block_corner_cut_off() {
    let mut program = block();
    extrude(
        &mut program,
        "corner",
        plane(FrameAxis::Z),
        polygon(&[[1.5, 2.5], [2.5, 1.5], [2.5, 2.5]]),
        Extents::blind(1.0),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_with_boss() {
    let mut program = block();
    extrude(
        &mut program,
        "boss",
        Some(EntityRef::Face {
            name: "extrude(body,end)".into(),
        }),
        circle(1.0, 1.0, 0.4),
        Extents::blind(0.5),
        join_to("extrude(body)"),
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
    extrude(
        &mut program,
        "hole",
        plane(FrameAxis::Z),
        circle(0.0, 0.0, 0.3),
        Extents::blind(1.5),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_drilled_off_axis() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "hole",
        plane(FrameAxis::Z),
        circle(0.4, 0.0, 0.2),
        Extents::blind(1.5),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_cylinder_with_flat() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "flat",
        plane(FrameAxis::Z),
        rectangle(0.6, -2.0, 2.0, 2.0),
        Extents::blind(1.5),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_pocketed() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "pocket",
        Some(EntityRef::Face {
            name: "extrude(body,end)".into(),
        }),
        circle(0.0, 0.0, 0.5),
        Extents::blind(-0.5),
        cut_from("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_boss() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "boss",
        Some(EntityRef::Face {
            name: "extrude(body,end)".into(),
        }),
        circle(0.0, 0.0, 0.4),
        Extents::blind(0.5),
        join_to("extrude(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_sphere() {
    stress(sphere(), 0.1);
}

#[test]
fn stress_hemisphere() {
    let mut program = sphere();
    extrude(
        &mut program,
        "half",
        plane(FrameAxis::Z),
        rectangle(-2.0, -2.0, 2.0, 2.0),
        Extents::blind(-2.0),
        cut_from("revolve(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_sphere_drilled() {
    let mut program = sphere();
    extrude(
        &mut program,
        "hole",
        plane(FrameAxis::Z),
        circle(0.0, 0.0, 0.3),
        both_ways(2.0),
        cut_from("revolve(body)"),
    );
    stress(program, 0.1);
}

#[test]
fn stress_sphere_with_side_cut_off() {
    let mut program = sphere();
    extrude(
        &mut program,
        "side",
        plane(FrameAxis::Z),
        rectangle(0.5, -2.0, 2.0, 2.0),
        both_ways(2.0),
        cut_from("revolve(body)"),
    );
    stress(program, 0.1);
}

/// A sphere with a cylinder standing on it, around its axis: the two meet
/// in a free-form rim — no face of revolution with a straight meridian on
/// one side of it — which a rolling ball rounds.
fn sphere_with_boss() -> Program {
    let mut program = sphere();
    extrude(
        &mut program,
        "boss",
        plane(FrameAxis::Z),
        circle(0.0, 0.0, 0.4),
        Extents::blind(1.5),
        join_to("revolve(body)"),
    );
    program
}

/// A pipe along `x` with a narrower branch standing on it: the two meet in
/// a saddle, a free-form edge round which a rolling ball rolls from face to
/// face of both.
fn pipe_tee() -> Program {
    let mut program = Program::new();
    extrude(
        &mut program,
        "pipe",
        plane(FrameAxis::X),
        circle(0.0, 0.0, 0.5),
        both_ways(1.0),
        Combine::NewBody,
    );
    extrude(
        &mut program,
        "branch",
        plane(FrameAxis::Z),
        circle(0.0, 0.0, 0.3),
        Extents::blind(1.0),
        join_to("extrude(pipe)"),
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
fn assert_builds<O: Operation>(program: Program, operation: O, args: O::Args) {
    let (part, _) = build(&program);
    if let Outcome::Wrong(why) = outcome(&part, operation, &args) {
        panic!("{why}");
    }
}

/// The hemisphere's rim — an arc of the circle where the flat meets the
/// sphere — bevelled: the chamfer rolls round the whole circle, its point
/// on the sphere moving across the seams of the sphere's quarters.
#[test]
fn chamfer_hemisphere_rim() {
    let mut program = sphere();
    extrude(
        &mut program,
        "half",
        plane(FrameAxis::Z),
        rectangle(-2.0, -2.0, 2.0, 2.0),
        Extents::blind(-2.0),
        cut_from("revolve(body)"),
    );
    let args = ChamferArgs {
        edges: vec!["revolve(body,body_sketch,c2,a0)".into()],
        distance: 0.1.into(),
        distance2: None,
    };
    assert_builds(program, Chamfer, args);
}

/// The cylinder's flat's upright side bevelled: a straight edge between the
/// flat and the cylinder, rolled.
#[test]
fn chamfer_cylinder_flat_side() {
    let mut program = cylinder();
    extrude(
        &mut program,
        "flat",
        plane(FrameAxis::Z),
        rectangle(0.6, -2.0, 2.0, 2.0),
        Extents::blind(1.5),
        cut_from("extrude(body)"),
    );
    let args = ChamferArgs {
        edges: vec!["combine(flat,extrude(body,body_sketch,c1),extrude(flat,flat_sketch,c7),combine(flat,extrude(body,body_sketch,c1,end),extrude(flat,flat_sketch,c7,end),0,1),combine(flat,extrude(body,body_sketch,c1,start),extrude(flat,flat_sketch,c7,start),0,1))".into()],
        distance: 0.1.into(),
        distance2: None,
    };
    assert_builds(program, Chamfer, args);
}
