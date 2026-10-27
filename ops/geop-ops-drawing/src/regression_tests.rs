//! Programs from tutorials and bug reports, rebuilt in Rust: drawings of
//! parts whose features were made as a user makes them.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::{part::Part, program::Program, runner::Runner};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::sketch::{
    figures::{disc, on_face, on_origin, rectangle_outline},
    whole_sketch,
};
use geop_ops_model::{
    Combine, RefId,
    testing::{middle, model_of},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{Annotation, DrawingArgs, DrawnView, ViewKind, compose, program_tests::registry};

/// `program` built on its own, in the operations of [`registry`].
fn build(program: &Program) -> Part {
    let runner = Runner::new(registry().operations());
    let mut part = Part::default();
    program
        .run(&runner, &mut part)
        .unwrap_or_else(|e| panic!("{e}"));
    part
}

/// A 40 x 40 x 10 plate with a hole of diameter 6 through its middle, made
/// as the Onshape and SolidWorks part tutorials make a hole: a circle
/// sketched on the top face and extruded "through all" as a cut. Then its
/// drawing, the bottom view with the hole's diameter dimensioned.
///
/// The hole's rims are the cylinder's intersections with the plate's
/// faces, which the boolean traces as fitted curves rather than keeping
/// the tool's circles, as it does every curve two faces meet in. The
/// drawing reconstructs the circle a rim lies on where it dimensions it
/// (`drawing::arc_of`), and dimensions it "⌀6", as it does the rims of the
/// hole operation's holes, which are arcs.
#[test]
fn diameter_of_a_hole_extruded_through_all() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 40.0, 10.0]),
    );
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_face("block(plate,end)"), disc([20.0, 20.0], 3.0)),
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents {
                side1: Extent::ThroughAll,
                reversed: true,
                ..Extents::blind(0.0)
            },
            output: Output::Solid(Combine::cut("block(plate)")),
            draft: 0.0,
        },
    );
    let built = build(&program);
    let part = model_of(&built);
    let model = part.topology();
    let mut rims: Vec<String> = (part.names().iter())
        .filter(|(id, _)| matches!(id, RefId::Edge(_)))
        .map(|(_, name)| name.to_string())
        .filter(|name| {
            let m = middle(model.get_edge(part.edge_id(name).unwrap()).unwrap());
            let (dx, dy) = (m[0].to_f64() - 20.0, m[1].to_f64() - 20.0);
            m[2].to_f64().abs() < 1e-6 && (dx * dx + dy * dy).sqrt() < 4.0
        })
        .collect();
    rims.sort();
    assert!(!rims.is_empty(), "no rim on the top");
    let args = DrawingArgs {
        views: vec![ViewKind::Bottom, ViewKind::Front],
        annotations: vec![Annotation::Radius {
            view: DrawnView::View(ViewKind::Bottom),
            edge: rims[0].clone(),
            diameter: true,
            label: [10.0, 10.0],
        }],
        ..Default::default()
    };
    let sheet = compose(&built.current(), &args, "", &[]).unwrap_or_else(|e| panic!("{e}"));
    let labels: Vec<&str> = sheet.labels.iter().map(|l| l.text.as_str()).collect();
    assert!(labels.contains(&"⌀6"), "{labels:?}");
}

/// `program` built in the operations of [`registry`] and the shell.
fn build_with_shell(program: &Program) -> Part {
    let registry = geop_ops_shell::register(registry()).unwrap();
    let runner = Runner::new(registry.operations());
    let mut part = Part::default();
    program
        .run(&runner, &mut part)
        .unwrap_or_else(|e| panic!("{e}"));
    part
}

/// The names of the edges of `part` whose middle `keep` accepts.
fn edges_where(part: &Part, keep: impl Fn([f64; 3]) -> bool) -> Vec<String> {
    let part = model_of(part);
    let model = part.topology();
    let mut names: Vec<String> = (part.names().iter())
        .filter(|(id, _)| matches!(id, RefId::Edge(_)))
        .map(|(_, name)| name.to_string())
        .filter(|name| {
            let m = middle(model.get_edge(part.edge_id(name).unwrap()).unwrap());
            keep(m.to_array().map(|c| c.to_f64()))
        })
        .collect();
    names.sort();
    names
}

/// An enclosure's drawing (every enclosure tutorial: a box, an upright
/// edge rounded, shelled open at the top, then drawn): a 40 x 20 x 10
/// block, its upright edge at the origin rounded by 3, shelled 1 thick
/// open at its top, and its front view.
///
/// Seen from the front, the blend and its inner copy turn away from the
/// view exactly along their edges with the side `x = 0` and its inner
/// copy: their silhouettes lie on those edges. The view failed, the search
/// for where the inner copy's traced silhouette meets the face's trim
/// exhausted (`curve_curve_crossings`): the silhouette came out `3.6e-12`
/// off the edge, and sharp, because `correct` returned its last sharpened
/// seed as the answer. The inner copy is a padded fit, and where its
/// silhouette lies is known only as well as the padding says; returned
/// as the step from that seed, unsharpened, the silhouette holds the edge,
/// and the search finds the two running along each other.
#[test]
fn front_view_of_a_shelled_box_with_a_rounded_edge() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 20.0, 10.0]),
    );
    let block = build_with_shell(&program);
    let upright = edges_where(&block, |m| {
        m[0].abs() < 1e-9 && m[1].abs() < 1e-9 && (m[2] - 5.0).abs() < 1e-9
    });
    assert_eq!(upright.len(), 1, "{upright:?}");
    program.push("round", geop_ops_fillet::FilletArgs::constant(upright, 3.0));
    program.push(
        "s",
        geop_ops_shell::ShellArgs {
            solid: geop_ops_model::solid("fillet(round)"),
            faces: vec![geop_core_program::element::ElementReference::new(
                geop_ops_model::model(),
                "block(box,end)",
            )],
            thickness: 1.0.into(),
            other_faces: Vec::new(),
            other_thickness: 0.2.into(),
            outward: false,
        },
    );
    let built = build_with_shell(&program);
    geop_ops_model::testing::assert_valid(&model_of(&built));
    let args = DrawingArgs {
        views: vec![ViewKind::Front],
        ..Default::default()
    };
    compose(&built.current(), &args, "", &[]).unwrap_or_else(|e| panic!("{e}"));
}

/// A plate with a round boss, its root bevelled (the SolidWorks "boss
/// with chamfer" tutorial), drawn: a 60 x 40 x 5 plate, a boss of radius 8
/// standing 20 high from its middle, the circle where they meet bevelled
/// 2, and its isometric view. The part builds valid; the view fails
/// deciding what hides a piece of the bevel's edge:
///
/// > curve_surface_crossings: exhausted max_nodes=20000 with 8028 pairs
/// > pending; the result would be incomplete
///
/// for "the ray from (-9.5, 0, 5.5) against face ..", the middle of the
/// bevel's straight edge at `y = 0`, from `(-8, 0, 7)` to `(-10, 0, 5)`:
/// a ray leaving the bevel's cone along it, towards the viewer; the ray's
/// start is 1e-7 wide. Rounded instead of bevelled, the view fails the same
/// way. Built from given solids instead — a block and a cylinder joined,
/// the same shape — the bevelled boss draws: the sketched and extruded
/// circle is in four arcs, the given cylinder's not.
///
/// Measured: seen along `(-1, 1, -1)`, the bevel's cone turns away from
/// the view where `sin θ − cos θ = 1`, at `θ = 90°` and `180°` exactly:
/// on two of the seams between its quarters. Those seams are silhouette
/// edges, and the ray from a point of one towards the eye is tangent to
/// its own cone face where it starts, which the crossing search cannot
/// isolate. Since the silhouette points are no longer sharpened (see
/// [`front_view_of_a_shelled_box_with_a_rounded_edge`]) the view fails one
/// step earlier, the cone's traced silhouette meeting the seam's pcurve
/// within its width over part of its length (`curve_curve_crossings`
/// exhausted in `face_silhouettes`). Both are a silhouette lying along an
/// edge; what is missing is to recognise that from the edge (the face seen
/// edge on along all of it), not from a search along it.
#[test]
#[ignore = "bug: the iso view of a plate with a boss whose root is bevelled fails: the cone's silhouettes lie exactly on two of its quarter seams, and the trim search (and, before it, the hidden-line ray tangent to the cone) exhausts along them"]
fn iso_view_of_a_boss_with_a_bevelled_root() {
    let mut program = Program::new();
    program.push(
        "s1",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            rectangle_outline(-30.0, -20.0, 30.0, 20.0),
        ),
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("s1"),
            extent: Extents::blind(5.0),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );
    program.push(
        "s2",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([0.0, 0.0], 8.0)),
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: whole_sketch("s2"),
            extent: Extents::blind(20.0),
            output: Output::Solid(Combine::join("extrude(plate)")),
            draft: 0.0,
        },
    );
    let before = build(&program);
    let root = edges_where(&before, |m| {
        (m[2] - 5.0).abs() < 1e-9 && (m[0] * m[0] + m[1] * m[1]).sqrt() < 9.0
    });
    assert!(!root.is_empty(), "no root edges");
    program.push("bevel", geop_ops_fillet::ChamferArgs::even(root, 2.0));
    let built = build(&program);
    geop_ops_model::testing::assert_valid(&model_of(&built));
    let args = DrawingArgs {
        views: vec![ViewKind::Iso],
        ..Default::default()
    };
    compose(&built.current(), &args, "", &[]).unwrap_or_else(|e| panic!("{e}"));
}
