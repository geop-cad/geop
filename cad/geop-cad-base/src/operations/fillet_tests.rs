//! Fillets and chamfers in programs: edges of extruded and cut solids,
//! picked by name, rounded and bevelled.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{Ring, ScalInF64 as S, Scalar};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::operation::Role;
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_fillet::{ChamferArgs, FilletArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::examples::{self, n};
use crate::{Command, Editor, Program};

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
}

/// The names of the edges of `part` whose curve `keep` accepts, sorted.
fn edges_where(part: &Part<S>, keep: impl Fn(&geop_core_topology::Edge<S>) -> bool) -> Vec<String> {
    let model = part.topology();
    let mut names: Vec<String> = model
        .edges
        .iter()
        .filter(|(_, e)| keep(e))
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    names.sort();
    names
}

/// The straight edges of `part` standing upright at `(x, y)`.
fn upright_edges_at(part: &Part<S>, x: f64, y: f64) -> Vec<String> {
    let model = part.topology();
    edges_where(part, |e| {
        [e.start_vertex, e.end_vertex].iter().all(|&v| {
            let p = model.vertices[&v].point;
            p[0].could_be_equal(S::from_f64(x)) && p[1].could_be_equal(S::from_f64(y))
        })
    })
}

/// The arcs of `part` around `(cx, cy)` of `radius`, at height `z`.
fn arcs_at(part: &Part<S>, [cx, cy, z]: [f64; 3], radius: f64) -> Vec<String> {
    edges_where(part, |e| {
        e.curve.as_arc().unwrap().is_some_and(|arc| {
            let c = arc.circle;
            c.center[0].could_be_equal(S::from_f64(cx))
                && c.center[1].could_be_equal(S::from_f64(cy))
                && c.center[2].could_be_equal(S::from_f64(z))
                && c.radius.could_be_equal(S::from_f64(radius))
        })
    })
}

/// Whether some vertex of `part` lies on the circle around `(cx, cy)` of
/// `radius`, at height `z`.
fn vertex_on_circle(part: &Part<S>, [cx, cy, z]: [f64; 3], radius: f64) -> bool {
    part.topology().vertices.values().any(|v| {
        let (dx, dy) = (
            v.point[0].sub(S::from_f64(cx)),
            v.point[1].sub(S::from_f64(cy)),
        );
        v.point[2].could_be_equal(S::from_f64(z))
            && dx
                .mul(dx)
                .add(dy.mul(dy))
                .could_be_equal(S::from_f64(radius * radius))
    })
}

/// The rim of the drilled hole rounded: picking one of its arcs rounds the
/// whole circle, the rim moving out to radius `0.4 + r` on the top and down
/// to `1 - r` in the hole.
#[test]
fn fillet_drill_hole_rim() {
    let mut program = examples::box_with_drill_hole();
    let before = program.build::<S>(&NoFiles).unwrap();
    let rim = arcs_at(&before, [1.0, 1.0, 1.0], 0.4);
    assert_eq!(rim.len(), 4, "{rim:?}");
    program.push(
        "round",
        FilletArgs {
            edges: vec![rim[0].clone()],
            radius: 0.1,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["fillet(round)"]);
    assert!(vertex_on_circle(&part, [1.0, 1.0, 1.0], 0.5));
    assert!(vertex_on_circle(&part, [1.0, 1.0, 0.9], 0.4));
    assert!(!vertex_on_circle(&part, [1.0, 1.0, 1.0], 0.4));
}

/// The four upright edges of the drilled box bevelled, then its rim too.
#[test]
fn chamfer_box_edges_and_rim() {
    let mut program = examples::box_with_drill_hole();
    let before = program.build::<S>(&NoFiles).unwrap();
    let mut edges = Vec::new();
    for (x, y) in [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)] {
        edges.extend(upright_edges_at(&before, x, y));
    }
    assert_eq!(edges.len(), 4, "{edges:?}");
    program.push(
        "bevel",
        ChamferArgs {
            edges,
            distance: 0.2,
            distance2: None,
        },
    );
    let rim = arcs_at(&before, [1.0, 1.0, 1.0], 0.4);
    program.push(
        "countersink",
        ChamferArgs {
            edges: vec![rim[2].clone()],
            distance: 0.1,
            distance2: Some(0.05),
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["chamfer(countersink)"]);
    assert!(upright_edges_at(&part, 0.0, 0.0).is_empty());
}

/// A program with a fillet and a chamfer reads back from JSON as written,
/// each under its own kind.
#[test]
fn fillet_and_chamfer_round_trip_through_json() {
    let mut program = examples::box_with_drill_hole();
    program.push(
        "round",
        FilletArgs {
            edges: vec!["a".into(), "b".into()],
            radius: 0.25,
        },
    );
    program.push(
        "bevel",
        ChamferArgs {
            edges: vec!["c".into()],
            distance: 0.1,
            distance2: Some(0.2),
        },
    );
    let json = serde_json::to_value(&program).unwrap();
    let steps = json["steps"].as_array().unwrap();
    let n = steps.len();
    assert_eq!(steps[n - 2]["operation"], "fillet");
    assert_eq!(steps[n - 1]["operation"], "chamfer");
    let back: Program = serde_json::from_value(json).unwrap();
    assert_eq!(back, program);
}

/// An L-shaped block, 1 high, its inner corner at `(1, 1)`.
fn l_block() -> Program {
    let mut s = Sketch::new();
    let corners = [
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 1.0],
        [1.0, 1.0],
        [1.0, 2.0],
        [0.0, 2.0],
    ];
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    let mut program = Program::new();
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: s,
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// The L-block's inner upright edge is concave: rounding it fills the
/// corner in, the round touching both walls `0.2` from the corner, and
/// after an outer edge is rounded too.
#[test]
fn fillet_l_block_inner_edge() {
    let mut program = l_block();
    let before = program.build::<S>(&NoFiles).unwrap();
    let inner = upright_edges_at(&before, 1.0, 1.0);
    let outer = upright_edges_at(&before, 2.0, 0.0);
    assert_eq!((inner.len(), outer.len()), (1, 1));
    program.push(
        "round",
        FilletArgs {
            edges: outer,
            radius: 0.2,
        },
    );
    program.push(
        "inner",
        FilletArgs {
            edges: inner,
            radius: 0.2,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["fillet(inner)"]);
    let model = part.topology();
    for p in [
        [1.2, 1.0, 0.0],
        [1.0, 1.2, 0.0],
        [1.2, 1.0, 1.0],
        [1.0, 1.2, 1.0],
    ] {
        let p = p.map(S::from_f64);
        assert!(
            model
                .vertices
                .values()
                .any(|v| (0..3).all(|k| v.point[k].could_be_equal(p[k]))),
            "no vertex at {p:?}"
        );
    }
    assert!(upright_edges_at(&part, 1.0, 1.0).is_empty());
}

/// The L-block's inner edge bevelled, filling the corner with a flat.
#[test]
fn chamfer_l_block_inner_edge() {
    let mut program = l_block();
    let before = program.build::<S>(&NoFiles).unwrap();
    program.push(
        "bevel",
        ChamferArgs {
            edges: upright_edges_at(&before, 1.0, 1.0),
            distance: 0.3,
            distance2: None,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(upright_edges_at(&part, 1.0, 1.0).is_empty());
}

/// A round boss joined onto a block: the circle where it stands on the
/// block's top is concave, and rounding it fills it in, out to radius
/// `0.3 + 0.1` on the top and up to `1.1` on the boss; its top rim, convex,
/// rounded at once.
#[test]
fn fillet_boss_on_block() {
    let mut block = Sketch::new();
    let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|c| block.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..4 {
        block.add_line(p[i], p[(i + 1) % 4]);
    }
    let mut boss = Sketch::new();
    let c = boss.add_point(n(0.5), n(0.5));
    boss.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push(
        "base",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: block,
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "base".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "extrude(block,end)".into(),
            }),
            sketch: boss,
            ..Default::default()
        },
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: "boss_sketch".into(),
            extent: Extents::blind(0.5),
            face: false,
            combine: Combine::Union {
                target: "extrude(block)".into(),
            },
        },
    );
    let before = program.build::<S>(&NoFiles).unwrap();
    let foot = arcs_at(&before, [0.5, 0.5, 1.0], 0.3);
    let rim = arcs_at(&before, [0.5, 0.5, 1.5], 0.3);
    assert!(!foot.is_empty() && !rim.is_empty(), "{foot:?} {rim:?}");
    program.push(
        "round",
        FilletArgs {
            edges: vec![foot[0].clone(), rim[0].clone()],
            radius: 0.1,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(vertex_on_circle(&part, [0.5, 0.5, 1.0], 0.4));
    assert!(vertex_on_circle(&part, [0.5, 0.5, 1.1], 0.3));
    assert!(vertex_on_circle(&part, [0.5, 0.5, 1.5], 0.2));
    assert!(!vertex_on_circle(&part, [0.5, 0.5, 1.0], 0.3));
}

/// A new fillet waits for its edges to be picked, as edges.
#[test]
fn new_fillet_picks_edges() {
    let mut editor = Editor::<S>::new();
    editor.handle(Command::LoadExample {
        name: "box_with_drill_hole".into(),
    });
    for kind in ["fillet", "chamfer"] {
        let update = editor.handle(Command::New { kind: kind.into() });
        let step = update.step.expect("a step is edited");
        assert_eq!(step.missing, ["edges"]);
        assert_eq!(step.presentation.pickable, [Role::Edge]);
        editor.handle(Command::Cancel);
    }
}
