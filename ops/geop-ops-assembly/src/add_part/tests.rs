use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::ScalInF64 as S,
    vector::Vector3,
};
use geop_ops::{
    Component, EntityRef, Operations,
    assembly::{Drag, Kind, MateKind},
    operation::Aspects,
    part::{ParamValue, State, pose_parameter},
    ui::{Control, PartView, StepEditEvent, StepEditor, Tone, Value},
};
use geop_ops_extrude_revolve::shapes::cube_solid;

use geop_core_math::scalars::Scalar;
use geop_ops::Design;

use super::*;
use crate::editor::PART;

/// Programs' parts by file name, built beforehand.
struct Shelf(BTreeMap<String, Arc<Component<S>>>);

impl Library<S> for Shelf {
    fn component(&self, file: &str, _: &State) -> GeopResult<Arc<Component<S>>> {
        self.0
            .get(file)
            .cloned()
            .ok_or_else(|| GeopError::new(format!("no file {file:?}")))
    }

    fn files(&self) -> Vec<String> {
        self.0.keys().cloned().collect()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        Err(GeopError::new(format!("no file {file:?} to read")))
    }
}

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// A shelf with `cube.geop`: the unit cube from the origin to `(1, 1, 1)`.
fn shelf() -> Shelf {
    let mut part = Part::new();
    cube_solid(&mut part, "c", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    let component = Component::new(
        "cube.geop".into(),
        part,
        BTreeSet::from(["cube.geop".into()]),
    );
    Shelf(BTreeMap::from([("cube.geop".into(), Arc::new(component))]))
}

fn args(fixed: bool, mates: &[(&str, MateKind, [EntityRef; 2])]) -> AddPartArgs {
    AddPartArgs {
        file: "cube.geop".into(),
        fixed,
        flexible: false,
        mates: mates
            .iter()
            .map(|(id, kind, entities)| {
                let mate = Mate {
                    kind: *kind,
                    entities: entities.to_vec(),
                    joints: Vec::new(),
                };
                (id.to_string(), mate)
            })
            .collect(),
        ..Default::default()
    }
}

fn n(x: f64) -> Design {
    Design::from_f64(x)
}

fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n)).unwrap()
}

/// Where `pose` puts the point `p`.
fn apply(pose: &Pose<Design>, p: [f64; 3]) -> [f64; 3] {
    let q = pose.apply(&Vector3::from_array(p.map(n)));
    [0, 1, 2].map(|k| q[k].to_f64())
}

fn at(position: [f64; 3]) -> Pose<Design> {
    pose(position, [0.0; 3])
}

/// The frame of the part placed as `instance`, or one of its axes or planes.
fn origin(instance: &str, component: Option<DatumComponent>) -> EntityRef {
    EntityRef::Datum {
        name: format!("{instance}/origin"),
        component,
    }
}

fn axis(instance: &str) -> EntityRef {
    origin(instance, Some(DatumComponent::Axis(FrameAxis::Z)))
}

fn base(instance: &str) -> EntityRef {
    origin(instance, Some(DatumComponent::Plane(FrameAxis::Z)))
}

fn point(part: &Part<S>, entity: &EntityRef) -> [f64; 3] {
    let p = Aspects::of(entity, part).unwrap().point.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// Asserts `a` is within `tol` of `b` in every coordinate.
#[track_caller]
fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < tol),
        "{a:?} is not within {tol} of {b:?}"
    );
}

/// How near the solver brings mates to holding: a billionth of the
/// assembly's size (see `geop_core_solve`), and the cube is 1 across.
const SOLVED: f64 = 1e-8;

/// `before` with the cube placed by `args` as the step `id`, where the
/// program's parameter `id.pose` puts it: `pose`.
fn place(
    before: Part<S>,
    id: &str,
    pose: Pose<Design>,
    args: &AddPartArgs,
    library: &Shelf,
) -> Part<S> {
    let state = State::from([(pose_parameter(id), ParamValue::Pose(pose))]);
    AddPart
        .apply(before.with_state(state), id, args, library)
        .unwrap()
}

/// The part with the cube placed as `a`, fixed at the origin.
fn with_a(library: &Shelf) -> Part<S> {
    place(
        Part::new(),
        "a",
        Pose::identity(),
        &args(true, &[]),
        library,
    )
}

/// A pointer looking straight down at `at`.
fn down_at(at: Vector3<S>) -> geop_ops::ui::Pointer<S> {
    geop_ops::ui::Pointer {
        ray: geop_core_math::primitives::Ray::try_new(
            at.add(&v(0.0, 0.0, 10.0)),
            v(0.0, 0.0, -1.0),
        )
        .unwrap(),
        reach: geop_ops::ui::Reach::Tube {
            radius: S::from_f64(0.01),
        },
    }
}

/// Where `moved` puts the part placed as `instance`.
fn moved_to(moved: &State, instance: &str) -> Pose<Design> {
    match moved[&pose_parameter(instance)] {
        ParamValue::Pose(pose) => pose,
        ref other => panic!("a pose, not {other:?}"),
    }
}

/// A placed part is where its pose puts it: its entities resolve there, and
/// it is drawn there, named behind its instance's name.
#[test]
fn a_placed_part_is_where_its_pose_puts_it() {
    let library = shelf();
    let pose = pose([2.0, 0.0, 0.0], [0.0, 0.0, 90.0]);
    let part = place(Part::new(), "p", pose, &args(false, &[]), &library);
    part.check_names().unwrap();
    assert_eq!(
        part.state().keys().collect::<Vec<_>>(),
        ["p.pose"],
        "the step declares where it puts the part"
    );
    assert_close(point(&part, &origin("p", None)), [2.0, 0.0, 0.0], 1e-12);

    // Drawn by reference: its component's view, where it is.
    let view = PartView::of(&part).unwrap();
    assert!(view.faces.is_empty());
    let [placed] = &view.instances[..] else {
        panic!("one part is placed: {:?}", view.instances);
    };
    assert_eq!(placed.name, "p");
    let component = placed.component().view().unwrap();
    assert!(!component.faces.is_empty());
    // Turned a quarter about z, the cube spans x in [1, 2] and y in [0, 1].
    for p in component
        .faces
        .iter()
        .flat_map(|f| f.triangles.iter().flatten())
    {
        let p = placed.frame.to_xyz(p);
        let [x, y] = [p[0].to_f64(), p[1].to_f64()];
        assert!((1.0 - 1e-9..=2.0 + 1e-9).contains(&x), "{x}");
        assert!((-1e-9..=1.0 + 1e-9).contains(&y), "{y}");
    }
    // And picked where it is, named behind its instance's name.
    let pointer = geop_ops::ui::Pointer {
        ray: geop_core_math::primitives::Ray::try_new(v(1.5, 0.5, 10.0), v(0.0, 0.0, -1.0))
            .unwrap(),
        reach: geop_ops::ui::Reach::Tube {
            radius: S::from_f64(0.01),
        },
    };
    let hit = view
        .pick(&pointer, &[geop_ops::operation::Role::Plane], None)
        .unwrap();
    assert!(
        matches!(&hit.entity, EntityRef::Face { name } if name.starts_with("p/")),
        "{hit:?}"
    );
    assert!((hit.point[2].to_f64() - 1.0).abs() < 1e-9, "{hit:?}");
}

/// The step places, it does not solve: solving its mates gives where the
/// part placed has to be — on the axis of the part before, at a distance
/// from its base, on the side it was put — and the part before, fixed,
/// stays.
#[test]
fn solving_the_mates_moves_a_part_onto_the_part_before() {
    let library = shelf();
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Concentric),
                [axis("b"), axis("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Distance { value: n(1.5) }),
                [base("b"), base("a")],
            ),
        ],
    );
    let part = place(with_a(&library), "b", at([5.0, 4.0, 3.0]), &mates, &library);
    assert!(!part.check_mates(|_| true).unwrap().converged);
    let (moved, report) = part.solve_mates(None, &[], &[]).unwrap();
    assert!(report.converged, "{report:?} {moved:?}");
    assert_eq!(moved.keys().collect::<Vec<_>>(), ["b.pose"], "a is fixed");
    let solved = place(
        with_a(&library),
        "b",
        moved_to(&moved, "b"),
        &mates,
        &library,
    );
    assert!(solved.check_mates(|_| true).unwrap().converged);
    assert_close(point(&solved, &origin("b", None)), [0.0, 0.0, 1.5], SOLVED);
}

/// The one operation, as a set an editor edits.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
enum Ops {
    AddPart(AddPartArgs),
}

/// Adding a mate in the editor arms its entities to be picked; once it has
/// both, picking is done.
#[test]
fn adding_a_mate_arms_its_entities() {
    let library = shelf();
    let before = with_a(&library);
    let context = Context::new(&before, "b", &library);
    let mut editor = StepEditor::new(Ops::AddPart(args(false, &[])), context, false);
    let built = editor.step().apply(before.clone(), "b", &library).unwrap();
    let view = PartView::of(&built).unwrap();
    let dialog = |key: &str, value: Value| StepEditEvent::Dialog {
        key: key.into(),
        value,
    };
    editor.handle(
        context,
        &view,
        &dialog("add_mate", Value::Choice("coincident".into())),
    );
    let presentation = editor.presentation(context, &built, &view);
    let Some(Control::Reference(entities)) = presentation.dialog.get("mate:m1:entities") else {
        panic!("the new mate's entities are a reference field");
    };
    assert!(entities.armed, "a new mate waits for its entities");

    let pair = vec![origin("b", None), origin("a", None)];
    editor.handle(
        context,
        &view,
        &dialog("mate:m1:entities", Value::Entities(pair.clone())),
    );
    let presentation = editor.presentation(context, &built, &view);
    assert!(
        presentation.dialog.get("mate:m1:entities").is_none(),
        "done picking"
    );
    let Ops::AddPart(args) = editor.step().clone();
    assert_eq!(args.mates["m1"].entities, pair);
}

/// Typing where the part goes sets the program's parameter.
#[test]
fn the_placement_fields_set_the_pose_parameter() {
    let library = shelf();
    let before = Part::new();
    let context = Context::new(&before, "b", &library);
    let mut editor = StepEditor::new(Ops::AddPart(args(false, &[])), context, false);
    let view = PartView::of(&before).unwrap();
    let dialog = |key: &str, v: f64| StepEditEvent::Dialog {
        key: key.into(),
        value: Value::Number(v),
    };
    editor.handle(context, &view, &dialog("x", 2.0));
    editor.handle(context, &view, &dialog("rotation_z", 90.0));
    let pose = moved_to(editor.state(), "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, 0.0, 0.0], 1e-12);
    assert_close(apply(&pose, [1.0, 0.0, 0.0]), [2.0, 1.0, 0.0], 1e-12);
}

/// Dragging the part asks for the point grabbed to be pulled where the
/// pointer is — the editor solves it — and solving with that pull turns a
/// part on an axis about the axis.
#[test]
fn a_drag_pulls_the_point_grabbed() {
    let library = shelf();
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Concentric),
                [axis("b"), axis("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Coincident),
                [base("b"), base("a")],
            ),
        ],
    );
    let part = place(with_a(&library), "b", Pose::identity(), &mates, &library);
    let before = with_a(&library);
    let state = part.inputs().clone();
    let context = Context::new(&before, "b", &library)
        .state(&state)
        .built(Some(&part));
    let (mut args, mut session) = (mates.clone(), PartSession::default());
    let mut state = state.clone();
    let mut drag = |to: [f64; 3], done: bool, session: &mut PartSession| {
        let edit = Edit {
            args: &mut args,
            session,
            selection: &mut Vec::new(),
            state: &mut state,
        };
        AddPart.event(
            context,
            edit,
            &CanvasEvent::Move {
                key: PART.into(),
                from: v(1.0, 0.0, 0.0),
                to: v(to[0], to[1], to[2]),
                pointer: down_at(v(to[0], to[1], to[2])),
                done,
                shift: false,
            },
        );
    };
    drag([0.0, 1.0, 0.0], false, &mut session);
    let drags = AddPart.form(context, &mates, &session, &[]).drags;
    let [
        Drag {
            parameter,
            local,
            target,
        },
    ] = drags.as_slice()
    else {
        panic!("one drag: {drags:?}");
    };
    assert_eq!(parameter, "b.pose");
    let plain = |p: &Vector3<S>| [0, 1, 2].map(|k| p[k].to_f64());
    assert_close(plain(local), [1.0, 0.0, 0.0], 1e-12);
    assert_close(plain(target), [0.0, 1.0, 0.0], 1e-12);
    let (moved, report) = part.solve_mates(None, &[], &drags).unwrap();
    assert!(report.converged);
    let corner = apply(&moved_to(&moved, "b"), [1.0, 0.0, 0.0]);
    assert_close(corner, [0.0, 1.0, 0.0], 1e-2);
    // Released, it pulls once more, and is gone at the next event.
    drag([0.0, 1.0, 0.0], true, &mut session);
    assert_eq!(AddPart.form(context, &mates, &session, &[]).drags, drags);
    let edit = Edit {
        args: &mut mates.clone(),
        session: &mut session,
        selection: &mut Vec::new(),
        state: &mut part.inputs().clone(),
    };
    AddPart.event(context, edit, &CanvasEvent::Leave);
    assert!(
        AddPart
            .form(context, &mates, &session, &[])
            .drags
            .is_empty(),
        "released"
    );
}

/// A fixed part goes where it is dragged: no mate moves it, so the drag
/// sets its pose.
#[test]
fn a_fixed_part_goes_where_it_is_dragged() {
    let library = shelf();
    let part = Part::new();
    let context = Context::new(&part, "a", &library);
    let mut state = State::new();
    let edit = Edit {
        args: &mut args(true, &[]),
        session: &mut PartSession::default(),
        selection: &mut Vec::new(),
        state: &mut state,
    };
    AddPart.event(
        context,
        edit,
        &CanvasEvent::Move {
            key: PART.into(),
            from: v(0.5, 0.5, 1.0),
            to: v(2.5, 0.0, 1.0),
            pointer: down_at(v(2.5, 0.0, 1.0)),
            done: true,
            shift: false,
        },
    );
    assert_close(
        apply(&moved_to(&state, "a"), [0.0; 3]),
        [2.0, -0.5, 0.0],
        1e-12,
    );
}

/// Mates that cannot all hold are listed as such.
#[test]
fn mates_that_cannot_hold_are_said_so() {
    let library = shelf();
    let before = with_a(&library);
    let args = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Distance { value: n(1.0) }),
                [base("b"), base("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Distance { value: n(2.0) }),
                [base("b"), base("a")],
            ),
        ],
    );
    let built = AddPart.apply(before.clone(), "b", &args, &library).unwrap();
    let context = Context::new(&before, "b", &library).built(Some(&built));
    let form = AddPart.form(context, &args, &PartSession::default(), &[]);
    let Some(Control::List { items, .. }) = form.dialog.get("mates") else {
        panic!("the mates are a list");
    };
    assert!(items.iter().any(|i| i.tone == Tone::Error), "{items:?}");
}

/// Without a file there is nothing to place.
#[test]
fn a_part_needs_a_file() {
    let mut args = args(true, &[]);
    args.file.clear();
    let err = AddPart.apply(Part::<S>::new(), "a", &args, &shelf());
    assert!(err.is_err());
}
