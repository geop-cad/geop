//! The engine is operation-free: an [`Editor`] over a registry of two test
//! operations and a drawer — no panel, no tool, no settler — edits their
//! programs the way a front end drives it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, DatumKind},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    operation::Operation,
    part::StepContext,
    placed::Located,
    reader::Reader,
    target::{Target, TargetReference},
};
use serde::{Deserialize, Serialize};

use super::{Command, Editor, Update};
use crate::{
    editable::{Context, Editable, OperationGroup, OperationInfo, OperationTier},
    editing::{CanvasEvent, Event, Form},
    registry::{Drawer, Registry, StructureItem},
    testing::{click_at, editor_with, from_above, handled},
    view::{PartView, ViewDatum},
};

/// A point a step put down: the target named after the step.
#[derive(Clone, Debug, PartialEq)]
struct Point(Vector3);

impl Target for Point {}

impl Kind for Point {
    const NAME: &'static str = "point";
}

impl HasElements<Point> for Point {
    type Element<'a> = &'a Point;

    fn element(&self, name: &str) -> GeopResult<&Point> {
        whole(self, name)
    }
}

/// The point the step `step` put down.
fn point(step: &str) -> ElementReference<Point, Point> {
    ElementReference::whole(TargetReference::new(step))
}

/// Puts a point down where it is clicked, on the plane `z = 0`.
struct Place;

impl Operation for Place {
    const KIND: &'static str = "place";
    type Args = Vector3;

    fn run(ctx: &mut StepContext, at: &Vector3) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&TargetReference::new(step), at, |_, at| Ok(Point(*at)))
    }
}

impl Editable for Place {
    const INFO: OperationInfo = OperationInfo::new(
        "Place",
        "Put a point down.",
        OperationGroup::Sketch,
        OperationTier::Big,
    );
    type EditingState = ();

    fn new_args(_: &Context) -> Vector3 {
        Vector3::zero()
    }

    fn presentation(_: &Context, _: &Vector3, _: &()) -> Form {
        Form::default()
    }

    fn process_user_event(_: &Context, at: &Vector3, _: &(), event: Event) -> (Vector3, ()) {
        let Event::Canvas(CanvasEvent::Click { pointer, .. }) = event else {
            return (*at, ());
        };
        let up = Vector3::axis(2);
        match pointer.ray.intersect_plane(&Vector3::zero(), &up) {
            // Where it was clicked, a free choice — so sharp — and on the
            // plane.
            Some((_, hit)) => {
                let [x, y, _] = hit.sharpen().to_array();
                (Vector3::from_array([x, y, Scalar::ZERO]), ())
            }
            None => (*at, ()),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct RaiseArgs {
    /// The step whose point is raised.
    from: String,
    by: f64,
}

/// Puts a point down above the point another step put down.
struct Raise;

impl Operation for Raise {
    const KIND: &'static str = "raise";
    type Args = RaiseArgs;

    fn run(ctx: &mut StepContext, args: &RaiseArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&TargetReference::new(step), args, |reader, args| {
            let below = reader.retrieve_element(&point(&args.from))?;
            let by = Vector3::from_array([0.0, 0.0, args.by].map(Scalar::from_f64));
            Ok(Point(below.0.add(&by)))
        })
    }
}

impl Editable for Raise {
    const INFO: OperationInfo = OperationInfo::new(
        "Raise",
        "Put a point down above another.",
        OperationGroup::Sketch,
        OperationTier::Small,
    );
    type EditingState = ();

    /// Above the last point put down, by one.
    fn new_args(context: &Context) -> RaiseArgs {
        let last = context.before.targets_of::<Point>().pop();
        RaiseArgs {
            from: last.map(|t| t.name().to_string()).unwrap_or_default(),
            by: 1.0,
        }
    }

    fn presentation(_: &Context, _: &RaiseArgs, _: &()) -> Form {
        Form::default()
    }

    fn process_user_event(_: &Context, args: &RaiseArgs, _: &(), _: Event) -> (RaiseArgs, ()) {
        (args.clone(), ())
    }
}

/// Draws every point as a point datum, and lists it.
struct Points;

impl Drawer for Points {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        for target in reader.targets_of::<Point>() {
            let Point(at) = reader.retrieve_element(&point(target.name()))?;
            view.datums.push(ViewDatum {
                name: target.name().to_string(),
                kind: DatumKind::Point,
                frame: CoordinateSystem::world_at(*at),
                components: Vec::new(),
            });
        }
        Ok(())
    }

    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        Ok((reader.targets_of::<Point>().iter())
            .map(|target| StructureItem::drawn("point", "Points", target.name()))
            .collect())
    }
}

/// What the editor draws, by name: where each point is.
fn drawn(update: &Update) -> Vec<(String, [f64; 3])> {
    let scene = update.scene.as_ref().expect("a scene");
    (scene.part.datums.iter())
        .map(|d| {
            (
                d.name.clone(),
                d.frame.origin().to_array().map(|c| c.to_f64()),
            )
        })
        .collect()
}

/// Puts a point down by a new step of `kind`, clicked at `(x, y)`, and
/// commits it.
fn put(editor: &mut Editor, kind: &str, x: f64, y: f64) -> Update {
    handled(editor, Command::New { kind: kind.into() });
    handled(editor, click_at(from_above(x, y)));
    handled(editor, Command::Commit)
}

#[test]
fn an_editor_of_a_registry_of_its_own_edits_programs() -> Result<(), GeopError> {
    let registry = (Registry::default().operation(Place))
        .and_then(|r| r.operation(Raise))
        .and_then(|r| r.drawer(Points))?;
    let mut editor = editor_with(registry, Default::default());

    let shown = handled(&mut editor, Command::Show);
    let program = shown.program.expect("the program is sent");
    let kinds: Vec<&str> = program.operations.iter().map(|o| o.kind).collect();
    assert_eq!(kinds, ["place", "raise"]);
    assert!(program.panels.is_empty() && program.tool.is_none());
    assert!(program.inspectors.is_empty() && program.examples.is_empty());

    // Two points clicked down, one raised above the second.
    put(&mut editor, "place", 2.0, 3.0);
    put(&mut editor, "place", 5.0, 0.0);
    put(&mut editor, "raise", 0.0, 0.0);
    // The commit shows what the step's preview showed: the viewer has it,
    // and is sent it whole when it asks.
    let update = handled(&mut editor, Command::Show);
    let ids: Vec<&str> = editor
        .program()
        .steps
        .iter()
        .map(|s| s.id.as_str())
        .collect();
    assert_eq!(ids, ["place1", "place2", "raise1"]);
    assert_eq!(
        drawn(&update),
        [
            ("place1".to_string(), [2.0, 3.0, 0.0]),
            ("place2".to_string(), [5.0, 0.0, 0.0]),
            ("raise1".to_string(), [5.0, 0.0, 1.0]),
        ]
    );
    let structure = update.scene.expect("a scene").structure;
    assert!(
        (structure.iter()).all(|i| i.kind_label == "Points" && i.visible == Some(true)),
        "{structure:?}"
    );

    // Run only the first step: only its point is drawn, and new steps go
    // after it.
    let update = handled(&mut editor, Command::Seek { marker: Some(1) });
    assert_eq!(
        update.program.as_ref().expect("the program is sent").marker,
        1
    );
    assert_eq!(drawn(&update), [("place1".to_string(), [2.0, 3.0, 0.0])]);
    let update = handled(&mut editor, Command::Seek { marker: None });
    assert_eq!(drawn(&update).len(), 3);

    // What no registry offers is refused by name, and changes nothing.
    let before = editor.program().clone();
    let refused = |editor: &mut Editor, command| editor.handle(command).error.unwrap_or_default();
    let panel = Command::Panel {
        key: "joints".into(),
        payload: serde_json::Value::Null,
    };
    assert!(refused(&mut editor, panel).contains("there is no panel `joints`"));
    let tool = Command::Tool {
        key: Some("drag".into()),
    };
    assert!(refused(&mut editor, tool).contains("there is no tool `drag`"));
    let example = Command::LoadExample { name: "box".into() };
    assert!(refused(&mut editor, example).contains("there is no example"));
    assert_eq!(*editor.program(), before);

    // Undone, the raised point goes; with no settler, the state is kept
    // as it is.
    let update = handled(&mut editor, Command::Undo);
    assert_eq!(drawn(&update).len(), 2);
    assert!(editor.program().state.is_empty());
    Ok(())
}

/// A bug report attaches what the user was doing: the step being edited,
/// which a failing step never is committed, with its arguments and its
/// error as the dialog shows it — besides the program and the kernel.
/// Asking changes nothing.
#[test]
fn a_report_carries_the_step_being_edited() -> Result<(), GeopError> {
    let registry = (Registry::default().operation(Place))
        .and_then(|r| r.operation(Raise))
        .and_then(|r| r.drawer(Points))?;
    let mut editor = editor_with(registry, Default::default());
    put(&mut editor, "place", 2.0, 3.0);
    // Raising a point no step put down: it does not build.
    handled(
        &mut editor,
        Command::New {
            kind: "raise".into(),
        },
    );
    let args = serde_json::json!({ "from": "nowhere", "by": 2.5 });
    let program = editor.program().clone();
    let shown = handled(&mut editor, Command::Report);
    let report = shown.report.expect("a report");
    let step = report.step.expect("the step being edited");
    assert_eq!(step.step.operation, "raise");
    assert_eq!((&step.id, step.index, &step.placed), (&None, 1, &None));
    assert_eq!(step.step.args["from"], "place1");
    assert!(step.error.is_none() && step.missing.is_empty(), "{step:?}");
    assert_eq!(report.program, program);
    assert_eq!(report.version, env!("CARGO_PKG_VERSION"));
    // Still editing it: the report changed nothing.
    assert!(shown.step.is_some() && shown.program.is_none());

    // Committed and opened again with an argument it fails with: the
    // error as the dialog says it.
    handled(&mut editor, Command::Commit);
    let mut failing = editor.program().clone();
    failing.steps[1].args = args.clone();
    handled(
        &mut editor,
        Command::Load {
            program: failing,
            path: None,
        },
    );
    handled(
        &mut editor,
        Command::Open {
            id: "raise1".into(),
            placed: None,
        },
    );
    let shown = handled(&mut editor, Command::Report);
    let report = shown.report.expect("a report");
    let step = report.step.expect("the step being edited");
    assert_eq!(step.id.as_deref(), Some("raise1"));
    assert_eq!(step.step.args, args);
    let error = step.error.expect("it does not build");
    assert_eq!(
        Some(&error),
        shown.step.as_ref().and_then(|s| s.error.as_ref())
    );
    assert!(error.contains("nowhere"), "{error}");

    // With no step open, the report is of the program alone.
    handled(&mut editor, Command::Cancel);
    let report = handled(&mut editor, Command::Report)
        .report
        .expect("a report");
    assert!(report.step.is_none());
    assert_eq!(report.program, *editor.program());
    Ok(())
}

/// A step edited with its preview off is not built — however long it would
/// take — and its dialog says nothing of whether it builds; the part shown
/// is the part before it. Committed, it is built then, once; previewed
/// again, it is built and shown.
#[test]
fn a_step_not_previewed_is_built_only_when_committed() -> Result<(), GeopError> {
    let registry = (Registry::default().operation(Place))
        .and_then(|r| r.operation(Raise))
        .and_then(|r| r.drawer(Points))?;
    let mut editor = editor_with(registry, Default::default());
    put(&mut editor, "place", 2.0, 3.0);
    let built = |editor: &Editor| -> Vec<String> {
        let part = editor.part();
        (part.targets_of::<Point>().iter())
            .map(|target| target.name().to_string())
            .collect()
    };

    handled(&mut editor, Command::Preview { preview: false });
    handled(
        &mut editor,
        Command::New {
            kind: "raise".into(),
        },
    );
    let clicked = handled(&mut editor, click_at(from_above(0.0, 0.0)));
    assert_eq!(clicked.step.expect("the step is edited").error, None);
    assert_eq!(built(&editor).len(), 1, "the raise is not built");
    let shown = handled(&mut editor, Command::Show);
    assert_eq!(drawn(&shown), [("place1".to_string(), [2.0, 3.0, 0.0])]);

    handled(&mut editor, Command::Preview { preview: true });
    assert_eq!(built(&editor).len(), 2, "previewed again, it is built");
    handled(&mut editor, Command::Preview { preview: false });
    assert_eq!(built(&editor).len(), 1);

    handled(&mut editor, Command::Commit);
    let shown = handled(&mut editor, Command::Show);
    assert_eq!(drawn(&shown).len(), 2, "committed, it is built");
    Ok(())
}

/// The key a front end sends, as the step being edited gets it.
fn key(key: &str) -> Command {
    Command::Event {
        event: crate::editing::StepEditEvent::Key { key: key.into() },
    }
}

/// With nothing in progress in the step, Enter commits it — one edit to
/// undo — and Escape cancels it; a step cancelled once edited is no edit
/// lost: undo opens it again, as it was left, and cancelled again as it
/// was, redo does.
#[test]
fn enter_commits_and_escape_cancels_a_step_at_rest() -> Result<(), GeopError> {
    let registry = (Registry::default().operation(Place)).and_then(|r| r.drawer(Points))?;
    let mut editor = editor_with(registry, Default::default());

    handled(
        &mut editor,
        Command::New {
            kind: "place".into(),
        },
    );
    let update = handled(&mut editor, key("Escape"));
    assert!(update.step.is_none(), "Escape cancels a step not edited");
    assert!(editor.program().steps.is_empty());
    let update = handled(&mut editor, Command::Undo);
    assert!(
        update.step.is_none(),
        "an untouched step is not opened again"
    );

    handled(
        &mut editor,
        Command::New {
            kind: "place".into(),
        },
    );
    handled(&mut editor, click_at(from_above(2.0, 3.0)));
    let update = handled(&mut editor, key("Escape"));
    assert!(update.step.is_none(), "Escape cancels an edited step too");
    assert!(editor.program().steps.is_empty());
    assert!(
        update.program.is_some_and(|p| p.can_undo),
        "the cancel is undone"
    );
    let update = handled(&mut editor, Command::Undo);
    let step = update.step.expect("undo opens the step cancelled again");
    assert!(step.can_undo, "with its own edit to undo");
    // Cancelled again as it was, it is that cancel again: undo goes on
    // past it, redo opens it once more.
    let update = handled(&mut editor, key("Escape"));
    assert!(update.step.is_none());
    assert!(update.program.is_some_and(|p| !p.can_undo && p.can_redo));
    let update = handled(&mut editor, Command::Redo);
    assert!(update.step.is_some(), "redo opens it again");
    let update = handled(&mut editor, key("Enter"));
    assert!(update.step.is_none(), "Enter commits");
    assert_eq!(
        editor.program().steps.len(),
        1,
        "the click it was cancelled with"
    );
    let update = handled(&mut editor, Command::Undo);
    assert!(update.program.is_some_and(|p| p.program.steps.is_empty()));
    Ok(())
}

/// What is lit is the editor's to say: what [`Command::Light`] points at
/// is sent as what it lights, once, and again only when it changes.
#[test]
fn the_editor_sends_what_is_lit_when_it_changes() -> Result<(), GeopError> {
    let registry = (Registry::default().operation(Place)).and_then(|r| r.drawer(Points))?;
    let mut editor = editor_with(registry, Default::default());
    let update = put(&mut editor, "place", 2.0, 3.0);
    let datums = update.scene.as_ref().map_or_else(
        || {
            handled(&mut editor, Command::Show)
                .scene
                .expect("a scene")
                .part
                .datum_target
                .clone()
        },
        |s| s.part.datum_target.clone(),
    );
    let point = PickedElement {
        target: Located::in_instance("", &datums),
        kind: "datum".into(),
        name: "place1".into(),
    };
    let light = |elements: Vec<PickedElement>| Command::Light { elements };

    let update = handled(&mut editor, light(vec![point.clone()]));
    let lit = update.lit.expect("what is lit changed");
    assert_eq!(lit.keys().collect::<Vec<_>>(), [""]);
    assert!(lit[""].datums.contains("place1"), "{lit:?}");
    assert!(
        update.scene.is_none() && update.program.is_none(),
        "lighting changes nothing else"
    );

    let again = handled(&mut editor, light(vec![point]));
    assert!(again.lit.is_none(), "the same lighting is not sent again");
    let off = handled(&mut editor, light(Vec::new()));
    assert_eq!(
        off.lit,
        Some(Default::default()),
        "nothing is lit once it leaves"
    );
    Ok(())
}
