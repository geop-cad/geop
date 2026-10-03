//! Editing a placed part: choosing its file, where it goes and whether it
//! stays put, its mates — and dragging it, as far as its mates let it.
//!
//! Where the part is, is the program's parameter `<id>.pose`: the placement
//! fields set it, and the editor solves the program's mates from there
//! after every edit (see `geop_cad_base::editor`). A drag is the step's to
//! ask for, the solving the editor's: while the part is dragged, the form
//! says what point of it is pulled where ([`Form::drags`]).

use std::collections::BTreeMap;

use geop_core_math::{primitives::Pose, scalars::Scalar, vector::Vector3};
use geop_ops::{
    Context, Design, EntityRef,
    assembly::{Drag, Mate, MateKind},
    operation::Role,
    part::{ParamValue, pose_parameter},
    ui::{
        Action, CanvasEvent, Choice, Control, Edit, Form, ListItem, Number, Shape, Style, Tone,
        Unit, Value, Visual,
    },
};

use crate::AddPartArgs;

/// The key of the placed part, as drawn to be dragged.
pub const PART: &str = "part";

/// What an entity of a mate of `kind` can be: what [`Mate::needs`] says.
fn mate_roles(kind: &MateKind) -> &'static [Role] {
    match kind {
        MateKind::Coincident | MateKind::Distance { .. } => {
            &[Role::Point, Role::Line, Role::Plane, Role::Round]
        }
        MateKind::Concentric => &[Role::Round, Role::Line],
        MateKind::Parallel | MateKind::Perpendicular | MateKind::Angle { .. } => {
            &[Role::Line, Role::Plane, Role::Round]
        }
    }
}

/// The kinds of mate, as added: by name, with a value to start from.
fn kinds() -> [(&'static str, MateKind, &'static str); 6] {
    [
        (
            "coincident",
            MateKind::Coincident,
            "Two points, a point and a line or a plane, two lines or two planes touch.",
        ),
        (
            "concentric",
            MateKind::Concentric,
            "Two round edges or faces, or lines, share their axis.",
        ),
        (
            "parallel",
            MateKind::Parallel,
            "Two lines or planes, or a line and a plane, run parallel.",
        ),
        (
            "perpendicular",
            MateKind::Perpendicular,
            "Two lines or planes stand square, or a line along a plane's normal.",
        ),
        (
            "distance",
            MateKind::Distance { value: Design::ONE },
            "Two points, a point and a line or plane, or parallel lines or planes, keep a distance.",
        ),
        (
            "angle",
            MateKind::Angle {
                value: Design::from_i64(90),
            },
            "Two lines or planes meet at an angle, in degrees.",
        ),
    ]
}

/// The temporary state of editing a placed part.
#[derive(Default)]
pub struct PartSession {
    /// The mate whose entities are picked.
    pub selected: Option<String>,
    /// While the part is dragged: what is pulled where — the point grabbed
    /// kept from the press on, so every event of the drag is measured from
    /// it.
    drag: Option<Drag<Design>>,
    /// Whether the drag was released: it pulls once more — for the editor
    /// to solve with where it was let go — and is gone at the next event.
    released: bool,
}

/// Where the part is placed as the step `id`: the program's parameter, or
/// where it was built — at first, before the parameter is solved for.
fn pose_of<S: Scalar>(context: Context<'_, S>) -> Pose<Design> {
    match context.state.get(&pose_parameter(context.id)) {
        Some(ParamValue::Pose(pose)) => *pose,
        _ => context
            .built
            .and_then(|part| part.instance(part.instance_id(context.id).ok()?).ok())
            .map_or(Pose::identity(), |instance| instance.pose.cast()),
    }
}

/// The value a kind of mate holds, if any — as the dialog shows it.
fn value_of(kind: &MateKind) -> Option<f64> {
    match *kind {
        MateKind::Distance { value } | MateKind::Angle { value } => Some(value.to_f64()),
        _ => None,
    }
}

/// The lowest `m1`, `m2`, ... not yet a mate's id.
fn fresh_mate_id(mates: &BTreeMap<String, Mate>) -> String {
    (1..)
        .map(|n| format!("m{n}"))
        .find(|id| !mates.contains_key(id))
        .expect("some number is free")
}

/// The file to place, whether it stays put, where it goes, its mates — the
/// one selected with its entities to pick — and the part as placed, to
/// drag.
pub(crate) fn form<'a, S: Scalar>(
    context: Context<'a, S>,
    args: &AddPartArgs,
    session: &PartSession,
) -> Form<'a, S, AddPartArgs, PartSession> {
    let mut f = Form::<S, AddPartArgs, PartSession>::new();
    let built = context
        .built
        .and_then(|part| Some((part, part.instance_id(context.id).ok()?)));

    let mut files = context.library.files();
    files.sort();
    if !args.file.is_empty() && !files.contains(&args.file) {
        files.push(args.file.clone());
    }
    if files.is_empty() {
        f.text(
            "no_files",
            "There are no other program files to place: make one first.",
            Tone::Hint,
        );
    }
    let options = std::iter::once(Choice::new("", "Choose a file…"))
        .chain(
            files
                .iter()
                .map(|file| Choice::new(file.clone(), file.clone())),
        )
        .collect();
    f.select("file", "file", args.file.clone(), options, |args, file| {
        args.file = file.to_string()
    });
    f.checkbox("fixed", "fixed in place", args.fixed, |args, fixed| {
        args.fixed = fixed;
    });
    f.checkbox(
        "flexible",
        "flexible: its parts move with this program's mates",
        args.flexible,
        |args, flexible| args.flexible = flexible,
    );

    // Shown as a position and angles — what people read — though a pose is
    // a quaternion: setting one angle rebuilds it from the three shown.
    f.heading("placement_heading", "Placement");
    let pose = pose_of(context);
    let (position, angles) = (pose.position(), pose.euler_degrees());
    let parameter = pose_parameter(context.id);
    for (k, axis) in ["x", "y", "z"].into_iter().enumerate() {
        let number = Number::new(axis, position[k].to_f64(), Unit::Length);
        let parameter = parameter.clone();
        f.on(axis, move |edit, value| {
            if let Value::Number(v) = value {
                let mut position = position;
                position[k] = Design::from_f64(v);
                let moved = ParamValue::Pose(pose.with_position(position));
                edit.state.insert(parameter.clone(), moved);
            }
        });
        f.dialog.push(axis, Control::Number(number));
    }
    for (k, axis) in ["x", "y", "z"].into_iter().enumerate() {
        let key = format!("rotation_{axis}");
        let number = Number::new(format!("rotation {axis}"), angles[k], Unit::Angle);
        let parameter = parameter.clone();
        f.on(&key, move |edit, value| {
            if let Value::Number(v) = value {
                let mut angles = angles;
                angles[k] = v;
                if let Ok(turned) = Pose::from_euler(position, angles.map(Design::from_f64)) {
                    edit.state
                        .insert(parameter.clone(), ParamValue::Pose(turned));
                }
            }
        });
        f.dialog.push(key, Control::Number(number));
    }

    f.heading("mates_heading", "Mates");
    let failed = built
        .and_then(|(part, _)| part.check_mates().ok())
        .map(|report| report.failed)
        .unwrap_or_default();
    let namer_prefix = format!("add_part({},", context.id);
    let items = args
        .mates
        .iter()
        .map(|(id, mate)| {
            let key = format!("mate:{id}");
            let mate_id = id.clone();
            f.on(key.clone(), move |edit, value| {
                let args = &mut *edit.args;
                match value {
                    Value::Press => {
                        edit.session.selected = (edit.session.selected.as_deref()
                            != Some(&mate_id))
                        .then(|| mate_id.clone());
                    }
                    Value::Remove => {
                        args.mates.remove(&mate_id);
                        if edit.session.selected.as_deref() == Some(&mate_id) {
                            edit.session.selected = None;
                        }
                    }
                    Value::Number(v) => {
                        if let Some(mate) = args.mates.get_mut(&mate_id) {
                            match &mut mate.kind {
                                MateKind::Distance { value } | MateKind::Angle { value } => {
                                    *value = Design::from_f64(v)
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            });
            let mut item = ListItem::new(key, mate.kind.label());
            let entities: Vec<String> = mate.entities.iter().map(EntityRef::label).collect();
            let holds = !failed.contains(&format!("{namer_prefix}{id})"));
            // Two entities of one part keep where they are to each other
            // whatever moves: such a mate holds nothing together.
            let one_part = mate.pair().and_then(|[a, b]| {
                let (a, b) = (a.split_instance()?.0, b.split_instance()?.0);
                (a == b).then_some(a)
            });
            item.detail = Some(match (mate.pair(), &one_part, holds) {
                (None, _, _) => {
                    format!("pick {} — {}", 2 - mate.entities.len().min(2), mate.needs())
                }
                (Some(_), Some(part), _) => {
                    format!(
                        "{} — both of {part}: holds nothing together",
                        entities.join(" & ")
                    )
                }
                (Some(_), None, true) => entities.join(" & "),
                (Some(_), None, false) => format!("{} — cannot hold", entities.join(" & ")),
            });
            let holds = holds && one_part.is_none();
            item.tone = match (mate.pair(), holds) {
                (None, _) => Tone::Hint,
                (Some(_), true) => Tone::Normal,
                (Some(_), false) => Tone::Error,
            };
            item.selected = session.selected.as_deref() == Some(id);
            item.removable = true;
            item.value = value_of(&mate.kind);
            item
        })
        .collect();
    f.list("mates", items, "None yet.");
    f.actions(
        "add_mate",
        kinds()
            .iter()
            .map(|(name, kind, doc)| Action::new(*name, kind.label()).title(*doc).group("Add"))
            .collect(),
        |edit, name| {
            if let Some((_, kind, _)) = kinds().iter().find(|(n, _, _)| *n == name) {
                let id = fresh_mate_id(&edit.args.mates);
                edit.args.mates.insert(
                    id.clone(),
                    Mate {
                        kind: *kind,
                        entities: Vec::new(),
                    },
                );
                edit.session.selected = Some(id);
            }
        },
    );

    if let Some((id, mate)) = session
        .selected
        .as_ref()
        .and_then(|id| args.mates.get_key_value(id))
    {
        let mate_id = id.clone();
        f.reference(
            &format!("mate:{id}:entities"),
            "entities",
            mate.entities.clone(),
            mate_roles(&mate.kind),
            None,
            true,
            move |edit, mut entities| {
                // Two at most: a third pick replaces the oldest.
                let extra = entities.len().saturating_sub(2);
                entities.drain(..extra);
                let complete = entities.len() == 2;
                if let Some(mate) = edit.args.mates.get_mut(&mate_id) {
                    mate.entities = entities;
                }
                if complete {
                    edit.session.selected = None;
                }
            },
        );
        f.text(
            "mate_hint",
            format!(
                "Pick two entities, each {} — of the placed part, of a part placed before, or of this part itself.",
                mate.needs()
            ),
            Tone::Hint,
        );
    }

    // A fixed part is moved by the event itself; nothing is solved for it.
    f.drags.extend(
        session
            .drag
            .iter()
            .filter(|_| !args.fixed)
            .map(|drag| Drag {
                parameter: drag.parameter.clone(),
                local: drag.local.map(|c| c.cast()),
                target: drag.target.map(|c| c.cast()),
            }),
    );
    if built.is_some() {
        let shape = Shape::Instance {
            name: context.id.to_string(),
        };
        f.visuals
            .push(Visual::new(PART, shape, Style::Free).draggable());
        f.text(
            "drag_hint",
            "Drag the part to move it as far as its mates let it.",
            Tone::Hint,
        );
    }
    f
}

/// A pointer or key event: the part dragged (see the module docs); Delete
/// removes the mate selected, Escape stops editing it.
pub(crate) fn event<S: Scalar>(
    context: Context<'_, S>,
    edit: Edit<'_, AddPartArgs, PartSession>,
    event: &CanvasEvent<S>,
) {
    let Edit {
        args,
        session,
        state,
        ..
    } = edit;
    if std::mem::take(&mut session.released) {
        session.drag = None;
    }
    match event {
        CanvasEvent::Move {
            key,
            from,
            to,
            done,
        } if key == PART => {
            let pose = pose_of(context);
            // Where the pointer is: a free choice, made by whoever moved it.
            let target: Vector3<Design> = to.map(|c| c.cast()).sharpen();
            let local = match &session.drag {
                Some(drag) => drag.local,
                None => pose.inverse().apply(&from.map(|c| c.cast())).sharpen(),
            };
            if args.fixed {
                // Nothing mates move: the part goes where it is dragged.
                let at = pose.apply(&local);
                let moved = pose.with_position(pose.position().add(&target.sub(&at)).sharpen());
                state.insert(pose_parameter(context.id), ParamValue::Pose(moved));
            }
            session.drag = Some(Drag {
                parameter: pose_parameter(context.id),
                local,
                target,
            });
            session.released = *done;
        }
        CanvasEvent::Key { key } if key == "Escape" => session.selected = None,
        CanvasEvent::Key { key } if key == "Delete" || key == "Backspace" => {
            if let Some(id) = session.selected.take() {
                args.mates.remove(&id);
            }
        }
        _ => {}
    }
}
