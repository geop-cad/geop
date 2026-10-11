//! Editing a placed part: choosing its file, where it goes and whether it
//! stays put, its mates — and dragging it, as far as its mates let it.
//!
//! Where the part is, is the program's state `<id>.pose`, which the editor
//! solves the program's mates for after every edit. The step only says
//! where it wants the part pulled ([`Form::pulls`]): while it is dragged,
//! what point of it is pulled where; once its placement is typed in, three
//! points of it pulled to where that puts them. It is dragged by a gizmo at
//! its origin — moved along or turned about its own axes or the world's —
//! or grabbed anywhere and dragged in the plane facing the eye. A part no
//! mate may move goes where it is pulled.
//!
//! A joint is made between frames, one picked on each part: a click on a
//! face, an edge or a point puts one there (see
//! [`geop_ops::operation::Aspects::frame_on`]), and the hover shows the
//! frame it would pick, as three arrows. A coupling is made of two joints
//! already made, offered by what they join.
//!
//! A joint's coordinates are the program's state too: set in the dialog,
//! they are held there while the step is edited ([`Pull::Value`]), and
//! the parts move to them.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{Pose, Quaternion},
    scalars::Scalar,
};
use geop_core_program::operation::Context;
use geop_ops::{
    Design, EntityRef, Namer, ParamValue,
    model::pose_parameter,
    operation::Input,
    parameters::{COLOR, ParameterKind, Values, validate_color},
    placed::{instance, library},
    solving::{SettingsState, check, settings},
    ui::{
        Action, CanvasEvent, Choice, Control, Edit, EditForm, Event, Gizmo, ListItem, Number, Pull,
        Shape, Style, Tone, Unit, Value, Visual,
    },
};

use crate::{
    AddPartArgs, Anchor, CouplingKind, JointKind, Kind, Mate, MateKind, Motion, Transmission,
    mechanism::mate_holds,
};

/// The form of a step placing a part, and what setting its fields does.
type PartForm = EditForm<'static, Design, AddPartArgs, PartEditing>;

/// The key of the placed part, as drawn to be dragged.
pub const PART: &str = "part";

/// The kinds of mate, as added: by name, with a value to start from, what
/// it does, and the group it is offered in.
fn kinds() -> Vec<(&'static str, MateKind, &'static str, &'static str)> {
    let constraint = |name, kind, doc| (name, MateKind::Constraint(kind), doc, "Add");
    let joint = |name, kind, doc| (name, MateKind::Joint(kind), doc, "Joint");
    let coupling = |name, transmission, value, reverse, doc| {
        let kind = CouplingKind {
            transmission,
            value,
            reverse,
        };
        (name, MateKind::Coupling(kind), doc, "Couple")
    };
    vec![
        (
            "fixed",
            MateKind::Anchor(Anchor::Fixed),
            "The part stays where it is: no mate moves it.",
            "Add",
        ),
        constraint(
            "coincident",
            Kind::Coincident,
            "Two points, a point and a line or a plane, two lines or two planes touch.",
        ),
        constraint(
            "concentric",
            Kind::Concentric,
            "Two round edges or faces, or lines, share their axis.",
        ),
        constraint(
            "parallel",
            Kind::Parallel,
            "Two lines or planes, or a line and a plane, run parallel.",
        ),
        constraint(
            "perpendicular",
            Kind::Perpendicular,
            "Two lines or planes stand square, or a line along a plane's normal.",
        ),
        constraint(
            "distance",
            Kind::Distance { value: Design::ONE },
            "Two points, a point and a line or plane, or parallel lines or planes, keep a distance.",
        ),
        constraint(
            "angle",
            Kind::Angle {
                value: Design::from_i64(90),
            },
            "Two lines or planes meet at an angle, in degrees.",
        ),
        joint(
            "revolute",
            JointKind::Revolute {
                min: None,
                max: None,
            },
            "The second part turns about the first one's frame's z axis, and nothing else — a hinge: pick a frame on each part.",
        ),
        joint(
            "slider",
            JointKind::Slider {
                min: None,
                max: None,
            },
            "The second part slides along the first one's frame's z axis, and nothing else: pick a frame on each part.",
        ),
        joint(
            "cylindrical",
            JointKind::Cylindrical,
            "The second part turns about the first one's frame's z axis and slides along it — a shaft in a bearing.",
        ),
        joint(
            "fastened",
            JointKind::Fastened,
            "The two parts are held together, their frames one: pick a frame on each part.",
        ),
        coupling(
            "gear",
            Transmission::Gear,
            Design::ONE,
            true,
            "Two joints that turn — two gears' hinges — turn together: the first `ratio` times per turn of the second. Add the two joints first.",
        ),
        coupling(
            "rack_pinion",
            Transmission::RackPinion,
            Design::ONE,
            false,
            "A joint that slides — a rack — moves as far as a joint that turns — its pinion — rolls its pitch circle. Add the two joints first.",
        ),
        coupling(
            "screw",
            Transmission::Screw,
            Design::from_ratio(1, 10).expect("ten is not zero"),
            false,
            "A joint that slides moves its lead per turn of a joint that turns — of one cylindrical joint, say. Add the joints first.",
        ),
    ]
}

/// What the editor keeps while a placed part is edited.
#[derive(Clone, Debug, Default)]
pub struct PartEditing {
    /// The mate whose entities are picked.
    pub selected: Option<String>,
    /// What points of the part are pulled where: while it is dragged — the
    /// point grabbed kept from the press on, so every event of the drag is
    /// measured from it; for a turn by the gizmo, three points of its frame
    /// — or after its placement was typed in.
    pulls: Vec<Pull<Design>>,
    /// Whether the pulls are done with: they pull once more — for the
    /// editor to solve with where they were let go — and are gone at the
    /// next event.
    released: bool,
    /// The joint coordinates set in the dialog, by state: kept where they
    /// were set while the step is edited, until the part is dragged.
    held: BTreeMap<String, f64>,
}

/// Where the part the step places is: the program's state, or — before
/// anything was solved for it — the origin.
fn pose_of(context: &Context<Input, ParamValue>) -> Pose<Design> {
    match context.before.retrieve_state(&pose_parameter(context.step)) {
        ParamValue::Pose(pose) => pose,
        _ => Pose::identity(),
    }
}

/// The part placed as `step` pulled to `target` as a whole.
fn pulled_to(step: &str, target: Pose<Design>) -> Vec<Pull<Design>> {
    vec![Pull::Pose {
        parameter: pose_parameter(step),
        target: target.map(|c| c.sharpen()),
    }]
}

/// The value a kind of mate holds, if any — as the dialog shows it.
fn value_of(kind: &MateKind) -> Option<f64> {
    match *kind {
        MateKind::Constraint(Kind::Distance { value } | Kind::Angle { value })
        | MateKind::Coupling(CouplingKind { value, .. }) => Some(value.to_f64()),
        _ => None,
    }
}

/// Sets the joint coordinate `parameter` to `value` and keeps it there
/// while the step is edited: the parts move to it (see [`Pull::Value`]).
fn set_joint(editing: &mut PartEditing, parameter: &str, value: f64) {
    editing.held.insert(parameter.to_string(), value);
}

/// The value a kind of mate holds is `v` now.
fn set_value(kind: &mut MateKind, v: f64) {
    let v = Design::from_f64(v);
    match kind {
        MateKind::Constraint(Kind::Distance { value } | Kind::Angle { value })
        | MateKind::Coupling(CouplingKind { value, .. }) => *value = v,
        _ => {}
    }
}

/// The fields of the mate `id` selected: the entities to pick for a
/// constraint or a joint; a joint's coordinates — set, they hold — and its
/// limits; the joints a coupling ties, and how.
fn selected(
    f: &mut PartForm,
    id: &str,
    mate: &Mate,
    joint: Option<&SettingsState>,
    joints: &[SettingsState],
) {
    if let MateKind::Coupling(kind) = mate.kind {
        // The joints' fields, the value's — its name and unit — as each
        // transmission reads.
        let (labels, value_name, unit) = match kind.transmission {
            Transmission::Gear => (["driving joint", "driven joint"], "ratio", Unit::Fraction),
            Transmission::RackPinion => (
                ["pinion's joint", "rack's joint"],
                "pitch radius",
                Unit::Length,
            ),
            Transmission::Screw => (["turning joint", "sliding joint"], "lead", Unit::Length),
        };
        // What each of the two joints has to do: turn or slide.
        let motions = kind.transmission.motions();
        for (k, label) in labels.into_iter().enumerate() {
            let moves =
                |j: &&SettingsState| j.values.iter().any(|v| v.measure == motions[k].measure());
            let options: Vec<Choice> = std::iter::once(Choice::new("", "Choose a joint…"))
                .chain(joints.iter().filter(moves).map(|j| {
                    let what = if j.between.is_empty() {
                        j.name.clone()
                    } else {
                        format!("{} — {}", j.name, j.between.join(" ↔ "))
                    };
                    Choice::new(j.name.clone(), format!("{} · {what}", j.kind))
                }))
                .collect();
            if options.len() == 1 {
                f.form.dialog.text(
                    &format!("mate:{id}:no_joint{k}"),
                    format!(
                        "No joint {}s yet: add one first — Joint, then {} — to the parts placed so far.",
                        motions[k].verb(),
                        (JointKind::ALL.iter())
                            .filter(|j| j.moves(motions[k]))
                            .map(JointKind::label)
                            .collect::<Vec<_>>()
                            .join(" or "),
                    ),
                    Tone::Error,
                );
            }
            let mate_id = id.to_string();
            let value = mate.joints.get(k).cloned().unwrap_or_default();
            f.select(
                &format!("mate:{id}:joint{k}"),
                label,
                value,
                options,
                true,
                move |args, joint| {
                    if let Some(mate) = args.mates.get_mut(&mate_id) {
                        mate.joints.resize(2, String::new());
                        mate.joints[k] = joint.to_string();
                        if mate.joints.iter().all(String::is_empty) {
                            mate.joints.clear();
                        }
                    }
                },
            );
        }
        let mate_id = id.to_string();
        f.number(
            &format!("mate:{id}:value"),
            Number::new(value_name, kind.value.to_f64(), unit),
            move |args, v| {
                if let Some(mate) = args.mates.get_mut(&mate_id) {
                    set_value(&mut mate.kind, v);
                }
            },
        );
        let mate_id = id.to_string();
        f.checkbox(
            &format!("mate:{id}:reverse"),
            "the other way round",
            kind.reverse,
            move |args, r| {
                if let Some(MateKind::Coupling(CouplingKind { reverse, .. })) =
                    args.mates.get_mut(&mate_id).map(|m| &mut m.kind)
                {
                    *reverse = r;
                }
            },
        );
        f.form.dialog.text(
            "mate_hint",
            format!(
                "Choose {} from the joints of the parts placed so far, here or in an earlier step.",
                mate.needs()
            ),
            Tone::Hint,
        );
        return;
    }

    if mate.is_fixed() {
        f.form.dialog.text(
            "mate_hint",
            "No mate moves this part: it goes where it is dragged.",
            Tone::Hint,
        );
        return;
    }

    let mate_id = id.to_string();
    f.reference(
        &format!("mate:{id}:entities"),
        if matches!(mate.kind, MateKind::Joint(_)) {
            "frames"
        } else {
            "entities"
        },
        mate.entities.clone(),
        mate.kind.roles(),
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
    let hint = match mate.kind {
        MateKind::Joint(_) => {
            "Click a face, an edge, a point or a datum on each of the two parts: where you click, a \
             frame is put, and the joint turns or slides along its z axis, the blue arrow. A planar \
             face has it at its middle, or at the corner or side you are near, along its normal; a \
             cylinder, on its axis; a circular edge, at its center, along its axis; a straight edge, \
             at its middle, or at the end you are near, along it; a point, along the world's z. \
             The first part holds, the second moves."
                .to_string()
        }
        _ => format!(
            "Pick two entities, each {} — of the placed part, of a part placed before, or of this part itself.",
            mate.needs()
        ),
    };
    f.form.dialog.text("mate_hint", hint, Tone::Hint);

    let MateKind::Joint(kind) = mate.kind else {
        return;
    };
    for motion in kind.motions() {
        let unit = match motion {
            Motion::Turn => Unit::Angle,
            Motion::Slide => Unit::Length,
        };
        let value = joint.and_then(|j| j.values.iter().find(|v| v.measure == motion.measure()));
        if let Some(value) = value {
            let key = format!("mate:{id}:{}", motion.name());
            let parameter = value.parameter.clone();
            f.on(&key, move |edit, value| {
                if let Value::Number(v) = value {
                    set_joint(edit.session, &parameter, v);
                }
            });
            f.form.dialog.push(
                key,
                Control::Number(Number::new(motion.name(), value.value, unit)),
            );
        }
        if !matches!(kind, JointKind::Revolute { .. } | JointKind::Slider { .. }) {
            continue;
        }
        let [min, max] = kind.limits(motion);
        let mate_id = id.to_string();
        // Limited to either side of where it is now, to start with.
        let here = value.map_or(0.0, |v| v.value);
        let reach = match motion {
            Motion::Turn => 90.0,
            Motion::Slide => 1.0,
        };
        f.checkbox(
            &format!("mate:{id}:limited"),
            format!("limit the {}", motion.name()),
            min.is_some() || max.is_some(),
            move |args, limited| {
                if let Some(MateKind::Joint(
                    JointKind::Revolute { min, max } | JointKind::Slider { min, max },
                )) = args.mates.get_mut(&mate_id).map(|m| &mut m.kind)
                {
                    let bound = |v: f64| limited.then(|| Design::from_f64(v));
                    *min = bound(here - reach);
                    *max = bound(here + reach);
                }
            },
        );
        for (k, (bound, label)) in [(min, "min"), (max, "max")].into_iter().enumerate() {
            let Some(bound) = bound else {
                continue;
            };
            let mate_id = id.to_string();
            f.number(
                &format!("mate:{id}:{label}"),
                Number::new(format!("{label} {}", motion.name()), bound.to_f64(), unit),
                move |args, v| {
                    if let Some(MateKind::Joint(
                        JointKind::Revolute { min, max } | JointKind::Slider { min, max },
                    )) = args.mates.get_mut(&mate_id).map(|m| &mut m.kind)
                    {
                        *[min, max][k] = Some(Design::from_f64(v));
                    }
                },
            );
        }
    }
}

/// The lowest `m1`, `m2`, ... not yet a mate's id.
fn fresh_mate_id(mates: &BTreeMap<String, Mate>) -> String {
    (1..)
        .map(|n| format!("m{n}"))
        .find(|id| !mates.contains_key(id))
        .expect("some number is free")
}

/// The key of the field of the placed part's parameter `name`.
fn parameter_key(name: &str) -> String {
    format!("parameter:{name}")
}

/// The placed part's own parameters, each as what it is given here: its
/// colour picked, a number on a slider, a table's row chosen from a list
/// to search.
fn parameters(f: &mut PartForm, context: &Context<Input, ParamValue>, args: &AddPartArgs) {
    if args.file.is_empty() {
        return;
    }
    let mut before = context.before;
    let Ok(library) = library(&mut before) else {
        return;
    };
    // As placed here: what a number not given reads follows what is.
    let Ok(placed) = (library.component(&args.file, &args.parameters))
        .or_else(|_| library.component(&args.file, &Values::new()))
    else {
        return;
    };
    let defined = &placed.parameters;
    if defined.is_empty() {
        return;
    }
    let built = placed.view();
    let given = |name: &str| match args.parameters.get(name) {
        Some(value) => Some(value.clone()),
        None => match built.retrieve_input(name) {
            Some(Input::Value(value)) => Some(value),
            _ => None,
        },
    };
    f.form.dialog.heading("parameters_heading", "Parameters");
    let set = |name: String| {
        move |args: &mut AddPartArgs, value: ParamValue| {
            args.parameters.insert(name.clone(), value);
        }
    };
    if defined.color.is_some() || args.parameters.contains_key(COLOR) {
        let color = match given(COLOR) {
            Some(ParamValue::Text(c)) => c,
            _ => "#a0a8b8".into(),
        };
        let set = set(COLOR.into());
        f.color(&parameter_key(COLOR), COLOR, color, move |args, c| {
            if validate_color(c).is_ok() {
                set(args, ParamValue::Text(c.to_string()));
            }
        });
    }
    for p in &defined.values {
        let key = parameter_key(&p.name);
        let set = set(p.name.clone());
        match &p.kind {
            ParameterKind::Number { min, max, .. } => {
                let value = match given(&p.name) {
                    Some(ParamValue::Number(v)) => v.to_f64(),
                    _ => continue,
                };
                // A slider over what the parameter offers — or, with no
                // range of its own, from nothing to twice its value.
                let lo = min.unwrap_or(0.0_f64.min(2.0 * value));
                let hi = max.unwrap_or(0.0_f64.max(2.0 * value)).max(lo + 1e-9);
                let number = Number::new(&p.name, value, Unit::Length).range(lo, hi);
                f.number(&key, number, move |args, v| {
                    set(args, ParamValue::Number(Design::from_f64(v)))
                });
            }
            ParameterKind::Table { rows, selected, .. } => {
                let value = match given(&p.name) {
                    Some(ParamValue::Text(row)) => row,
                    _ => selected.clone(),
                };
                let options = (rows.iter())
                    .map(|r| Choice::new(r.name.clone(), r.name.clone()))
                    .collect();
                f.select(&key, &p.name, value, options, true, move |args, row| {
                    set(args, ParamValue::Text(row.to_string()))
                });
            }
        }
    }
}

/// The file to place, whether it stays put, where it goes, its mates — the
/// one selected with its entities to pick — and the part as placed, to
/// drag.
pub(crate) fn form(
    context: &Context<Input, ParamValue>,
    args: &AddPartArgs,
    editing: &PartEditing,
) -> PartForm {
    let mut f = PartForm::default();
    let step = context.step.to_string();
    let built = context
        .built
        .filter(|view| view.retrieve_target(&instance(&step)).is_ok());

    let mut before = context.before;
    let mut files: Vec<String> = match library(&mut before) {
        Ok(library) => (library.files().into_iter())
            .filter(|f| geop_ops::program::is_program(f))
            .collect(),
        Err(_) => Vec::new(),
    };
    files.sort();
    if !args.file.is_empty() && !files.contains(&args.file) {
        files.push(args.file.clone());
    }
    if files.is_empty() {
        f.form.dialog.text(
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
    f.select(
        "file",
        "file",
        args.file.clone(),
        options,
        true,
        |args, file| args.file = file.to_string(),
    );

    parameters(&mut f, context, args);

    // Shown as a position and angles — what people read — though a pose is
    // a quaternion: setting one angle rebuilds it from the three shown.
    f.form.dialog.heading("placement_heading", "Placement");
    let pose = pose_of(context);
    let (position, angles) = (pose.position(), pose.euler_degrees());
    for (k, axis) in ["x", "y", "z"].into_iter().enumerate() {
        let number = Number::new(axis, position[k].to_f64(), Unit::Length);
        let step = step.clone();
        f.on(axis, move |edit, value| {
            if let Value::Number(v) = value {
                let mut position = position;
                position[k] = Design::from_f64(v);
                edit.session.pulls = pulled_to(&step, pose.with_position(position));
                edit.session.released = true;
            }
        });
        f.form.dialog.push(axis, Control::Number(number));
    }
    for (k, axis) in ["x", "y", "z"].into_iter().enumerate() {
        let key = format!("rotation_{axis}");
        let number = Number::new(format!("rotation {axis}"), angles[k], Unit::Angle);
        let step = step.clone();
        f.on(&key, move |edit, value| {
            if let Value::Number(v) = value {
                let mut angles = angles;
                angles[k] = v;
                if let Ok(turned) = Pose::from_euler(position, angles.map(Design::from_f64)) {
                    edit.session.pulls = pulled_to(&step, turned);
                    edit.session.released = true;
                }
            }
        });
        f.form.dialog.push(key, Control::Number(number));
    }

    f.form.dialog.heading("mates_heading", "Mates");
    // Its own mates — named after the step — not every mate of the part.
    let namer = |id: &str| match Namer::new("add_part", &step) {
        Ok(namer) => namer.name(&[id]),
        Err(_) => String::new(),
    };
    // Every joint of the part as built, its coordinates where they are.
    let joints = built.map(settings).unwrap_or_default();
    let items = (args.mates.iter())
        .map(|(id, mate)| {
            let key = format!("mate:{id}");
            let mate_id = id.clone();
            let joint = joints.iter().find(|j| j.name == namer(id)).cloned();
            let first = joint.as_ref().and_then(|j| j.values.first().cloned());
            let parameter = first.as_ref().map(|v| v.parameter.clone());
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
                    Value::Number(v) => match (args.mates.get_mut(&mate_id), &parameter) {
                        (Some(mate), None) => set_value(&mut mate.kind, v),
                        (Some(_), Some(parameter)) => set_joint(edit.session, parameter, v),
                        (None, _) => {}
                    },
                    _ => {}
                }
            });
            let mut item = ListItem::new(key, mate.kind.label());
            // A fixed mate holds this step's part: it needs no entity, and
            // puts no constraint on where it is.
            let complete = mate.is_complete() || mate.is_fixed();
            let holds = mate.is_fixed() || built.is_some_and(|view| mate_holds(view, &namer(id)));
            // Two entities of one part keep where they are to each other
            // whatever moves: such a mate holds nothing together.
            let one_part = mate.pair().and_then(|[a, b]| {
                let (a, b) = (a.split_instance()?.0, b.split_instance()?.0);
                (a == b).then_some(a)
            });
            let what = match mate.kind {
                MateKind::Anchor(_) => "holds this part where it is".to_string(),
                MateKind::Coupling(_) => mate.joints.join(" & "),
                _ => {
                    let entities: Vec<String> =
                        mate.entities.iter().map(EntityRef::label).collect();
                    entities.join(" & ")
                }
            };
            item.detail = Some(match (complete, &one_part, holds) {
                (false, _, _) => match mate.kind {
                    MateKind::Coupling(_) => format!("choose {}", mate.needs()),
                    _ => format!("pick {} — {}", 2 - mate.entities.len().min(2), mate.needs()),
                },
                (true, Some(part), _) => {
                    format!("{what} — both of {part}: holds nothing together")
                }
                (true, None, true) => what,
                (true, None, false) => format!("{what} — cannot hold"),
            });
            let holds = holds && one_part.is_none();
            item.tone = match (complete, holds) {
                (false, _) => Tone::Hint,
                (true, true) => Tone::Normal,
                (true, false) => Tone::Error,
            };
            item.selected = editing.selected.as_deref() == Some(id);
            item.removable = true;
            item.value = (first.map(|v| v.value)).or_else(|| value_of(&mate.kind));
            item
        })
        .collect();
    f.form.dialog.list("mates", items, "None yet.");
    f.actions(
        "add_mate",
        (kinds().iter())
            .map(|(name, kind, doc, group)| {
                Action::new(*name, kind.label()).title(*doc).group(*group)
            })
            .collect(),
        |edit, name| {
            if let Some((_, kind, _, _)) = kinds().iter().find(|(n, ..)| *n == name) {
                let id = fresh_mate_id(&edit.args.mates);
                let mate = Mate {
                    kind: *kind,
                    entities: Vec::new(),
                    joints: Vec::new(),
                };
                edit.args.mates.insert(id.clone(), mate);
                edit.session.selected = Some(id);
            }
        },
    );

    if let Some((id, mate)) =
        (editing.selected.as_ref()).and_then(|id| args.mates.get_key_value(id))
    {
        let joint = joints.iter().find(|j| j.name == namer(id));
        selected(&mut f, id, mate, joint, &joints);
    }

    if let Some(view) = built
        && let Ok(report) = check(view)
        && !report.failed.is_empty()
    {
        f.form.dialog.text(
            "conflict",
            format!("These mates do not hold: {}.", report.failed.join(", ")),
            Tone::Error,
        );
    }
    f.form.pulls.extend(editing.pulls.iter().cloned());
    f.form
        .pulls
        .extend(editing.held.iter().map(|(parameter, value)| Pull::Value {
            parameter: parameter.clone(),
            value: Design::from_f64(*value),
        }));
    if built.is_some() {
        let shape = Shape::Instance { name: step.clone() };
        f.form
            .visuals
            .push(Visual::new(PART, shape, Style::Free).draggable());
        if let Ok(axes) = pose.rotation().rotation_columns()
            && let (Ok(x), Ok(y), Ok(z)) = (
                axes[0].normalize(),
                axes[1].normalize(),
                axes[2].normalize(),
            )
        {
            f.form.gizmo = Some(Gizmo::new(position).translate().rotate().local([x, y, z]));
        }
        f.form.dialog.text(
            "drag_hint",
            "Drag the part, or the gizmo at its origin, to move it as far as its mates let it.",
            Tone::Hint,
        );
    }
    f
}

/// The step's arguments and editing state after `event`: a field set, or
/// the part dragged — Delete removes the mate selected, Escape stops
/// editing it.
pub(crate) fn process(
    context: &Context<Input, ParamValue>,
    args: &AddPartArgs,
    editing: &PartEditing,
    event: Event<Design>,
) -> (AddPartArgs, PartEditing) {
    let (mut next, mut session, mut selection) = (args.clone(), editing.clone(), Vec::new());
    if std::mem::take(&mut session.released) {
        session.pulls.clear();
    }
    let edit = Edit {
        args: &mut next,
        session: &mut session,
        selection: &mut selection,
    };
    match event {
        Event::Field { key, value } => form(context, args, editing).set(&key, edit, value),
        Event::Select(_) => {}
        Event::Canvas(event) => canvas(context, edit, &event),
    }
    (next, session)
}

/// A pointer or key event: the part dragged (see the module docs); Delete
/// removes the mate selected, Escape stops editing it.
fn canvas(
    context: &Context<Input, ParamValue>,
    edit: Edit<'_, AddPartArgs, PartEditing>,
    event: &CanvasEvent<Design>,
) {
    let Edit { args, session, .. } = edit;
    match event {
        CanvasEvent::Move {
            key,
            from,
            to,
            done,
            ..
        } if key == PART => {
            let pose = pose_of(context);
            // Where the pointer is: a free choice, made by whoever moved it.
            let target = to.sharpen();
            let local = match session.pulls.first() {
                Some(Pull::Point { local, .. }) => *local,
                _ => pose.inverse().apply(from).sharpen(),
            };
            // A drag moves the joints set in the dialog too.
            session.held.clear();
            session.pulls = vec![Pull::Point {
                parameter: pose_parameter(context.step),
                local,
                target,
            }];
            session.released = *done;
        }
        // Moved or turned by the gizmo, from where it stood when the drag
        // started: at the part's origin, along its own axes — where the
        // part was.
        CanvasEvent::Gizmo { drag, from, done } => {
            let (Some(motion), Some(axes)) = (drag.turn(), from.axes) else {
                return;
            };
            let Ok(start) = Quaternion::from_rotation_columns(axes)
                .and_then(|rotation| Pose::new(from.at, rotation))
            else {
                return;
            };
            session.held.clear();
            session.pulls = pulled_to(context.step, motion.compose(&start));
            session.released = *done;
        }
        CanvasEvent::Cancel => session.selected = None,
        CanvasEvent::Delete => {
            if let Some(id) = session.selected.take() {
                args.mates.remove(&id);
            }
        }
        _ => {}
    }
}
