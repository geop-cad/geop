//! Editing a placed part: choosing its file, where it goes and whether it
//! stays put, its mates — and dragging it, as far as its mates let it.
//!
//! Where the part is, is the program's parameter `<id>.pose`: the placement
//! fields set it, and the editor solves the program's mates from there
//! after every edit (see `geop_cad_base::editor`). A drag is the step's to
//! ask for, the solving the editor's: while the part is dragged, the form
//! says what point of it is pulled where ([`Form::drags`]).
//!
//! A joint's coordinates are parameters of the program too: set in the
//! dialog, they are held there while the step is edited ([`Form::holds`]),
//! and the parts move to them.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{primitives::Pose, scalars::Scalar, vector::Vector3};
use geop_ops::{
    Context, Design, EntityRef,
    assembly::{CouplingKind, Drag, JointInfo, JointKind, Kind, Mate, MateKind, Motion},
    operation::Role,
    parameters::{COLOR, ParameterKind, validate_color},
    part::{ParamValue, State, pose_parameter},
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
        MateKind::Constraint(Kind::Coincident | Kind::Distance { .. }) => {
            &[Role::Point, Role::Line, Role::Plane, Role::Round]
        }
        MateKind::Constraint(Kind::Concentric) => &[Role::Round, Role::Line],
        MateKind::Constraint(Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. }) => {
            &[Role::Line, Role::Plane, Role::Round]
        }
        MateKind::Joint(_) => &[Role::Circle, Role::Round, Role::Point, Role::Line, Role::Plane],
        MateKind::Coupling(_) => &[],
    }
}

/// The kinds of mate, as added: by name, with a value to start from, what
/// it does, and the group it is offered in.
fn kinds() -> Vec<(&'static str, MateKind, &'static str, &'static str)> {
    let constraint = |name, kind, doc| (name, MateKind::Constraint(kind), doc, "Add");
    let joint = |name, kind, doc| (name, MateKind::Joint(kind), doc, "Joint");
    let coupling = |name, kind, doc| (name, MateKind::Coupling(kind), doc, "Couple");
    vec![
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
            "The second part turns about the first one's axis, and nothing else: pick a circular edge or a datum of each.",
        ),
        joint(
            "slider",
            JointKind::Slider {
                min: None,
                max: None,
            },
            "The second part slides along the first one's axis, and nothing else.",
        ),
        joint(
            "cylindrical",
            JointKind::Cylindrical,
            "The second part turns about the first one's axis and slides along it.",
        ),
        joint(
            "fastened",
            JointKind::Fastened,
            "The two parts are held together where their circular edges or datums meet.",
        ),
        coupling(
            "gear",
            CouplingKind::Gear {
                ratio: Design::ONE,
                reverse: true,
            },
            "Two turning joints turn together: the first `ratio` times per turn of the second.",
        ),
        coupling(
            "rack_pinion",
            CouplingKind::RackPinion {
                radius: Design::ONE,
                reverse: false,
            },
            "A sliding joint moves as far as a turning joint's pitch circle rolls.",
        ),
        coupling(
            "screw",
            CouplingKind::Screw {
                lead: Design::from_ratio(1, 10).expect("ten is not zero"),
                reverse: false,
            },
            "A sliding joint moves its lead per turn of a turning joint — of one cylindrical joint, say.",
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
    /// The joint coordinates set in the dialog, by the parameters they are:
    /// kept where they were set while the step is edited, until the part is
    /// dragged.
    held: BTreeSet<String>,
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
        MateKind::Constraint(Kind::Distance { value } | Kind::Angle { value })
        | MateKind::Coupling(
            CouplingKind::Gear { ratio: value, .. }
            | CouplingKind::RackPinion { radius: value, .. }
            | CouplingKind::Screw { lead: value, .. },
        ) => Some(value.to_f64()),
        _ => None,
    }
}

/// Sets the joint coordinate `parameter` to `value` and keeps it there
/// while the step is edited: the parts move to it (see [`Form::holds`]).
fn set_joint(state: &mut State, session: &mut PartSession, parameter: &str, value: f64) {
    state.insert(
        parameter.to_string(),
        ParamValue::Number(Design::from_f64(value)),
    );
    session.held.insert(parameter.to_string());
}

/// The value a kind of mate holds is `v` now.
fn set_value(kind: &mut MateKind, v: f64) {
    let v = Design::from_f64(v);
    match kind {
        MateKind::Constraint(Kind::Distance { value } | Kind::Angle { value }) => *value = v,
        MateKind::Coupling(CouplingKind::Gear { ratio: value, .. })
        | MateKind::Coupling(CouplingKind::RackPinion { radius: value, .. })
        | MateKind::Coupling(CouplingKind::Screw { lead: value, .. }) => *value = v,
        _ => {}
    }
}

/// The name of a coupling's value, as its field reads.
fn value_name(kind: &CouplingKind<Design>) -> &'static str {
    match kind {
        CouplingKind::Gear { .. } => "ratio",
        CouplingKind::RackPinion { .. } => "pitch radius",
        CouplingKind::Screw { .. } => "lead",
    }
}

/// The fields of the mate `id` selected: the entities to pick for a
/// constraint or a joint; a joint's coordinates — set, they hold — and its
/// limits; the joints a coupling ties, and how.
fn selected<'a, S: Scalar>(
    f: &mut Form<'a, S, AddPartArgs, PartSession>,
    id: &str,
    mate: &Mate,
    joint: Option<&JointInfo>,
    joints: &[JointInfo],
) {
    if let MateKind::Coupling(kind) = mate.kind {
        let options: Vec<Choice> = std::iter::once(Choice::new("", "Choose a joint…"))
            .chain(
                joints
                    .iter()
                    .map(|j| Choice::new(j.name.clone(), format!("{} — {}", j.name, j.kind))),
            )
            .collect();
        let [first, second] = match kind {
            CouplingKind::Gear { .. } => ["driving joint", "driven joint"],
            CouplingKind::RackPinion { .. } => ["pinion's joint", "rack's joint"],
            CouplingKind::Screw { .. } => ["turning joint", "sliding joint"],
        };
        for (k, label) in [first, second].into_iter().enumerate() {
            let mate_id = id.to_string();
            let value = mate.joints.get(k).cloned().unwrap_or_default();
            f.select(
                &format!("mate:{id}:joint{k}"),
                label,
                value,
                options.clone(),
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
        let (value, reverse) = match kind {
            CouplingKind::Gear { ratio, reverse } => (ratio, reverse),
            CouplingKind::RackPinion { radius, reverse } => (radius, reverse),
            CouplingKind::Screw { lead, reverse } => (lead, reverse),
        };
        let unit = match kind {
            CouplingKind::Gear { .. } => Unit::Fraction,
            _ => Unit::Length,
        };
        let mate_id = id.to_string();
        f.number(
            &format!("mate:{id}:value"),
            Number::new(value_name(&kind), value.to_f64(), unit),
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
            reverse,
            move |args, r| {
                if let Some(MateKind::Coupling(
                    CouplingKind::Gear { reverse, .. }
                    | CouplingKind::RackPinion { reverse, .. }
                    | CouplingKind::Screw { reverse, .. },
                )) = args.mates.get_mut(&mate_id).map(|m| &mut m.kind)
                {
                    *reverse = r;
                }
            },
        );
        f.text(
            "mate_hint",
            format!("Choose {}: the joints of the parts placed so far.", mate.needs()),
            Tone::Hint,
        );
        return;
    }

    let mate_id = id.to_string();
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

    let MateKind::Joint(kind) = mate.kind else {
        return;
    };
    for motion in kind.motions() {
        let unit = match motion {
            Motion::Turn => Unit::Angle,
            Motion::Slide => Unit::Length,
        };
        if let Some(value) = joint.and_then(|j| j.values.iter().find(|v| v.motion == motion.name()))
        {
            let key = format!("mate:{id}:{}", motion.name());
            let parameter = value.parameter.clone();
            f.on(&key, move |edit, value| {
                if let Value::Number(v) = value {
                    set_joint(edit.state, edit.session, &parameter, v);
                }
            });
            f.dialog
                .push(key, Control::Number(Number::new(motion.name(), value.value, unit)));
        }
        if !matches!(kind, JointKind::Revolute { .. } | JointKind::Slider { .. }) {
            continue;
        }
        let [min, max] = kind.limits(motion);
        let mate_id = id.to_string();
        // Limited to either side of where it is now, to start with.
        let here = joint
            .and_then(|j| j.values.iter().find(|v| v.motion == motion.name()))
            .map_or(0.0, |v| v.value);
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
fn parameters<S: Scalar>(
    f: &mut Form<'_, S, AddPartArgs, PartSession>,
    context: Context<'_, S>,
    args: &AddPartArgs,
) {
    if args.file.is_empty() {
        return;
    }
    // As placed here: what a number not given reads follows what is.
    let Ok(component) = context
        .library
        .component(&args.file, &args.parameters)
        .or_else(|_| context.library.component(&args.file, &State::new()))
    else {
        return;
    };
    let defined = component.part.parameters();
    if defined.is_empty() {
        return;
    }
    let built = component.part.inputs();
    let given = |name: &str| args.parameters.get(name).or(built.get(name));
    f.heading("parameters_heading", "Parameters");
    let set = |name: String| {
        move |args: &mut AddPartArgs, value: ParamValue| {
            args.parameters.insert(name.clone(), value);
        }
    };
    if defined.color.is_some() || args.parameters.contains_key(COLOR) {
        let color = match given(COLOR) {
            Some(ParamValue::Text(c)) => c.clone(),
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
                    Some(ParamValue::Text(row)) => row.clone(),
                    _ => selected.clone(),
                };
                let options = rows
                    .iter()
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
    f.select(
        "file",
        "file",
        args.file.clone(),
        options,
        true,
        |args, file| args.file = file.to_string(),
    );
    f.checkbox("fixed", "fixed in place", args.fixed, |args, fixed| {
        args.fixed = fixed;
    });
    f.checkbox(
        "flexible",
        "flexible: its parts move with this program's mates",
        args.flexible,
        |args, flexible| args.flexible = flexible,
    );

    parameters(&mut f, context, args);

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
    // Every joint of the part as built, its coordinates where they are.
    let joints = built
        .and_then(|(part, _)| part.joints().ok())
        .unwrap_or_default();
    let namer = |id: &str| format!("add_part({},{id})", context.id);
    let items = args
        .mates
        .iter()
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
                        (Some(_), Some(parameter)) => {
                            set_joint(edit.state, edit.session, parameter, v)
                        }
                        (None, _) => {}
                    },
                    _ => {}
                }
            });
            let mut item = ListItem::new(key, mate.kind.label());
            let holds = !failed.contains(&namer(id));
            let complete = mate.is_complete();
            // Two entities of one part keep where they are to each other
            // whatever moves: such a mate holds nothing together.
            let one_part = mate.pair().and_then(|[a, b]| {
                let (a, b) = (a.split_instance()?.0, b.split_instance()?.0);
                (a == b).then_some(a)
            });
            let what = match mate.kind {
                MateKind::Coupling(_) => mate.joints.join(" & "),
                _ => {
                    let entities: Vec<String> = mate.entities.iter().map(EntityRef::label).collect();
                    entities.join(" & ")
                }
            };
            item.detail = Some(match (complete, &one_part, holds) {
                (false, _, _) => match mate.kind {
                    MateKind::Coupling(_) => {
                        format!("choose {}", mate.needs())
                    }
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
            item.selected = session.selected.as_deref() == Some(id);
            item.removable = true;
            item.value = first.map(|v| v.value).or_else(|| value_of(&mate.kind));
            item
        })
        .collect();
    f.list("mates", items, "None yet.");
    f.actions(
        "add_mate",
        kinds()
            .iter()
            .map(|(name, kind, doc, group)| {
                Action::new(*name, kind.label()).title(*doc).group(*group)
            })
            .collect(),
        |edit, name| {
            if let Some((_, kind, _, _)) = kinds().iter().find(|(n, ..)| *n == name) {
                let id = fresh_mate_id(&edit.args.mates);
                edit.args.mates.insert(
                    id.clone(),
                    Mate {
                        kind: *kind,
                        entities: Vec::new(),
                        joints: Vec::new(),
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
        selected(&mut f, id, mate, joints.iter().find(|j| j.name == namer(id)), &joints);
    }

    if let Some((part, _)) = built
        && let Ok(freedom) = part.mate_freedom()
    {
        if let Some(dof) = freedom.parts.get(context.id) {
            let text = match dof {
                0 => "Held fast: it cannot move.".to_string(),
                1 => "Free to move in 1 way.".to_string(),
                n => format!("Free to move in {n} ways."),
            };
            f.text("freedom", text, Tone::Hint);
        }
        if !freedom.conflicting.is_empty() {
            f.text(
                "conflict",
                format!(
                    "These mates cannot all hold: {}.",
                    freedom.conflicting.join(", ")
                ),
                Tone::Error,
            );
        }
    }
    f.holds.extend(session.held.iter().cloned());

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
            ..
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
            // A drag moves the joints set in the dialog too.
            session.held.clear();
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
