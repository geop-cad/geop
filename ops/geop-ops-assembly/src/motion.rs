//! Motion studies: joint coordinates driven by a time table — a step,
//! [`Drive`], whose study ([`MotionStudy`]) the model's vocabulary holds —
//! played and checked for collisions in the motion panel
//! ([`MotionPanel`]), and written frame by frame ([`MotionExport`]).
//!
//! A motion is played by solving: at each time its drivers' values are
//! set, as the joints panel sets a coordinate, and everything else that is
//! free gives way (see [`solving::set`]). Each frame starts from the one
//! before, so a linkage keeps the branch it is on.

use std::{cell::RefCell, collections::BTreeMap};

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{
        Cell, Choice, Column, ColumnKind, ListItem, Number, Pull, Table, TableEdit, TableRow, Tone,
        Unit, Value,
    },
    editor::{Changed, EditorContext, Rerun},
    output::{Content, Export, Exporter, ExporterInfo, Output},
    part_operation,
    part_operation::PartOperation,
    registry::Panel,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{part::StepContext, program::Program, reader::Reader, values::StateValues};
use geop_ops_model::{
    parameters::stem,
    placed::{
        motion::{MotionStudy, motion_studies, motion_study},
        placed_parts,
    },
};
use serde::{Deserialize, Serialize};

use crate::solving::{self, Measure, SettingsState, settings};

/// Drives joint coordinates by a time table: a motion study — see the
/// module docs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Drive;

/// A motion study, as its step holds it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionArgs {
    #[serde(flatten)]
    pub study: MotionStudy,
}

/// The joint coordinates of the part `reader` reads a motion can drive,
/// each with what it is called: `fore m1 angle`.
fn coordinates(reader: &Reader) -> Vec<(String, String, Measure, SettingsState)> {
    (settings(reader).into_iter())
        .flat_map(|joint| {
            (joint.values.clone().into_iter()).map(move |value| {
                let coordinate = value.state.rsplit('.').next().unwrap_or_default();
                let label = format!("{} {coordinate}", joint.label);
                (value.state.clone(), label, value.measure, joint.clone())
            })
        })
        .collect()
}

/// The study `args` holds, checked against the part `reader` reads: every
/// driver one of its joint coordinates. The generator of its target.
fn study(reader: &Reader, args: &MotionArgs) -> GeopResult<MotionStudy> {
    args.study.validate()?;
    let known = coordinates(reader);
    for driver in &args.study.drivers {
        if !known.iter().any(|(state, ..)| state == driver) {
            let names: Vec<&str> = known.iter().map(|(state, ..)| state.as_str()).collect();
            return Err(GeopError::new(format!(
                "{driver:?} is no joint coordinate of this part: a motion drives one of {}",
                match names.is_empty() {
                    true => "its joints' coordinates, and it has no joint yet".to_string(),
                    false => names.join(", "),
                }
            )));
        }
    }
    Ok(args.study.clone())
}

/// What the dialog keeps while a motion is edited: the time it is shown
/// at, if one was chosen.
#[derive(Clone, Debug, Default)]
pub struct MotionSession {
    preview: Option<f64>,
}

part_operation!(Drive(MotionArgs));

impl PartOperation for Drive {
    const KIND: &'static str = "motion";
    const INFO: OperationInfo = OperationInfo::new(
        "Motion",
        "Drive joints by a time table, to play in the motion panel, check for collisions and export frame by frame.",
        OperationGroup::Assembly,
        OperationTier::Small,
    )
    .icon(crate::icons::MOTION);
    type Args = MotionArgs;
    type Session = MotionSession;

    /// The first joint coordinate of the part driven from where it is: an
    /// angle a whole turn, or up to its limit, over two seconds; a
    /// distance by one.
    fn new_args(context: &Context) -> MotionArgs {
        let first = coordinates(&context.before).into_iter().next();
        let (drivers, keys) = match first {
            None => (Vec::new(), vec![vec![0.0], vec![2.0]]),
            Some((state, _, measure, joint)) => {
                let value = joint.values.iter().find(|v| v.state == state);
                let (at, max) = value.map_or((0.0, None), |v| (v.value, v.max));
                let to = max.unwrap_or(match measure {
                    Measure::Angle => at + 360.0,
                    Measure::Distance => at + 1.0,
                });
                (vec![state], vec![vec![0.0, at], vec![2.0, to]])
            }
        };
        MotionArgs {
            study: MotionStudy {
                drivers,
                keys,
                frames: 25,
            },
        }
    }

    fn run(ctx: &mut StepContext, args: &MotionArgs) -> GeopResult<()> {
        let step = ctx.step().to_string();
        ctx.produce(&motion_study(&step), args, study)
            .with_context(with_context!("motion({step})"))
    }

    /// The drivers, the time table — a row per keyframe, a column per
    /// driver — how many frames, and a time to show the motion at: the
    /// joints held at their values then, everything else giving way.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &MotionArgs,
        editing: &FormState<MotionSession>,
    ) -> EditForm<'a, MotionArgs, MotionSession> {
        let mut f = EditForm::<MotionArgs, MotionSession>::default();
        let known = coordinates(&context.before);
        let label = |state: &str| {
            (known.iter().find(|(s, ..)| s == state)).map_or(state.to_string(), |k| k.1.clone())
        };
        let study = &args.study;
        let items = (study.drivers.iter().enumerate())
            .map(|(d, driver)| {
                let key = format!("driver:{driver}");
                f.on(key.clone(), move |Edit { args, .. }, value| {
                    if matches!(value, Value::Remove) {
                        let study = &mut args.study;
                        study.drivers.remove(d);
                        for key in &mut study.keys {
                            key.remove(d + 1);
                        }
                    }
                });
                let mut item = ListItem::new(key, label(driver));
                item.removable = true;
                item
            })
            .collect();
        f.form
            .dialog
            .list("drivers", items, "None yet: add a joint to drive.");
        let options: Vec<Choice> = std::iter::once(Choice::new("", "Drive a joint…"))
            .chain(
                (known.iter())
                    .filter(|(state, ..)| !study.drivers.contains(state))
                    .map(|(state, label, ..)| Choice::new(state.clone(), label.clone())),
            )
            .collect();
        if options.len() == 1 && study.drivers.is_empty() {
            f.form.dialog.text(
                "no_joint",
                "This part has no joint to drive yet: place parts and join them first.",
                Tone::Error,
            );
        }
        let values: BTreeMap<String, f64> = (known.iter())
            .filter_map(|(state, _, _, joint)| {
                let value = joint.values.iter().find(|v| v.state == *state)?;
                Some((state.clone(), value.value))
            })
            .collect();
        f.select(
            "add_driver",
            "drive",
            "",
            options,
            true,
            move |Edit { args, .. }, state| {
                if state.is_empty() || args.study.drivers.iter().any(|d| d == state) {
                    return;
                }
                // Held where it is at every keyframe, to start with.
                let at = values.get(state).copied().unwrap_or(0.0);
                args.study.drivers.push(state.to_string());
                for key in &mut args.study.keys {
                    key.push(at);
                }
            },
        );
        let unit = |driver: &str| match known.iter().find(|(s, ..)| s == driver) {
            Some((_, _, Measure::Angle, _)) => Unit::Angle,
            _ => Unit::Length,
        };
        let seconds = ColumnKind::Number {
            unit: Unit::Fraction,
        };
        let mut columns = vec![Column::new("time (s)", seconds)];
        columns.extend((study.drivers.iter()).map(|driver| {
            let kind = ColumnKind::Number { unit: unit(driver) };
            Column::new(label(driver), kind)
        }));
        let rows = (study.keys.iter())
            .map(|key| {
                let mut row = TableRow::new(key.iter().map(|v| Cell::number(*v)).collect());
                row.removable = study.keys.len() > 1;
                row
            })
            .collect();
        f.table(
            "keys",
            Table::new(columns, rows, "No keyframe: add a row."),
            |Edit { args, .. }, edit| {
                let keys = &mut args.study.keys;
                match edit {
                    TableEdit::Cell { row, column, text } => {
                        if let (Some(key), Ok(value)) = (keys.get_mut(row), text.trim().parse())
                            && column < key.len()
                        {
                            key[column] = value;
                        }
                    }
                    // A second on, holding where the last one is.
                    TableEdit::AddRow => {
                        let next = match keys.last() {
                            Some(last) => {
                                let mut next = last.clone();
                                next[0] += 1.0;
                                next
                            }
                            None => vec![0.0; args.study.drivers.len() + 1],
                        };
                        keys.push(next);
                    }
                    TableEdit::RemoveRow { row } if row < keys.len() && keys.len() > 1 => {
                        keys.remove(row);
                    }
                    TableEdit::MoveRow { row, to } if row < keys.len() && to < keys.len() => {
                        let key = keys.remove(row);
                        keys.insert(to, key);
                    }
                    _ => {}
                }
            },
        );
        f.number(
            "frames",
            Number::new("frames", study.frames as f64, Unit::Count).range(2.0, 200.0),
            |Edit { args, .. }, frames| args.study.frames = frames.round().max(2.0) as usize,
        );
        let (start, end) = study.span();
        let shown = editing.session.preview.unwrap_or(start);
        f.number(
            "time",
            Number::new("show at (s)", shown, Unit::Fraction).range(start, end.max(start + 1e-9)),
            |Edit { session, .. }, t| session.preview = Some(t),
        );
        if let Some(t) = editing.session.preview
            && study.validate().is_ok()
        {
            f.form.pulls.extend(
                study
                    .at(t)
                    .into_iter()
                    .map(|(parameter, value)| Pull::Value {
                        parameter,
                        value: Scalar::from_f64(value),
                    }),
            );
        }
        f.form.dialog.text(
            "hint",
            "Each row is a keyframe: its time, then where each joint driven is; between two, they move evenly. Play it, check it for collisions and export it in the motion panel.",
            Tone::Hint,
        );
        f
    }
}

/// The values a study `study` drives, at the time `t`, as a solve sets
/// them.
fn driven(study: &MotionStudy, t: f64) -> BTreeMap<String, Scalar> {
    (study.at(t).into_iter())
        .map(|(state, value)| (state, Scalar::from_f64(value)))
        .collect()
}

/// The program `rerun` runs, last run with `state`, solved with the
/// drivers of `study` set to their values at `t`: the values of the state
/// that change. Fails, naming the time, where the joints cannot be there.
fn solved_at(
    rerun: &mut Rerun,
    state: &StateValues,
    study: &MotionStudy,
    t: f64,
) -> GeopResult<StateValues> {
    let (moved, report) = solving::set(rerun, state, &driven(study, t))?;
    if !report.converged {
        return Err(GeopError::new(format!(
            "at {t} s the joints cannot be where the motion drives them: {} do not hold",
            report.failed.join(", ")
        )));
    }
    Ok(moved)
}

/// The motion study `name` of the part `reader` reads — or, with no name,
/// its last.
fn find(reader: &Reader, name: Option<&str>) -> GeopResult<MotionStudy> {
    let studies = motion_studies(reader);
    let found = match name {
        Some(name) => studies.iter().find(|(step, _)| step == name),
        None => studies.last(),
    };
    found.map(|(_, study)| (*study).clone()).ok_or_else(|| {
        let names: Vec<&str> = studies.iter().map(|(step, _)| step.as_str()).collect();
        GeopError::new(match (name, names.is_empty()) {
            (_, true) => "this part has no motion study: add a Motion step".to_string(),
            (Some(name), false) => {
                format!("no motion study {name:?}: there are {}", names.join(", "))
            }
            (None, false) => unreachable!("a last study where there are some"),
        })
    })
}

/// A motion study as the panel lists it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StudyState {
    /// The id of its step.
    pub name: String,
    /// When it starts and ends, in seconds.
    pub start: f64,
    pub end: f64,
    pub frames: usize,
    /// The joint coordinates it drives, by state.
    pub drivers: Vec<String>,
}

/// Two parts found overlapping at a time of a motion.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Collision {
    pub time: f64,
    pub a: String,
    pub b: String,
}

/// What the last check of a study found.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Checked {
    pub study: String,
    /// Each pair of solids that overlap, at each frame they do, in time.
    pub collisions: Vec<Collision>,
}

/// What the motion panel shows: the studies of the part shown, the one
/// played last and the time it was shown at, and what the last check
/// found.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct MotionState {
    pub studies: Vec<StudyState>,
    /// The study played last, and the time it was shown at.
    pub shown: Option<(String, f64)>,
    pub checked: Option<Checked>,
}

/// What the motion panel is sent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum MotionCommand {
    /// Show the study `study` at the time `time`, in seconds: its joints
    /// set to their values then, from where they are, everything else free
    /// giving way.
    Seek { study: String, time: f64 },
    /// Play the study `study` through, frame by frame, and find the parts
    /// that overlap at each: then show it where it was.
    Check { study: String },
}

/// The motion panel: see the module docs.
#[derive(Default)]
pub struct MotionPanel {
    shown: RefCell<Option<(String, f64)>>,
    checked: RefCell<Option<Checked>>,
}

/// The pairs of solids an interference report — as the inspector of that
/// kind writes it — says overlap.
fn overlapping(report: &serde_json::Value) -> Vec<(String, String)> {
    let found = report["found"].as_array().cloned().unwrap_or_default();
    (found.iter())
        .filter(|pair| pair["contact"] == "overlap")
        .filter_map(|pair| {
            let name = |k: &str| pair[k].as_str().map(str::to_string);
            Some((name("a")?, name("b")?))
        })
        .collect()
}

/// What checks a motion for collisions, by kind: the interference
/// inspector, wherever an editor is composed with one.
const INTERFERENCE: &str = "interference";

impl Panel for MotionPanel {
    const KEY: &'static str = "motion";
    type State = MotionState;
    type Command = MotionCommand;

    fn state(&self, _: &Program, part: &Reader) -> MotionState {
        let studies: Vec<StudyState> = (motion_studies(part).into_iter())
            .map(|(name, study)| {
                let (start, end) = study.span();
                StudyState {
                    name,
                    start,
                    end,
                    frames: study.frames,
                    drivers: study.drivers.clone(),
                }
            })
            .collect();
        let known = |name: &str| studies.iter().any(|s| s.name == name);
        let shown = (self.shown.borrow().clone()).filter(|(name, _)| known(name));
        let checked = (self.checked.borrow().clone()).filter(|c| known(&c.study));
        MotionState {
            studies,
            shown,
            checked,
        }
    }

    fn command(&self, ctx: &mut EditorContext, command: MotionCommand) -> GeopResult<Changed> {
        ctx.idle()?;
        ctx.run_all();
        match command {
            MotionCommand::Seek { study: name, time } => {
                let study = find(&ctx.part(), Some(&name))?;
                let mut rerun = ctx.rerun();
                let state = rerun.program.state.clone();
                let moved = solved_at(&mut rerun, &state, &study, time)?;
                ctx.move_state(moved);
                *self.shown.borrow_mut() = Some((name, time));
                Ok(Changed::Run)
            }
            MotionCommand::Check { study: name } => {
                let study = find(&ctx.part(), Some(&name))?;
                let before = ctx.program().state.clone();
                let mut collisions = Vec::new();
                let mut checked = Ok(());
                for time in study.times() {
                    let mut rerun = ctx.rerun();
                    let state = rerun.program.state.clone();
                    let moved = match solved_at(&mut rerun, &state, &study, time) {
                        Ok(moved) => moved,
                        Err(e) => {
                            checked = Err(e);
                            break;
                        }
                    };
                    ctx.move_state(moved);
                    ctx.run_all();
                    let inspection = ctx.inspect(INTERFERENCE, &[]).map_err(|e| {
                        e.with_context("collisions are checked by the interference inspector")
                    });
                    match inspection {
                        Ok(inspection) => {
                            collisions.extend(
                                (overlapping(&inspection.report).into_iter())
                                    .map(|(a, b)| Collision { time, a, b }),
                            )
                        }
                        Err(e) => {
                            checked = Err(e);
                            break;
                        }
                    }
                }
                // Shown where it was before the check.
                ctx.move_state(before);
                ctx.run_all();
                checked?;
                *self.checked.borrow_mut() = Some(Checked {
                    study: name,
                    collisions,
                });
                Ok(Changed::Run)
            }
        }
    }
}

/// Writes a motion study frame by frame: at each, the values its drivers
/// are set to and where every placed part is.
pub struct MotionExport;

/// Which motion study to write.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct MotionOptions {
    /// The id of its step: the part's last, if not given.
    pub study: Option<String>,
}

/// One frame of a motion, as written: the time, the drivers' values, and
/// every placed part's pose — its position, and its turn as a unit
/// quaternion `[w, x, y, z]`.
#[derive(Serialize)]
struct Frame {
    time: f64,
    values: BTreeMap<String, f64>,
    poses: BTreeMap<String, [f64; 7]>,
}

impl Exporter for MotionExport {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "motion",
        label: "Motion",
        doc: "joint values and where each placed part is, at every frame of a motion study",
        formats: &["csv", "json"],
    };
    type Options = MotionOptions;

    fn export(output: &mut Output, format: &str, options: MotionOptions) -> GeopResult<Export> {
        let study = find(&output.part(), options.study.as_deref())?;
        let mut state = output.file.state.clone();
        let mut frames = Vec::new();
        let mut made = Ok(());
        for time in study.times() {
            let mut rerun = Rerun {
                runner: output.runner,
                part: &mut *output.part,
                program: output.file,
            };
            match solved_at(&mut rerun, &state, &study, time) {
                Ok(moved) => state.extend(moved),
                Err(e) => {
                    made = Err(e);
                    break;
                }
            }
            let reader = output.part.current();
            let poses = (placed_parts(&reader)?.into_iter())
                .map(|placed| {
                    let pose = placed.located.world;
                    let (p, q) = (pose.position(), pose.rotation());
                    let f = |s: Scalar| s.to_f64();
                    let numbers = [p[0], p[1], p[2], q.w, q.x, q.y, q.z].map(f);
                    (placed.name, numbers)
                })
                .collect();
            let values = study.at(time).into_iter().collect();
            frames.push(Frame {
                time,
                values,
                poses,
            });
        }
        // The part as it was, for whatever is asked of it next.
        let inputs = Default::default();
        (output.runner).run_with(output.part, output.file, &inputs, None);
        made?;
        let name = stem(output.file_name());
        let (text, media_type) = match format {
            "json" => {
                let text = serde_json::to_string_pretty(&frames)
                    .map_err(|e| GeopError::new(format!("writing the motion: {e}")))?;
                (text, "application/json")
            }
            _ => (csv(&study, &frames), "text/csv"),
        };
        Ok(Export {
            name: format!("{name}.{format}"),
            media_type: media_type.into(),
            content: Content::Text(text),
        })
    }
}

/// `frames` as comma separated values: a row a frame, its time, the
/// drivers' values in `study`'s order, then each part's position and turn.
fn csv(study: &MotionStudy, frames: &[Frame]) -> String {
    let parts: Vec<&String> = frames
        .first()
        .map_or(Vec::new(), |f| f.poses.keys().collect());
    let mut header = vec!["time".to_string()];
    header.extend(study.drivers.iter().cloned());
    for part in &parts {
        for field in ["x", "y", "z", "qw", "qx", "qy", "qz"] {
            header.push(format!("{part}.{field}"));
        }
    }
    let quoted: Vec<String> = header
        .iter()
        .map(|h| format!("\"{}\"", h.replace('"', "\"\"")))
        .collect();
    let mut out = quoted.join(",") + "\n";
    for frame in frames {
        let mut row = vec![frame.time];
        row.extend(study.drivers.iter().map(|d| frame.values[d]));
        for part in &parts {
            row.extend(frame.poses[*part]);
        }
        let row: Vec<String> = row.iter().map(|v| v.to_string()).collect();
        out += &(row.join(",") + "\n");
    }
    out
}
