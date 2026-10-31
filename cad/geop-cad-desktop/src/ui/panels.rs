//! The program's panels, each shown by what its key maps to — the one
//! place the app names a panel (as `web/src/ProgramPanels.tsx`): the
//! configurations, then the joints. A panel of a key not here is not
//! shown.

use eframe::egui::{self, RichText, Ui};
use geop_core_editor::editor::Command;
use serde::Deserialize;

use super::GeopApp;

/// What the joints panel shows (see `geop_ops_assembly::JointsState`).
#[derive(Deserialize, Default)]
struct JointsState {
    #[serde(default)]
    joints: Vec<JointInfo>,
    #[serde(default)]
    conflicting: Vec<String>,
}

#[derive(Deserialize)]
struct JointInfo {
    name: String,
    label: String,
    kind: String,
    #[serde(default)]
    between: Vec<String>,
    values: Vec<JointValue>,
}

#[derive(Deserialize)]
struct JointValue {
    parameter: String,
    motion: String,
    value: f64,
    min: Option<f64>,
    max: Option<f64>,
}

/// The key of the configurations panel (see
/// `geop_ops_model::parameters::ConfigurationsPanel`).
const CONFIGURATIONS: &str = "configurations";

/// What the configurations panel shows (see
/// `geop_ops_model::parameters::ConfigurationsState`).
#[derive(Deserialize, Default)]
struct ConfigurationsState {
    #[serde(default)]
    tables: Vec<TableChoice>,
}

#[derive(Deserialize)]
struct TableChoice {
    step: String,
    input: String,
    rows: Vec<String>,
    selected: String,
}

/// The command suppressing the step `id` in the configuration built, or
/// running it again: what the timeline's button and Ctrl+E send.
pub fn suppress_step(id: &str) -> Command {
    Command::Panel {
        key: CONFIGURATIONS.into(),
        payload: serde_json::json!({ "command": "suppress", "step": id }),
    }
}

pub fn program_panels(window: &mut GeopApp, ui: &mut Ui) {
    configurations(window, ui);
    joints_panel(window, ui);
    motion_panel(window, ui);
}

/// The configuration built, and every other table the part is looked up
/// in, each a list to switch; and a button adding a configuration.
fn configurations(window: &mut GeopApp, ui: &mut Ui) {
    let Some(state) = (window.app.program.as_ref())
        .and_then(|p| p.panels.get(CONFIGURATIONS))
        .cloned()
    else {
        return;
    };
    let Ok(state) = serde_json::from_value::<ConfigurationsState>(state) else {
        return;
    };
    let enabled = window.app.ready();
    let idle = window.app.step.is_none();
    let mut command = None;
    ui.horizontal_wrapped(|ui| {
        for table in &state.tables {
            let label = match table.input.as_str() {
                "configuration" => "Configuration",
                input => input,
            };
            ui.label(RichText::new(label).color(window.theme.weak()));
            ui.add_enabled_ui(enabled, |ui| {
                egui::ComboBox::from_id_salt(("table", &table.step))
                    .selected_text(&table.selected)
                    .show_ui(ui, |ui| {
                        for row in &table.rows {
                            if ui.selectable_label(*row == table.selected, row).clicked() {
                                command = Some(serde_json::json!({
                                    "command": "select", "input": table.input, "row": row,
                                }));
                            }
                        }
                    });
            });
        }
        let has = state.tables.iter().any(|t| t.input == "configuration");
        let (text, tip) = match has {
            true => ("+", "Add a configuration as this one is, and build it"),
            false => (
                "+ Configuration",
                "Add a configuration: the part as it is stays the Default one",
            ),
        };
        if ui
            .add_enabled(enabled && idle, egui::Button::new(text).small())
            .on_hover_text(tip)
            .clicked()
        {
            command = Some(serde_json::json!({ "command": "add" }));
        }
    });
    if let Some(payload) = command {
        window.app.command(Command::Panel {
            key: CONFIGURATIONS.into(),
            payload,
        });
    }
}

/// The joints panel: each joint's coordinates, to set.
fn joints_panel(window: &mut GeopApp, ui: &mut Ui) {
    let Some(state) = (window.app.program.as_ref())
        .and_then(|p| p.panels.get("joints"))
        .cloned()
    else {
        return;
    };
    let Ok(state) = serde_json::from_value::<JointsState>(state) else {
        return;
    };
    if state.joints.is_empty() && state.conflicting.is_empty() {
        return;
    }
    let theme = window.theme;
    let enabled = window.app.ready() && window.app.step.is_none();
    ui.label(
        RichText::new(if state.joints.is_empty() {
            "Mates"
        } else {
            "Joints"
        })
        .strong(),
    );
    let mut set = None;
    for joint in &state.joints {
        for v in &joint.values {
            ui.horizontal(|ui| {
                let between = if joint.between.is_empty() {
                    String::new()
                } else {
                    format!(" — {}", joint.between.join(" and "))
                };
                ui.add(egui::Label::new(&joint.label).truncate())
                    .on_hover_text(format!(
                        "{} joint {}: {}{between}",
                        joint.kind, joint.name, v.motion
                    ));
                let unit = if v.motion == "angle" { "°" } else { "" };
                if let (Some(min), Some(max)) = (v.min, v.max) {
                    let mut value = v.value;
                    let step = if v.motion == "angle" { 1.0 } else { 0.01 };
                    let r = ui.add_enabled(
                        enabled,
                        egui::Slider::new(&mut value, min..=max)
                            .step_by(step)
                            .suffix(unit),
                    );
                    if r.changed() {
                        set = Some((v.parameter.clone(), value));
                    }
                } else {
                    let id = ui.id().with(&v.parameter);
                    let (given, _) = super::dialog::typed(
                        ui,
                        id,
                        &format!("{}", (v.value * 1e4).round() / 1e4),
                        |t| egui::TextEdit::singleline(t).desired_width(70.0),
                    );
                    if let Some(value) = given.and_then(|t| t.trim().parse::<f64>().ok())
                        && value != v.value
                    {
                        set = Some((v.parameter.clone(), value));
                    }
                    // A distance is called by its coordinate: `x`, `distance`.
                    let coordinate = v.parameter.rsplit('.').next().unwrap_or_default();
                    ui.label(
                        RichText::new(if unit.is_empty() { coordinate } else { unit })
                            .color(theme.weak()),
                    );
                }
            });
        }
    }
    if !state.conflicting.is_empty() {
        ui.label(
            RichText::new(format!("cannot all hold: {}", state.conflicting.join(", ")))
                .color(theme.error()),
        );
    }
    ui.separator();
    if let Some((parameter, value)) = set {
        window.app.command(Command::Panel {
            key: "joints".into(),
            payload: serde_json::json!({ "command": "set", "parameter": parameter, "value": value }),
        });
    }
}

/// What the motion panel shows (see `geop_ops_assembly::MotionState`).
#[derive(Deserialize, Default)]
struct MotionState {
    #[serde(default)]
    studies: Vec<StudyState>,
    shown: Option<(String, f64)>,
    checked: Option<Checked>,
}

#[derive(Deserialize)]
struct StudyState {
    name: String,
    start: f64,
    end: f64,
    frames: usize,
    #[serde(default)]
    drivers: Vec<String>,
}

#[derive(Deserialize)]
struct Checked {
    study: String,
    collisions: Vec<Collision>,
}

#[derive(Deserialize)]
struct Collision {
    time: f64,
    a: String,
    b: String,
}

/// A study being played: the time it started from, and when, by the
/// clock of the window's input.
#[derive(Clone, Copy)]
struct Playing {
    from: f64,
    began: f64,
}

/// The motion panel: each motion study to scrub through, play and pause,
/// and check for collisions — as `web/src/MotionPanel.tsx` shows it.
fn motion_panel(window: &mut GeopApp, ui: &mut Ui) {
    let Some(state) = (window.app.program.as_ref())
        .and_then(|p| p.panels.get("motion"))
        .cloned()
    else {
        return;
    };
    let Ok(state) = serde_json::from_value::<MotionState>(state) else {
        return;
    };
    if state.studies.is_empty() {
        return;
    }
    let theme = window.theme;
    let enabled = window.app.ready() && window.app.step.is_none();
    ui.label(RichText::new("Motion").strong());
    let mut sent = None;
    for study in &state.studies {
        let id = ui.id().with(("motion", &study.name));
        let shown = match &state.shown {
            Some((name, time)) if *name == study.name => *time,
            _ => study.start,
        };
        let mut playing: Option<Playing> = ui.ctx().data(|d| d.get_temp(id));
        let now = ui.input(|i| i.time);
        ui.horizontal(|ui| {
            ui.add(egui::Label::new(&study.name).truncate())
                .on_hover_text(format!("drives {}", study.drivers.join(", ")));
            let label = if playing.is_some() { "❚❚" } else { "▶" };
            if ui.add_enabled(enabled, egui::Button::new(label)).clicked() {
                playing = match playing {
                    Some(_) => None,
                    None => Some(Playing {
                        from: if shown >= study.end {
                            study.start
                        } else {
                            shown
                        },
                        began: now,
                    }),
                };
            }
            let mut time = shown;
            let step = (study.end - study.start) / study.frames.max(2) as f64 / 4.0;
            let slider = egui::Slider::new(&mut time, study.start..=study.end)
                .step_by(step.max(1e-3))
                .suffix(" s");
            if ui
                .add_enabled(enabled && playing.is_none(), slider)
                .changed()
            {
                sent = Some(("seek", study.name.clone(), time));
            }
        });
        // Played on: the next frame once the last is shown, at the time
        // that has passed since it started — never out of order.
        if let Some(play) = playing {
            if !window.app.busy() {
                let t = (play.from + now - play.began).min(study.end);
                sent = Some(("seek", study.name.clone(), t));
                if t >= study.end {
                    playing = None;
                }
            }
            ui.ctx().request_repaint();
        }
        ui.ctx().data_mut(|d| match playing {
            Some(play) => {
                d.insert_temp(id, play);
            }
            None => {
                d.remove::<Playing>(id);
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    enabled && playing.is_none(),
                    egui::Button::new("Check collisions"),
                )
                .clicked()
            {
                sent = Some(("check", study.name.clone(), 0.0));
            }
            if let Some(checked) = state.checked.as_ref().filter(|c| c.study == study.name) {
                let said = match checked.collisions.len() {
                    0 => format!("no collision in {} frames", study.frames),
                    n => format!("{n} collision{}", if n == 1 { "" } else { "s" }),
                };
                ui.label(RichText::new(said).color(theme.weak()));
            }
        });
        if let Some(checked) = state.checked.as_ref().filter(|c| c.study == study.name) {
            for c in &checked.collisions {
                let text = format!("{:.2} s: {} × {}", c.time, c.a, c.b);
                let row = ui.add(
                    egui::Label::new(RichText::new(text).color(theme.error()))
                        .sense(egui::Sense::click()),
                );
                if enabled && row.on_hover_text("Show it then").clicked() {
                    sent = Some(("seek", study.name.clone(), c.time));
                }
            }
        }
    }
    ui.separator();
    if let Some((command, study, time)) = sent {
        let payload = match command {
            "seek" => serde_json::json!({ "command": "seek", "study": study, "time": time }),
            _ => serde_json::json!({ "command": "check", "study": study }),
        };
        window.app.command(Command::Panel {
            key: "motion".into(),
            payload,
        });
    }
}
