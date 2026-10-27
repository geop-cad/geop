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
    joints(window, ui);
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

/// The joints, each coordinate set by a slider or typed; and which mates
/// cannot all hold.
fn joints(window: &mut GeopApp, ui: &mut Ui) {
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
                    ui.label(
                        RichText::new(if unit.is_empty() {
                            v.motion.as_str()
                        } else {
                            unit
                        })
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
