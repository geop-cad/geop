//! The fields for drawing a sketch: the tools, each a button of its own;
//! what the sketch projects; one line on what to do next and one on how
//! the sketch stands; and its constraints.

use geop_ops::ui::{Choice, Control, Number, Prompt, Unit};

use super::*;
use crate::references::Source;

/// What the tool in hand asks for next.
fn tool_hint(s: &SketchSession) -> String {
    let placed = s.draft.placed.len();
    let text = match s.tool {
        Tool::Select => {
            "Select, or drag points and curves · double-click a dimension to give its value"
        }
        Tool::Constrain(tool) => {
            let info = tool.info();
            let place = if tool.is_dimension() {
                " · then click where its value goes"
            } else {
                ""
            };
            return format!(
                "{}: {}{place} · Esc to stop",
                info.label,
                info.doc.to_lowercase()
            );
        }
        Tool::Trim => {
            "Click a curve, or drag across curves, to remove them up to where they meet others · Esc to stop"
        }
        Tool::Modify(ModifyTool::Offset) => {
            "Click curves to offset their chains, then click where the offset goes · Esc to stop"
        }
        Tool::Modify(ModifyTool::Mirror) if s.modify.mirror_line.is_some() => {
            "Click curves to mirror them · Esc to stop"
        }
        Tool::Modify(ModifyTool::Mirror) => {
            "Click the line to mirror what is selected across · Esc to stop"
        }
        Tool::Modify(ModifyTool::LinearPattern) => {
            "Click a line to repeat what is selected along, towards its end clicked nearer · Esc to stop"
        }
        Tool::Modify(ModifyTool::CircularPattern) => {
            "Click the point — or circle — to repeat what is selected round · Esc to stop"
        }
        Tool::Draw(tool) => match (tool, placed) {
            (DrawTool::Line, 0) => "Click where the line starts",
            (DrawTool::Line, _) if s.draft.arc => {
                "Click where the tangent arc ends · move back onto its start for a line"
            }
            (DrawTool::Line, _) => {
                "Click the next point · move back onto the end for a tangent arc · Esc ends"
            }
            (DrawTool::Rectangle, 0) => "Click one corner",
            (DrawTool::Rectangle, _) => "Click the opposite corner",
            (DrawTool::CenterRectangle, 0) => "Click the center",
            (DrawTool::CenterRectangle, _) => "Click a corner",
            (DrawTool::ThreePointRectangle, 0) => "Click where one side starts",
            (DrawTool::ThreePointRectangle, 1) => "Click where that side ends",
            (DrawTool::ThreePointRectangle, _) => "Click to give the width",
            (DrawTool::Circle, 0) => "Click the center",
            (DrawTool::Circle, _) => "Click a point on the circle",
            (DrawTool::ThreePointCircle, _) => "Click three points on the circle",
            (DrawTool::Arc, 0) => "Click where the arc starts",
            (DrawTool::Arc, 1) => "Click where it ends",
            (DrawTool::Arc, _) => "Click a point it passes through",
            (DrawTool::CenterArc, 0) => "Click the center",
            (DrawTool::CenterArc, 1) => "Click where the arc starts",
            (DrawTool::CenterArc, _) => "Move round, and click where it ends",
            (DrawTool::TangentArc, 0) => "Click the end of a curve to continue",
            (DrawTool::TangentArc, _) => "Click where the arc ends",
            (DrawTool::Polygon, 0) => "Click the center",
            (DrawTool::Polygon, _) if s.circumscribed => "Click the middle of a side",
            (DrawTool::Polygon, _) => "Click a corner",
            (DrawTool::Slot, 0) => "Click the first center",
            (DrawTool::Slot, 1) => "Click the second center",
            (DrawTool::Slot, _) => "Click to give the width",
            (DrawTool::ArcSlot, 0) => "Click the center of its arc",
            (DrawTool::ArcSlot, 1) => "Click where its arc starts",
            (DrawTool::ArcSlot, 2) => "Move round, and click where its arc ends",
            (DrawTool::ArcSlot, _) => "Click to give the width",
            (DrawTool::Spline, _) => "Click control points · double-click or Enter finishes",
            (DrawTool::Point, _) => "Click to place a point",
            (DrawTool::Fillet | DrawTool::Chamfer, _) => "Click the corner where two lines meet",
        },
    };
    let snapping = if matches!(s.tool, Tool::Draw(_)) {
        " · Shift: no snapping"
    } else {
        ""
    };
    format!("{text}{snapping}")
}

/// The options of the tool in hand: a polygon's sides, how many copies a
/// pattern makes and how far apart, which sides an offset goes to.
fn tool_options<'a, S: Scalar>(
    d: &mut Form<'a, S, AddSketchArgs, SketchSession>,
    s: &SketchSession,
) {
    let number = |d: &mut Form<'a, S, AddSketchArgs, SketchSession>,
                  key: &str,
                  number: Number<S>,
                  set: fn(&mut SketchSession, f64)| {
        d.on(key, move |edit, value| {
            if let Value::Number(v) = value {
                set(edit.session, v);
            }
        });
        d.dialog.push(key, Control::Number(number));
    };
    let checkbox = |d: &mut Form<'a, S, AddSketchArgs, SketchSession>,
                    key: &str,
                    label: &str,
                    value: bool,
                    set: fn(&mut SketchSession, bool)| {
        d.on(key, move |edit, value| {
            if let Value::Bool(b) = value {
                set(edit.session, b);
            }
        });
        let control = Control::Checkbox {
            label: label.into(),
            value,
        };
        d.dialog.push(key, control);
    };
    let count = |d: &mut Form<'a, S, AddSketchArgs, SketchSession>| {
        let field = Number::new("copies", s.modify.count as f64, Unit::Count).range(2.0, 24.0);
        number(d, "count", field, |s, n| {
            s.modify.count = (n.round() as usize).clamp(2, 1000);
        });
    };
    match s.tool {
        Tool::Draw(DrawTool::Polygon) => {
            let sides = Number::new("sides", s.sides as f64, Unit::Count).range(3.0, 12.0);
            number(d, "sides", sides, |s, n| {
                s.sides = (n.round() as usize).clamp(3, 64);
            });
            checkbox(
                d,
                "circumscribed",
                "Sides touch the circle",
                s.circumscribed,
                |s, b| s.circumscribed = b,
            );
        }
        Tool::Modify(ModifyTool::LinearPattern) => {
            count(d);
            let spacing = Number::new("spacing", s.modify.spacing, Unit::Length);
            number(d, "spacing", spacing, |s, v| {
                if v > 0.0 {
                    s.modify.spacing = v;
                }
            });
        }
        Tool::Modify(ModifyTool::CircularPattern) => {
            count(d);
            let pitch = s.modify.pitch.unwrap_or(360.0 / s.modify.count as f64);
            let angle = Number::new("angle between copies", pitch, Unit::Angle);
            number(d, "pitch", angle, |s, v| s.modify.pitch = Some(v));
        }
        Tool::Modify(ModifyTool::Offset) => {
            checkbox(d, "both", "Both sides", s.modify.both, |s, b| {
                s.modify.both = b
            });
            d.on("corners", move |edit, value| {
                if let Value::Choice(choice) = value {
                    edit.session.modify.corners = match choice.as_str() {
                        "extend" => Corners::Extend,
                        _ => Corners::Round,
                    };
                }
            });
            let value = match s.modify.corners {
                Corners::Round => "round",
                Corners::Extend => "extend",
            };
            d.dialog.push(
                "corners",
                Control::Select {
                    label: "corners".into(),
                    value: value.into(),
                    options: vec![
                        Choice::new("round", "Rounded"),
                        Choice::new("extend", "Extended"),
                    ],
                    searchable: false,
                },
            );
        }
        _ => {}
    }
}

/// How the sketch stands: solved or not, how free it still is, how many
/// closed regions it has.
fn status(sketch: &Sketch, s: &SketchSession) -> (String, Tone) {
    let profile = sketch.curves.values().any(|c| !c.construction && !c.fixed);
    let regions = match sketch.regions() {
        Ok(regions) => format!(
            "{} region{}",
            regions.len(),
            if regions.len() == 1 { "" } else { "s" }
        ),
        // Nothing drawn yet is no problem of the profile's.
        Err(_) if !profile => "nothing drawn yet".into(),
        Err(e) => e.root_message().to_string(),
    };
    match &s.solved {
        Some(Solved::Failed { error }) => (error.clone(), Tone::Error),
        Some(Solved::Solved { report }) if !report.converged => (
            format!(
                "{} constraint{} conflict · {regions}",
                report.failed_constraints.len(),
                if report.failed_constraints.len() == 1 {
                    ""
                } else {
                    "s"
                }
            ),
            Tone::Error,
        ),
        Some(Solved::Solved { report }) if report.dof == 0 => {
            (format!("Fully constrained · {regions}"), Tone::Success)
        }
        Some(Solved::Solved { report }) => (
            format!(
                "{} degree{} of freedom · {regions}",
                report.dof,
                if report.dof == 1 { "" } else { "s" }
            ),
            Tone::Hint,
        ),
        None => (regions, Tone::Hint),
    }
}

/// The fields for drawing in the sketch of `args` on `before`, lying in
/// `frame`, with `keys` the keys of the visuals selected.
pub(super) fn draw_dialog<'a, S: Scalar>(
    d: &mut Form<'a, S, AddSketchArgs, SketchSession>,
    before: &'a Part<S>,
    args: &AddSketchArgs,
    s: &SketchSession,
    keys: &[String],
    frame: &CoordinateSystem<S>,
) {
    let sketch = &args.sketch;
    let (picks, selected_constraints) = selected(sketch, keys);

    let shortcut =
        |s: Option<&str>| s.map_or(String::new(), |s| format!(" ({})", s.to_uppercase()));
    let mut tools: Vec<Action> = DrawTool::ALL
        .iter()
        .map(|i| {
            let group = if i.tool.modifies() { "Modify" } else { "Draw" };
            Action::new(i.name, i.label)
                .title(format!("{}{}", i.label, shortcut(i.shortcut)))
                .icon(i.name)
                .group(group)
                .active(s.tool == Tool::Draw(i.tool))
        })
        .collect();
    tools.push(
        Action::new("trim", "Trim")
            .title(format!(
                "Trim{}: click a curve, or drag across curves, to remove them up to where they meet others",
                shortcut(Some(TRIM_SHORTCUT))
            ))
            .icon("trim")
            .group("Modify")
            .active(s.tool == Tool::Trim),
    );
    let curves_selected = !selected_curves(sketch, keys).is_empty();
    for i in &ModifyTool::ALL {
        let active = s.tool == Tool::Modify(i.tool);
        let action = Action::new(i.name, i.label)
            .icon(i.name)
            .group("Modify")
            .active(active);
        tools.push(if i.tool.needs_selection() && !curves_selected && !active {
            action.disabled(format!("{}: select the curves to copy first", i.label))
        } else {
            action.title(format!("{}{}", i.label, shortcut(i.shortcut)))
        });
    }
    tools.push(
        Action::new("construction", "Construction")
            .title(
                "Construction geometry (X): the curves selected, or — with none — what is drawn next",
            )
            .icon("construction")
            .group("Modify")
            .active(s.construction),
    );
    d.actions("tool", tools, move |edit, name| {
        editing(before, edit, |e| match name {
            "construction" => e.toggle_construction(),
            "trim" => e.take(Tool::Trim),
            name => {
                if let Some(tool) = DrawTool::by_name(name) {
                    e.take(Tool::Draw(tool));
                } else if let Some(tool) = ModifyTool::by_name(name) {
                    e.take(Tool::Modify(tool));
                }
            }
        });
    });
    tool_options(d, s);
    if let [Pick::Curve(by)] = picks[..]
        && let Some(count) = sketch.pattern_count(by)
    {
        d.on("pattern_count", move |edit, value| {
            if let Value::Number(n) = value {
                let count = (n.round().max(2.0)) as usize;
                editing(before, edit, |e| e.set_pattern_count(by, count));
            }
        });
        let field = Number::new("pattern count", count as f64, Unit::Count).range(2.0, 24.0);
        d.dialog.push("pattern_count", Control::Number(field));
    }

    let constraints: Vec<Action> = ConstraintTool::ALL
        .iter()
        .map(|i| {
            let group = if i.tool.is_dimension() {
                "Dimension"
            } else {
                "Constrain"
            };
            let action = Action::new(i.name, i.label)
                .icon(i.name)
                .group(group)
                .active(s.tool == Tool::Constrain(i.tool));
            if i.tool.fit(sketch, &picks).usable() {
                action.title(format!("{}: {}{}", i.label, i.doc, shortcut(i.shortcut)))
            } else {
                action.disabled(format!("{}: not for what is selected", i.label))
            }
        })
        .collect();
    d.actions("constrain", constraints, move |edit, name| {
        if let Some(tool) = ConstraintTool::by_name(name) {
            editing(before, edit, |e| e.press_constraint(tool));
        }
    });

    let projected: Vec<_> = args
        .references
        .iter()
        .filter_map(|r| match &r.source {
            Source::Projection { entity } => Some(entity.clone()),
            Source::Frame => None,
        })
        .collect();
    d.reference(
        "project",
        "project",
        projected,
        &[Role::Point, Role::Edge, Role::Plane, Role::Round],
        None,
        true,
        move |edit, entities| editing(before, edit, |e| e.project(entities)),
    );
    d.optional("project");

    match &s.error {
        Some(error) => d.text("hint", error.clone(), Tone::Error),
        None => d.text("hint", tool_hint(s), Tone::Hint),
    };
    let (status, tone) = status(sketch, s);
    d.text("status", status, tone);

    let failed = match &s.solved {
        Some(Solved::Solved { report }) => report.failed_constraints.clone(),
        _ => Vec::new(),
    };
    let items: Vec<ListItem> = sketch
        .constraints
        .iter()
        .map(|(&id, c)| {
            let key = format!("constraint:{}", id.0);
            d.on(key.clone(), move |edit, value| {
                editing(before, edit, |e| e.constraint(id, &value))
            });
            let mut item = ListItem::new(key, constraints::name(c));
            item.selected = selected_constraints.contains(&id);
            item.removable = true;
            item.tone = if failed.contains(&id) {
                Tone::Error
            } else {
                Tone::Normal
            };
            item.text =
                args.formulas.get(&id).cloned().or_else(|| {
                    constraints::value(c).map(|v| format!("{}", (v * 1e6).round() / 1e6))
                });
            item
        })
        .collect();
    d.heading(
        "constraints_heading",
        format!("Constraints ({})", sketch.constraints.len()),
    );
    d.list("constraints", items, "None yet.");

    if let Some(id) = s.prompt
        && let Some(c) = sketch.constraints.get(&id)
        && let Some(value) = constraints::value(c)
        && let Some((_, at)) = constraints::glyph(sketch, c, false)
    {
        d.on("prompt", move |edit, value| {
            if let Value::Text(text) = value {
                editing(before, edit, |e| e.set_value(id, &text));
            }
        });
        d.prompt = Some(Prompt {
            key: "prompt".into(),
            label: constraints::name(c),
            value: args
                .formulas
                .get(&id)
                .cloned()
                .unwrap_or_else(|| format!("{}", (value * 1e6).round() / 1e6)),
            at: to_world(frame, at),
        });
    }
}
