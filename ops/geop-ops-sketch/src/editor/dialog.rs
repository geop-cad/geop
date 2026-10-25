//! The fields for drawing a sketch: the tools, each a button of its own;
//! what the sketch projects; one line on what to do next and one on how
//! the sketch stands; and its constraints.

use geop_core_editor::editing::{Choice, Number, Prompt, Unit};

use crate::{icons, references::Source};
use geop_ops_model::sketch::fields::constraint_status;

use super::*;

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
fn tool_options(d: &mut EditForm<'_, S, AddSketchArgs, SketchSession>, s: &SketchSession) {
    let count = |d: &mut EditForm<'_, S, AddSketchArgs, SketchSession>| {
        let field = Number::new("copies", s.modify.count as f64, Unit::Count).range(2.0, 24.0);
        d.number("count", field, |edit, n| {
            edit.session.modify.count = (n.round() as usize).clamp(2, 1000);
        });
    };
    match s.tool {
        Tool::Draw(DrawTool::Polygon) => {
            let sides = Number::new("sides", s.sides as f64, Unit::Count).range(3.0, 12.0);
            d.number("sides", sides, |edit, n| {
                edit.session.sides = (n.round() as usize).clamp(3, 64);
            })
            .checkbox(
                "circumscribed",
                "Sides touch the circle",
                s.circumscribed,
                |edit, b| edit.session.circumscribed = b,
            );
        }
        Tool::Modify(ModifyTool::LinearPattern) => {
            count(d);
            let spacing = Number::new("spacing", s.modify.spacing, Unit::Length);
            d.number("spacing", spacing, |edit, v| {
                if v > 0.0 {
                    edit.session.modify.spacing = v;
                }
            });
        }
        Tool::Modify(ModifyTool::CircularPattern) => {
            count(d);
            let pitch = s.modify.pitch.unwrap_or(360.0 / s.modify.count as f64);
            let angle = Number::new("angle between copies", pitch, Unit::Angle);
            d.number("pitch", angle, |edit, v| {
                edit.session.modify.pitch = Some(v)
            });
        }
        Tool::Modify(ModifyTool::Offset) => {
            let value = match s.modify.corners {
                Corners::Round => "round",
                Corners::Extend => "extend",
            };
            let options = vec![
                Choice::new("round", "Rounded"),
                Choice::new("extend", "Extended"),
            ];
            d.checkbox("both", "Both sides", s.modify.both, |edit, b| {
                edit.session.modify.both = b
            })
            .select(
                "corners",
                "corners",
                value,
                options,
                false,
                |edit, choice| {
                    edit.session.modify.corners = match choice {
                        "extend" => Corners::Extend,
                        _ => Corners::Round,
                    };
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
        Some(Solved::Solved { report }) => constraint_status(
            report.converged,
            report.failed_constraints.len(),
            report.dof,
            &regions,
        ),
        None => (regions, Tone::Hint),
    }
}

/// The fields for drawing in the sketch of `args` on the part `view`
/// shows, lying in `frame`, with `keys` the keys of the visuals selected.
pub(super) fn draw_dialog<'a>(
    d: &mut EditForm<'a, S, AddSketchArgs, SketchSession>,
    view: &'a Reader<'a>,
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
                .icon(icons::named(i.name))
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
            .icon(icons::TRIM)
            .group("Modify")
            .active(s.tool == Tool::Trim),
    );
    let curves_selected = !selected_curves(sketch, keys).is_empty();
    for i in &ModifyTool::ALL {
        let active = s.tool == Tool::Modify(i.tool);
        let action = Action::new(i.name, i.label)
            .icon(icons::named(i.name))
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
            .icon(icons::CONSTRUCTION)
            .group("Modify")
            .active(s.construction),
    );
    d.actions("tool", tools, move |edit, name| {
        editing(view, edit, |e| match name {
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
        let field = Number::new("pattern count", count as f64, Unit::Count).range(2.0, 24.0);
        d.number("pattern_count", field, move |edit, n| {
            let count = (n.round().max(2.0)) as usize;
            editing(view, edit, |e| e.set_pattern_count(by, count));
        });
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
                .icon(icons::named(i.name))
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
            editing(view, edit, |e| e.press_constraint(tool));
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
    d.pick(
        "project",
        "project",
        view,
        &projected,
        true,
        move |edit, entities| editing(view, edit, |e| e.project(entities)),
    );
    d.form.dialog.optional("project");

    match &s.error {
        Some(error) => d.form.dialog.text("hint", error.clone(), Tone::Error),
        None => d.form.dialog.text("hint", tool_hint(s), Tone::Hint),
    };
    let (status, tone) = status(sketch, s);
    d.form.dialog.text("status", status, tone);

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
                editing(view, edit, |e| e.constraint(id, &value))
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
                args.formulas.get(&id).map(|f| f.to_string()).or_else(|| {
                    constraints::value(c).map(|v| format!("{}", (v * 1e6).round() / 1e6))
                });
            item
        })
        .collect();
    d.form.dialog.heading(
        "constraints_heading",
        format!("Constraints ({})", sketch.constraints.len()),
    );
    d.form.dialog.list("constraints", items, "None yet.");

    if let Some(id) = s.prompt
        && let Some(c) = sketch.constraints.get(&id)
        && let Some(value) = constraints::value(c)
        && let Some((_, at)) = constraints::glyph(sketch, c, false)
    {
        d.on("prompt", move |edit, value| {
            if let Value::Text(text) = value {
                editing(view, edit, |e| e.set_value(id, &text));
            }
        });
        d.form.prompt = Some(Prompt {
            key: "prompt".into(),
            label: constraints::name(c),
            value: (args.formulas.get(&id))
                .map(|f| f.to_string())
                .unwrap_or_else(|| format!("{}", (value * 1e6).round() / 1e6)),
            at: to_world(frame, at),
        });
    }
}
