//! The fields for drawing a sketch.

use super::*;

/// What each tool asks for next.
fn tool_hint(tool: Tool, draft: Option<&Draft>) -> &'static str {
    match (tool, draft) {
        (Tool::Select, _) => {
            "Click to select (adds to the selection) · drag points or curves · drag empty space to pan"
        }
        (Tool::Line, None) => "Click the start point",
        (Tool::Line, Some(_)) => "Click the next point · right-click / Esc ends the chain",
        (Tool::Rectangle, None) => "Click one corner",
        (Tool::Rectangle, Some(_)) => "Click the opposite corner",
        (Tool::Arc, None) => "Click the start point",
        (Tool::Arc, Some(Draft::Arc { end: None, .. })) => "Click the end point",
        (Tool::Arc, Some(_)) => "Click a point the arc passes through",
        (Tool::Circle, None) => "Click the center",
        (Tool::Circle, Some(_)) => "Click a point on the circle",
        (Tool::Spline, _) => "Click control points · double-click / Enter / right-click finishes",
        (Tool::Point, _) => "Click to place a point",
    }
}

/// The fields for drawing in the sketch on `before`: tools, constraints,
/// and how the sketch stands, with `keys` the keys of the visuals selected.
pub(super) fn draw_dialog<'a, S: Scalar>(
    d: &mut Form<'a, S, AddSketchArgs, SketchSession>,
    before: &'a Part<S>,
    sketch: &Sketch,
    s: &SketchSession,
    keys: &[String],
) {
    let (selection, selected_constraints) = selected(sketch, keys);
    d.heading("draw_heading", "Draw");
    d.actions(
        "tool",
        Tool::ALL
            .iter()
            .map(|&(tool, name, label, shortcut)| {
                Action::new(name, label)
                    .title(format!("Shortcut: {shortcut}"))
                    .active(s.tool == tool)
            })
            .collect(),
        move |edit, name| {
            if let Some(tool) = Tool::by_name(name) {
                editing(before, edit, |e| e.take(tool));
            }
        },
    );
    d.text("tool_hint", tool_hint(s.tool, s.draft.as_ref()), Tone::Hint);

    d.heading("constrain_heading", "Constrain");
    let options = constraints::options(sketch, &selection);
    if options.is_empty() && selection.curves.is_empty() {
        d.text(
            "constrain_hint",
            "Select points and curves (Select tool) to see the constraints that apply.",
            Tone::Hint,
        );
    }
    if !options.is_empty() {
        d.actions(
            "constrain",
            options
                .iter()
                .map(|o| Action::new(o.label, o.label).title(o.title))
                .collect(),
            move |edit, label| editing(before, edit, |e| e.constrain(label)),
        );
    }
    let mut edits = Vec::new();
    if !selection.curves.is_empty() {
        edits.push(Action::new("construction", "Construction"));
    }
    if !selection.is_empty() || !selected_constraints.is_empty() {
        edits.push(Action::new("delete", "Delete"));
    }
    if !edits.is_empty() {
        d.actions("edit", edits, move |edit, action| {
            editing(before, edit, |e| match action {
                "construction" => e.toggle_construction(),
                "delete" => e.delete_selection(),
                _ => {}
            })
        });
    }

    d.heading("status_heading", "Status");
    let (status, tone) = match &s.solved {
        Some(Solved::Failed { error }) => (error.clone(), Tone::Error),
        Some(Solved::Solved { report }) if !report.converged => (
            format!(
                "Over-constrained or conflicting: {} constraint(s) cannot be met",
                report.failed_constraints.len()
            ),
            Tone::Error,
        ),
        Some(Solved::Solved { report }) if report.dof == 0 => {
            ("Fully constrained".into(), Tone::Success)
        }
        Some(Solved::Solved { report }) => (
            format!(
                "{} degree{} of freedom",
                report.dof,
                if report.dof == 1 { "" } else { "s" }
            ),
            Tone::Hint,
        ),
        None => (String::new(), Tone::Hint),
    };
    d.text("status", status, tone);
    let curves = sketch.curves.values().filter(|c| !c.construction).count();
    let regions = match sketch.regions() {
        Ok(regions) => format!(
            "{} closed region{}",
            regions.len(),
            if regions.len() == 1 { "" } else { "s" }
        ),
        Err(e) => e.root_message().to_string(),
    };
    d.text(
        "counts",
        format!(
            "{curves} curve{} · {regions}",
            if curves == 1 { "" } else { "s" }
        ),
        Tone::Hint,
    );

    d.heading("constraints_heading", "Constraints");
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
            item.value = constraints::value(c).map(|v| {
                if matches!(c, Constraint::Angle { .. }) {
                    v.to_degrees()
                } else {
                    v
                }
            });
            item
        })
        .collect();
    d.list("constraints", items, "None yet.");
    if let Some([x, y]) = s.cursor {
        d.text("cursor", format!("x {x:.3} · y {y:.3}"), Tone::Hint);
    }
}
