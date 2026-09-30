//! What the sketch looks like while it is edited.

use super::*;

/// What the sketch looks like in its plane `frame`, with what is selected,
/// hovered and being drawn in `s`.
pub(super) fn visuals<S: Scalar>(
    sketch: &Sketch,
    s: &SketchSession,
    frame: &CoordinateSystem<S>,
) -> Vec<Visual<S>> {
    let report = match &s.solved {
        Some(Solved::Solved { report }) => Some(report),
        _ => None,
    };
    let failed: Vec<&Constraint> = report
        .map(|r| {
            r.failed_constraints
                .iter()
                .filter_map(|k| sketch.constraints.get(k))
                .collect()
        })
        .unwrap_or_default();
    let failed_points: Vec<PointId> = failed.iter().flat_map(|c| c.points()).collect();
    let failed_curves: Vec<CurveId> = failed.iter().flat_map(|c| c.curves()).collect();
    let hovered = |key: &str| s.hover.as_deref() == Some(key);
    let world = |p: P2| to_world(frame, p);
    let positions = sketch.positions();
    let mut out = Vec::new();

    out.push(Visual::new(
        "origin",
        Shape::Point {
            at: world([0.0, 0.0]),
        },
        if hovered("origin") {
            Style::Hover
        } else {
            Style::Guide
        },
    ));
    if let Ok(regions) = sketch.regions() {
        for (i, region) in regions.iter().enumerate() {
            let uv = |polyline: Vec<P2>| -> Vec<Vector2<S>> {
                polyline
                    .into_iter()
                    .map(|p| Vector2::from_array(p.map(S::from_f64)))
                    .collect()
            };
            let outer = uv(region.outer.polyline(sketch, &positions));
            let holes: Vec<_> = region
                .holes
                .iter()
                .map(|h| uv(h.polyline(sketch, &positions)))
                .collect();
            out.push(Visual::new(
                format!("region{i}"),
                Shape::region(frame, &outer, &holes),
                Style::Region,
            ));
        }
    }
    for (&id, curve) in &sketch.curves {
        let key = id.to_string();
        let style = if s.selection.curves.contains(&id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if failed_curves.contains(&id) {
            Style::Failed
        } else if curve.construction {
            Style::Construction
        } else if report.is_none_or(|r| r.free_curves.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        out.push(Visual::new(
            key,
            Shape::Polyline {
                points: curve_polyline(sketch, &positions, id)
                    .into_iter()
                    .map(world)
                    .collect(),
            },
            style,
        ));
        if let CurveKind::Spline { control_points } = &curve.kind {
            out.push(Visual::new(
                format!("hull{}", id.0),
                Shape::Polyline {
                    points: control_points
                        .iter()
                        .map(|&p| world(pt(sketch, p)))
                        .collect(),
                },
                Style::Guide,
            ));
        }
    }
    if let (Some(draft), Some(cursor)) = (&s.draft, s.cursor) {
        let preview = draft_preview(sketch, draft, cursor);
        if preview.len() >= 2 {
            out.push(Visual::new(
                "draft",
                Shape::Polyline {
                    points: preview.into_iter().map(world).collect(),
                },
                Style::Draft,
            ));
        }
    }
    for (&id, p) in &sketch.points {
        let key = id.to_string();
        let style = if s.selection.points.contains(&id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if failed_points.contains(&id) {
            Style::Failed
        } else if report.is_none_or(|r| r.free_points.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        out.push(Visual::new(key, Shape::Point { at: world(p.xy()) }, style));
    }
    // Glyphs of one spot stack sideways, so each can be read and clicked.
    let mut anchors: Vec<P2> = Vec::new();
    for (&id, c) in &sketch.constraints {
        let Some((text, at)) = constraints::glyph(sketch, c) else {
            continue;
        };
        let stack = anchors.iter().filter(|&&a| dist(a, at) < 1e-9).count();
        anchors.push(at);
        let key = id.to_string();
        let style = if s.selected_constraint == Some(id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if report.is_some_and(|r| r.failed_constraints.contains(&id)) {
            Style::Failed
        } else {
            Style::Fixed
        };
        out.push(Visual::new(
            key,
            Shape::Label {
                at: world(at),
                text,
                offset: frame
                    .u()
                    .prod_scalar(S::from_f64(1.5 + 2.5 * stack as f64))
                    .add(&frame.v().prod_scalar(S::from_f64(1.3))),
            },
            style,
        ));
    }
    out
}

/// What the curve being drawn would be with its next point at `cursor`.
fn draft_preview(sketch: &Sketch, draft: &Draft, cursor: P2) -> Vec<P2> {
    // Previewed as the sketch itself would draw the curve: in a copy, with
    // a point at the cursor.
    let mut temp = sketch.clone();
    let at = temp.add_point(cursor[0], cursor[1]);
    let curve = match draft {
        Draft::Line { start } => temp.add_line(*start, at),
        Draft::Arc { start, end: None } => temp.add_line(*start, at),
        Draft::Arc {
            start,
            end: Some(end),
        } => {
            let sweep = sweep_through(pt(&temp, *start), pt(&temp, *end), cursor);
            if !sweep.is_finite() {
                return Vec::new();
            }
            temp.add_arc_with_sweep(*start, *end, sweep)
        }
        Draft::Circle { center } => {
            let radius = dist(pt(&temp, *center), cursor);
            temp.add_circle(*center, radius)
        }
        Draft::Rectangle { corner } => {
            let [x0, y0] = pt(&temp, *corner);
            return vec![[x0, y0], [cursor[0], y0], cursor, [x0, cursor[1]], [x0, y0]];
        }
        Draft::Spline { points } => {
            let mut points = points.clone();
            points.push(at);
            temp.add_spline(points)
        }
    };
    curve_polyline(&temp, &temp.positions(), curve)
}
