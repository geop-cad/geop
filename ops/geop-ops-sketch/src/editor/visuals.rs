//! What the sketch looks like while it is edited.

use super::drawing::{Hints, construct};
use super::snap::curve_mid;
use super::*;
use crate::references::{Source, X_AXIS, Y_AXIS};

/// The far ends of the sketch's own axes: only there to give the axes a
/// direction, so never drawn.
fn axis_ends(args: &AddSketchArgs) -> Vec<PointId> {
    args.references
        .iter()
        .filter(|r| r.source == Source::Frame)
        .flat_map(|r| [X_AXIS, Y_AXIS].map(|k| r.points.get(k).copied()))
        .flatten()
        .collect()
}

/// The polyline `curve` is drawn as: the sketch's own axes across all of
/// it, any other curve as it is.
pub(super) fn drawn(args: &AddSketchArgs, curve: CurveId) -> Vec<P2> {
    let sketch = &args.sketch;
    let ends = axis_ends(args);
    if let Some(CurveKind::Line { start, end }) = sketch.curves.get(&curve).map(|c| &c.kind)
        && ends.contains(end)
    {
        let extent = sketch
            .points
            .iter()
            .filter(|(id, _)| !ends.contains(id))
            .map(|(&id, _)| {
                let [x, y] = pt(sketch, id);
                x.abs().max(y.abs())
            })
            .fold(1.0, f64::max);
        let (o, d) = (
            pt(sketch, *start),
            sub(pt(sketch, *end), pt(sketch, *start)),
        );
        let reach = 10.0 * extent;
        return vec![
            sub(o, crate::geometry::scale(d, reach)),
            add(o, crate::geometry::scale(d, reach)),
        ];
    }
    polyline(sketch, curve)
}

/// What the sketch of `args` looks like in its plane `frame`, with what
/// is being drawn in `s`: its points and curves selectable — and, unless
/// given from outside, draggable — its constraints' glyphs selectable and
/// dimensions' values draggable, where the pointer snaps to, and a
/// dimension being placed with `selection`.  What is selected or hovered the editor
/// draws so.
pub(super) fn visuals<S: Scalar>(
    args: &AddSketchArgs,
    s: &SketchSession,
    selection: &[String],
    frame: &CoordinateSystem<S>,
) -> Vec<Visual<S>> {
    let sketch = &args.sketch;
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
    let world = |p: P2| to_world(frame, p);
    let hidden = axis_ends(args);
    let mut out = Vec::new();

    if let Ok(regions) = sketch.regions() {
        for (i, region) in regions.iter().enumerate() {
            let uv = |polyline: Vec<P2>| -> Vec<Vector2<S>> {
                polyline
                    .into_iter()
                    .map(|p| Vector2::from_array(p.map(S::from_f64)))
                    .collect()
            };
            let outer = uv(loop_polyline(sketch, &region.outer));
            let holes: Vec<_> = region
                .holes
                .iter()
                .map(|h| uv(loop_polyline(sketch, h)))
                .collect();
            out.push(Visual::new(
                format!("region{i}"),
                Shape::region(frame, &outer, &holes),
                Style::Region,
            ));
        }
    }
    for (&id, curve) in &sketch.curves {
        let style = if failed_curves.contains(&id) {
            Style::Failed
        } else if curve.fixed {
            Style::Reference
        } else if curve.construction {
            Style::Construction
        } else if report.is_none_or(|r| r.free_curves.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        let visual = Visual::new(
            id.to_string(),
            Shape::Polyline {
                points: drawn(args, id).into_iter().map(world).collect(),
            },
            style,
        )
        .selectable();
        out.push(if curve.fixed {
            visual
        } else {
            visual.draggable()
        });
        if let CurveKind::Spline {
            control_points,
            shape: None,
        } = &curve.kind
        {
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
    if let Some(cursor) = s.cursor {
        for (i, points) in draft_preview(args, s, cursor).into_iter().enumerate() {
            if points.len() >= 2 {
                out.push(Visual::new(
                    format!("draft{i}"),
                    Shape::Polyline {
                        points: points.into_iter().map(world).collect(),
                    },
                    Style::Draft,
                ));
            }
        }
    }
    for (&id, point) in &sketch.points {
        if hidden.contains(&id) {
            continue;
        }
        let style = if failed_points.contains(&id) {
            Style::Failed
        } else if report.is_none_or(|r| r.free_points.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        let visual = Visual::new(
            id.to_string(),
            Shape::Point {
                at: world(xy(sketch, id)),
            },
            style,
        )
        .selectable();
        out.push(if point.fixed {
            visual
        } else {
            visual.draggable()
        });
    }
    if let Some(snap) = s.snap {
        let at = match snap {
            Snap::Point(p) => Some(pt(sketch, p)),
            Snap::Midpoint(c) => curve_mid(sketch, c),
            Snap::Intersection(..) | Snap::OnCurve(_) => s.cursor,
        };
        if let Some(at) = at {
            out.push(Visual::new(
                "snap",
                Shape::Point { at: world(at) },
                Style::Snap,
            ));
        }
    }
    // Glyphs of one spot stack sideways, so each can be read and clicked;
    // a dimension placed by the designer is where it was put, with its
    // dimension lines.
    let style_of = |id: &ConstraintId| {
        if report.is_some_and(|r| r.failed_constraints.contains(id)) {
            Style::Failed
        } else {
            Style::Fixed
        }
    };
    let mut anchors: Vec<P2> = Vec::new();
    for (&id, c) in &sketch.constraints {
        let formula = args.formulas.contains_key(&id);
        let Some((text, at)) = constraints::glyph(sketch, c, formula) else {
            continue;
        };
        let dimension = constraints::value(c).is_some();
        let label = match args.labels.get(&id) {
            Some(&offset) if dimension => {
                let placed = add(at, offset);
                dimension_visuals(
                    &mut out,
                    frame,
                    &id.to_string(),
                    sketch,
                    c,
                    placed,
                    Style::Guide,
                );
                Shape::Label {
                    at: world(placed),
                    text,
                    offset: Vector3::zero(),
                }
            }
            _ => {
                let stack = anchors.iter().filter(|&&a| dist(a, at) < 1e-9).count();
                anchors.push(at);
                Shape::Label {
                    at: world(at),
                    text,
                    offset: frame
                        .u()
                        .prod_scalar(S::from_f64(1.5 + 2.5 * stack as f64))
                        .add(&frame.v().prod_scalar(S::from_f64(1.3))),
                }
            }
        };
        let visual = Visual::new(id.to_string(), label, style_of(&id)).selectable();
        // A dimension's value is dragged out of the way of the geometry.
        out.push(if dimension {
            visual.draggable()
        } else {
            visual
        });
    }
    // A dimension being placed, where the pointer would put it.
    if let (Tool::Constrain(tool), Some(cursor)) = (s.tool, s.cursor)
        && tool.is_dimension()
    {
        let (picks, _) = selected(sketch, selection);
        if let Some(c) = tool
            .build(sketch, &picks)
            .and_then(|cs| cs.into_iter().next())
            && let Some((text, _)) = constraints::glyph(sketch, &c, false)
        {
            dimension_visuals(&mut out, frame, "placing", sketch, &c, cursor, Style::Draft);
            out.push(Visual::new(
                "placing",
                Shape::Label {
                    at: world(cursor),
                    text,
                    offset: Vector3::zero(),
                },
                Style::Draft,
            ));
        }
    }
    out
}

/// The lines of the dimension `c` with its value at `label`, as visuals
/// keyed after `key`, drawn in `style`.
fn dimension_visuals<S: Scalar>(
    out: &mut Vec<Visual<S>>,
    frame: &CoordinateSystem<S>,
    key: &str,
    sketch: &Sketch,
    c: &Constraint,
    label: P2,
    style: Style,
) {
    for (i, line) in constraints::dimension_lines(sketch, c, label)
        .into_iter()
        .enumerate()
    {
        out.push(Visual::new(
            format!("{key}/{i}"),
            Shape::Polyline {
                points: line.into_iter().map(|p| to_world(frame, p)).collect(),
            },
            style,
        ));
    }
}

/// What would be drawn with the next point at `cursor`, as polylines: the
/// curves the tool would build, if that is its last point — else a line
/// through the points placed so far, to the cursor.
fn draft_preview(args: &AddSketchArgs, s: &SketchSession, cursor: P2) -> Vec<Vec<P2>> {
    let Tool::Draw(tool) = s.tool else {
        return Vec::new();
    };
    let draft = &s.draft;
    if draft.placed.is_empty() {
        return Vec::new();
    }
    let mut placed = draft.placed.clone();
    placed.push(Placed {
        at: cursor,
        snap: s.snap,
    });
    let complete = tool.needs().is_none_or(|n| placed.len() == n);
    if complete {
        // Previewed as the sketch itself would draw it: built in a copy.
        let mut temp = args.sketch.clone();
        let first_new = temp.next_id;
        let hints = Hints {
            sides: s.sides,
            tangent_to: draft.previous,
            sweep: draft.sweep,
            min_size: 0.0,
        };
        if construct(tool, draft.arc, &mut temp, &placed, &hints).is_some() {
            return temp
                .curves
                .range(CurveId(first_new)..)
                .map(|(&c, _)| polyline(&temp, c))
                .collect();
        }
    }
    vec![placed.iter().map(|p| p.at).collect()]
}
