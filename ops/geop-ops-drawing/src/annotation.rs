//! What is added on a drawing's sheet by hand: dimensions between what its
//! views show, notes and centre marks ([`Annotation`]) — and what in a view
//! can be picked to add them ([`Candidate`]).
//!
//! An annotation refers to what it measures by the entities' names and the
//! view's, so it follows the part through every rebuild, and its value is
//! measured anew each time: a drawing changes nothing, it only annotates.
//! Its value is placed where the designer dragged it, as an offset on the
//! paper from what it measures, and drawn the way a sketch draws its
//! dimensions (see [`geop_core_sketch::dimension`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_sketch::{
    dimension::Measure,
    plain::{add, cross, dist, dot, perp, scale, sub, unit},
};
use geop_core_topology::EdgeId;
use geop_ops::{Part, RefId, operation::INSTANCE_SEPARATOR};
use serde::{Deserialize, Serialize};

use crate::{
    drawing::{
        ARROW_LENGTH, ARROW_WIDTH, CENTER_OVERSHOOT, Layout, Placed, Placement, TEXT, length_label,
        placed_edge, placed_vertex,
    },
    scene::Scene,
    sheet::{Anchor, Label, Layer, P, Shape, Sheet},
    view::{DrawnView, ViewFrame},
};

/// Which way a distance is measured on the paper.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Along {
    /// Along the line between its points.
    #[default]
    Aligned,
    Horizontal,
    Vertical,
}

/// A point of a view, by the entity that defines it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Target {
    Vertex { name: String },
    /// The centre of a circular edge.
    Center { edge: String },
    /// The middle of an edge: where a note's leader points at it.
    Edge { name: String },
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Target::Vertex { name } | Target::Edge { name } => f.write_str(name),
            Target::Center { edge } => write!(f, "the centre of {edge}"),
        }
    }
}

/// Where a note's leader points: a point of a view.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Leader {
    pub view: DrawnView,
    pub target: Target,
}

/// A dimension, a note or a centre mark added on the sheet, in one of its
/// views. `label` is where its value or text goes: on the paper, in
/// millimetres, from the point it is placed from — the middle between a
/// distance's points, a circle's centre, an angle's vertex, a leader's
/// target — or, for a note without a leader, from the sheet's corner.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Annotation {
    /// The distance between two points as `view` shows them — along the
    /// line between them, or horizontally or vertically on the paper.
    Distance {
        view: DrawnView,
        from: Target,
        to: Target,
        #[serde(default)]
        along: Along,
        label: [f64; 2],
    },
    /// The radius of a circular edge `view` sees round — or, `diameter`,
    /// its diameter.
    Radius {
        view: DrawnView,
        edge: String,
        #[serde(default)]
        diameter: bool,
        label: [f64; 2],
    },
    /// The angle between two straight edges, as `view` shows them: the one
    /// of the four between their lines that `label` is in.
    Angle {
        view: DrawnView,
        a: String,
        b: String,
        label: [f64; 2],
    },
    /// Text, with a leader to a point of a view if it has one.
    Note {
        text: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        leader: Option<Leader>,
        label: [f64; 2],
    },
    /// Two centre lines across a circular edge `view` sees round.
    CenterMark { view: DrawnView, edge: String },
}

/// An annotation as it is drawn on the sheet: its lines, each on its
/// layer, its arrowheads, and its text.
#[derive(Clone, Debug, PartialEq)]
pub struct Drawn {
    pub lines: Vec<(Layer, Vec<P>)>,
    /// Filled triangles.
    pub arrows: Vec<[P; 3]>,
    pub text: Option<Text>,
    /// Where its value is shown, and dragged by.
    pub label: P,
    /// The point it is placed from (see [`Annotation`]).
    pub anchor: P,
}

/// A line of text on the sheet, the middle of its baseline at `at`, turned
/// `angle` degrees counter-clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct Text {
    pub text: String,
    pub at: P,
    pub angle: f64,
}

impl Drawn {
    /// The dimension `measure`, its value `text` shown at `label`, as a
    /// sketch draws one, with drafting's arrowheads: a linear one's text
    /// along its dimension line, reading left to right or bottom to top,
    /// just off it; any other's level, just above the label.
    pub(crate) fn measure(measure: Measure, label: P, text: String) -> Drawn {
        let at = match measure {
            Measure::Linear { along, .. } => {
                let mut u = unit(along).unwrap_or([1.0, 0.0]);
                if u[0] < 0.0 || (u[0] == 0.0 && u[1] < 0.0) {
                    u = scale(u, -1.0);
                }
                Text {
                    text,
                    at: add(label, perp(u)),
                    angle: u[1].atan2(u[0]).to_degrees(),
                }
            }
            _ => Text {
                text,
                at: add(label, [0.0, 1.0]),
                angle: 0.0,
            },
        };
        Drawn {
            lines: measure
                .lines(label)
                .into_iter()
                .map(|line| (Layer::Dimension, line))
                .collect(),
            arrows: measure
                .arrows(label)
                .into_iter()
                .map(|(tip, d)| arrowhead(tip, d))
                .collect(),
            text: Some(at),
            label,
            anchor: label,
        }
    }

    /// Puts it on `sheet`.
    pub fn draw(&self, sheet: &mut Sheet) {
        for (layer, line) in &self.lines {
            let shape = match line[..] {
                [a, b] => Shape::Line(a, b),
                _ => Shape::Polyline(line.clone()),
            };
            sheet.stroke(*layer, shape);
        }
        for arrow in &self.arrows {
            sheet.stroke(Layer::Dimension, Shape::Filled(arrow.to_vec()));
        }
        if let Some(text) = &self.text {
            sheet.labels.push(Label {
                layer: Layer::Dimension,
                at: text.at,
                height: TEXT,
                angle: text.angle,
                anchor: Anchor::Middle,
                text: text.text.clone(),
            });
        }
    }
}

/// An arrowhead with its tip at `tip`, pointing along the unit vector `d`.
fn arrowhead(tip: P, d: P) -> [P; 3] {
    let base = sub(tip, scale(d, ARROW_LENGTH));
    let side = scale(perp(d), ARROW_WIDTH / 2.0);
    [tip, add(base, side), sub(base, side)]
}

impl Annotation {
    /// The view it is in; a note's without a leader is in none.
    pub fn view(&self) -> Option<DrawnView> {
        match self {
            Annotation::Distance { view, .. }
            | Annotation::Radius { view, .. }
            | Annotation::Angle { view, .. }
            | Annotation::CenterMark { view, .. } => Some(*view),
            Annotation::Note { leader, .. } => leader.as_ref().map(|l| l.view),
        }
    }

    /// Where its value or text goes, from the point it is placed from (see
    /// [`Annotation`]); none for a centre mark, which has none.
    pub fn label_mut(&mut self) -> Option<&mut [f64; 2]> {
        match self {
            Annotation::Distance { label, .. }
            | Annotation::Radius { label, .. }
            | Annotation::Angle { label, .. }
            | Annotation::Note { label, .. } => Some(label),
            Annotation::CenterMark { .. } => None,
        }
    }

    /// What kind of annotation it is, as a list names it.
    pub fn kind(&self) -> &'static str {
        match self {
            Annotation::Distance { along, .. } => match along {
                Along::Aligned => "Distance",
                Along::Horizontal => "Horizontal distance",
                Along::Vertical => "Vertical distance",
            },
            Annotation::Radius { diameter: false, .. } => "Radius",
            Annotation::Radius { diameter: true, .. } => "Diameter",
            Annotation::Angle { .. } => "Angle",
            Annotation::Note { .. } => "Note",
            Annotation::CenterMark { .. } => "Centre mark",
        }
    }

    /// It in words, naming what it refers to.
    pub fn describe(&self) -> String {
        let kind = self.kind();
        match self {
            Annotation::Distance { view, from, to, .. } => {
                format!("{kind} from {from} to {to} in the {view} view")
            }
            Annotation::Radius { view, edge, .. } | Annotation::CenterMark { view, edge } => {
                format!("{kind} of {edge} in the {view} view")
            }
            Annotation::Angle { view, a, b, .. } => {
                format!("{kind} between {a} and {b} in the {view} view")
            }
            Annotation::Note { text, leader, .. } => match leader {
                Some(Leader { view, target }) => {
                    format!("{kind} {text:?} pointing at {target} in the {view} view")
                }
                None => format!("{kind} {text:?}"),
            },
        }
    }

    /// How it is drawn on `layout`'s sheet, measured anew in `part` — or
    /// why it cannot be: its view is gone, an entity it names is, or the
    /// view does not show what it measures.
    pub fn drawn<S: Scalar>(&self, part: &Part<S>, layout: &Layout) -> GeopResult<Drawn> {
        let paper_scale = layout.scale;
        match self {
            Annotation::Distance {
                view,
                from,
                to,
                along,
                label,
            } => {
                let (p, frame) = measuring::<S>(layout, *view)?;
                let (a, b) = (paper(part, &frame, from)?, paper(part, &frame, to)?);
                let (direction, value) = match along {
                    Along::Aligned => (sub(b, a), dist(a, b)),
                    Along::Horizontal => ([1.0, 0.0], (b[0] - a[0]).abs()),
                    Along::Vertical => ([0.0, 1.0], (b[1] - a[1]).abs()),
                };
                if value == 0.0 {
                    return Err(GeopError::new(format!(
                        "{from} and {to} are no distance apart in the {view} view"
                    )));
                }
                let (a, b) = (p.place(paper_scale, a), p.place(paper_scale, b));
                let anchor = middle(a, b);
                let mut drawn = Drawn::measure(
                    Measure::Linear {
                        a,
                        b,
                        along: direction,
                    },
                    add(anchor, *label),
                    length_label(value),
                );
                drawn.anchor = anchor;
                Ok(drawn)
            }
            Annotation::Radius {
                view,
                edge,
                diameter,
                label,
            } => {
                let (p, frame) = measuring::<S>(layout, *view)?;
                let (center, radius) = round(part, p, &frame, edge, paper_scale)?;
                let text = match diameter {
                    true => format!("⌀{}", length_label(2.0 * radius)),
                    false => format!("R{}", length_label(radius)),
                };
                let measure = Measure::Radial {
                    center,
                    radius: radius * paper_scale,
                    diameter: *diameter,
                };
                let mut drawn = Drawn::measure(measure, add(center, *label), text);
                drawn.anchor = center;
                Ok(drawn)
            }
            Annotation::Angle { view, a, b, label } => {
                let (p, frame) = measuring::<S>(layout, *view)?;
                let side = |edge: &str| -> GeopResult<(P, P)> {
                    let curve = placed_edge(part, edge)?;
                    if curve.as_line()?.is_none() {
                        return Err(GeopError::new(format!(
                            "the edge {edge} is not straight: only straight edges make an angle"
                        )));
                    }
                    let (t0, t1) = curve.domain();
                    let on = |t| -> GeopResult<P> {
                        let q = frame.project_point(&curve.evaluate(t)?);
                        Ok(p.place(paper_scale, [q[0].to_f64(), q[1].to_f64()]))
                    };
                    Ok((on(t0)?, on(t1)?))
                };
                let ((a0, a1), (b0, b1)) = (side(a)?, side(b)?);
                let (da, db) = (sub(a1, a0), sub(b1, b0));
                let denom = cross(da, db);
                if denom == 0.0 {
                    return Err(GeopError::new(format!(
                        "{a} and {b} are parallel in the {view} view: they make no angle"
                    )));
                }
                let vertex = add(a0, scale(da, cross(sub(b0, a0), db) / denom));
                let label = add(vertex, *label);
                let (from, to, sweep) = sector(da, db, sub(label, vertex));
                let mut drawn = Drawn::measure(
                    Measure::Angular {
                        vertex,
                        from,
                        to,
                        sweep,
                    },
                    label,
                    format!("{}°", length_label(sweep.to_degrees())),
                );
                drawn.anchor = vertex;
                Ok(drawn)
            }
            Annotation::Note {
                text,
                leader,
                label,
            } => {
                let anchor = match leader {
                    Some(Leader { view, target }) => {
                        let p = layout.placement(*view)?;
                        p.place(paper_scale, paper(part, &frame_of(*view)?, target)?)
                    }
                    None => [0.0, 0.0],
                };
                let at = add(anchor, *label);
                let mut drawn = Drawn {
                    lines: Vec::new(),
                    arrows: Vec::new(),
                    text: Some(Text {
                        text: text.clone(),
                        at: add(at, [0.0, 1.0]),
                        angle: 0.0,
                    }),
                    label: at,
                    anchor,
                };
                if leader.is_some()
                    && let Some(d) = unit(sub(anchor, at))
                {
                    drawn.lines.push((Layer::Dimension, vec![at, anchor]));
                    drawn.arrows.push(arrowhead(anchor, d));
                }
                Ok(drawn)
            }
            Annotation::CenterMark { view, edge } => {
                let (p, frame) = measuring::<S>(layout, *view)?;
                let (c, radius) = round(part, p, &frame, edge, paper_scale)?;
                let reach = radius * paper_scale + CENTER_OVERSHOOT;
                Ok(Drawn {
                    lines: vec![
                        (Layer::Center, vec![[c[0] - reach, c[1]], [c[0] + reach, c[1]]]),
                        (Layer::Center, vec![[c[0], c[1] - reach], [c[0], c[1] + reach]]),
                    ],
                    arrows: Vec::new(),
                    text: None,
                    label: c,
                    anchor: c,
                })
            }
        }
    }
}

/// The point halfway between `a` and `b`.
fn middle(a: P, b: P) -> P {
    scale(add(a, b), 0.5)
}

/// The frame the view `view` looks along. Refused for the section view:
/// its lines are of the part cut, which nothing of the part names.
fn frame_of<S: Scalar>(view: DrawnView) -> GeopResult<ViewFrame<S>> {
    match view {
        DrawnView::View(kind) => kind.frame(),
        DrawnView::Section => Err(GeopError::new(
            "the section view cannot be annotated: annotate the views of the whole part",
        )),
    }
}

/// The view `view` to measure in: where it is placed, and the frame it
/// looks along. Refused for the isometric view, which foreshortens every
/// length and angle.
fn measuring<S: Scalar>(
    layout: &Layout,
    view: DrawnView,
) -> GeopResult<(&Placement, ViewFrame<S>)> {
    if !view.orthographic() {
        return Err(GeopError::new(
            "the isometric view foreshortens every length and angle: dimension in another view",
        ));
    }
    Ok((layout.placement(view)?, frame_of(view)?))
}

/// Where `target` is on the paper of the view looking along `frame`, in
/// model units.
fn paper<S: Scalar>(part: &Part<S>, frame: &ViewFrame<S>, target: &Target) -> GeopResult<P> {
    let point = match target {
        Target::Vertex { name } => placed_vertex(part, name)?,
        Target::Center { edge } => {
            placed_edge(part, edge)?
                .as_arc()?
                .ok_or_else(|| GeopError::new(format!("the edge {edge} is not circular")))?
                .circle
                .center
        }
        Target::Edge { name } => {
            let curve = placed_edge(part, name)?;
            let (t0, t1) = curve.domain();
            // Any point of it would do: its middle, sharp.
            curve.evaluate(t0.add(t1).mul(S::from_f64(0.5)).sharpen())?
        }
    };
    let q = frame.project_point(&point);
    Ok([q[0].to_f64(), q[1].to_f64()])
}

/// The centre of the circular edge `edge` on the sheet drawn at `scale`,
/// and its radius — refused unless the view placed at `p`, looking along
/// `frame`, sees it round.
fn round<S: Scalar>(
    part: &Part<S>,
    p: &Placement,
    frame: &ViewFrame<S>,
    edge: &str,
    scale: f64,
) -> GeopResult<(P, f64)> {
    let arc = placed_edge(part, edge)?.as_arc()?.ok_or_else(|| {
        GeopError::new(format!(
            "the edge {edge} is not circular: only a circle has a radius"
        ))
    })?;
    if !seen_round(frame, &arc.circle.normal) {
        return Err(GeopError::new(format!(
            "the {} view does not see the circle {edge} round: dimension it in a view along its axis",
            p.view
        )));
    }
    let c = frame.project_point(&arc.circle.center);
    Ok((
        p.place(scale, [c[0].to_f64(), c[1].to_f64()]),
        arc.circle.radius.to_f64(),
    ))
}

/// Whether a view looking along `frame` sees a circle about `normal` round.
fn seen_round<S: Scalar>(frame: &ViewFrame<S>, normal: &Vector3<S>) -> bool {
    frame
        .direction
        .vector()
        .prod_cross(normal)
        .could_be_equal(&Vector3::zero())
}

/// The sides along `da` or `-da` and `db` or `-db` between which `towards`
/// points, and the angle between them, counter-clockwise from the first:
/// of the four angles two lines make, the one a label there dimensions.
fn sector(da: P, db: P, towards: P) -> (P, P, f64) {
    let ccw = |from: P, to: P| {
        let a = cross(from, to).atan2(dot(from, to));
        if a < 0.0 { a + std::f64::consts::TAU } else { a }
    };
    let mut first = None;
    for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
        let (mut from, mut to) = (scale(da, sa), scale(db, sb));
        let mut sweep = ccw(from, to);
        if sweep > std::f64::consts::PI {
            (from, to) = (to, from);
            sweep = std::f64::consts::TAU - sweep;
        }
        first.get_or_insert((from, to, sweep));
        if ccw(from, towards) <= sweep {
            return (from, to, sweep);
        }
    }
    first.expect("four sides tried")
}

/// What can be picked in a view to annotate it.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    pub view: DrawnView,
    pub what: Pickable,
}

/// A point or an edge of a view, where it is on the sheet.
#[derive(Clone, Debug, PartialEq)]
pub enum Pickable {
    /// A vertex, or the centre of a circle the view sees round.
    Point { target: Target, at: P },
    /// A piece of an edge, drawn as `points`.
    Edge {
        name: String,
        points: Vec<P>,
        shape: EdgeShape,
    },
}

/// What an edge is, for what it can be dimensioned by.
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeShape {
    /// Straight, between these two vertices.
    Line { start: String, end: String },
    /// A circle or an arc the view sees round; `full` if it is a whole
    /// circle, its ends one vertex.
    Circle { full: bool },
    Other,
}

/// Points along each piece of a line a candidate is drawn with.
const POINTS_PER_PIECE: usize = 12;

/// What can be picked in the views `placed` of `scene` at `scale`: the
/// vertices of the edges each draws, the centres of the circles it sees
/// round, and the edges themselves — each by its name in the part drawn.
/// The section view's lines are of the part cut, whose entities the part
/// does not name: nothing can be picked in it.
pub(crate) fn candidates<S: Scalar>(
    scene: &Scene<'_, S>,
    placed: &[Placed<S>],
    scale: f64,
) -> GeopResult<Vec<Candidate>> {
    let mut out: Vec<Candidate> = Vec::new();
    for p in placed.iter().filter(|p| matches!(p.kind, DrawnView::View(_))) {
        let placement = Placement::of(p);
        let place = |q: P| placement.place(scale, q);
        let frame = &p.view.frame;
        let on_sheet = |q: &Vector3<S>| {
            let v = frame.project_point(q);
            place([v[0].to_f64(), v[1].to_f64()])
        };
        // The pieces of each edge drawn, by body and edge.
        type Pieces = Vec<Vec<P>>;
        let mut edges: Vec<((usize, EdgeId), Pieces)> = Vec::new();
        for line in &p.view.lines {
            let Some(edge) = line.edge else { continue };
            let (t0, t1) = line.curve.domain();
            let mut points = Vec::with_capacity(POINTS_PER_PIECE + 1);
            for k in 0..=POINTS_PER_PIECE {
                // Where along it is a free choice, but its ends are its own.
                let t = match k {
                    0 => t0,
                    POINTS_PER_PIECE => t1,
                    k => {
                        let f = S::from_f64(k as f64 / POINTS_PER_PIECE as f64);
                        t0.add(t1.sub(t0).mul(f)).sharpen()
                    }
                };
                let q = line.curve.evaluate(t)?;
                points.push(place([q[0].to_f64(), q[1].to_f64()]));
            }
            let key = (line.body, edge);
            match edges.iter_mut().find(|(k, _)| *k == key) {
                Some((_, pieces)) => pieces.push(points),
                None => edges.push((key, vec![points])),
            }
        }
        let add_point = |out: &mut Vec<Candidate>, target: Target, at: P| {
            let known = out.iter().any(|c| {
                c.view == p.kind && matches!(&c.what, Pickable::Point { target: t, .. } if *t == target)
            });
            if !known {
                out.push(Candidate {
                    view: p.kind,
                    what: Pickable::Point { target, at },
                });
            }
        };
        for ((body, edge), pieces) in edges {
            let b = &scene.bodies[body];
            let model = b.model();
            let named = |id: RefId| {
                b.part.name_of(id).map(|n| match b.path.as_str() {
                    "" => n.to_string(),
                    path => format!("{path}{INSTANCE_SEPARATOR}{n}"),
                })
            };
            let Some(name) = named(edge.into()) else {
                continue;
            };
            let e = model.get_edge(edge)?;
            let world = |q: Vector3<S>| match &b.pose {
                Some(pose) => pose.apply(&q),
                None => q,
            };
            let ends = [e.start_vertex, e.end_vertex].map(|v| named(v.into()));
            for (v, end) in [e.start_vertex, e.end_vertex].into_iter().zip(&ends) {
                if let Some(end) = end {
                    let at = on_sheet(&world(model.get_vertex(v)?.point));
                    add_point(&mut out, Target::Vertex { name: end.clone() }, at);
                }
            }
            let curve = match &b.pose {
                Some(pose) => e.curve.transform(&pose.motion()),
                None => e.curve.clone(),
            };
            let shape = if curve.as_line()?.is_some() {
                match ends {
                    [Some(start), Some(end)] => EdgeShape::Line { start, end },
                    _ => EdgeShape::Other,
                }
            } else if let Some(arc) = curve.as_arc()?
                && seen_round(frame, &arc.circle.normal)
            {
                let target = Target::Center { edge: name.clone() };
                add_point(&mut out, target, on_sheet(&arc.circle.center));
                EdgeShape::Circle {
                    full: e.start_vertex == e.end_vertex,
                }
            } else {
                EdgeShape::Other
            };
            for points in pieces {
                out.push(Candidate {
                    view: p.kind,
                    what: Pickable::Edge {
                        name: name.clone(),
                        points,
                        shape: shape.clone(),
                    },
                });
            }
        }
    }
    Ok(out)
}
