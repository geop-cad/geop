//! [`Drawing`]: a step describing a 2-D drawing of the part as the steps
//! before it built it — its views, a section, the title block, and the
//! dimensions, notes and centre marks added on its sheet. It changes
//! nothing; the drawing is made from it when it is downloaded
//! ([`crate::drawing::compose`]).
//!
//! Edited, it shows its sheet in the viewport — laid in the world's `xy`
//! plane, a millimetre of paper to a unit, faced head on as a sketch's plane
//! is — and is annotated there, the way a sketch is dimensioned: a tool
//! taken up in the dialog picks what the views show (vertices, circles'
//! centres, edges), the dimension then follows the pointer, and a click
//! puts its value down. A value is dragged where it goes; Delete removes
//! what is selected.
//!
//! The sheet shown is the one downloaded, but for its date: laying it out
//! is the costly part, so it is kept in the session for as long as the
//! part and what the views depend on are the same (see
//! [`DrawingSession::show`]).

use std::{any::Any, cell::RefCell, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::plain::{add, sub};
use geop_ops::{
    Context, Library, Part,
    operation::{Operation, Role},
    ui::{
        Action, CanvasEvent, Choice, Edit, Extent, Form, InHand, ListItem, Pointer, Shape, Style,
        Tone, Value, Visual, hit::hit_visuals,
    },
};
use serde::{Deserialize, Serialize};

use crate::{
    annotation::{Along, Annotation, Candidate, Drawn, EdgeShape, Leader, Pickable, Target},
    drawing::{
        DrawingArgs, Layout, PartsListLine, Projected, Projection, SCALES, SheetSize, annotated,
        arrange, placed_edge, placed_vertex, project, scale_label,
    },
    sheet::{Anchor, Layer, P, Sheet},
    view::{DrawnView, ViewKind},
};

/// Describes a drawing of the part: see [`DrawingArgs`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Drawing;

/// A tool for annotating the sheet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tool {
    /// None: a click selects, a value is dragged.
    #[default]
    Select,
    /// Whatever the picks call for: the distance between two points, a
    /// line's length, the angle between two lines, a circle's diameter or
    /// an arc's radius.
    Dimension,
    Horizontal,
    Vertical,
    Radius,
    Diameter,
    Angle,
    Note,
    CenterMark,
}

/// The tools, as the dialog offers them: each with its name, its icon and
/// what it does.
const TOOLS: [(Tool, &str, &str); 8] = [
    (
        Tool::Dimension,
        "distance",
        "Dimension — two points, a line, two lines or a circle",
    ),
    (
        Tool::Horizontal,
        "distance_x",
        "Horizontal distance — two points or a line",
    ),
    (
        Tool::Vertical,
        "distance_y",
        "Vertical distance — two points or a line",
    ),
    (Tool::Radius, "radius", "Radius of a circle or an arc"),
    (Tool::Diameter, "diameter", "Diameter of a circle or an arc"),
    (Tool::Angle, "angle", "Angle between two lines"),
    (
        Tool::Note,
        "note",
        "Note — click what it points at, then where its text goes; or click empty paper",
    ),
    (Tool::CenterMark, "center_mark", "Centre mark on a circle"),
];

impl Tool {
    fn by_name(name: &str) -> Option<Tool> {
        TOOLS.iter().find(|t| t.1 == name).map(|t| t.0)
    }

    fn label(self) -> &'static str {
        TOOLS.iter().find(|t| t.0 == self).map_or("", |t| t.2)
    }
}

/// What editing a drawing keeps between events — and what the editor
/// gives it that a drawing cannot work out itself: the bill of materials
/// (see [`DrawingSession::show`]).
pub struct DrawingSession {
    tool: Tool,
    /// What was picked for the tool so far, all in one view.
    picks: Vec<Candidate>,
    /// What a click would pick.
    hover: Option<Candidate>,
    /// Where the pointer is on the sheet.
    cursor: Option<P>,
    /// The text the next note gets.
    note: String,
    /// The annotation whose value is being dragged, and where its value was
    /// when it was grabbed.
    drag: Option<(usize, [f64; 2])>,
    /// Why the last annotation picked could not be added.
    refused: Option<String>,
    /// The bill of materials the editor gave, for the part of which
    /// revision and whether it was asked for: the sheet is shown once it
    /// has been given one.
    parts: Option<(u64, bool, Result<Vec<PartsListLine>, String>)>,
    /// The views as last projected — a `Result<Projected<S>, String>` —
    /// for the part of which revision, and what of the arguments they
    /// depend on (see [`Projected::depends_on`]).
    projected: Kept<dyn Any>,
    /// The sheet as last laid out: for the part of which revision, and the
    /// arguments without their annotations.
    laid_out: Kept<Result<Layout, String>>,
}

/// What a session keeps of what it worked out: for the part of which
/// revision, from which arguments, and what.
type Kept<T> = RefCell<Option<(u64, DrawingArgs, Rc<T>)>>;

impl Default for DrawingSession {
    fn default() -> Self {
        Self {
            tool: Tool::Select,
            picks: Vec::new(),
            hover: None,
            cursor: None,
            note: "NOTE".into(),
            drag: None,
            refused: None,
            parts: None,
            projected: RefCell::new(None),
            laid_out: RefCell::new(None),
        }
    }
}

impl DrawingSession {
    /// Whether the editor still has to give it the bill of materials of the
    /// part of `revision` (see [`Part::revision`]), asked for or not —
    /// `bom` — before its sheet can be shown.
    pub fn wants_parts(&self, revision: u64, bom: bool) -> bool {
        self.parts
            .as_ref()
            .is_none_or(|(r, b, _)| *r != revision || *b != bom)
    }

    /// Shows the sheet in the viewport, with `parts` the bill of materials
    /// of the part of `revision` if `bom` asks for one — or why it could
    /// not be made. A drawing cannot list the parts placed itself: the
    /// editor, which knows the program's files, does.
    pub fn show(&mut self, revision: u64, bom: bool, parts: GeopResult<Vec<PartsListLine>>) {
        self.parts = Some((revision, bom, parts.map_err(|e| e.to_string())));
        // Laid out with the parts it had: to be laid out again.
        *self.laid_out.get_mut() = None;
    }

    /// The tool in hand.
    pub fn tool(&self) -> Tool {
        self.tool
    }

    /// `args`' drawing of `part`, as downloaded — dated `date`, with
    /// `parts` its bill of materials if `args` asks for one — from the
    /// views as projected for the viewport (see [`crate::compose`]).
    pub fn compose<S: Scalar>(
        &self,
        part: &Part<S>,
        args: &DrawingArgs,
        date: &str,
        parts: &[PartsListLine],
    ) -> GeopResult<Sheet> {
        let projected = self.projected(part, args);
        let projected = (*projected).as_ref().map_err(|e| GeopError::new(e.clone()))?;
        annotated(part, args, &arrange(part, args, date, parts, projected)?)
    }

    /// `args`' sheet as drawn of `part` — laid out once, and kept while
    /// `part` and what the views depend on are the same. None until the
    /// editor has shown the sheet (see [`DrawingSession::show`]): a form
    /// listed among the steps lays out nothing.
    fn layout<S: Scalar>(
        &self,
        part: &Part<S>,
        args: &DrawingArgs,
    ) -> Option<Rc<Result<Layout, String>>> {
        let revision = part.revision();
        // Until the editor has given the parts for this part and these
        // arguments — on its way, after an edit — there is nothing to show.
        let (_, _, parts) = self
            .parts
            .as_ref()
            .filter(|(r, bom, _)| *r == revision && *bom == args.bom)?;
        let mut key = args.clone();
        key.annotations.clear();
        if let Some((r, a, laid)) = &*self.laid_out.borrow()
            && *r == revision
            && *a == key
        {
            return Some(laid.clone());
        }
        let laid = Rc::new(match (parts, &*self.projected(part, args)) {
            (Ok(parts), Ok(projected)) => arrange(part, args, "", parts, projected)
                .map_err(|e| e.root_message().to_string()),
            (Err(e), _) => Err(format!("the bill of materials: {e}")),
            (_, Err(e)) => Err(e.clone()),
        });
        *self.laid_out.borrow_mut() = Some((revision, key, laid.clone()));
        Some(laid)
    }

    /// `args`' views of `part`, projected — once, and kept while `part`
    /// and what they depend on are the same.
    fn projected<S: Scalar>(
        &self,
        part: &Part<S>,
        args: &DrawingArgs,
    ) -> Rc<Result<Projected<S>, String>> {
        let revision = part.revision();
        let key = Projected::<S>::depends_on(args);
        if let Some((r, a, projected)) = &*self.projected.borrow()
            && *r == revision
            && *a == key
            && let Ok(projected) = projected.clone().downcast::<Result<Projected<S>, String>>()
        {
            return projected;
        }
        let projected = Rc::new(project(part, args).map_err(|e| e.root_message().to_string()));
        *self.projected.borrow_mut() = Some((revision, key, projected.clone()));
        projected
    }
}

fn label(kind: ViewKind) -> &'static str {
    match kind {
        ViewKind::Front => "Front",
        ViewKind::Top => "Top",
        ViewKind::Right => "Right",
        ViewKind::Left => "Left",
        ViewKind::Bottom => "Bottom",
        ViewKind::Back => "Back",
        ViewKind::Iso => "Isometric",
    }
}

/// The plane the sheet lies in: the world's `xy` plane, a millimetre of
/// paper to a unit.
fn paper<S: Scalar>() -> CoordinateSystem<S> {
    CoordinateSystem::world_at(Vector3::zero())
}

/// A point of the sheet where the viewport has it.
fn world<S: Scalar>(p: P) -> Vector3<S> {
    Vector3::from_array([S::from_f64(p[0]), S::from_f64(p[1]), S::ZERO])
}

/// Where `pointer` points on the sheet.
fn on_sheet<S: Scalar>(pointer: &Pointer<S>) -> Option<P> {
    let plane = paper::<S>();
    let (_, at) = pointer.ray.intersect_plane(plane.origin(), plane.w())?;
    Some([at[0].to_f64(), at[1].to_f64()])
}

/// The key of the visual of the `i`-th annotation's value, which selects
/// and drags it.
fn annotation_key(i: usize) -> String {
    format!("annotation{i}")
}

/// The annotation a visual's key belongs to.
fn annotation_of(key: &str) -> Option<usize> {
    key.strip_prefix("annotation")?
        .split('/')
        .next()?
        .parse()
        .ok()
}

/// What a candidate is, for what a tool can make of it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Point,
    Line,
    Circle,
    Other,
}

fn kind(c: &Candidate) -> Kind {
    match &c.what {
        Pickable::Point { .. } => Kind::Point,
        Pickable::Edge { shape, .. } => match shape {
            EdgeShape::Line { .. } => Kind::Line,
            EdgeShape::Circle { .. } => Kind::Circle,
            EdgeShape::Other => Kind::Other,
        },
    }
}

/// The picks a tool takes, complete: every pick in one view — one that
/// shows lengths as they are, but for a note.
fn patterns(tool: Tool) -> &'static [&'static [Kind]] {
    use Kind::*;
    match tool {
        Tool::Select => &[],
        Tool::Dimension => &[&[Point, Point], &[Line], &[Line, Line], &[Circle]],
        Tool::Horizontal | Tool::Vertical => &[&[Point, Point], &[Line]],
        Tool::Radius | Tool::Diameter | Tool::CenterMark => &[&[Circle]],
        Tool::Angle => &[&[Line, Line]],
        Tool::Note => &[&[Point], &[Line], &[Circle], &[Other]],
    }
}

/// Whether `picks` are all `tool` needs, or can still grow into it.
fn fits(tool: Tool, picks: &[&Candidate]) -> bool {
    let Some(first) = picks.first() else {
        return true;
    };
    if picks.iter().any(|c| c.view != first.view)
        || (tool != Tool::Note && !first.view.orthographic())
    {
        return false;
    }
    let kinds: Vec<Kind> = picks.iter().map(|c| kind(c)).collect();
    patterns(tool)
        .iter()
        .any(|p| p.len() >= kinds.len() && p[..kinds.len()] == kinds[..])
}

/// What `tool` makes of `picks`, if they are all it needs — its value not
/// yet placed.
fn build(tool: Tool, picks: &[Candidate]) -> Option<Annotation> {
    let refs: Vec<&Candidate> = picks.iter().collect();
    let view = picks.first()?.view;
    if !fits(tool, &refs) || !patterns(tool).iter().any(|p| p.len() == picks.len()) {
        return None;
    }
    let point = |c: &Candidate| match &c.what {
        Pickable::Point { target, .. } => Some(target.clone()),
        _ => None,
    };
    let edge = |c: &Candidate| match &c.what {
        Pickable::Edge { name, shape, .. } => Some((name.clone(), shape.clone())),
        _ => None,
    };
    let distance = |along: Along| -> Option<Annotation> {
        let (from, to) = match picks {
            [a, b] => (point(a)?, point(b)?),
            [line] => match edge(line)?.1 {
                EdgeShape::Line { start, end } => {
                    (Target::Vertex { name: start }, Target::Vertex { name: end })
                }
                _ => return None,
            },
            _ => return None,
        };
        Some(Annotation::Distance {
            view,
            from,
            to,
            along,
            label: [0.0, 0.0],
        })
    };
    let radius = |diameter: bool| {
        Some(Annotation::Radius {
            view,
            edge: edge(&picks[0])?.0,
            diameter,
            label: [0.0, 0.0],
        })
    };
    match tool {
        Tool::Select => None,
        Tool::Dimension => match picks {
            [a, b] if kind(a) == Kind::Line => Some(Annotation::Angle {
                view,
                a: edge(a)?.0,
                b: edge(b)?.0,
                label: [0.0, 0.0],
            }),
            [c] if kind(c) == Kind::Circle => {
                radius(matches!(edge(c)?.1, EdgeShape::Circle { full: true }))
            }
            _ => distance(Along::Aligned),
        },
        Tool::Horizontal => distance(Along::Horizontal),
        Tool::Vertical => distance(Along::Vertical),
        Tool::Radius => radius(false),
        Tool::Diameter => radius(true),
        Tool::Angle => Some(Annotation::Angle {
            view,
            a: edge(&picks[0])?.0,
            b: edge(&picks[1])?.0,
            label: [0.0, 0.0],
        }),
        Tool::CenterMark => Some(Annotation::CenterMark {
            view,
            edge: edge(&picks[0])?.0,
        }),
        Tool::Note => {
            let target = match &picks[0].what {
                Pickable::Point { target, .. } => target.clone(),
                Pickable::Edge { name, .. } => Target::Edge { name: name.clone() },
            };
            Some(Annotation::Note {
                text: String::new(),
                leader: Some(Leader { view, target }),
                label: [0.0, 0.0],
            })
        }
    }
}

/// `annotation` with its value placed at `at` on the sheet, measured in
/// `part` as `layout` draws it — or why it cannot be drawn.
fn placed_at<S: Scalar>(
    mut annotation: Annotation,
    part: &Part<S>,
    layout: &Layout,
    at: P,
) -> GeopResult<(Annotation, Drawn)> {
    let anchor = annotation.drawn(part, layout)?.anchor;
    if let Some(label) = annotation.label_mut() {
        *label = sub(at, anchor);
    }
    let drawn = annotation.drawn(part, layout)?;
    Ok((annotation, drawn))
}

/// How a candidate is drawn, and hit.
fn candidate_shape<S: Scalar>(c: &Candidate) -> Shape<S> {
    match &c.what {
        Pickable::Point { at, .. } => Shape::Point { at: world(*at) },
        Pickable::Edge { points, .. } => Shape::Polyline {
            points: points.iter().map(|&p| world(p)).collect(),
        },
    }
}

/// The candidate of `layout` under `pointer` that `tool` can take next,
/// with `picks` picked: one that adds to them, or starts afresh.
fn under<'l, S: Scalar>(
    layout: &'l Layout,
    pointer: &Pointer<S>,
    tool: Tool,
    picks: &[Candidate],
) -> Option<&'l Candidate> {
    let takes = |c: &Candidate| {
        let mut more: Vec<&Candidate> = picks.iter().collect();
        more.push(c);
        fits(tool, &more) || fits(tool, &[c])
    };
    let usable: Vec<&Candidate> = layout.candidates.iter().filter(|c| takes(c)).collect();
    let visuals: Vec<Visual<S>> = usable
        .iter()
        .enumerate()
        .map(|(i, c)| Visual::new(i.to_string(), candidate_shape(c), Style::Free))
        .collect();
    let hit = hit_visuals(&visuals, pointer, None, |_| true)?;
    usable.get(hit.visual.key.parse::<usize>().ok()?).copied()
}

/// The style a layer's strokes are shown in: the part's lines as a
/// sketch's fixed geometry, hidden lines as construction, the rest as
/// guides.
fn style(layer: Layer) -> Style {
    match layer {
        Layer::Visible | Layer::Border | Layer::Cut | Layer::Thread => Style::Fixed,
        Layer::Hidden => Style::Construction,
        Layer::Center | Layer::Dimension | Layer::Hatch | Layer::Bend => Style::Guide,
    }
}

/// Points every circle a stroke draws is shown through.
const CIRCLE_POINTS: usize = 48;

/// The sheet's strokes and texts as visuals, under keys starting `key`.
fn sheet_visuals<S: Scalar>(sheet: &Sheet, key: &str, out: &mut Vec<Visual<S>>) {
    use crate::sheet::Shape as Stroke;
    let arc = |center: P, radius: f64, start: f64, sweep: f64| -> Vec<Vector3<S>> {
        let n = ((CIRCLE_POINTS as f64 * sweep / std::f64::consts::TAU).ceil() as usize).max(2);
        (0..=n)
            .map(|k| {
                let a = start + sweep * k as f64 / n as f64;
                world([center[0] + radius * a.cos(), center[1] + radius * a.sin()])
            })
            .collect()
    };
    for (i, stroke) in sheet.strokes.iter().enumerate() {
        let shape = match &stroke.shape {
            Stroke::Line(a, b) => Shape::Polyline {
                points: vec![world(*a), world(*b)],
            },
            Stroke::Polyline(points) => Shape::Polyline {
                points: points.iter().map(|&p| world(p)).collect(),
            },
            Stroke::Circle { center, radius } => Shape::Polyline {
                points: arc(*center, *radius, 0.0, std::f64::consts::TAU),
            },
            Stroke::Arc {
                center,
                radius,
                start,
                end,
            } => Shape::Polyline {
                points: arc(
                    *center,
                    *radius,
                    *start,
                    (end - start).rem_euclid(std::f64::consts::TAU),
                ),
            },
            Stroke::Filled(points) => Shape::Triangles {
                triangles: (1..points.len().saturating_sub(1))
                    .map(|k| [points[0], points[k], points[k + 1]].map(world))
                    .collect(),
            },
        };
        out.push(Visual::new(
            format!("{key}/{i}"),
            shape,
            style(stroke.layer),
        ));
    }
    for (i, text) in sheet.labels.iter().enumerate() {
        // Shown centred where it is written: an average glyph is about
        // 0.6 of the text's height wide.
        let half = 0.3 * text.height * text.text.chars().count() as f64;
        let along = match text.anchor {
            Anchor::Start => half,
            Anchor::Middle => 0.0,
            Anchor::End => -half,
        };
        let (s, c) = text.angle.to_radians().sin_cos();
        let up = text.height / 2.0;
        let at = [
            text.at[0] + along * c - up * s,
            text.at[1] + along * s + up * c,
        ];
        // A dimension's value as a sketch shows one; the title block's,
        // the parts list's and the views' captions as plain text.
        let style = match text.layer {
            Layer::Dimension => Style::Fixed,
            _ => Style::Paper,
        };
        out.push(Visual::new(
            format!("{key}/text{i}"),
            Shape::Label {
                at: world(at),
                text: text.text.clone(),
                offset: Vector3::zero(),
            },
            style,
        ));
    }
}

/// An annotation as visuals: its lines and arrowheads under `key/…`, in
/// `style`, and its value under `key`, in `value` — or, for a centre mark,
/// which has none, its first line. What is under `key` is what selects it
/// and drags it.
fn annotation_visuals<S: Scalar>(
    drawn: &Drawn,
    key: &str,
    style: Style,
    value: Style,
) -> Vec<Visual<S>> {
    let mut out = Vec::new();
    for (i, (_, line)) in drawn.lines.iter().enumerate() {
        let shape = Shape::Polyline {
            points: line.iter().map(|&p| world(p)).collect(),
        };
        out.push(match (i, &drawn.text) {
            (0, None) => Visual::new(key, shape, style),
            _ => Visual::new(format!("{key}/{i}"), shape, style),
        });
    }
    for (i, arrow) in drawn.arrows.iter().enumerate() {
        out.push(Visual::new(
            format!("{key}/arrow{i}"),
            Shape::Triangles {
                triangles: vec![arrow.map(world)],
            },
            style,
        ));
    }
    if let Some(text) = &drawn.text {
        out.push(Visual::new(
            key,
            Shape::Label {
                at: world(drawn.label),
                text: text.text.clone(),
                offset: Vector3::zero(),
            },
            value,
        ));
    }
    out
}

/// What the tool in hand asks for next.
fn hint(s: &DrawingSession) -> String {
    let complete = build(s.tool, &s.picks).is_some();
    match s.tool {
        Tool::Select => "Take up a tool to add dimensions, notes and centre marks · drag a value \
                         to move it · Delete removes what is selected"
            .into(),
        Tool::Note if s.picks.is_empty() => {
            "Note: click what it points at, or empty paper to put it there · Esc to stop".into()
        }
        Tool::Note => "Click where the note's text goes · Esc to start again".into(),
        Tool::CenterMark => "Click a circle the view sees round · Esc to stop".into(),
        _ if complete => "Click where its value goes · Esc to start again".into(),
        tool => format!("{}: click it in a view · Esc to stop", tool.label()),
    }
}

/// The fields and visuals of annotating `layout`'s sheet of `part`: the
/// tools, what they ask for, the annotations — each with its value, or why
/// it cannot be drawn — and the sheet itself in the viewport, with what
/// is picked, what a click would pick, and the annotation being placed.
fn annotate<'a, S: Scalar>(
    f: &mut Form<'a, S, DrawingArgs, DrawingSession>,
    part: &Part<S>,
    args: &DrawingArgs,
    s: &DrawingSession,
    selection: &[String],
    layout: &Layout,
) {
    let sheet = &layout.sheet;
    f.focus = Some(paper());
    f.sheet = Some(Extent {
        center: world([sheet.width / 2.0, sheet.height / 2.0]),
        size: S::from_f64(sheet.width.hypot(sheet.height)),
    });
    f.tool = match s.tool {
        Tool::Select => InHand::Nothing,
        _ => InHand::Clicks,
    };
    sheet_visuals(sheet, "sheet", &mut f.visuals);

    f.heading("annotate_heading", "Annotate");
    let actions = TOOLS
        .iter()
        .map(|&(tool, name, label)| {
            Action::new(name, label)
                .icon(name)
                .active(s.tool == tool)
        })
        .collect();
    f.actions("tools", actions, |edit, name| {
        let s = edit.session;
        let tool = Tool::by_name(name).unwrap_or_default();
        s.tool = if s.tool == tool { Tool::Select } else { tool };
        s.picks.clear();
        s.hover = None;
        s.refused = None;
    });
    f.text("tool_hint", hint(s), Tone::Hint);
    if let Some(why) = &s.refused {
        f.text("refused", format!("Not added: {why}"), Tone::Error);
    }
    if s.tool == Tool::Note {
        let mut item = ListItem::new("note_text", "Text");
        item.text = Some(s.note.clone());
        f.on("note_text", |edit, value| {
            if let Value::Text(text) = value {
                edit.session.note = text;
            }
        });
        f.list("note", vec![item], "");
    }

    let mut items = Vec::new();
    for (i, annotation) in args.annotations.iter().enumerate() {
        let key = annotation_key(i);
        let mut item = ListItem::new(format!("annotation:{i}"), annotation.kind());
        item.removable = true;
        item.selected = selection.contains(&key);
        match annotation.drawn(part, layout) {
            Ok(drawn) => {
                let value = drawn.text.as_ref().map(|t| t.text.clone());
                let view = annotation.view().map(|v| v.name().to_string());
                item.detail = match (value, view) {
                    (Some(value), Some(view)) => Some(format!("{value} · {view}")),
                    (value, view) => value.or(view),
                };
                let style = if item.selected {
                    Style::Selected
                } else {
                    Style::Guide
                };
                // Its value — or a centre mark's line — is selected and
                // dragged, unless a tool is in hand.
                f.visuals.extend(
                    annotation_visuals(&drawn, &key, style, Style::Fixed)
                        .into_iter()
                        .map(|v| match v.key == key {
                            true => v.selectable().draggable(),
                            false => v,
                        }),
                );
            }
            Err(e) => {
                item.tone = Tone::Error;
                item.detail = Some(e.root_message().to_string());
            }
        }
        if let Annotation::Note { text, .. } = annotation {
            item.text = Some(text.clone());
        }
        items.push(item);
        f.on(format!("annotation:{i}"), move |edit, value| match value {
            Value::Remove if i < edit.args.annotations.len() => {
                edit.args.annotations.remove(i);
                edit.selection.clear();
                edit.session.drag = None;
            }
            Value::Text(text) => {
                if let Some(Annotation::Note { text: t, .. }) = edit.args.annotations.get_mut(i) {
                    *t = text;
                }
            }
            Value::Press => *edit.selection = vec![annotation_key(i)],
            _ => {}
        });
    }
    f.list(
        "annotations",
        items,
        "None yet: the views show their overall sizes.",
    );

    // What is picked, what a click would pick, and what it would place.
    for (k, pick) in s.picks.iter().enumerate() {
        f.visuals.push(Visual::new(
            format!("picked{k}"),
            candidate_shape(pick),
            Style::Selected,
        ));
    }
    if let Some(hover) = &s.hover {
        f.visuals
            .push(Visual::new("hover", candidate_shape(hover), Style::Hover));
    }
    if let (Some(annotation), Some(cursor)) = (build(s.tool, &s.picks), s.cursor)
        && let Ok((_, drawn)) = placed_at(annotation, part, layout, cursor)
    {
        f.visuals.extend(annotation_visuals(
            &drawn,
            "placing",
            Style::Draft,
            Style::Draft,
        ));
    }
}

impl DrawingSession {
    /// A click on the sheet at `at`, with the tool in hand: what is under
    /// the pointer — `hit` — picked for it, or, once it has all it needs,
    /// what it makes added to `args`, its value at `at`.
    fn click<S: Scalar>(
        &mut self,
        args: &mut DrawingArgs,
        part: &Part<S>,
        layout: &Layout,
        at: P,
        hit: Option<Candidate>,
    ) {
        self.refused = None;
        let mut more: Vec<&Candidate> = self.picks.iter().collect();
        let grows = hit.as_ref().is_some_and(|h| {
            more.push(h);
            fits(self.tool, &more) && !(self.tool == Tool::Note && !self.picks.is_empty())
        });
        if grows && let Some(hit) = &hit {
            self.picks.push(hit.clone());
            // A centre mark has nothing to place: it is added at once.
            if self.tool == Tool::CenterMark {
                self.add(args, part, layout, at);
            }
            return;
        }
        if self.tool == Tool::Note && self.picks.is_empty() {
            // Empty paper: the note goes there, pointing at nothing.
            args.annotations.push(Annotation::Note {
                text: self.note.clone(),
                leader: None,
                label: at,
            });
            return;
        }
        if build(self.tool, &self.picks).is_some() {
            self.add(args, part, layout, at);
            return;
        }
        // What was clicked cannot be part of it: it starts afresh.
        self.picks = hit
            .filter(|h| fits(self.tool, &[h]))
            .into_iter()
            .collect();
    }

    /// Adds what the tool makes of the picks, its value at `at`.
    fn add<S: Scalar>(&mut self, args: &mut DrawingArgs, part: &Part<S>, layout: &Layout, at: P) {
        let Some(mut annotation) = build(self.tool, &self.picks) else {
            return;
        };
        if let Annotation::Note { text, .. } = &mut annotation {
            *text = self.note.clone();
        }
        self.picks.clear();
        match placed_at(annotation, part, layout, at) {
            Ok((annotation, _)) => args.annotations.push(annotation),
            Err(e) => self.refused = Some(e.root_message().to_string()),
        }
    }
}

/// Whether the drawing `args` describe has the view `view`.
fn shows(args: &DrawingArgs, view: DrawnView) -> bool {
    match view {
        DrawnView::View(kind) => args.views.contains(&kind),
        DrawnView::Section => args.section.is_some(),
    }
}

/// Checks that `target` names something in `part`.
fn resolve_target<S: Scalar>(part: &Part<S>, target: &Target) -> GeopResult<()> {
    match target {
        Target::Vertex { name } => placed_vertex(part, name).map(|_| ()),
        Target::Edge { name } => placed_edge(part, name).map(|_| ()),
        Target::Center { edge } => circular(part, edge),
    }
}

/// Checks that `edge` is a circular edge of `part`.
fn circular<S: Scalar>(part: &Part<S>, edge: &str) -> GeopResult<()> {
    match placed_edge(part, edge)?.as_arc()? {
        Some(_) => Ok(()),
        None => Err(GeopError::new(format!(
            "the edge {edge} is not circular: only a circle has a radius or a centre"
        ))),
    }
}

/// Checks that `edge` is a straight edge of `part`.
fn straight<S: Scalar>(part: &Part<S>, edge: &str) -> GeopResult<()> {
    match placed_edge(part, edge)?.as_line()? {
        Some(_) => Ok(()),
        None => Err(GeopError::new(format!(
            "the edge {edge} is not straight: only straight edges make an angle"
        ))),
    }
}

impl Operation for Drawing {
    type Args = DrawingArgs;
    type Session = DrawingSession;

    /// Front, top, right and isometric views, third angle, on A3.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> DrawingArgs {
        DrawingArgs::default()
    }

    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &DrawingArgs,
        session: &DrawingSession,
        selection: &[String],
    ) -> Form<'a, S, DrawingArgs, DrawingSession> {
        let mut f = Form::<S, DrawingArgs, DrawingSession>::new();
        f.download(
            "download",
            vec![Choice::new("svg", "SVG"), Choice::new("dxf", "DXF")],
        );
        if let Some(laid) = session.layout(context.before, args) {
            match &*laid {
                Ok(layout) => annotate(&mut f, context.before, args, session, selection, layout),
                Err(e) => {
                    f.text(
                        "sheet_error",
                        format!("The sheet cannot be laid out: {e}"),
                        Tone::Error,
                    );
                }
            }
        }

        f.heading("views_heading", "Views");
        for kind in ViewKind::ALL {
            f.checkbox(
                &format!("view:{}", kind.name()),
                label(kind),
                args.views.contains(&kind),
                move |args: &mut DrawingArgs, on| {
                    args.views.retain(|k| *k != kind);
                    if on {
                        args.views.push(kind);
                        args.views
                            .sort_by_key(|k| ViewKind::ALL.iter().position(|a| a == k));
                    }
                },
            );
        }
        f.select(
            "projection",
            "projection",
            match args.projection {
                Projection::ThirdAngle => "third_angle",
                Projection::FirstAngle => "first_angle",
            },
            vec![
                Choice::new("third_angle", "Third angle"),
                Choice::new("first_angle", "First angle"),
            ],
            false,
            |args, value| {
                args.projection = match value {
                    "first_angle" => Projection::FirstAngle,
                    _ => Projection::ThirdAngle,
                }
            },
        );
        f.checkbox(
            "hidden_lines",
            "Hidden lines",
            args.hidden_lines,
            |args, on| args.hidden_lines = on,
        );
        f.checkbox(
            "tangent_edges",
            "Tangent edges",
            args.tangent_edges,
            |args, on| args.tangent_edges = on,
        );
        f.reference(
            "section",
            "section plane",
            args.section.iter().cloned().collect(),
            &[Role::Plane],
            None,
            false,
            |edit, picked| edit.args.section = picked.into_iter().next(),
        );
        f.optional("section");

        f.heading("sheet_heading", "Sheet");
        f.select(
            "sheet",
            "paper",
            args.sheet.name(),
            SheetSize::ALL
                .iter()
                .map(|s| Choice::new(s.name(), s.name().to_uppercase()))
                .collect(),
            false,
            |args, value| {
                if let Some(s) = SheetSize::ALL.into_iter().find(|s| s.name() == value) {
                    args.sheet = s;
                }
            },
        );
        let mut scales = vec![Choice::new("fit", "To fit")];
        scales.extend(
            SCALES
                .iter()
                .map(|&s| Choice::new(scale_label(s), scale_label(s))),
        );
        f.select(
            "scale",
            "scale",
            args.scale.map(scale_label).unwrap_or_else(|| "fit".into()),
            scales,
            false,
            |args, value| {
                args.scale = SCALES.iter().copied().find(|&s| scale_label(s) == value);
            },
        );
        f.checkbox("bom", "Bill of materials", args.bom, |args, on| {
            args.bom = on
        });
        let mut title = Vec::new();
        for (key, caption, value) in [
            ("title:name", "Part name", &args.name),
            ("title:material", "Material", &args.material),
        ] {
            f.on(key, move |edit, value| {
                if let Value::Text(text) = value {
                    match key {
                        "title:name" => edit.args.name = text,
                        _ => edit.args.material = text,
                    }
                }
            });
            let mut item = ListItem::new(key, caption);
            item.text = Some(value.clone());
            title.push(item);
        }
        f.list("title", title, "");
        f
    }

    /// The pointer on the sheet: a tool picks what the views show and puts
    /// down what it makes; a value is dragged where it goes; Escape lets
    /// go of the picks, then of the tool; Delete removes the annotations
    /// selected.
    fn event<S: Scalar>(
        &self,
        context: Context<'_, S>,
        edit: Edit<'_, DrawingArgs, DrawingSession>,
        event: &CanvasEvent<S>,
    ) {
        let Edit {
            args,
            session: s,
            selection,
            ..
        } = edit;
        let Some(laid) = s.layout(context.before, args) else {
            return;
        };
        let Ok(layout) = &*laid else {
            return;
        };
        let part = context.before;
        match event {
            CanvasEvent::Hover { pointer, .. } => {
                s.cursor = on_sheet(pointer);
                s.hover = match s.tool {
                    Tool::Select => None,
                    tool => under(layout, pointer, tool, &s.picks).cloned(),
                };
            }
            CanvasEvent::Leave => {
                s.cursor = None;
                s.hover = None;
            }
            CanvasEvent::Click {
                pointer,
                button: geop_ops::ui::Button::Primary,
                double: false,
                ..
            } if s.tool != Tool::Select => {
                let Some(at) = on_sheet(pointer) else {
                    return;
                };
                let hit = under(layout, pointer, s.tool, &s.picks).cloned();
                s.click(args, part, layout, at, hit);
                s.hover = under(layout, pointer, s.tool, &s.picks).cloned();
            }
            CanvasEvent::Click {
                button: geop_ops::ui::Button::Secondary,
                ..
            } => s.picks.clear(),
            CanvasEvent::Move {
                key,
                from,
                to,
                done,
                ..
            } => {
                let Some(i) = annotation_of(key).filter(|_| !key.contains('/')) else {
                    return;
                };
                let Some(annotation) = args.annotations.get_mut(i) else {
                    return;
                };
                let grabbed = match s.drag {
                    Some((j, grabbed)) if j == i => grabbed,
                    _ => {
                        let grabbed = annotation.label_mut().map_or([0.0, 0.0], |l| *l);
                        s.drag = Some((i, grabbed));
                        grabbed
                    }
                };
                let moved = [
                    to[0].to_f64() - from[0].to_f64(),
                    to[1].to_f64() - from[1].to_f64(),
                ];
                if let Some(label) = annotation.label_mut() {
                    *label = add(grabbed, moved);
                }
                if *done {
                    s.drag = None;
                }
            }
            CanvasEvent::Key { key } if key == "Escape" => {
                s.refused = None;
                if s.picks.is_empty() {
                    s.tool = Tool::Select;
                }
                s.picks.clear();
                s.hover = None;
            }
            CanvasEvent::Key { key } if key == "Delete" || key == "Backspace" => {
                let mut gone: Vec<usize> =
                    selection.iter().filter_map(|k| annotation_of(k)).collect();
                gone.sort_unstable();
                gone.dedup();
                for i in gone.into_iter().rev() {
                    if i < args.annotations.len() {
                        args.annotations.remove(i);
                    }
                }
                selection.clear();
            }
            _ => {}
        }
    }

    /// Checks that what the drawing refers to is there — its section
    /// plane, and what each annotation names, in a view the drawing has;
    /// it changes nothing. Whether each view sees what it is asked to
    /// measure is checked when the drawing is laid out.
    fn apply<S: Scalar>(
        &self,
        part: Part<S>,
        operation_id: &str,
        args: &DrawingArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("drawing({operation_id})");
        if args.views.is_empty() && args.section.is_none() {
            return Err(GeopError::new("a drawing needs a view")).with_context(ctx);
        }
        if let Some(plane) = &args.section {
            plane.resolve_plane(&part).with_context(ctx)?;
        }
        if let Some(scale) = args.scale
            && !(scale.is_finite() && scale > 0.0)
        {
            return Err(GeopError::new(format!(
                "the scale {scale} is not a positive number"
            )))
            .with_context(ctx);
        }
        for (index, annotation) in args.annotations.iter().enumerate() {
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "drawing({operation_id}) annotation {}: {}",
                    index + 1,
                    annotation.describe()
                ))
            };
            if let Some(view) = annotation.view()
                && !shows(args, view)
            {
                return Err(GeopError::new(format!(
                    "the drawing has no {view} view any more: show it again, or remove the {}",
                    annotation.kind().to_lowercase()
                )))
                .with_context(&ctx);
            }
            match annotation {
                Annotation::Distance { from, to, .. } => {
                    resolve_target(&part, from).with_context(&ctx)?;
                    resolve_target(&part, to).with_context(&ctx)?;
                }
                Annotation::Radius { edge, .. } | Annotation::CenterMark { edge, .. } => {
                    circular(&part, edge).with_context(&ctx)?;
                }
                Annotation::Angle { a, b, .. } => {
                    straight(&part, a).with_context(&ctx)?;
                    straight(&part, b).with_context(&ctx)?;
                }
                Annotation::Note { leader, .. } => {
                    if let Some(leader) = leader {
                        resolve_target(&part, &leader.target).with_context(&ctx)?;
                    }
                }
            }
        }
        Ok(part)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FRONT: DrawnView = DrawnView::View(ViewKind::Front);

    fn point(view: DrawnView, name: &str) -> Candidate {
        Candidate {
            view,
            what: Pickable::Point {
                target: Target::Vertex { name: name.into() },
                at: [0.0, 0.0],
            },
        }
    }

    fn edge(view: DrawnView, name: &str, shape: EdgeShape) -> Candidate {
        Candidate {
            view,
            what: Pickable::Edge {
                name: name.into(),
                points: Vec::new(),
                shape,
            },
        }
    }

    fn line(name: &str) -> Candidate {
        let shape = EdgeShape::Line {
            start: format!("{name}.start"),
            end: format!("{name}.end"),
        };
        edge(FRONT, name, shape)
    }

    /// The dimension tool makes what its picks call for: two points their
    /// distance, a line its length, two lines their angle, a circle its
    /// diameter and an arc its radius — and takes nothing from two views,
    /// or from the isometric view, which foreshortens what it shows.
    #[test]
    fn the_dimension_tool_makes_what_its_picks_call_for() {
        let made = |picks: &[Candidate]| build(Tool::Dimension, picks);
        assert!(matches!(
            made(&[point(FRONT, "a"), point(FRONT, "b")]),
            Some(Annotation::Distance {
                along: Along::Aligned,
                ..
            })
        ));
        let Some(Annotation::Distance { from, to, .. }) = made(&[line("l")]) else {
            panic!("a line's length");
        };
        assert_eq!(
            (from.to_string(), to.to_string()),
            ("l.start".into(), "l.end".into())
        );
        assert!(matches!(
            made(&[line("l"), line("m")]),
            Some(Annotation::Angle { .. })
        ));
        let round = |full| edge(FRONT, "c", EdgeShape::Circle { full });
        assert!(matches!(
            made(&[round(true)]),
            Some(Annotation::Radius { diameter: true, .. })
        ));
        assert!(matches!(
            made(&[round(false)]),
            Some(Annotation::Radius {
                diameter: false,
                ..
            })
        ));
        assert!(made(&[point(FRONT, "a")]).is_none(), "one point is not enough");
        let top = DrawnView::View(ViewKind::Top);
        assert!(!fits(Tool::Dimension, &[&point(FRONT, "a"), &point(top, "b")]));
        let iso = DrawnView::View(ViewKind::Iso);
        assert!(!fits(Tool::Dimension, &[&point(iso, "a")]));
        assert!(fits(Tool::Note, &[&point(iso, "a")]), "a note points into any view");
        assert!(matches!(
            build(Tool::Horizontal, &[line("l")]),
            Some(Annotation::Distance {
                along: Along::Horizontal,
                ..
            })
        ));
    }
}
