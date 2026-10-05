//! What a drawing shows ([`DrawingArgs`]), and composing it on a sheet
//! ([`compose`]): the views laid out in third- or first-angle projection,
//! each with its visible and hidden lines, centre marks on its circles and
//! its overall size dimensioned, the dimensions asked for, a section view
//! with its cut hatched, a border and a title block — and, if asked for,
//! the parts list of the parts placed in it above the title block, each
//! line ballooned where its part is drawn.

use geop_core_geometry::{
    intersection::curve_curve_overlaps_and_crossings,
    nurb_curve::{NurbCurve2D, NurbCurve3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_sketch::dimension::Measure;
use geop_core_topology::{FaceId, Model};
use geop_ops::{EntityRef, Part};
use geop_ops_inspect::bodies::resolve;
use serde::{Deserialize, Serialize};

use crate::{
    MAX_NODES,
    hidden_lines::{LineKind, ProjectedView, ViewOptions},
    min_subdivision_size,
    scene::{Body, Scene},
    section::{cut_faces, section_part},
    sheet::{Anchor, Layer, P, Shape, Sheet, lift, strokes_of},
    annotation::{Annotation, Candidate, Drawn, candidates},
    view::{DrawnView, ViewFrame, ViewKind},
};

/// Which side of the front view the other views go.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Projection {
    /// The top view above the front view, the right view to its right —
    /// each view where it is seen from (ASME).
    #[default]
    ThirdAngle,
    /// The top view below the front view, the right view to its left —
    /// each view where it is projected onto (ISO).
    FirstAngle,
}

impl Projection {
    pub fn label(self) -> &'static str {
        match self {
            Projection::ThirdAngle => "THIRD ANGLE",
            Projection::FirstAngle => "FIRST ANGLE",
        }
    }
}

/// The paper, in landscape.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SheetSize {
    A4,
    #[default]
    A3,
    A2,
    A1,
    A0,
}

impl SheetSize {
    pub const ALL: [SheetSize; 5] = [
        SheetSize::A4,
        SheetSize::A3,
        SheetSize::A2,
        SheetSize::A1,
        SheetSize::A0,
    ];

    pub fn name(self) -> &'static str {
        match self {
            SheetSize::A4 => "a4",
            SheetSize::A3 => "a3",
            SheetSize::A2 => "a2",
            SheetSize::A1 => "a1",
            SheetSize::A0 => "a0",
        }
    }

    /// Width and height in millimetres.
    pub fn size(self) -> (f64, f64) {
        match self {
            SheetSize::A4 => (297.0, 210.0),
            SheetSize::A3 => (420.0, 297.0),
            SheetSize::A2 => (594.0, 420.0),
            SheetSize::A1 => (841.0, 594.0),
            SheetSize::A0 => (1189.0, 841.0),
        }
    }
}

/// What a drawing of a part shows.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrawingArgs {
    #[serde(default = "default_views")]
    pub views: Vec<ViewKind>,
    #[serde(default)]
    pub projection: Projection,
    #[serde(default)]
    pub sheet: SheetSize,
    /// Paper length per model length; the largest standard scale that fits
    /// the sheet if not given.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scale: Option<f64>,
    #[serde(default)]
    pub tangent_edges: bool,
    #[serde(default = "yes")]
    pub hidden_lines: bool,
    /// The plane of a section view, `A-A`: the part cut there, seen from
    /// the side its normal points to, the cut hatched.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub section: Option<EntityRef>,
    /// The dimensions, notes and centre marks added on the sheet, each in
    /// one of its views (see [`Annotation`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub annotations: Vec<Annotation>,
    /// The part's name, for the title block.
    #[serde(default)]
    pub name: String,
    /// What it is made of, for the title block.
    #[serde(default)]
    pub material: String,
    /// A bill of materials of the parts placed in it, as a table above the
    /// title block: each part's item number, quantity, name, designation
    /// and material (see [`PartsListLine`]) — and, with parts placed, a
    /// balloon per line with its item number, pointing at its part.
    #[serde(default)]
    pub bom: bool,
}

/// A line of a drawing's bill of materials: a kind of part placed — its
/// item number, how many there are, what it is called, what it is ordered
/// as (`ISO 4762 M4x12`, empty for a part that is made) and made of — and
/// where it is placed, as the entities of placed parts are named: `screw`,
/// `arm/screw` for one placed in the part placed as `arm`, `""` for the
/// part drawn. Who lists the parts placed is the caller's: the drawing lays
/// them out, and balloons them where they are placed.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PartsListLine {
    pub item: String,
    pub quantity: u64,
    pub name: String,
    pub designation: String,
    pub material: String,
    pub placements: Vec<String>,
}

fn default_views() -> Vec<ViewKind> {
    vec![
        ViewKind::Front,
        ViewKind::Top,
        ViewKind::Right,
        ViewKind::Iso,
    ]
}

fn yes() -> bool {
    true
}

impl Default for DrawingArgs {
    fn default() -> Self {
        DrawingArgs {
            views: default_views(),
            projection: Projection::default(),
            sheet: SheetSize::default(),
            scale: None,
            tangent_edges: false,
            hidden_lines: true,
            section: None,
            annotations: Vec::new(),
            name: String::new(),
            material: String::new(),
            bom: false,
        }
    }
}

/// The scales a drawing is made at, largest first.
pub const SCALES: [f64; 15] = [
    50.0, 20.0, 10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001,
];
/// The margin between the paper's edge and its border, in millimetres.
const MARGIN: f64 = 10.0;
/// The space around each view, for its dimensions, in millimetres.
const GAP: f64 = 28.0;
/// The title block's size, in millimetres.
const TITLE_WIDTH: f64 = 180.0;
const TITLE_HEIGHT: f64 = 40.0;
/// Text heights, in millimetres.
pub(crate) const TEXT: f64 = 3.5;
const TITLE_TEXT: f64 = 7.0;
/// Arrowheads, in millimetres.
pub(crate) const ARROW_LENGTH: f64 = 3.0;
pub(crate) const ARROW_WIDTH: f64 = 1.0;
/// How far dimension lines stand off what they measure, in millimetres.
pub(crate) const DIMENSION_OFFSET: f64 = 10.0;
/// How far centre lines reach past their circle, in millimetres.
pub(crate) const CENTER_OVERSHOOT: f64 = 3.0;
/// Hatch line spacing, in millimetres.
const HATCH_SPACING: f64 = 3.0;
/// The bill of materials' rows, and its columns' captions and widths — as
/// wide, together, as the title block it stands on — in millimetres.
const LIST_ROW: f64 = 6.0;
const LIST_COLUMNS: [(&str, f64); 5] = [
    ("ITEM", 14.0),
    ("QTY", 12.0),
    ("NAME", 54.0),
    ("DESIGNATION", 60.0),
    ("MATERIAL", 40.0),
];
const LIST_TEXT: f64 = 2.5;
/// Item-number balloons: their radius, how far they stand off the box of
/// the view they go around — past its overall dimensions — and the paper
/// that view needs around it for them; the dot their leaders end in.
const BALLOON_RADIUS: f64 = 4.0;
const BALLOON_OFFSET: f64 = 22.0;
const BALLOON_MARGIN: f64 = BALLOON_OFFSET + BALLOON_RADIUS;
const LEADER_DOT: f64 = 0.5;

/// A scale as written: `1:2`, `5:1`.
pub fn scale_label(scale: f64) -> String {
    if scale >= 1.0 {
        format!("{}:1", length_label(scale))
    } else {
        format!("1:{}", length_label(1.0 / scale))
    }
}

/// A length as written on a drawing: to a hundredth, without trailing
/// zeros.
pub fn length_label(x: f64) -> String {
    let s = format!("{x:.2}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

/// A view placed on the sheet.
pub(crate) struct Placed<S: Scalar> {
    pub(crate) kind: DrawnView,
    pub(crate) view: ProjectedView<S>,
    /// Hatched regions of a section view, as boundary curves on the paper
    /// in model units, one list per cut face.
    hatched: Vec<Hatched<S>>,
    /// Its box on the paper in model units, `[x_min, y_min, x_max, y_max]`.
    pub(crate) extents: [f64; 4],
    /// Where its box's centre lands on the sheet.
    pub(crate) center: P,
    /// The paper it needs around its box, besides the gap between views,
    /// in millimetres: for balloons.
    margin: f64,
}

/// Its column and row in the layout grid, row 0 at the top: the front view
/// in the middle, the others around it as `projection` says.
fn cell(view: DrawnView, projection: Projection) -> (usize, usize) {
    let third = projection == Projection::ThirdAngle;
    match view {
        DrawnView::View(ViewKind::Front) => (1, 1),
        DrawnView::View(ViewKind::Top) => (1, if third { 0 } else { 2 }),
        DrawnView::View(ViewKind::Bottom) => (1, if third { 2 } else { 0 }),
        DrawnView::View(ViewKind::Right) => (if third { 2 } else { 0 }, 1),
        DrawnView::View(ViewKind::Left) => (if third { 0 } else { 2 }, 1),
        DrawnView::View(ViewKind::Back) => (3, 1),
        DrawnView::View(ViewKind::Iso) => (if third { 2 } else { 0 }, if third { 0 } else { 2 }),
        DrawnView::Section => (if third { 2 } else { 0 }, if third { 2 } else { 0 }),
    }
}

/// The faces a drawing of `part` shows: every face of its solids and
/// sheets, in a fixed order.
pub fn drawn_faces<S: Scalar>(model: &Model<S>) -> Vec<FaceId> {
    let mut faces: Vec<FaceId> = model.faces.keys().copied().collect();
    faces.sort_by_key(|f| f.0);
    faces
}

/// The part's drawing as `args` describe it, dated `date`, with `parts`
/// its bill of materials if `args` asks for one: its layout (see
/// [`layout`]), with its annotations drawn on it. One that does not resolve
/// — its view gone, an entity it names gone — is refused, naming it.
pub fn compose<S: Scalar>(
    part: &Part<S>,
    args: &DrawingArgs,
    date: &str,
    parts: &[PartsListLine],
) -> GeopResult<Sheet> {
    annotated(part, args, &layout(part, args, date, parts)?)
}

/// `layout`'s sheet, of `part`, with `args`' annotations drawn on it (see
/// [`compose`]).
pub fn annotated<S: Scalar>(
    part: &Part<S>,
    args: &DrawingArgs,
    layout: &Layout,
) -> GeopResult<Sheet> {
    let mut sheet = layout.sheet.clone();
    for (index, annotation) in args.annotations.iter().enumerate() {
        annotation
            .drawn(part, &layout)
            .map_err(|e| {
                e.with_context(format!(
                    "annotation {}: {}",
                    index + 1,
                    annotation.describe()
                ))
            })?
            .draw(&mut sheet);
    }
    Ok(sheet)
}

/// A drawing laid out on its sheet: the sheet with everything on it but
/// the annotations, where its views are placed on it, the scale they are
/// drawn at, and what can be picked in them to annotate (see
/// [`Candidate`]).
#[derive(Clone, Debug)]
pub struct Layout {
    pub sheet: Sheet,
    pub(crate) views: Vec<Placement>,
    pub scale: f64,
    pub candidates: Vec<Candidate>,
}

/// Where a view is placed on the sheet: the middle of its box on its
/// paper, in model units, lands on `center`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Placement {
    pub(crate) view: DrawnView,
    mid: P,
    center: P,
}

impl Placement {
    pub(crate) fn of<S: Scalar>(p: &Placed<S>) -> Self {
        Placement {
            view: p.kind,
            mid: [
                (p.extents[0] + p.extents[2]) / 2.0,
                (p.extents[1] + p.extents[3]) / 2.0,
            ],
            center: p.center,
        }
    }

    /// Where a point of its paper, in model units, lands on the sheet
    /// drawn at `scale`.
    pub(crate) fn place(&self, scale: f64, q: P) -> P {
        [
            self.center[0] + scale * (q[0] - self.mid[0]),
            self.center[1] + scale * (q[1] - self.mid[1]),
        ]
    }
}

impl Layout {
    /// The view `view` placed, or why there is none.
    pub(crate) fn placement(&self, view: DrawnView) -> GeopResult<&Placement> {
        self.views.iter().find(|p| p.view == view).ok_or_else(|| {
            GeopError::new(format!(
                "the drawing has no {view} view any more: show it again, or remove what is in it"
            ))
        })
    }
}

/// The part's drawing as `args` describe it, without its annotations (see
/// [`compose`]): its views projected (see [`project`]) and laid out (see
/// [`arrange`]), dated `date`, with `parts` its bill of materials if `args`
/// asks for one.
pub fn layout<S: Scalar>(
    part: &Part<S>,
    args: &DrawingArgs,
    date: &str,
    parts: &[PartsListLine],
) -> GeopResult<Layout> {
    arrange(part, args, date, parts, &project(part, args)?)
}

/// A drawing's views projected, each with its lines visible and hidden —
/// and the section's, with its cut faces to hatch: what laying a drawing
/// out costs. It depends on no more of the drawing's arguments than
/// [`Projected::depends_on`] keeps, so changing the paper, the scale, the title
/// block or the bill of materials projects nothing again.
#[derive(Clone)]
pub struct Projected<S: Scalar> {
    views: Vec<(DrawnView, ProjectedView<S>, Vec<Hatched<S>>)>,
}

impl<S: Scalar> Projected<S> {
    /// What of `args` projecting the views depends on: the views, the
    /// lines drawn, the section.
    pub fn depends_on(args: &DrawingArgs) -> DrawingArgs {
        DrawingArgs {
            views: args.views.clone(),
            hidden_lines: args.hidden_lines,
            tangent_edges: args.tangent_edges,
            section: args.section.clone(),
            ..DrawingArgs::default()
        }
    }
}

/// The views of `part` that `args` ask for, projected (see [`Projected`]):
/// its own solids and sheets, and every part placed in it, however deep,
/// where it is placed (see [`Scene`]).
pub fn project<S: Scalar>(part: &Part<S>, args: &DrawingArgs) -> GeopResult<Projected<S>> {
    let ctx = |e: GeopError| e.with_context("project(drawing)");
    let scene = Scene::of(part).with_context(&ctx)?;
    let options = ViewOptions {
        tangent_edges: args.tangent_edges,
        hidden_lines: args.hidden_lines,
    };
    let mut views = Vec::new();
    if scene.bodies.is_empty() {
        return Ok(Projected { views });
    }
    for &kind in &args.views {
        if views.iter().any(|(v, _, _)| *v == DrawnView::View(kind)) {
            continue;
        }
        let frame = kind.frame()?;
        let view = scene
            .project(&frame, &options)
            .map_err(|e| ctx(e.with_context(format!("the {} view", kind.name()))))?;
        views.push((DrawnView::View(kind), view, Vec::new()));
    }
    if let Some(plane) = &args.section {
        let (view, hatched) = section_view(part, &scene, plane, &options)
            .map_err(|e| ctx(e.with_context(format!("the section view on {}", plane.label()))))?;
        views.push((DrawnView::Section, view, hatched));
    }
    Ok(Projected { views })
}

/// The views `projected` of `part` laid out on the sheet `args` describe
/// (see [`layout`]), dated `date`, with `parts` its bill of materials if
/// `args` asks for one.
///
/// With a bill of materials, each line of it placed in the drawing gets a
/// balloon with its item number, its leader pointing at its part in one
/// view (see [`balloon_view`]). A bill too long for the sheet is refused.
pub fn arrange<S: Scalar>(
    part: &Part<S>,
    args: &DrawingArgs,
    date: &str,
    parts: &[PartsListLine],
    projected: &Projected<S>,
) -> GeopResult<Layout> {
    let ctx = |e: GeopError| e.with_context("arrange(drawing)");
    let (width, height) = args.sheet.size();
    let list_height = match args.bom {
        true => LIST_ROW * (parts.len() + 1) as f64,
        false => 0.0,
    };
    if list_height > height - 2.0 * MARGIN - TITLE_HEIGHT {
        return Err(GeopError::new(format!(
            "the bill of materials has {} lines, too many for an {} sheet: choose a larger one",
            parts.len(),
            args.sheet.name().to_uppercase()
        )));
    }
    let scene = Scene::of(part).with_context(&ctx)?;
    if scene.bodies.is_empty() && !args.bom {
        return Err(GeopError::new(
            "the part has nothing to draw: it has no faces",
        ));
    }
    let options = ViewOptions {
        tangent_edges: args.tangent_edges,
        hidden_lines: args.hidden_lines,
    };
    let mut placed: Vec<Placed<S>> = projected
        .views
        .iter()
        .filter_map(|(kind, view, hatched)| {
            let e = view.extents()?;
            Some(Placed {
                kind: *kind,
                extents: [e[0], e[1], e[2], e[3]].map(|x| x.to_f64()),
                view: view.clone(),
                hatched: hatched.clone(),
                center: [0.0, 0.0],
                margin: 0.0,
            })
        })
        .collect();
    if placed.is_empty() && !args.bom {
        return Err(GeopError::new("no view of the drawing shows any line"));
    }
    let mut sheet = Sheet {
        width,
        height,
        ..Sheet::default()
    };
    if placed.is_empty() {
        // Its bill of materials alone.
        let scale = args.scale.unwrap_or(1.0);
        draw_frame(&mut sheet, args, scale, date);
        draw_parts_list(&mut sheet, parts);
        return Ok(Layout {
            sheet,
            views: Vec::new(),
            scale,
            candidates: Vec::new(),
        });
    }
    let balloons = match args.bom && scene.bodies.iter().any(|b| !b.path.is_empty()) {
        true => balloon_view(&scene, &placed, parts)?,
        false => None,
    };
    if let Some((index, _)) = &balloons {
        placed[*index].margin = BALLOON_MARGIN;
    }

    // The grid: each occupied column as wide as its widest view, each row
    // as high as its highest, in model units — and with as much paper
    // either side of its views as the widest margin of one asks for.
    let mut columns: Vec<(usize, f64, f64)> = Vec::new();
    let mut rows: Vec<(usize, f64, f64)> = Vec::new();
    for p in &placed {
        let (c, r) = cell(p.kind, args.projection);
        let (w, h) = (p.extents[2] - p.extents[0], p.extents[3] - p.extents[1]);
        grow(&mut columns, c, w, p.margin);
        grow(&mut rows, r, h, p.margin);
    }
    columns.sort_by_key(|&(c, _, _)| c);
    rows.sort_by_key(|&(r, _, _)| r);
    let model_width: f64 = columns.iter().map(|&(_, w, _)| w).sum();
    let model_height: f64 = rows.iter().map(|&(_, h, _)| h).sum();
    let paper_width: f64 = columns.iter().map(|&(_, _, m)| 2.0 * m).sum();
    let paper_height: f64 = rows.iter().map(|&(_, _, m)| 2.0 * m).sum();
    let area_width = width - 2.0 * MARGIN;
    let area_height = height - 2.0 * MARGIN - TITLE_HEIGHT - list_height;
    let fit = ((area_width - paper_width - GAP * (columns.len() + 1) as f64)
        / model_width.max(1e-300))
    .min((area_height - paper_height - GAP * (rows.len() + 1) as f64) / model_height.max(1e-300));
    let scale = match args.scale {
        Some(s) if s.is_finite() && s > 0.0 => s,
        Some(s) => {
            return Err(GeopError::new(format!(
                "the scale {s} is not a positive number"
            )));
        }
        None => SCALES
            .iter()
            .copied()
            .find(|&s| s <= fit)
            .unwrap_or(SCALES[SCALES.len() - 1]),
    };
    // Centred on the sheet's area above the title block and the bill.
    let used_width = scale * model_width + paper_width + GAP * (columns.len() - 1) as f64;
    let used_height = scale * model_height + paper_height + GAP * (rows.len() - 1) as f64;
    let left = MARGIN + (area_width - used_width) / 2.0;
    let top = height - MARGIN - (area_height - used_height) / 2.0;
    for p in &mut placed {
        let (c, r) = cell(p.kind, args.projection);
        let ci = columns.iter().position(|&(k, _, _)| k == c).unwrap_or(0);
        let ri = rows.iter().position(|&(k, _, _)| k == r).unwrap_or(0);
        let x: f64 = left
            + columns[..ci]
                .iter()
                .map(|&(_, w, m)| scale * w + 2.0 * m + GAP)
                .sum::<f64>()
            + columns[ci].2
            + scale * columns[ci].1 / 2.0;
        let y: f64 = top
            - rows[..ri]
                .iter()
                .map(|&(_, h, m)| scale * h + 2.0 * m + GAP)
                .sum::<f64>()
            - rows[ri].2
            - scale * rows[ri].1 / 2.0;
        p.center = [x, y];
    }

    for p in &placed {
        draw_view(&mut sheet, p, scale).with_context(&ctx)?;
    }
    draw_threads(&mut sheet, &scene, &placed, scale, &options).with_context(&ctx)?;
    if let Some((index, targets)) = &balloons {
        draw_balloons(&mut sheet, &placed[*index], targets, scale).with_context(&ctx)?;
    }
    draw_frame(&mut sheet, args, scale, date);
    if args.bom {
        draw_parts_list(&mut sheet, parts);
    }
    let candidates = candidates(&scene, &placed, scale).with_context(&ctx)?;
    Ok(Layout {
        sheet,
        views: placed.iter().map(Placement::of).collect(),
        scale,
        candidates,
    })
}

/// What a balloon points at: an item number, and a point of its part on
/// the paper of its view, in model units.
type Target = (String, P);

/// The view the balloons go around, by its index in `placed`, and each
/// balloon's target: the view in which the most lines of the bill show a
/// visible line of their parts — the isometric one first among equals,
/// which sees each part from where most of it shows. A balloon's leader
/// points at the point of its part's lines in the view farthest out from
/// the view's centre, of those a quarter, half and three quarters along
/// each: a visible one, or a hidden one if that is all it has there. A
/// point far out is one a leader reaches without crossing other parts, and
/// one inside a line is no corner other lines share.
///
/// A line of the bill with no faces drawn — a wire, or a part placed
/// nowhere in the drawing — gets no balloon.
fn balloon_view<S: Scalar>(
    scene: &Scene<'_, S>,
    placed: &[Placed<S>],
    parts: &[PartsListLine],
) -> GeopResult<Option<(usize, Vec<Target>)>> {
    // The line of the bill each body is counted in.
    let mut line_of: Vec<Option<usize>> = vec![None; scene.bodies.len()];
    let paths: std::collections::HashMap<&str, usize> = parts
        .iter()
        .enumerate()
        .flat_map(|(k, line)| line.placements.iter().map(move |p| (p.as_str(), k)))
        .collect();
    for (b, body) in scene.bodies.iter().enumerate() {
        line_of[b] = paths.get(body.path.as_str()).copied();
    }
    let mut order: Vec<usize> = (0..placed.len())
        .filter(|&i| matches!(placed[i].kind, DrawnView::View(_)))
        .collect();
    order.sort_by_key(|&i| placed[i].kind != DrawnView::View(ViewKind::Iso));
    let mut best: Option<(usize, usize, Vec<Target>)> = None;
    for i in order {
        let p = &placed[i];
        let middle = [
            (p.extents[0] + p.extents[2]) / 2.0,
            (p.extents[1] + p.extents[3]) / 2.0,
        ];
        // Per line of the bill: whether its point is seen, how far out it
        // is, and where.
        let mut found: Vec<Option<(bool, f64, P)>> = vec![None; parts.len()];
        for l in &p.view.lines {
            let Some(k) = line_of[l.body] else { continue };
            let (t0, t1) = l.curve.domain();
            for f in [0.25, 0.5, 0.75] {
                let q = l
                    .curve
                    .evaluate(t0.add(t1.sub(t0).mul(S::from_f64(f))).sharpen())?;
                let q = [q[0].to_f64(), q[1].to_f64()];
                let out = (q[0] - middle[0]).hypot(q[1] - middle[1]);
                if found[k].is_none_or(|(visible, o, _)| (l.visible, out) > (visible, o)) {
                    found[k] = Some((l.visible, out, q));
                }
            }
        }
        let seen = found.iter().flatten().filter(|f| f.0).count();
        let targets = found
            .iter()
            .zip(parts)
            .filter_map(|(f, line)| f.map(|(_, _, q)| (line.item.clone(), q)))
            .collect();
        if best.as_ref().is_none_or(|(_, s, _)| seen > *s) {
            best = Some((i, seen, targets));
        }
    }
    Ok(best
        .filter(|(_, _, targets)| !targets.is_empty())
        .map(|(i, _, targets)| (i, targets)))
}

/// The point `s` along the rectangle `r` (`[x_min, y_min, x_max, y_max]`),
/// counter-clockwise from its lower left corner.
fn along_rectangle(r: [f64; 4], s: f64) -> P {
    let (w, h) = (r[2] - r[0], r[3] - r[1]);
    let s = s.rem_euclid(2.0 * (w + h));
    if s < w {
        [r[0] + s, r[1]]
    } else if s < w + h {
        [r[2], r[1] + s - w]
    } else if s < 2.0 * w + h {
        [r[2] - (s - w - h), r[3]]
    } else {
        [r[0], r[3] - (s - 2.0 * w - h)]
    }
}

/// How far along the rectangle `r` (see [`along_rectangle`]) the ray from
/// `from`, inside it, along `d` leaves it.
fn rectangle_exit(r: [f64; 4], from: P, d: P) -> f64 {
    let (w, h) = (r[2] - r[0], r[3] - r[1]);
    // The side the ray reaches first, and how far along the rectangle.
    let mut best = (f64::INFINITY, 0.0);
    // Bottom, right, top, left: the coordinate fixed on each, its value,
    // and where along the rectangle it starts.
    let sides = [
        (1, r[1], 0.0),
        (0, r[2], w),
        (1, r[3], w + h),
        (0, r[0], 2.0 * w + h),
    ];
    for (k, &(axis, value, start)) in sides.iter().enumerate() {
        if d[axis] == 0.0 {
            continue;
        }
        let t = (value - from[axis]) / d[axis];
        if t <= 0.0 || t >= best.0 {
            continue;
        }
        let q = add(from, times(d, t));
        let along = match k {
            0 => q[0] - r[0],
            1 => q[1] - r[1],
            2 => r[2] - q[0],
            _ => r[3] - q[1],
        };
        best = (t, start + along);
    }
    best.1
}

/// The balloons around the view `p`: each a circle with its item number,
/// on a rectangle standing `BALLOON_OFFSET` off the view's box, its leader
/// running to a dot on its target. Each goes where the ray from the view's
/// centre through its target meets the rectangle, and they are spread
/// along it, in that order, until none overlaps the next: so leaders run
/// outwards, and do not cross.
fn draw_balloons<S: Scalar>(
    sheet: &mut Sheet,
    p: &Placed<S>,
    targets: &[Target],
    scale: f64,
) -> GeopResult<()> {
    let place = placer(p, scale);
    let [x0, y0, x1, y1] = p.extents;
    let (lo, hi) = (place([x0, y0]), place([x1, y1]));
    let ring = [
        lo[0] - BALLOON_OFFSET,
        lo[1] - BALLOON_OFFSET,
        hi[0] + BALLOON_OFFSET,
        hi[1] + BALLOON_OFFSET,
    ];
    let perimeter = 2.0 * (ring[2] - ring[0] + ring[3] - ring[1]);
    let spacing = 2.0 * BALLOON_RADIUS + BALLOON_RADIUS / 2.0;
    if targets.len() as f64 * spacing > perimeter {
        return Err(GeopError::new(format!(
            "{} balloons do not fit around the view: choose a larger sheet",
            targets.len()
        )));
    }
    let mut balloons: Vec<(f64, &Target)> = targets
        .iter()
        .map(|target| {
            let d = sub(place(target.1), p.center);
            let d = if d == [0.0, 0.0] { [0.0, 1.0] } else { d };
            (rectangle_exit(ring, p.center, d), target)
        })
        .collect();
    balloons.sort_by(|a, b| a.0.total_cmp(&b.0));
    // Spread forwards — and, should the last then run into the first round
    // the rectangle, backwards from there.
    for k in 1..balloons.len() {
        balloons[k].0 = balloons[k].0.max(balloons[k - 1].0 + spacing);
    }
    let n = balloons.len();
    if n > 1 && balloons[n - 1].0 > balloons[0].0 + perimeter - spacing {
        balloons[n - 1].0 = balloons[0].0 + perimeter - spacing;
        for k in (0..n - 1).rev() {
            balloons[k].0 = balloons[k].0.min(balloons[k + 1].0 - spacing);
        }
    }
    for (s, (item, target)) in balloons {
        let center = along_rectangle(ring, s);
        let at = place(*target);
        let toward = unit(sub(at, center));
        sheet.stroke(
            Layer::Dimension,
            Shape::Circle {
                center,
                radius: BALLOON_RADIUS,
            },
        );
        sheet.stroke(
            Layer::Dimension,
            Shape::Line(add(center, times(toward, BALLOON_RADIUS)), at),
        );
        sheet.stroke(
            Layer::Dimension,
            Shape::Circle {
                center: at,
                radius: LEADER_DOT,
            },
        );
        sheet.label(
            Layer::Dimension,
            [center[0], center[1] - TEXT / 2.0],
            TEXT,
            Anchor::Middle,
            item.clone(),
        );
    }
    Ok(())
}

/// The bill of materials as a table standing on the title block, as wide
/// as it: its captions in the bottom row, the first item above them. A
/// text too long for its column is cut short, ending in `…`.
fn draw_parts_list(sheet: &mut Sheet, parts: &[PartsListLine]) {
    let x0 = sheet.width - MARGIN - TITLE_WIDTH;
    let y0 = MARGIN + TITLE_HEIGHT;
    let rows = parts.len() + 1;
    let y1 = y0 + LIST_ROW * rows as f64;
    for k in 1..=rows {
        let y = y0 + LIST_ROW * k as f64;
        sheet.stroke(Layer::Border, Shape::Line([x0, y], [x0 + TITLE_WIDTH, y]));
    }
    let mut x = x0;
    for (_, w) in LIST_COLUMNS {
        sheet.stroke(Layer::Border, Shape::Line([x, y0], [x, y1]));
        x += w;
    }
    sheet.stroke(Layer::Border, Shape::Line([x, y0], [x, y1]));
    let row = |sheet: &mut Sheet, k: usize, cells: [&str; 5]| {
        let mut x = x0;
        for ((_, w), text) in LIST_COLUMNS.iter().zip(cells) {
            // An average glyph is about 0.6 of the text's height wide.
            let fits = ((w - 2.0) / (0.6 * LIST_TEXT)) as usize;
            let text = match text.chars().count() > fits {
                true => format!(
                    "{}…",
                    text.chars()
                        .take(fits.saturating_sub(1))
                        .collect::<String>()
                ),
                false => text.to_string(),
            };
            let at = [
                x + 1.0,
                y0 + LIST_ROW * k as f64 + (LIST_ROW - LIST_TEXT) / 2.0,
            ];
            sheet.label(Layer::Border, at, LIST_TEXT, Anchor::Start, text);
            x += w;
        }
    };
    row(sheet, 0, LIST_COLUMNS.map(|(caption, _)| caption));
    for (k, line) in parts.iter().enumerate() {
        let quantity = line.quantity.to_string();
        let cells = [
            line.item.as_str(),
            quantity.as_str(),
            line.name.as_str(),
            line.designation.as_str(),
            line.material.as_str(),
        ];
        row(sheet, k + 1, cells);
    }
}

/// How a view sees a thread's axis.
enum ThreadSeen {
    /// Square to the axis: the thread from the side.
    Side,
    /// Along the axis: the thread end on.
    End,
}

/// The part's cosmetic threads, as drafting draws them, in every
/// orthographic view seeing their axis from the side or end on: from the
/// side two thin lines along the thread, at its minor diameter on a shaft
/// and at its major diameter in a hole; end on, three quarters of a thin
/// circle at that diameter. Each on [`Layer::Hidden`] where the threaded
/// face is hidden there, and labelled with its designation in the first
/// view that draws it. A view seeing the axis obliquely, and the section
/// view, draw no threads.
///
/// The threads of the parts placed are drawn where they are placed, and
/// not labelled: the bill of materials designates the parts they are on.
fn draw_threads<S: Scalar>(
    sheet: &mut Sheet,
    scene: &Scene<'_, S>,
    placed: &[Placed<S>],
    scale: f64,
    options: &ViewOptions,
) -> GeopResult<()> {
    for body in &scene.bodies {
        let mut threads: Vec<_> = body.part.threads().collect();
        threads.sort_by_key(|(name, _)| *name);
        for (name, thread) in threads {
            let ctx = |e: GeopError| match body.path.as_str() {
                "" => e.with_context(format!("the cosmetic thread {name}")),
                path => e.with_context(format!(
                    "the cosmetic thread {name} of the part placed as {path}"
                )),
            };
            let (mut start, mut along) = (thread.axis.point, thread.axis.direction);
            if let Some(pose) = &body.pose {
                let motion = pose.motion();
                start = motion.apply(&start);
                along = motion.rotate(&along);
            }
            let seen = Seen {
                scene,
                placed,
                scale,
                options,
            };
            draw_thread(sheet, &seen, thread, start, along, body.path.is_empty())
                .with_context(&ctx)?;
        }
    }
    Ok(())
}

/// What drawing a cosmetic thread looks at: the bodies drawn, the views
/// placed and the sheet's scale.
struct Seen<'a, 'p, S: Scalar> {
    scene: &'a Scene<'p, S>,
    placed: &'a [Placed<S>],
    scale: f64,
    options: &'a ViewOptions,
}

/// The cosmetic thread `thread`, from `start` along the unit vector
/// `along`, in every view that sees it (see [`draw_threads`]), labelled
/// in the first if `label`.
fn draw_thread<S: Scalar>(
    sheet: &mut Sheet,
    seen: &Seen<'_, '_, S>,
    thread: &geop_ops::part::CosmeticThread<S>,
    start: Vector3<S>,
    along: Vector3<S>,
    label: bool,
) -> GeopResult<()> {
    let Seen {
        scene,
        placed,
        scale,
        options,
    } = *seen;
    let end = start.add(&along.prod_scalar(S::from_f64(thread.length)));
    let drawn = if thread.internal {
        thread.major_diameter
    } else {
        thread.minor_diameter
    } / 2.0;
    let mut labelled = !label;
    for p in placed
        .iter()
        .filter(|p| matches!(p.kind, DrawnView::View(k) if k != ViewKind::Iso))
    {
        let frame = &p.view.frame;
        let look = frame.direction.vector();
        let seen = if look.prod_dot(&along).could_be_equal(S::ZERO) {
            ThreadSeen::Side
        } else if look
            .prod_cross(&along)
            .to_array()
            .iter()
            .all(|c| c.could_be_equal(S::ZERO))
        {
            ThreadSeen::End
        } else {
            continue;
        };
        let place = placer(p, scale);
        let at = |q: &Vector3<S>| {
            let v = frame.project_point(q);
            place([v[0].to_f64(), v[1].to_f64()])
        };
        // Whether the threaded face is seen: from the side, at its
        // point nearest the eye halfway along — on a shaft that is
        // in front, in a hole behind the material round it; end on,
        // where the thread is drawn at its end nearer the eye.
        let (probe, label_at) = match seen {
            ThreadSeen::Side => {
                let middle = start.add(&along.prod_scalar(S::from_f64(thread.length / 2.0)));
                (
                    middle.add(&frame.toward_eye().prod_scalar(thread.radius)),
                    at(&end),
                )
            }
            ThreadSeen::End => {
                let near = if frame
                    .direction
                    .dot(&start)
                    .definitely_greater(frame.direction.dot(&end))
                {
                    end
                } else {
                    start
                };
                let frame_there = geop_ops::operation::frame_along(near, &along)?;
                let side = *frame_there.u();
                let c = at(&near);
                let reach = scale * drawn * std::f64::consts::FRAC_1_SQRT_2;
                (
                    near.add(&side.prod_scalar(S::from_f64(drawn))),
                    [c[0] + reach, c[1] + reach],
                )
            }
        };
        let visible = scene.point_seen(frame, probe)?;
        if !visible && !options.hidden_lines {
            continue;
        }
        let layer = if visible {
            Layer::Thread
        } else {
            Layer::Hidden
        };
        match seen {
            ThreadSeen::Side => {
                let off = look
                    .prod_cross(&along)
                    .normalize()?
                    .prod_scalar(S::from_f64(drawn));
                for off in [off, off.neg()] {
                    sheet.stroke(layer, Shape::Line(at(&start.add(&off)), at(&end.add(&off))));
                }
            }
            ThreadSeen::End => {
                // Open in the quarter up and to the right, as drafting
                // leaves it, a little turned.
                sheet.stroke(
                    layer,
                    Shape::Arc {
                        center: at(&start),
                        radius: scale * drawn,
                        start: 100f64.to_radians(),
                        end: 10f64.to_radians(),
                    },
                );
            }
        }
        if !labelled {
            sheet.label(
                Layer::Dimension,
                [label_at[0] + 1.5, label_at[1] + 1.5],
                TEXT,
                Anchor::Start,
                thread.designation.clone(),
            );
            labelled = true;
        }
    }
    Ok(())
}

/// Makes `cells` hold `key` at least `size`, with at least `margin`.
fn grow(cells: &mut Vec<(usize, f64, f64)>, key: usize, size: f64, margin: f64) {
    match cells.iter_mut().find(|(k, _, _)| *k == key) {
        Some((_, s, m)) => {
            *s = s.max(size);
            *m = m.max(margin);
        }
        None => cells.push((key, size, margin)),
    }
}

/// Where a point of the view `p`, in model units on its paper, lands on
/// the sheet.
pub(crate) fn placer<S: Scalar>(p: &Placed<S>, scale: f64) -> impl Fn(P) -> P + '_ {
    let mid = [
        (p.extents[0] + p.extents[2]) / 2.0,
        (p.extents[1] + p.extents[3]) / 2.0,
    ];
    move |q: P| {
        [
            p.center[0] + scale * (q[0] - mid[0]),
            p.center[1] + scale * (q[1] - mid[1]),
        ]
    }
}

/// The view `p`'s lines, hatching, centre marks, overall dimensions and
/// label.
fn draw_view<S: Scalar>(sheet: &mut Sheet, p: &Placed<S>, scale: f64) -> GeopResult<()> {
    let place = placer(p, scale);
    let curves: Vec<(Layer, &NurbCurve2D<S>)> = p
        .view
        .lines
        .iter()
        .map(|line| {
            let layer = if line.visible {
                Layer::Visible
            } else {
                Layer::Hidden
            };
            (layer, &line.curve)
        })
        .collect();
    sheet.strokes.extend(strokes_of(&curves, &place, scale)?);
    for (boundary, mirrored) in &p.hatched {
        for (a, b) in hatch(boundary, scale, *mirrored)? {
            sheet.stroke(Layer::Hatch, Shape::Line(place(a), place(b)));
        }
    }
    if p.kind.orthographic() {
        center_marks(sheet, p, scale)?;
        // The overall size: the width below the view, the height to its
        // right.
        let [x0, y0, x1, y1] = p.extents;
        let (a, b) = (place([x0, y0]), place([x1, y0]));
        let below = [(a[0] + b[0]) / 2.0, a[1] - DIMENSION_OFFSET];
        Drawn::measure(
            Measure::Linear {
                a,
                b,
                along: [1.0, 0.0],
            },
            below,
            length_label(x1 - x0),
        )
        .draw(sheet);
        let (a, b) = (place([x1, y0]), place([x1, y1]));
        let right = [a[0] + DIMENSION_OFFSET, (a[1] + b[1]) / 2.0];
        Drawn::measure(
            Measure::Linear {
                a,
                b,
                along: [0.0, 1.0],
            },
            right,
            length_label(y1 - y0),
        )
        .draw(sheet);
    }
    if p.kind == DrawnView::Section {
        let [_, y0, _, _] = p.extents;
        let below = place([(p.extents[0] + p.extents[2]) / 2.0, y0]);
        sheet.label(
            Layer::Dimension,
            [below[0], below[1] - DIMENSION_OFFSET - 2.5 * TEXT],
            TEXT * 1.4,
            Anchor::Middle,
            "SECTION A-A",
        );
    }
    Ok(())
}

/// Centre marks on the circles the view `p` sees round: on every centre
/// with a circular edge turning through half a turn or more around it, two
/// centre lines across its largest such circle.
fn center_marks<S: Scalar>(sheet: &mut Sheet, p: &Placed<S>, scale: f64) -> GeopResult<()> {
    // (centre, radius, sweep in radians)
    let mut circles: Vec<(Vector3<S>, S, f64)> = Vec::new();
    for line in p.view.lines.iter().filter(|l| l.kind == LineKind::Edge) {
        let lifted = lift(&line.curve)?;
        let Some(arc) = lifted.as_arc()? else {
            continue;
        };
        let sweep = arc.sweep();
        let (center, radius) = (arc.circle.center, arc.circle.radius);
        match circles
            .iter_mut()
            .find(|(c, r, _)| c.could_be_equal(&center) && r.could_be_equal(radius))
        {
            Some((_, _, s)) => *s += sweep,
            None => circles.push((center, radius, sweep)),
        }
    }
    let mut marks: Vec<(Vector3<S>, f64)> = Vec::new();
    for (center, radius, sweep) in circles {
        if sweep < std::f64::consts::PI * 0.999 {
            continue;
        }
        let r = radius.to_f64();
        match marks.iter_mut().find(|(c, _)| c.could_be_equal(&center)) {
            Some((_, m)) => *m = m.max(r),
            None => marks.push((center, r)),
        }
    }
    let place = placer(p, scale);
    for (center, r) in marks {
        let c = place([center[0].to_f64(), center[1].to_f64()]);
        let reach = r * scale + CENTER_OVERSHOOT;
        sheet.stroke(
            Layer::Center,
            Shape::Line([c[0] - reach, c[1]], [c[0] + reach, c[1]]),
        );
        sheet.stroke(
            Layer::Center,
            Shape::Line([c[0], c[1] - reach], [c[0], c[1] + reach]),
        );
    }
    Ok(())
}

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}

fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}

fn times(a: P, s: f64) -> P {
    [a[0] * s, a[1] * s]
}

fn unit(a: P) -> P {
    let n = (a[0] * a[0] + a[1] * a[1]).sqrt();
    if n > 0.0 {
        times(a, 1.0 / n)
    } else {
        [1.0, 0.0]
    }
}

/// Where the vertex `name` of `part` — or of a part placed in it, named as
/// `part` names it — is.
pub(crate) fn placed_vertex<S: Scalar>(part: &Part<S>, name: &str) -> GeopResult<Vector3<S>> {
    let (owner, local, pose) = resolve(part, &EntityRef::Vertex { name: name.into() })?;
    let point = owner
        .topology()
        .get_vertex(owner.vertex_id(&local.label())?)?
        .point;
    Ok(match pose {
        Some(pose) => pose.apply(&point),
        None => point,
    })
}

/// The curve of the edge `name` of `part` — or of a part placed in it,
/// named as `part` names it — where it is.
pub(crate) fn placed_edge<S: Scalar>(part: &Part<S>, name: &str) -> GeopResult<NurbCurve3D<S>> {
    let (owner, local, pose) = resolve(part, &EntityRef::Edge { name: name.into() })?;
    let curve = &owner
        .topology()
        .get_edge(owner.edge_id(&local.label())?)?
        .curve;
    Ok(match pose {
        Some(pose) => curve.transform(&pose.motion()),
        None => curve.clone(),
    })
}

/// The border and the title block.
fn draw_frame(sheet: &mut Sheet, args: &DrawingArgs, scale: f64, date: &str) {
    let (w, h) = (sheet.width, sheet.height);
    let corners =
        |x0: f64, y0: f64, x1: f64, y1: f64| vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]];
    sheet.stroke(
        Layer::Border,
        Shape::Polyline(corners(MARGIN, MARGIN, w - MARGIN, h - MARGIN)),
    );
    let (x0, y0) = (w - MARGIN - TITLE_WIDTH, MARGIN);
    let (x1, y1) = (w - MARGIN, MARGIN + TITLE_HEIGHT);
    sheet.stroke(Layer::Border, Shape::Polyline(corners(x0, y0, x1, y1)));
    let row = TITLE_HEIGHT / 4.0;
    let half = x0 + TITLE_WIDTH / 2.0;
    for k in 1..4 {
        let y = y0 + row * k as f64;
        sheet.stroke(Layer::Border, Shape::Line([x0, y], [x1, y]));
    }
    // The bottom two rows hold two fields each.
    sheet.stroke(
        Layer::Border,
        Shape::Line([half, y0], [half, y0 + 2.0 * row]),
    );
    let name = if args.name.is_empty() {
        "PART"
    } else {
        args.name.as_str()
    };
    let field = |sheet: &mut Sheet, x: f64, y: f64, caption: &str, value: &str| {
        sheet.label(
            Layer::Border,
            [x + 2.0, y + row - 3.0],
            2.0,
            Anchor::Start,
            caption,
        );
        sheet.label(
            Layer::Border,
            [x + 2.0, y + 1.5],
            TEXT,
            Anchor::Start,
            value,
        );
    };
    sheet.label(
        Layer::Border,
        [x0 + 3.0, y0 + 3.0 * row + 1.5],
        TITLE_TEXT * 0.8,
        Anchor::Start,
        name,
    );
    let material = if args.material.is_empty() {
        "-"
    } else {
        args.material.as_str()
    };
    field(sheet, x0, y0 + 2.0 * row, "MATERIAL", material);
    field(sheet, x0, y0 + row, "SCALE", &scale_label(scale));
    field(sheet, half, y0 + row, "UNITS", "mm");
    field(sheet, x0, y0, "DATE", date);
    field(sheet, half, y0, "PROJECTION", args.projection.label());
}

/// A region of a section view to hatch: its boundary curves on the paper,
/// in model units, and whether its hatching is mirrored — which it is on
/// every other part cut, so that two parts cut side by side tell apart.
type Hatched<S> = (Vec<NurbCurve2D<S>>, bool);

/// A body's part cut, with the cutting plane's origin and normal in its
/// own frame.
type Cut<S> = (Part<S>, Vector3<S>, Vector3<S>);

/// The drawing's bodies (`scene`, of `part`) cut at `plane`, seen from the
/// side its normal points to, and their cut faces' boundaries for hatching.
///
/// Each body is cut in its own part's frame (see [`section_part`]): a body
/// wholly on the side the normal points to is gone, one wholly on the
/// other side, or with no solid to cut, is drawn whole. Refused if no body
/// has a solid to cut.
fn section_view<S: Scalar>(
    part: &Part<S>,
    scene: &Scene<'_, S>,
    plane: &EntityRef,
    options: &ViewOptions,
) -> GeopResult<(ProjectedView<S>, Vec<Hatched<S>>)> {
    let frame_of = plane.resolve_plane(part)?;
    let normal = frame_of.w().normalize()?;
    let origin = *frame_of.origin();
    if scene.bodies.iter().all(|b| b.model().solids.is_empty()) {
        return Err(GeopError::new("the part has no solid to cut"));
    }
    // Up on the paper: the world axis least along the line of sight.
    let up = {
        let axis = (0..3)
            .min_by(|&a, &b| {
                normal[a]
                    .to_f64()
                    .abs()
                    .total_cmp(&normal[b].to_f64().abs())
            })
            .unwrap_or(2);
        let axis = if normal[2].to_f64().abs() < 0.9 {
            2
        } else {
            axis
        };
        let mut up = Vector3::zero();
        up[axis] = S::ONE;
        up
    };
    let frame = ViewFrame::looking(normal.neg(), up)?;
    // Each body as it is drawn: whole, or its part cut where the plane is
    // in its own frame.
    let mut cuts: Vec<(usize, Option<Cut<S>>)> = Vec::new();
    for (k, body) in scene.bodies.iter().enumerate() {
        if body.model().solids.is_empty() {
            cuts.push((k, None));
            continue;
        }
        match scene.side_of(k, &origin, &normal) {
            Some(true) => continue,
            Some(false) => cuts.push((k, None)),
            None => {
                let (o, n) = match &body.pose {
                    Some(pose) => {
                        let back = pose.inverse().motion();
                        (back.apply(&origin), back.rotate(&normal))
                    }
                    None => (origin, normal),
                };
                let cut = section_part(body.part, &o, &n).map_err(|e| {
                    e.with_context(format!("cutting the part placed as {:?}", body.path))
                })?;
                cuts.push((k, Some((cut, o, n))));
            }
        }
    }
    let bodies = cuts
        .iter()
        .map(|(k, cut)| {
            let body = &scene.bodies[*k];
            match cut {
                None => Body {
                    path: body.path.clone(),
                    part: body.part,
                    faces: body.faces.clone(),
                    pose: body.pose,
                },
                Some((cut, _, _)) => Body {
                    path: body.path.clone(),
                    part: cut,
                    faces: drawn_faces(cut.topology()),
                    pose: body.pose,
                },
            }
        })
        .filter(|b| !b.faces.is_empty())
        .collect();
    let options = ViewOptions {
        hidden_lines: false,
        ..*options
    };
    let view = Scene::new(bodies)?.project(&frame, &options)?;
    let mut hatched = Vec::new();
    for (index, (k, (cut, o, n))) in cuts
        .iter()
        .filter_map(|(k, cut)| cut.as_ref().map(|c| (k, c)))
        .enumerate()
    {
        let model = cut.topology();
        let motion = scene.bodies[*k].pose.map(|p| p.motion());
        for face in cut_faces(model, &drawn_faces(model), o, n)? {
            let mut boundary = Vec::new();
            for coedge in model.iterate_face_coedges(face) {
                if let geop_core_topology::CoedgeGeometry::Edge(edge) =
                    model.get_coedge(coedge)?.geometry
                {
                    let curve = &model.get_edge(edge)?.curve;
                    let curve = match &motion {
                        Some(m) => curve.transform(m),
                        None => curve.clone(),
                    };
                    boundary.push(frame.project_curve(&curve)?);
                }
            }
            hatched.push((boundary, index % 2 == 1));
        }
    }
    Ok((view, hatched))
}

/// Hatch lines across the region `boundary` bounds (model units on the
/// paper), at 45 degrees — or, `mirrored`, at 135 — `HATCH_SPACING` apart
/// on a sheet at `scale`.
///
/// Each line is cut where it crosses the boundary, and alternate stretches
/// are inside. A line through a corner of the boundary, or along it, cannot
/// be counted that way; where to put the lines is a free choice, so such a
/// line is moved a little instead.
fn hatch<S: Scalar>(
    boundary: &[NurbCurve2D<S>],
    scale: f64,
    mirrored: bool,
) -> GeopResult<Vec<(P, P)>> {
    if mirrored {
        let flip = |c: &NurbCurve2D<S>| {
            let points = c
                .control_points
                .iter()
                .map(|q| Vector3::from_array([q[0].neg(), q[1], q[2]]))
                .collect();
            NurbCurve2D::try_new(c.degree, points, c.knot_vector.clone())
        };
        let flipped = boundary.iter().map(flip).collect::<GeopResult<Vec<_>>>()?;
        let back = |p: P| [-p[0], p[1]];
        return Ok(hatch(&flipped, scale, false)?
            .into_iter()
            .map(|(a, b)| (back(a), back(b)))
            .collect());
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for curve in boundary {
        for q in &curve.control_points {
            for k in 0..2 {
                let x = q[k].div(q[2])?.to_f64();
                lo[k] = lo[k].min(x);
                hi[k] = hi[k].max(x);
            }
        }
    }
    if !(lo[0] < hi[0] && lo[1] < hi[1]) {
        return Ok(Vec::new());
    }
    // Lines `x - y = c`, from the box's lower right to upper left corner.
    let spacing = HATCH_SPACING / scale * std::f64::consts::SQRT_2;
    let (c_lo, c_hi) = (lo[0] - hi[1], hi[0] - lo[1]);
    let reach = (hi[0] - lo[0]) + (hi[1] - lo[1]) + 1.0;
    let mut out = Vec::new();
    let mut c = c_lo + spacing / 2.0;
    while c < c_hi {
        for shift in [0.0, 0.37, -0.29] {
            let cc = c + shift * spacing;
            // From a point of the line left of the box to one right of it.
            let x_start = lo[0] - 1.0;
            let a = [x_start, x_start - cc];
            let b = [x_start + reach, x_start + reach - cc];
            match hatch_line(boundary, a, b)? {
                Some(stretches) => {
                    out.extend(stretches);
                    break;
                }
                None => continue,
            }
        }
        c += spacing;
    }
    Ok(out)
}

/// The stretches of the segment from `a` to `b` inside `boundary`, or
/// `None` if it passes through a corner of it or runs along it.
fn hatch_line<S: Scalar>(
    boundary: &[NurbCurve2D<S>],
    a: P,
    b: P,
) -> GeopResult<Option<Vec<(P, P)>>> {
    let point = |p: P| Vector3::from_array([S::from_f64(p[0]), S::from_f64(p[1]), S::ONE]);
    let line = NurbCurve2D::try_new(
        1,
        vec![point(a), point(b)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )?;
    let mut ts: Vec<f64> = Vec::new();
    for curve in boundary {
        let (overlaps, crossings) =
            curve_curve_overlaps_and_crossings(&line, curve, MAX_NODES, min_subdivision_size())?;
        if !overlaps.is_empty() {
            return Ok(None);
        }
        let (s0, s1) = curve.domain();
        for (t, s) in crossings {
            if s.could_be_equal(s0) || s.could_be_equal(s1) {
                return Ok(None);
            }
            ts.push(t.midpoint().to_f64());
        }
    }
    if ts.len() % 2 == 1 {
        return Ok(None);
    }
    ts.sort_by(f64::total_cmp);
    let at = |t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
    Ok(Some(
        ts.chunks(2)
            .map(|pair| (at(pair[0]), at(pair[1])))
            .collect(),
    ))
}
