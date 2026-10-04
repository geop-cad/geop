//! Hidden-line removal: a part's edges and silhouettes, projected onto the
//! paper, each piece of them classified visible or hidden.
//!
//! What a view draws is the edges where faces meet at an angle (tangent
//! edges, where they meet smoothly, only if asked), and the silhouettes of
//! curved faces (see [`crate::silhouette`]). Along such a curve, what is in
//! front of it changes only where its projection meets the projection of
//! another — passing behind an edge or a silhouette — or turns back on
//! itself, where the curve runs along the line of sight. So:
//!
//! 1. every curve is projected (exactly: the projection is linear, so a
//!    NURBS curve projects to a NURBS curve with the same knots);
//! 2. each is cut wherever its projection crosses or runs along another's
//!    ([`curve_curve_overlaps_and_crossings`]) and wherever it is stationary
//!    along the paper's axes ([`NurbCurve::stationary_parameters`]), which
//!    holds every point where it turns back — and leaves every piece
//!    monotone along both axes, so the pieces' ends span the view;
//! 3. one point of each piece decides it: a ray from there towards the eye
//!    either crosses a face first (hidden) or does not (visible).
//!
//! A ray crossing a face's trim right at its boundary has met an edge in
//! front, whose projection the piece then runs along: the piece is drawn by
//! that edge already, and is classified hidden, so that the last step drops
//! it with every other piece lying on one already drawn — a hidden line
//! under a visible one is never drawn, nor any line twice.
//!
//! [`NurbCurve::stationary_parameters`]: geop_core_geometry::nurb_curve::NurbCurve::stationary_parameters

use std::collections::HashMap;

use geop_core_geometry::{
    contains::curve::curve_could_contain,
    intersection::{
        curve_curve_overlaps_and_crossings, curve_surface_crossings, curve_surface_overlaps,
        refine_crossing,
    },
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
};
use geop_core_topology::{
    CoedgeGeometry, CoedgeId, EdgeId, FaceId, Model,
    contains::face::{PointClassification, face_contains},
};

use crate::{
    MAX_NODES, min_subdivision_size,
    silhouette::{Silhouette, edge_crossings, face_silhouettes},
    view::ViewFrame,
};

/// Newton steps projecting a point onto a surface.
const PROJECT_ITERATIONS: usize = 20;
/// Seed for the containment tests.
const FACE_CONTAINS_SEED: u64 = 0xD_0A_11;
/// Where a search for where two lines cross on the paper hands over when
/// one with the fine handoff could not settle (see [`meetings`]): far
/// below what a sheet shows at any scale it is drawn at.
const COARSE_SUBDIVISION: f64 = 1e-5;

/// What a line of a view shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineKind {
    /// An edge where two faces meet at an angle, or a face's free edge.
    Edge,
    /// An edge where two faces meet smoothly.
    Tangent,
    /// Where a curved face turns away from the eye.
    Silhouette,
}

/// What to draw.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ViewOptions {
    /// Whether tangent edges are drawn.
    pub tangent_edges: bool,
    /// Whether hidden lines are kept.
    pub hidden_lines: bool,
}

impl Default for ViewOptions {
    fn default() -> Self {
        ViewOptions {
            tangent_edges: false,
            hidden_lines: true,
        }
    }
}

/// A piece of a line of a view, visible or hidden throughout.
#[derive(Clone, Debug)]
pub struct ViewLine<S: Scalar> {
    /// On the paper, in model units.
    pub curve: NurbCurve2D<S>,
    /// The same piece in space.
    pub curve3: NurbCurve3D<S>,
    pub kind: LineKind,
    pub visible: bool,
    /// The edge it is a piece of, if it is one: of the model of its body.
    pub edge: Option<EdgeId>,
    /// Which body of the scene it is a line of (see [`crate::scene`]): 0
    /// in a view of one model.
    pub body: usize,
    /// Which curve it is a piece of, and where: its parameter range there.
    pub(crate) source: usize,
    pub(crate) range: (S, S),
}

/// A part seen from one direction.
#[derive(Clone, Debug)]
pub struct ProjectedView<S: Scalar> {
    pub frame: ViewFrame<S>,
    pub lines: Vec<ViewLine<S>>,
}

impl<S: Scalar> ProjectedView<S> {
    /// The box the view's lines span on the paper, `[x_min, y_min, x_max,
    /// y_max]`, each an enclosure: the ends of its pieces, which are
    /// monotone along both axes. `None` for a view without lines.
    pub fn extents(&self) -> Option<[S; 4]> {
        let mut b: Option<[S; 4]> = None;
        for line in &self.lines {
            let (t0, t1) = line.curve.domain();
            for t in [t0, t1] {
                let Ok(p) = line.curve.evaluate(t) else {
                    continue;
                };
                let (x, y) = (p[0], p[1]);
                b = Some(match b {
                    None => [x, y, x, y],
                    Some([a, c, e, f]) => [a.min(x), c.min(y), e.max(x), f.max(y)],
                });
            }
        }
        b
    }
}

/// A curve to draw, before it is cut into pieces.
struct Source<S: Scalar> {
    curve3: NurbCurve3D<S>,
    curve: NurbCurve2D<S>,
    kind: LineKind,
    edge: Option<EdgeId>,
    silhouette: Option<Silhouette<S>>,
    /// The faces it lies on: those an edge bounds, a silhouette's face.
    on: Vec<FaceId>,
    /// Which curve it is a piece of, once curves are cut into monotone
    /// pieces.
    origin: usize,
    /// Its projection's box, `[x_lo, y_lo, x_hi, y_hi]`, holding every
    /// control point's whole enclosure.
    bounds: [f64; 4],
}

/// The box holding `curve`'s control points (and so the curve).
pub(crate) fn bounds<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<[f64; 4]> {
    let mut b = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for q in &curve.control_points {
        let (x, y) = (q[0].div(q[2])?, q[1].div(q[2])?);
        b[0] = b[0].min(x.lower().to_f64());
        b[1] = b[1].min(y.lower().to_f64());
        b[2] = b[2].max(x.upper().to_f64());
        b[3] = b[3].max(y.upper().to_f64());
    }
    Ok(b)
}

pub(crate) fn boxes_meet(a: &[f64; 4], b: &[f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

/// Whether `curve` projects to a single point: a line along the line of
/// sight, seen end on.
fn is_point<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<bool> {
    let first = curve.evaluate(curve.domain().0)?;
    for q in &curve.control_points {
        let p = Vector2::from_array([q[0].div(q[2])?, q[1].div(q[2])?]);
        if !p.could_be_equal(&first) {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The straight line from `a` to `b`, on `[0, 1]`.
fn line3<S: Scalar>(a: Vector3<S>, b: Vector3<S>) -> GeopResult<NurbCurve3D<S>> {
    NurbCurve3D::try_new(
        1,
        vec![
            Vector4::from_array([a[0], a[1], a[2], S::ONE]),
            Vector4::from_array([b[0], b[1], b[2], S::ONE]),
        ],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// The coedges of every edge of `faces`.
fn edge_coedges<S: Scalar>(model: &Model<S>, faces: &[FaceId]) -> Vec<(EdgeId, Vec<CoedgeId>)> {
    let mut order = Vec::new();
    let mut map: HashMap<EdgeId, Vec<CoedgeId>> = HashMap::new();
    for &face_id in faces {
        for coedge_id in model.iterate_face_coedges(face_id) {
            if let CoedgeGeometry::Edge(edge_id) = model.coedges[&coedge_id].geometry {
                let entry = map.entry(edge_id).or_default();
                if entry.is_empty() {
                    order.push(edge_id);
                }
                entry.push(coedge_id);
            }
        }
    }
    order
        .into_iter()
        .map(|e| {
            let coedges = map.remove(&e).unwrap_or_default();
            (e, coedges)
        })
        .collect()
}

/// How the edge `edge_id`, between the faces of `coedges`, is drawn seen
/// along `d`: sharp, it is an edge; smooth, it is a tangent edge — or, where
/// the faces turn away from the eye along it, their silhouette.
fn edge_kind<S: Scalar>(
    model: &Model<S>,
    edge_id: EdgeId,
    coedges: &[CoedgeId],
    d: &Vector3<S>,
) -> GeopResult<LineKind> {
    let [a, b] = coedges else {
        return Ok(LineKind::Edge);
    };
    let curve = &model.get_edge(edge_id)?.curve;
    let (t0, t1) = curve.domain();
    let point = curve.evaluate(t0.add(t1).div(S::TWO)?.sharpen())?;
    let normal = |coedge_id: CoedgeId| -> GeopResult<Vector3<S>> {
        let coedge = model.get_coedge(coedge_id)?;
        let surface = &model.get_face(coedge.face)?.surface;
        let (s0, s1) = coedge.pcurve.domain();
        let seed = coedge.pcurve.evaluate(s0.add(s1).div(S::TWO)?.sharpen())?;
        let (u, v) = surface.project(point, seed[0], seed[1], PROJECT_ITERATIONS)?;
        surface.normal(u, v)
    };
    let (Ok(na), Ok(nb)) = (normal(*a), normal(*b)) else {
        return Ok(LineKind::Edge);
    };
    if !na.prod_cross(&nb).could_be_equal(&Vector3::zero()) {
        return Ok(LineKind::Edge);
    }
    Ok(if na.prod_dot(d).could_be_equal(S::ZERO) {
        LineKind::Silhouette
    } else {
        LineKind::Tangent
    })
}

/// A face that can hide something, with the box holding it.
pub(crate) struct Occluder<S: Scalar> {
    face: FaceId,
    surface: NurbSurface3D<S>,
    /// `[lo, hi]` per axis, holding every control point's whole enclosure.
    pub(crate) bounds: [[f64; 2]; 3],
}

impl<S: Scalar> Occluder<S> {
    pub(crate) fn of(model: &Model<S>, face: FaceId) -> GeopResult<Self> {
        let surface = model.get_face(face)?.surface.clone();
        let mut bounds = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
        for q in &surface.control_points {
            for (k, b) in bounds.iter_mut().enumerate() {
                let x = q[k].div(q[3])?;
                b[0] = b[0].min(x.lower().to_f64());
                b[1] = b[1].max(x.upper().to_f64());
            }
        }
        Ok(Occluder {
            face,
            surface,
            bounds,
        })
    }

    /// Whether the segment from `a` to `b` could meet the box.
    fn could_meet(&self, a: &Vector3<S>, b: &Vector3<S>) -> bool {
        (0..3).all(|k| {
            let (x, y) = (a[k], b[k]);
            let lo = x.lower().min(y.lower()).to_f64();
            let hi = x.upper().max(y.upper()).to_f64();
            lo <= self.bounds[k][1] && self.bounds[k][0] <= hi
        })
    }
}

/// Whether `point` is seen from the eye: whether a ray from it towards the
/// eye (`toward_eye`, `length` long) leaves every face of `occluders`
/// behind. A crossing at the point itself — it lies on the faces it bounds
/// — is no crossing; one right on a face's trim boundary means an edge is
/// in front (see the module docs) and counts as hiding it.
pub(crate) fn seen<S: Scalar>(
    model: &Model<S>,
    occluders: &[Occluder<S>],
    point: Vector3<S>,
    toward_eye: &Vector3<S>,
    length: S,
) -> GeopResult<bool> {
    let far = point.add(&toward_eye.prod_scalar(length));
    let ray = line3(point, far)?;
    let mut behind_an_edge = false;
    for occluder in occluders.iter().filter(|o| o.could_meet(&point, &far)) {
        let (face_id, surface) = (occluder.face, &occluder.surface);
        let ctx =
            |e: GeopError| e.with_context(format!("the ray from {point:?} against face {face_id}"));
        let overlaps = curve_surface_overlaps(&ray, surface, MAX_NODES, min_subdivision_size())
            .with_context(&ctx)?;
        if !overlaps.is_empty() {
            // The ray runs in the face's surface: the face is seen edge on
            // here, and an edge-on face hides nothing.
            continue;
        }
        let crossings = curve_surface_crossings(&ray, surface, MAX_NODES, min_subdivision_size())
            .with_context(&ctx)?;
        for (t, uv) in crossings {
            let (t, uv) = if t.definitely_greater(S::ZERO) {
                (t, uv)
            } else {
                refine_crossing(&ray, surface, t, uv)
            };
            if !t.definitely_greater(S::ZERO) {
                continue;
            }
            match face_contains(
                model,
                face_id,
                uv[0].midpoint(),
                uv[1].midpoint(),
                MAX_NODES,
                min_subdivision_size(),
                FACE_CONTAINS_SEED,
            )
            .with_context(&ctx)?
            {
                PointClassification::Inside => return Ok(false),
                PointClassification::Outside => {}
                PointClassification::OnCoedge | PointClassification::OnVertex => {
                    behind_an_edge = true
                }
            }
        }
    }
    Ok(!behind_an_edge)
}

/// The ray length that clears every occluder from anywhere on them: twice
/// the diagonal of their boxes' box.
pub(crate) fn ray_length<'o, S: Scalar>(
    occluders: impl IntoIterator<Item = &'o Occluder<S>>,
) -> S {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for o in occluders {
        for k in 0..3 {
            lo[k] = lo[k].min(o.bounds[k][0]);
            hi[k] = hi[k].max(o.bounds[k][1]);
        }
    }
    let d2: f64 = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum();
    S::from_f64(2.0 * d2.sqrt() + 1.0)
}

/// The parameter boxes in `boxes` merged where they overlap, as sharp cut
/// points strictly inside `(t0, t1)`: where to cut is a free choice within
/// each.
pub(crate) fn cut_points<S: Scalar>(mut boxes: Vec<S>, (t0, t1): (S, S)) -> Vec<S> {
    boxes.sort_by(|a, b| a.lower().to_f64().total_cmp(&b.lower().to_f64()));
    let mut merged: Vec<S> = Vec::new();
    for b in boxes {
        match merged.last_mut() {
            Some(last) if last.could_be_equal(b) => *last = last.union(b),
            _ => merged.push(b),
        }
    }
    merged
        .into_iter()
        .filter(|b| b.definitely_greater(t0) && b.definitely_less(t1))
        .map(|b| b.midpoint())
        .collect()
}

/// The lines of the faces `faces` of `model` seen through `frame`, each
/// piece visible or hidden. Every face both draws and hides.
pub fn project_view<S: Scalar>(
    model: &Model<S>,
    faces: &[FaceId],
    frame: &ViewFrame<S>,
    options: &ViewOptions,
) -> GeopResult<ProjectedView<S>> {
    let d = frame.direction.vector();
    // Each curve with the parameters it is cut at before anything else.
    let mut sources: Vec<(Source<S>, Vec<S>)> = Vec::new();
    let mut add = |curve3: NurbCurve3D<S>,
                   kind: LineKind,
                   edge: Option<EdgeId>,
                   silhouette: Option<Silhouette<S>>,
                   on: Vec<FaceId>,
                   cuts: Vec<S>|
     -> GeopResult<()> {
        let curve = frame.project_curve(&curve3)?;
        if is_point(&curve)? {
            return Ok(());
        }
        let bounds = bounds(&curve)?;
        sources.push((
            Source {
                curve3,
                curve,
                kind,
                edge,
                silhouette,
                on,
                origin: sources.len(),
                bounds,
            },
            cuts,
        ));
        Ok(())
    };
    for (edge_id, coedges) in edge_coedges(model, faces) {
        let ctx = |e: GeopError| e.with_context(format!("project_view: edge {edge_id}"));
        let kind = edge_kind(model, edge_id, &coedges, &d).with_context(&ctx)?;
        if kind == LineKind::Tangent && !options.tangent_edges {
            continue;
        }
        let curve = model.get_edge(edge_id)?.curve.clone();
        // Where it crosses the silhouettes of the faces it bounds.
        let mut cuts = Vec::new();
        let mut on = Vec::new();
        for &coedge_id in &coedges {
            let face = model.get_coedge(coedge_id)?.face;
            on.push(face);
            cuts.extend(edge_crossings(model, face, coedge_id, &curve, &d).with_context(&ctx)?);
        }
        add(curve, kind, Some(edge_id), None, on, cuts).with_context(&ctx)?;
    }
    for &face_id in faces {
        for silhouette in face_silhouettes(model, face_id, &d)? {
            add(
                silhouette.curve.clone(),
                LineKind::Silhouette,
                None,
                Some(silhouette),
                vec![face_id],
                Vec::new(),
            )?;
        }
    }

    // Every curve cut where it is stationary along either axis of the
    // paper, before the curves are compared: then each runs one way along
    // both, and a curve seen edge on — its projection a segment it runs
    // back and forth along — becomes segments that overlap others plainly
    // instead of folding onto themselves.
    let axes = [
        Vector2::from_array([S::ONE, S::ZERO]),
        Vector2::from_array([S::ZERO, S::ONE]),
    ];
    let mut monotone = Vec::with_capacity(sources.len());
    for (source, mut stationary) in sources {
        for axis in &axes {
            stationary.extend(
                source
                    .curve
                    .stationary_parameters(axis, MAX_NODES, min_subdivision_size())
                    .map_err(|e| {
                        e.with_context(format!(
                            "project_view: the {:?} of edge {:?}",
                            source.kind, source.edge
                        ))
                    })?,
            );
        }
        let domain = source.curve.domain();
        let mut ends = vec![domain.0];
        ends.extend(cut_points(stationary, domain));
        ends.push(domain.1);
        if ends.len() == 2 {
            monotone.push(source);
            continue;
        }
        for pair in ends.windows(2) {
            let curve = source.curve.sub_curve(pair[0], pair[1])?;
            if is_point(&curve)? {
                continue;
            }
            monotone.push(Source {
                curve3: source.curve3.sub_curve(pair[0], pair[1])?,
                bounds: bounds(&curve)?,
                curve,
                kind: source.kind,
                edge: source.edge,
                silhouette: source.silhouette.clone(),
                on: source.on.clone(),
                origin: source.origin,
            });
        }
    }
    let sources = monotone;

    // Where each piece has to be cut.
    let mut cuts: Vec<Vec<S>> = vec![Vec::new(); sources.len()];
    for i in 0..sources.len() {
        for j in i + 1..sources.len() {
            if !boxes_meet(&sources[i].bounds, &sources[j].bounds) {
                continue;
            }
            // An edge meets the silhouette of a face it bounds where it
            // crosses it on the face — found already, and cut at — and
            // touches it there on the paper, tangentially: no search
            // here could isolate that.
            let bounds_silhouette = |a: &Source<S>, b: &Source<S>| {
                a.edge.is_some()
                    && b.kind == LineKind::Silhouette
                    && b.edge.is_none()
                    && b.on.iter().any(|f| a.on.contains(f))
            };
            if bounds_silhouette(&sources[i], &sources[j])
                || bounds_silhouette(&sources[j], &sources[i])
            {
                continue;
            }
            let (a, b) = (&sources[i], &sources[j]);
            let (a_cuts, b_cuts) = meetings((&a.curve, &a.curve3), (&b.curve, &b.curve3)).map_err(|e| {
                let describe = |s: &Source<S>| match s.edge {
                    Some(edge) => format!("{:?} of edge {edge}", s.kind),
                    None => format!(
                        "{:?} of face {:?}",
                        s.kind,
                        s.silhouette.as_ref().map(|s| s.face)
                    ),
                };
                e.with_context(format!(
                    "project_view: where the {} and the {} cross on the paper",
                    describe(&sources[i]),
                    describe(&sources[j])
                ))
            })?;
            cuts[i].extend(a_cuts);
            cuts[j].extend(b_cuts);
        }
    }

    // Each piece, decided by its middle.
    let toward_eye = frame.toward_eye();
    let occluders = faces
        .iter()
        .map(|&f| Occluder::of(model, f))
        .collect::<GeopResult<Vec<_>>>()?;
    let length = ray_length(&occluders);
    // Every piece, in order along each curve: its source, its range, and
    // its visibility or why no point of it could be decided.
    let mut pieces: Vec<(usize, S, S, Result<bool, GeopError>)> = Vec::new();
    for (index, (source, cuts)) in sources.iter().zip(cuts).enumerate() {
        let domain = source.curve.domain();
        let mut ends = vec![domain.0];
        ends.extend(cut_points(cuts, domain));
        ends.push(domain.1);
        for pair in ends.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "project_view: the piece [{a:?}, {b:?}] of a line of kind {:?} (edge {:?})",
                    source.kind, source.edge
                ))
            };
            // Any point of the piece decides it; the middle first, and two
            // more should a ray from there graze a face too closely for
            // its search to settle — as one does from where an edge
            // crosses the silhouette of a face it bounds, the ray leaving
            // tangent to that face.
            let mut outcome = Err(GeopError::new("no point tried"));
            for f in [0.5, 0.25, 0.75] {
                let decide = || -> GeopResult<bool> {
                    let t = a.add(b.sub(a).mul(S::from_f64(f))).sharpen();
                    let mut point = source.curve3.evaluate(t)?;
                    if let Some(silhouette) = &source.silhouette {
                        point = silhouette.exact_point(model, &d, &point)?;
                    }
                    seen(model, &occluders, point, &toward_eye, length)
                };
                outcome = decide().map_err(ctx);
                if outcome.is_ok() {
                    break;
                }
            }
            pieces.push((index, a, b, outcome));
        }
    }
    // A piece none of whose points could be decided is a sliver next to
    // such a graze: it takes the visibility of its nearest neighbour along
    // the same curve.
    let known: Vec<Option<bool>> = pieces.iter().map(|p| p.3.as_ref().ok().copied()).collect();
    let origin = |k: usize| sources[pieces[k].0].origin;
    let mut lines = Vec::new();
    for k in 0..pieces.len() {
        let visible = match known[k] {
            Some(v) => v,
            None => {
                let along = |i: usize| (origin(i) == origin(k)).then_some(known[i]).flatten();
                let neighbour = (1..pieces.len()).find_map(|step| {
                    let before = k.checked_sub(step).and_then(along);
                    before.or_else(|| (k + step < pieces.len()).then(|| along(k + step)).flatten())
                });
                match neighbour {
                    Some(v) => v,
                    None => {
                        let (_, _, _, outcome) = pieces.swap_remove(k);
                        return Err(outcome.unwrap_err());
                    }
                }
            }
        };
        let (index, a, b, _) = &pieces[k];
        let source = &sources[*index];
        if !visible && !options.hidden_lines {
            continue;
        }
        lines.push(ViewLine {
            curve: source.curve.sub_curve(*a, *b)?,
            curve3: source.curve3.sub_curve(*a, *b)?,
            kind: source.kind,
            visible,
            edge: source.edge,
            body: 0,
            source: *index,
            range: (*a, *b),
        });
    }

    Ok(ProjectedView {
        frame: *frame,
        lines: rejoin(&sources, drop_drawn(lines, |_, _| true)?)?,
    })
}

/// Where the projections of `a` and `b` cross or begin and end running
/// along each other: the parameters to cut each at.
/// `handoff` is where the search hands over (see `AGENTS.md`).
fn search<S: Scalar>(
    a: &NurbCurve2D<S>,
    b: &NurbCurve2D<S>,
    handoff: S,
) -> GeopResult<(Vec<S>, Vec<S>)> {
    let (overlaps, crossings) = curve_curve_overlaps_and_crossings(a, b, MAX_NODES, handoff)?;
    let (mut on_a, mut on_b) = (Vec::new(), Vec::new());
    for (s, t) in crossings {
        on_a.push(s);
        on_b.push(t);
    }
    for o in overlaps {
        on_a.extend([o.start.t, o.end.t]);
        on_b.extend([o.start.partner, o.end.partner]);
    }
    Ok((on_a, on_b))
}

/// Where the pieces `a` and `b`, each on the paper and in space, have to
/// be cut for each other (see [`search`]).
///
/// Two curves meeting at a common end often meet there tangentially on the
/// paper — an edge running into another smoothly, seen from the side —
/// and a subdivision search cannot isolate a tangency: it runs out of
/// nodes. Their common end is an end of both pieces already, so nothing
/// needs cutting there. Should the search fail, it is run again on the
/// halves of the two, leaving out the two halves that meet at the common
/// end. A crossing within those two halves, other than at the common end,
/// would go unseen: a known limit, taken rather than failing the view.
///
/// Two curves touching elsewhere — a screw's head seen from above, round,
/// touching the line a flat of the nut under it is seen as — come closer
/// than a fine handoff along a stretch the search cannot get through
/// either. Should all else fail, the search is run once more with a
/// coarser handoff ([`COARSE_SUBDIVISION`]): the touch comes out as one
/// wide cluster, cut at a free point inside it, and two crossings closer
/// together than that are taken as one.
pub(crate) fn meetings<S: Scalar>(
    (a, a3): (&NurbCurve2D<S>, &NurbCurve3D<S>),
    (b, b3): (&NurbCurve2D<S>, &NurbCurve3D<S>),
) -> GeopResult<(Vec<S>, Vec<S>)> {
    let fine = || -> GeopResult<(Vec<S>, Vec<S>)> {
        let error = match search(a, b, min_subdivision_size()) {
            Ok(found) => return Ok(found),
            Err(e) => e,
        };
        let ends = |c: &NurbCurve3D<S>| -> GeopResult<[Vector3<S>; 2]> {
            let (t0, t1) = c.domain();
            Ok([c.evaluate(t0)?, c.evaluate(t1)?])
        };
        let (ea, eb) = (ends(a3)?, ends(b3)?);
        let shared = (0..2)
            .flat_map(|i| (0..2).map(move |j| (i, j)))
            .find(|&(i, j)| ea[i].could_be_equal(&eb[j]));
        let ends_context = format!("their ends in space: {ea:?} and {eb:?}");
        let Some((end_a, end_b)) = shared else {
            return Err(error.with_context(ends_context));
        };
        let error = error.with_context(ends_context);
        let halves = |c: &NurbCurve2D<S>| -> GeopResult<[NurbCurve2D<S>; 2]> {
            let (l, r) = c.split_mid()?;
            Ok([l, r])
        };
        let (ha, hb) = (halves(a)?, halves(b)?);
        let (mut on_a, mut on_b) = (Vec::new(), Vec::new());
        for (i, half_a) in ha.iter().enumerate() {
            for (j, half_b) in hb.iter().enumerate() {
                if i == end_a && j == end_b {
                    continue;
                }
                let (x, y) = search(half_a, half_b, min_subdivision_size()).map_err(|e| {
                    e.with_context(format!(
                        "searched again without the common end, after: {error}"
                    ))
                })?;
                on_a.extend(x);
                on_b.extend(y);
            }
        }
        Ok((on_a, on_b))
    };
    fine().or_else(|error| {
        search(a, b, S::from_f64(COARSE_SUBDIVISION)).map_err(|e| {
            e.with_context(format!(
                "searched again with a coarser handoff, after: {error}"
            ))
        })
    })
}

/// `lines` with the pieces of one curve that follow on from each other, and
/// are both visible or both hidden, joined again: the cuts that did not
/// change anything undone, so a circle seen round stays a circle.
fn rejoin<S: Scalar>(
    sources: &[Source<S>],
    mut lines: Vec<ViewLine<S>>,
) -> GeopResult<Vec<ViewLine<S>>> {
    lines.sort_by(|a, b| {
        (a.source, a.range.0.to_f64())
            .partial_cmp(&(b.source, b.range.0.to_f64()))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut joined: Vec<ViewLine<S>> = Vec::new();
    for line in lines {
        if let Some(last) = joined.last_mut()
            && last.source == line.source
            && last.visible == line.visible
            && last.range.1.could_be_equal(line.range.0)
        {
            last.range.1 = line.range.1;
            continue;
        }
        joined.push(line);
    }
    // A closed curve cut at its start joins across it, too.
    let mut out: Vec<ViewLine<S>> = Vec::new();
    for mut line in joined {
        let source = &sources[line.source];
        let (t0, t1) = source.curve.domain();
        let whole = line.range.0.could_be_equal(t0) && line.range.1.could_be_equal(t1);
        if whole {
            line.curve = source.curve.clone();
            line.curve3 = source.curve3.clone();
        } else {
            line.curve = source.curve.sub_curve(line.range.0, line.range.1)?;
            line.curve3 = source.curve3.sub_curve(line.range.0, line.range.1)?;
        }
        out.push(line);
    }
    Ok(out)
}

/// `lines` without every piece lying on one drawn before it — visible ones
/// first, so a hidden line under a visible one goes — of the pieces
/// `compare` says to compare it with. Pieces are cut wherever their
/// projections meet, so a piece lies on another all along or only touches
/// it at its ends: its middle tells which.
pub(crate) fn drop_drawn<S: Scalar>(
    mut lines: Vec<ViewLine<S>>,
    compare: impl Fn(&ViewLine<S>, &ViewLine<S>) -> bool,
) -> GeopResult<Vec<ViewLine<S>>> {
    lines.sort_by_key(|l| !l.visible);
    let mut kept: Vec<(ViewLine<S>, [f64; 4])> = Vec::new();
    for line in lines {
        let (a, b) = line.curve.domain();
        let mid = line.curve.evaluate(a.add(b).div(S::TWO)?.sharpen())?;
        let here = bounds(&line.curve)?;
        let mut drawn = false;
        for (other, there) in &kept {
            if boxes_meet(&here, there)
                && compare(&line, other)
                && curve_could_contain(&other.curve, &mid, MAX_NODES, min_subdivision_size())?
                    .is_some()
            {
                drawn = true;
                break;
            }
        }
        if !drawn {
            kept.push((line, here));
        }
    }
    Ok(kept.into_iter().map(|(l, _)| l).collect())
}
