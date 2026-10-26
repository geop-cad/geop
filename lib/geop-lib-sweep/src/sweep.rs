//! Sweeping planar profiles along a [`Path`] — straight, for an extrude,
//! around an axis, for a revolve, along a chain of curves, for a sweep (see
//! [`crate::path_sweep`]) — into a solid or a sheet, described whole as a
//! [`BodySpec`] and built in one go (see [`geop_core_topology::build`]).
//! [`skin`] builds the same grid through a different section at every
//! station, as a loft does (see [`crate::loft`]). The grid itself — its
//! vertices, edges, walls and caps, and their names — is built by
//! [`crate::skin`], as a blend's tool's is; what is the
//! sweep's own is where its sections are: placed by the path's frames.
//!
//! A sweep is a grid: every profile joint sits at every station of the path
//! (a vertex), every profile curve lies at every station (an edge), every
//! joint travels along every span between two stations (a *lateral* edge),
//! and every curve sweeps a *wall* face through every span. A solid swept
//! along an open path is closed off by two flat *caps*, at the first and
//! last station; along a closed path — a full turn — it needs none.
//!
//! A revolve's axis makes two things degenerate. A joint on the axis is a
//! *pole*: it stays put, so it is a single vertex at every station, with no
//! lateral edges — the walls next to it close over it with a degenerate
//! coedge sitting at the pole (see [`CoedgeOn::Vertex`]). A curve lying on
//! the axis sweeps no wall at all; with caps, it is the one edge both caps
//! share.
//!
//! **Parametrization.** A wall is the tensor product of its span (`u`, from
//! one station to the next: degree 1 for a line, a rational quadratic for an
//! arc, a B-spline for any other curve) and its curve (`v`, the curve's own
//! degree and knots). So its normal `∂u × ∂v` points out of the material
//! exactly when the profile runs counter-clockwise in its stations' `(e1,
//! e2)` and the path runs along `-(e1 × e2)` — or clockwise, along `e1 ×
//! e2` — see [`Path::along_normal`].
//! Orienting the profile accordingly is the caller's part; [`sweep`] builds
//! the caps to match.

use std::collections::BTreeMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D, embed_point},
    nurb_surface::{FlatPatch, NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::build::{BodySpec, BuiltBody, CoedgeOn};
use geop_ops_model::{BodyNames, NamedModel, Namer, Profile};

use crate::skin::{self, Strip, StripCap, StripGeometry};

/// Where the profile's plane is at one station of a path: a profile point
/// `(x, y)` lies at `origin + x e1 + y e2`.
#[derive(Clone, Debug)]
pub struct Frame {
    pub origin: Vector3,
    pub e1: Vector3,
    pub e2: Vector3,
}

impl Frame {
    /// The profile point `p` at this station.
    pub fn point(&self, p: &Vector2) -> Vector3 {
        self.origin
            .add(&self.e1.prod_scalar(p[0]))
            .add(&self.e2.prod_scalar(p[1]))
    }

    /// The profile curve `curve` at this station. An affine map of its
    /// control points, so exact for any NURBS.
    pub fn curve(&self, curve: &NurbCurve2D) -> GeopResult<NurbCurve3D> {
        curve.embed(&self.origin, &self.e1, &self.e2)
    }
}

/// How the profile travels from one station to the next.
#[derive(Clone, Debug)]
pub enum Span {
    /// In a straight line: degree 1.
    Line,
    /// Along a NURBS of `degree` on the clamped `knots` over `[0, 1]`, whose
    /// inner control rows are the span's first section placed by each of
    /// `middle`, weighted by its weight (none: one): along circular arcs —
    /// a rational quadratic (see [`Span::arc`]) — translated along a curve,
    /// or along any other path, approximated by sections of the profile
    /// skinned together (see [`crate::path_sweep`]).
    Curve {
        degree: usize,
        knots: Vec<Scalar>,
        middle: Vec<(Frame, Option<Scalar>)>,
    },
    /// From one section to another of a different shape, along a NURBS of
    /// `degree` on the clamped `knots` over `[0, 1]`, each inner control row
    /// the sum of the span's first section placed by the first frame and
    /// its last section placed by the second, each times its weight: the
    /// interior of a loft shaped by guide curves (see [`crate::loft`]).
    Blend {
        degree: usize,
        knots: Vec<Scalar>,
        middle: Vec<[(Frame, Scalar); 2]>,
    },
    /// Along a NURBS of `degree` on the clamped `knots` over `[0, 1]`, each
    /// inner control row a sum of the sections at any stations — each
    /// placed by its own station's frame, times its coefficient: a loft
    /// running smoothly through its sections, its tangent at each taken
    /// from its neighbours (see [`crate::loft`]).
    Through {
        degree: usize,
        knots: Vec<Scalar>,
        middle: Vec<Vec<(usize, Scalar)>>,
    },
}

impl Span {
    /// Along circular arcs: a rational quadratic whose middle control row is
    /// the profile at `middle`, weighted `weight`.
    pub fn arc(middle: Frame, weight: Scalar) -> Self {
        Span::Curve {
            degree: 2,
            knots: vec![
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ONE,
                Scalar::ONE,
                Scalar::ONE,
            ],
            middle: vec![(middle, Some(weight))],
        }
    }
}

/// One control row of a span (see [`Path::rows`]).
enum Row<'a> {
    /// The span's first section, placed by a frame, its weight multiplied
    /// by the weight given, if any.
    First(&'a Frame, Option<Scalar>),
    /// The span's last section, placed by a frame.
    Last(&'a Frame),
    /// The sum of the first section placed by one frame and the last placed
    /// by another, each times its weight.
    Both(&'a [(Frame, Scalar); 2]),
    /// The sum of the sections at the stations given, each placed by its
    /// own station's frame, times its coefficient.
    Stations(&'a [(usize, Scalar)]),
}

impl Row<'_> {
    /// The row's control point for the homogeneous profile point `at[s]`
    /// of the section at every station `s` of `stations`, in span `(a, b)`:
    /// from station `a` to station `b`.
    fn point(&self, stations: &[Frame], (a, b): (usize, usize), at: &[Vector3]) -> Vector4 {
        let place =
            |p: &Vector3, frame: &Frame| embed_point(p, &frame.origin, &frame.e1, &frame.e2);
        match self {
            Row::First(frame, weight) => weighted(place(&at[a], frame), *weight),
            Row::Last(frame) => place(&at[b], frame),
            Row::Both([(fa, wa), (fb, wb)]) => {
                let (pa, pb) = (place(&at[a], fa), place(&at[b], fb));
                Vector4::from_array(std::array::from_fn(|k| pa[k].mul(*wa).add(pb[k].mul(*wb))))
            }
            Row::Stations(terms) => {
                // A coefficient of one multiplies nothing, and the sum starts
                // at the first term, not at zero: rounding by either would
                // only widen the row.
                let term = |&(s, coefficient): &(usize, Scalar)| -> Vector4 {
                    let p = place(&at[s], &stations[s]);
                    if coefficient.is_exactly(Scalar::ONE) {
                        p
                    } else {
                        Vector4::from_array(std::array::from_fn(|k| p[k].mul(coefficient)))
                    }
                };
                let mut terms = terms.iter().map(term);
                let first = terms.next().expect("a row of at least one section");
                terms.fold(first, |sum, t| {
                    Vector4::from_array(std::array::from_fn(|k| sum[k].add(t[k])))
                })
            }
        }
    }
}

/// What a profile is swept along: its stations, and the spans between
/// consecutive ones — around to the first again for a `closed` path (a full
/// turn), which has as many spans as stations; an open one has one fewer.
#[derive(Clone, Debug)]
pub struct Path {
    pub stations: Vec<Frame>,
    pub spans: Vec<Span>,
    pub closed: bool,
    /// Whether the path runs along `e1 × e2` of its stations rather than
    /// against it — which way the profile must wind (see the module docs).
    pub along_normal: bool,
    /// What each station is called in the names of what lies on it: `start`
    /// and `end`, or `a0`, `a1`, ...
    pub station_names: Vec<String>,
    /// What each span is called in the names of what it sweeps — `q0`, `q1`,
    /// ... — or nothing, for a path of one span that needs no telling apart.
    pub span_names: Vec<Option<String>>,
}

impl Path {
    /// Station `j`, counting on around a closed path.
    fn station(&self, j: usize) -> usize {
        if self.closed {
            j % self.stations.len()
        } else {
            j
        }
    }

    /// The control rows of span `j`: its first station's, its inner rows,
    /// its last station's. A station's row has no weight: multiplying by
    /// one would only widen every enclosure by rounding.
    fn rows(&self, j: usize) -> Vec<Row<'_>> {
        let (a, b) = (&self.stations[j], &self.stations[self.station(j + 1)]);
        let middle: Vec<Row<'_>> = match &self.spans[j] {
            Span::Line => Vec::new(),
            Span::Curve { middle, .. } => middle.iter().map(|(m, w)| Row::First(m, *w)).collect(),
            Span::Blend { middle, .. } => middle.iter().map(Row::Both).collect(),
            Span::Through { middle, .. } => middle.iter().map(|m| Row::Stations(m)).collect(),
        };
        std::iter::once(Row::First(a, None))
            .chain(middle)
            .chain(std::iter::once(Row::Last(b)))
            .collect()
    }

    fn degree(&self, j: usize) -> usize {
        match &self.spans[j] {
            Span::Line => 1,
            Span::Curve { degree, .. }
            | Span::Blend { degree, .. }
            | Span::Through { degree, .. } => *degree,
        }
    }

    fn knots(&self, j: usize) -> Vec<Scalar> {
        match &self.spans[j] {
            Span::Line => vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            Span::Curve { knots, .. } | Span::Blend { knots, .. } | Span::Through { knots, .. } => {
                knots.clone()
            }
        }
    }

    /// The stations span `j` runs between.
    fn ends(&self, j: usize) -> (usize, usize) {
        (j, self.station(j + 1))
    }

    /// The path along span `j` of a profile point, `points[s]` in the
    /// section at station `s` (see [`skin`]).
    fn lateral(&self, j: usize, points: &[Vector2]) -> GeopResult<NurbCurve3D> {
        let at: Vec<Vector3> = points
            .iter()
            .map(|p| Vector3::from_array([p[0], p[1], Scalar::ONE]))
            .collect();
        let control_points = self
            .rows(j)
            .iter()
            .map(|row| row.point(&self.stations, self.ends(j), &at))
            .collect();
        NurbCurve::try_new(self.degree(j), control_points, self.knots(j))
    }

    /// The wall a profile curve sweeps through span `j`, `curves[s]` in the
    /// section at station `s` — compatible curves: `u` along the span, `v`
    /// along the curve.
    fn wall(&self, j: usize, curves: &[&NurbCurve2D]) -> GeopResult<NurbSurface3D> {
        let a = curves[j];
        let rows = self.rows(j);
        let mut control_points = Vec::with_capacity(rows.len() * a.control_points.len());
        for row in &rows {
            for k in 0..a.control_points.len() {
                let at: Vec<Vector3> = curves.iter().map(|c| c.control_points[k]).collect();
                control_points.push(row.point(&self.stations, self.ends(j), &at));
            }
        }
        NurbSurface::try_new(
            self.degree(j),
            a.degree,
            control_points,
            self.knots(j),
            a.knot_vector.clone(),
        )
    }

    fn check(&self) -> GeopResult<()> {
        let stations = self.stations.len();
        let spans = self.spans.len();
        let expected = if self.closed { stations } else { stations - 1 };
        if spans == 0 || spans != expected {
            return Err(GeopError::new(format!(
                "sweep: a {} path of {stations} stations cannot have {spans} spans",
                if self.closed { "closed" } else { "open" }
            )));
        }
        if self.station_names.len() != stations || self.span_names.len() != spans {
            return Err(GeopError::new("sweep: a name for every station and span"));
        }
        Ok(())
    }
}

/// The homogeneous point `p` with its weight multiplied by `weight`, if any.
fn weighted(p: Vector4, weight: Option<Scalar>) -> Vector4 {
    match weight {
        None => p,
        Some(w) => Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), p[3].mul(w)]),
    }
}

/// One profile loop or chain to sweep, and which of its joints are poles
/// and which of its curves lie on the axis (see the module docs). A curve
/// on the axis runs between two poles.
#[derive(Clone, Debug)]
pub struct SweepLoop {
    pub profile: Profile,
    /// Per joint: whether it lies on the axis.
    pub poles: Vec<bool>,
    /// Per curve: whether it lies along the axis.
    pub on_axis: Vec<bool>,
    /// Whether the loop is collapsed to one point, every curve degenerate
    /// there: the apex a loft to a point comes to (see [`skin`]).
    pub apex: bool,
}

impl SweepLoop {
    /// `profile`, clear of any axis.
    pub fn plain(profile: Profile) -> Self {
        Self {
            poles: vec![false; profile.joint_names.len()],
            on_axis: vec![false; profile.curves.len()],
            profile,
            apex: false,
        }
    }

    /// The same loop traversed the other way; every flag stays with its
    /// curve or joint (see [`Profile::reversed`]).
    pub fn reversed(&self) -> Self {
        let n = self.profile.curves.len();
        let poles = if self.profile.is_closed() {
            (0..n).map(|m| self.poles[(n - m) % n]).collect()
        } else {
            self.poles.iter().rev().copied().collect()
        };
        Self {
            profile: self.profile.reversed(),
            poles,
            on_axis: self.on_axis.iter().rev().copied().collect(),
            apex: self.apex,
        }
    }

    fn joints(&self) -> usize {
        self.profile.joint_names.len()
    }

    /// The joint curve `i` ends at.
    fn next(&self, i: usize) -> usize {
        (i + 1) % self.joints()
    }

    /// Checks the profile is a chain of clamped curves on `[0, 1]`, each
    /// starting where the previous one ends — a closed one of at least two,
    /// since every vertex sits at a joint — with a flag for every joint and
    /// curve, and every curve on the axis between two poles.
    fn check(&self) -> GeopResult<()> {
        let profile = &self.profile;
        profile.check_names()?;
        let curves = &profile.curves;
        let n = curves.len();
        if n == 0 || (profile.is_closed() && n < 2) {
            return Err(GeopError::new(format!(
                "sweep: a profile of {n} curve(s) is too short"
            )));
        }
        if self.poles.len() != self.joints() || self.on_axis.len() != n {
            return Err(GeopError::new("sweep: a flag for every joint and curve"));
        }
        for (i, curve) in curves.iter().enumerate() {
            let (t0, t1) = curve.domain();
            if !(t0.could_be_equal(Scalar::ZERO) && t1.could_be_equal(Scalar::ONE)) {
                return Err(GeopError::new(format!(
                    "sweep: curve {} has domain ({t0:?}, {t1:?}), not [0, 1]",
                    profile.curve_names[i]
                )));
            }
            if i + 1 < n || profile.is_closed() {
                let next = &curves[(i + 1) % n];
                let (end, start) = (curve.end()?, next.start()?);
                if !end.could_be_equal(&start) {
                    return Err(GeopError::new(format!(
                        "sweep: curve {} ends at {end:?}, but the next one starts at {start:?}",
                        profile.curve_names[i]
                    )));
                }
            }
            if self.on_axis[i] && !(self.poles[i] && self.poles[self.next(i)]) {
                return Err(GeopError::new(format!(
                    "sweep: curve {} lies on the axis, but its ends are not poles",
                    profile.curve_names[i]
                )));
            }
        }
        Ok(())
    }
}

/// Sweeps `loops` along `path`: into a solid named `solid`, or, without
/// one, into sheets — a shell of walls per loop, with no caps.
///
/// A solid's loops are closed, the first its outer boundary and the rest
/// holes, each oriented as the module docs say; along an open path it gets
/// a cap at either end, `N(start)` and `N(end)`. The rest is named after the
/// profile's curves `X` and joints `P` (see [`Profile`]) and the path's
/// station names `s` and span names `q`:
///
/// | entity | name |
/// |---|---|
/// | wall swept by `X` through span `q` | `N(X,q)`, or `N(X)` for an unnamed span |
/// | `X` at station `s` | `N(X,s)`; `N(X)` for a curve on the axis |
/// | lateral edge swept by `P` through `q` | `N(P,q)`, or `N(P)` |
/// | vertex of `P` at `s` | `N(P,s)`; `N(P)` for a pole |
///
/// The solid's shells are the connected sets of faces: one, unless a full
/// turn leaves a hole — or a pocket closed off by the axis — sweeping a void
/// of its own.
pub fn sweep(
    model: &mut NamedModel,
    namer: &Namer,
    path: &Path,
    loops: &[SweepLoop],
    solid: Option<&str>,
) -> GeopResult<BuiltBody> {
    let sections = vec![loops.to_vec(); path.stations.len()];
    skin(model, namer, path, &sections, solid)
}

/// [`sweep`]s `loops` as they are drawn — the outer loop counter-clockwise
/// in the stations' `(e1, e2)`, the holes clockwise — each run the other
/// way first if the path runs along `e1 × e2`, where a counter-clockwise
/// outer loop would sweep walls facing inwards.
pub fn sweep_drawn(
    model: &mut NamedModel,
    namer: &Namer,
    path: &Path,
    loops: &[SweepLoop],
    solid: Option<&str>,
) -> GeopResult<BuiltBody> {
    if path.along_normal {
        let reversed: Vec<SweepLoop> = loops.iter().map(SweepLoop::reversed).collect();
        sweep(model, namer, path, &reversed, solid)
    } else {
        sweep(model, namer, path, loops, solid)
    }
}

/// Like [`sweep`], but with a section of its own at every station of
/// `path`: `sections[s]` is what lies at station `s`, in its frame — as a
/// loft passes through profiles of different shapes. Each section has the
/// same loops, each loop the same number of curves and the same flags, and
/// the `i`-th curves of all of them are compatible: of one degree, on one
/// knot vector (see `NurbCurve::compatible`). A wall then runs from one
/// section's curve to the next one's; a span's inner rows carry its first
/// station's section on.
///
/// Everything is named after the first section's curves and joints, as
/// [`sweep`] names it.
pub fn skin(
    model: &mut NamedModel,
    namer: &Namer,
    path: &Path,
    sections: &[Vec<SweepLoop>],
    solid: Option<&str>,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!(
        "skin({}, {} section(s) of {} loop(s), solid={solid:?})",
        namer.root(),
        sections.len(),
        sections.first().map_or(0, Vec::len)
    );
    path.check().with_context(ctx)?;
    check_sections(path, sections, solid.is_some()).with_context(ctx)?;
    let loops = &sections[0];
    let geometry: Vec<Placed<'_>> = (0..loops.len())
        .map(|l| Placed { path, sections, l })
        .collect();
    let apexes: Vec<Vec<bool>> = (0..loops.len())
        .map(|l| sections.iter().map(|section| section[l].apex).collect())
        .collect();
    let strips: Vec<Strip<'_>> = loops
        .iter()
        .zip(&geometry)
        .zip(&apexes)
        .map(|((lp, geometry), apex)| Strip {
            namer,
            joint_names: &lp.profile.joint_names,
            curve_names: &lp.profile.curve_names,
            station_names: &path.station_names,
            span_names: &path.span_names,
            closed: path.closed,
            poles: &lp.poles,
            on_axis: &lp.on_axis,
            apex,
            glued: Vec::new(),
            geometry,
        })
        .collect();
    let mut caps = Vec::new();
    if solid.is_some() && !path.closed {
        // Each parametrized so that it runs the outer loop counter-clockwise.
        for (s, forward, name) in [(0, true, "start"), (path.stations.len() - 1, false, "end")] {
            let (frame, section) = (&path.stations[s], &sections[s]);
            // A section collapsed to its apex needs no cap.
            if section.iter().any(|lp| lp.apex) {
                if section.len() > 1 {
                    return Err(GeopError::new(format!(
                        "skin: the section at {} comes to a point, and has holes",
                        path.station_names[s]
                    )))
                    .with_context(ctx);
                }
                continue;
            }
            let patch = FlatPatch::around(
                frame.origin,
                frame.e1,
                frame.e2,
                section.iter().flat_map(|l| &l.profile.curves),
                path.along_normal == forward,
            )?;
            let loops = section
                .iter()
                .enumerate()
                .map(|(l, lp)| {
                    let pcurves = lp
                        .profile
                        .curves
                        .iter()
                        .map(|c| patch.pcurve(c))
                        .collect::<GeopResult<_>>()?;
                    Ok((l, pcurves))
                })
                .collect::<GeopResult<_>>()?;
            caps.push(StripCap {
                name: namer.name(&[name]),
                station: s,
                forward,
                surface: patch.surface()?,
                loops,
            });
        }
    }

    let mut spec = BodySpec::new(solid.is_some());
    let mut names = BodyNames {
        solid: solid.map(str::to_string),
        ..BodyNames::default()
    };
    skin::skin(&mut spec, &mut names, &strips, &caps).with_context(ctx)?;
    spec.shells = connected_faces(&spec);
    model.build_body(spec, names).with_context(ctx)
}

/// Loop `l` of `sections`, each placed by its station's frame of `path`:
/// what [`skin`] builds.
struct Placed<'a> {
    path: &'a Path,
    sections: &'a [Vec<SweepLoop>],
    l: usize,
}

impl Placed<'_> {
    fn profile(&self, s: usize) -> &Profile {
        &self.sections[s][self.l].profile
    }
}

impl StripGeometry for Placed<'_> {
    fn joint(&self, s: usize, i: usize) -> GeopResult<Vector3> {
        Ok(self.path.stations[s].point(&self.profile(s).joint(i)?))
    }

    fn curve(&self, s: usize, i: usize) -> GeopResult<NurbCurve3D> {
        self.path.stations[s].curve(&self.profile(s).curves[i])
    }

    fn lateral(&self, j: usize, i: usize) -> GeopResult<NurbCurve3D> {
        let points = (0..self.sections.len())
            .map(|s| self.profile(s).joint(i))
            .collect::<GeopResult<Vec<_>>>()?;
        self.path.lateral(j, &points)
    }

    fn wall(&self, j: usize, i: usize) -> GeopResult<NurbSurface3D> {
        let curves: Vec<&NurbCurve2D> = (0..self.sections.len())
            .map(|s| &self.profile(s).curves[i])
            .collect();
        self.path.wall(j, &curves)
    }
}

/// Checks that `sections` fit `path` and each other, as [`skin`] needs:
/// one per station, each a valid set of loops — closed ones, for a solid —
/// all with the same loops, curves and flags, the `i`-th curves of all of
/// them on one degree and one knot vector.
fn check_sections(path: &Path, sections: &[Vec<SweepLoop>], solid: bool) -> GeopResult<()> {
    if sections.len() != path.stations.len() {
        return Err(GeopError::new(format!(
            "skin: {} sections for {} stations",
            sections.len(),
            path.stations.len()
        )));
    }
    let first = &sections[0];
    if first.is_empty() {
        return Err(GeopError::new("sweep: nothing to sweep"));
    }
    let same = |a: &Scalar, b: &Scalar| a.is_subset_of(*b) && b.is_subset_of(*a);
    for (s, section) in sections.iter().enumerate() {
        if section.len() != first.len() {
            return Err(GeopError::new(format!(
                "skin: section {s} has {} loops, the first {}",
                section.len(),
                first.len()
            )));
        }
        for (l, (lp, lp0)) in section.iter().zip(first).enumerate() {
            lp.check()?;
            if solid && !lp.profile.is_closed() {
                return Err(GeopError::new(
                    "sweep: a solid is swept from closed loops only",
                ));
            }
            if lp.profile.curves.len() != lp0.profile.curves.len()
                || lp.profile.is_closed() != lp0.profile.is_closed()
                || lp.poles != lp0.poles
                || lp.on_axis != lp0.on_axis
            {
                return Err(GeopError::new(format!(
                    "skin: loop {l} of section {s} does not match the first section's"
                )));
            }
            for (i, (c, c0)) in lp
                .profile
                .curves
                .iter()
                .zip(&lp0.profile.curves)
                .enumerate()
            {
                if c.degree != c0.degree
                    || c.knot_vector.len() != c0.knot_vector.len()
                    || !c
                        .knot_vector
                        .iter()
                        .zip(&c0.knot_vector)
                        .all(|(a, b)| same(a, b))
                {
                    return Err(GeopError::new(format!(
                        "skin: curve {i} of loop {l} of section {s} (degree {}, knots {:?}) is not compatible with the first section's (degree {}, knots {:?})",
                        c.degree, c.knot_vector, c0.degree, c0.knot_vector
                    )));
                }
            }
        }
    }
    Ok(())
}

/// The faces of `spec` grouped into connected sets — faces sharing an edge
/// are connected — each in the order of its faces' indices, and the sets in
/// the order of their first face.
fn connected_faces(spec: &BodySpec) -> Vec<Vec<usize>> {
    let mut sets = UnionFind::new(spec.faces.len());
    let mut user_of_edge: BTreeMap<usize, usize> = BTreeMap::new();
    for (f, face) in spec.faces.iter().enumerate() {
        for coedge in std::iter::once(&face.outer).chain(&face.holes).flatten() {
            if let CoedgeOn::Edge(e, _) = coedge.on {
                match user_of_edge.get(&e) {
                    Some(&other) => sets.union(f, other),
                    None => {
                        user_of_edge.insert(e, f);
                    }
                }
            }
        }
    }
    sets.groups()
}
