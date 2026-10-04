//! Sweeping planar profiles along a [`Path`] — straight, for an extrude,
//! around an axis, for a revolve, along a chain of curves, for a sweep (see
//! [`crate::path_sweep`]) — into a solid or a sheet, described whole as a
//! [`BodySpec`] and built in one go (see [`geop_core_topology::build`]).
//! [`skin`] builds the same grid through a different section at every
//! station, as a loft does (see [`crate::loft`]).
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
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    Sense,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{BodyNames, Namer, Part};

use crate::common::{Profile, bilinear, embed_curve, embed_point, end_point, line2, start_point};

/// Where the profile's plane is at one station of a path: a profile point
/// `(x, y)` lies at `origin + x e1 + y e2`.
#[derive(Clone, Debug)]
pub struct Frame<S: Scalar> {
    pub origin: Vector3<S>,
    pub e1: Vector3<S>,
    pub e2: Vector3<S>,
}

impl<S: Scalar> Frame<S> {
    /// The profile point `p` at this station.
    pub fn point(&self, p: &Vector2<S>) -> Vector3<S> {
        self.origin
            .add(&self.e1.prod_scalar(p[0]))
            .add(&self.e2.prod_scalar(p[1]))
    }

    /// The profile curve `curve` at this station. An affine map of its
    /// control points, so exact for any NURBS.
    pub fn curve(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve3D<S>> {
        embed_curve(curve, &self.origin, &self.e1, &self.e2)
    }
}

/// How the profile travels from one station to the next.
#[derive(Clone, Debug)]
pub enum Span<S: Scalar> {
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
        knots: Vec<S>,
        middle: Vec<(Frame<S>, Option<S>)>,
    },
    /// From one section to another of a different shape, along a NURBS of
    /// `degree` on the clamped `knots` over `[0, 1]`, each inner control row
    /// the sum of the span's first section placed by the first frame and
    /// its last section placed by the second, each times its weight: the
    /// interior of a loft shaped by guide curves (see [`crate::loft`]).
    Blend {
        degree: usize,
        knots: Vec<S>,
        middle: Vec<[(Frame<S>, S); 2]>,
    },
}

impl<S: Scalar> Span<S> {
    /// Along circular arcs: a rational quadratic whose middle control row is
    /// the profile at `middle`, weighted `weight`.
    pub fn arc(middle: Frame<S>, weight: S) -> Self {
        Span::Curve {
            degree: 2,
            knots: vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
            middle: vec![(middle, Some(weight))],
        }
    }
}

/// One control row of a span (see [`Path::rows`]).
enum Row<'a, S: Scalar> {
    /// The span's first section, placed by a frame, its weight multiplied
    /// by the weight given, if any.
    First(&'a Frame<S>, Option<S>),
    /// The span's last section, placed by a frame.
    Last(&'a Frame<S>),
    /// The sum of the first section placed by one frame and the last placed
    /// by another, each times its weight.
    Both(&'a [(Frame<S>, S); 2]),
}

impl<S: Scalar> Row<'_, S> {
    /// The row's control point for the homogeneous profile points `a` in
    /// the first section and `b` in the last.
    fn point(&self, a: &Vector3<S>, b: &Vector3<S>) -> Vector4<S> {
        let place =
            |p: &Vector3<S>, frame: &Frame<S>| embed_point(p, &frame.origin, &frame.e1, &frame.e2);
        match self {
            Row::First(frame, weight) => weighted(place(a, frame), *weight),
            Row::Last(frame) => place(b, frame),
            Row::Both([(fa, wa), (fb, wb)]) => {
                let (pa, pb) = (place(a, fa), place(b, fb));
                Vector4::from_array(std::array::from_fn(|k| pa[k].mul(*wa).add(pb[k].mul(*wb))))
            }
        }
    }
}

/// What a profile is swept along: its stations, and the spans between
/// consecutive ones — around to the first again for a `closed` path (a full
/// turn), which has as many spans as stations; an open one has one fewer.
#[derive(Clone, Debug)]
pub struct Path<S: Scalar> {
    pub stations: Vec<Frame<S>>,
    pub spans: Vec<Span<S>>,
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

impl<S: Scalar> Path<S> {
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
    fn rows(&self, j: usize) -> Vec<Row<'_, S>> {
        let (a, b) = (&self.stations[j], &self.stations[self.station(j + 1)]);
        let middle: Vec<Row<'_, S>> = match &self.spans[j] {
            Span::Line => Vec::new(),
            Span::Curve { middle, .. } => middle.iter().map(|(m, w)| Row::First(m, *w)).collect(),
            Span::Blend { middle, .. } => middle.iter().map(Row::Both).collect(),
        };
        std::iter::once(Row::First(a, None))
            .chain(middle)
            .chain(std::iter::once(Row::Last(b)))
            .collect()
    }

    fn degree(&self, j: usize) -> usize {
        match &self.spans[j] {
            Span::Line => 1,
            Span::Curve { degree, .. } | Span::Blend { degree, .. } => *degree,
        }
    }

    fn knots(&self, j: usize) -> Vec<S> {
        match &self.spans[j] {
            Span::Line => vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            Span::Curve { knots, .. } | Span::Blend { knots, .. } => knots.clone(),
        }
    }

    /// The path along span `j` of a profile point: `a` in the span's first
    /// station's section and `b` in its last one's (see [`skin`]).
    fn lateral(&self, j: usize, a: &Vector2<S>, b: &Vector2<S>) -> GeopResult<NurbCurve3D<S>> {
        let homogeneous = |p: &Vector2<S>| Vector3::from_array([p[0], p[1], S::ONE]);
        let (a, b) = (homogeneous(a), homogeneous(b));
        let control_points = self.rows(j).iter().map(|row| row.point(&a, &b)).collect();
        NurbCurve::try_new(self.degree(j), control_points, self.knots(j))
    }

    /// The wall a profile curve sweeps through span `j`, `a` in the span's
    /// first station's section and `b` in its last one's — compatible
    /// curves: `u` along the span, `v` along the curve.
    fn wall(
        &self,
        j: usize,
        a: &NurbCurve2D<S>,
        b: &NurbCurve2D<S>,
    ) -> GeopResult<NurbSurface3D<S>> {
        let control_points = self
            .rows(j)
            .iter()
            .flat_map(|row| {
                a.control_points
                    .iter()
                    .zip(&b.control_points)
                    .map(move |(pa, pb)| row.point(pa, pb))
            })
            .collect();
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
fn weighted<S: Scalar>(p: Vector4<S>, weight: Option<S>) -> Vector4<S> {
    match weight {
        None => p,
        Some(w) => Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), p[3].mul(w)]),
    }
}

/// One profile loop or chain to sweep, and which of its joints are poles
/// and which of its curves lie on the axis (see the module docs). A curve
/// on the axis runs between two poles.
#[derive(Clone, Debug)]
pub struct SweepLoop<S: Scalar> {
    pub profile: Profile<S>,
    /// Per joint: whether it lies on the axis.
    pub poles: Vec<bool>,
    /// Per curve: whether it lies along the axis.
    pub on_axis: Vec<bool>,
}

impl<S: Scalar> SweepLoop<S> {
    /// `profile`, clear of any axis.
    pub fn plain(profile: Profile<S>) -> Self {
        Self {
            poles: vec![false; profile.joint_names.len()],
            on_axis: vec![false; profile.curves.len()],
            profile,
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
            if !(t0.could_be_equal(S::ZERO) && t1.could_be_equal(S::ONE)) {
                return Err(GeopError::new(format!(
                    "sweep: curve {} has domain ({t0:?}, {t1:?}), not [0, 1]",
                    profile.curve_names[i]
                )));
            }
            if i + 1 < n || profile.is_closed() {
                let next = &curves[(i + 1) % n];
                let (end, start) = (end_point(curve)?, start_point(next)?);
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

    /// Joint `i`'s position in the profile plane.
    fn joint(&self, i: usize) -> GeopResult<Vector2<S>> {
        let curves = &self.profile.curves;
        if i < curves.len() {
            start_point(&curves[i])
        } else {
            end_point(&curves[i - 1])
        }
    }
}

/// A flat cap, at one station: the box around the profile in the
/// station's plane, as a bilinear patch — parametrized along `(x, y)`, or
/// along `(y, x)` if `swapped`, whichever makes its loops wind the way a
/// face's must (see [`sweep`]).
struct Cap<'a, S: Scalar> {
    frame: &'a Frame<S>,
    lo: Vector2<S>,
    size: Vector2<S>,
    swapped: bool,
}

impl<'a, S: Scalar> Cap<'a, S> {
    fn new(frame: &'a Frame<S>, loops: &[SweepLoop<S>], swapped: bool) -> GeopResult<Self> {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for curve in loops.iter().flat_map(|l| &l.profile.curves) {
            for cp in &curve.control_points {
                for k in 0..2 {
                    let x = cp[k].div(cp[2])?;
                    lo[k] = lo[k].min(x.lower().to_f64());
                    hi[k] = hi[k].max(x.upper().to_f64());
                }
            }
        }
        // The cap spans exactly this box. It encloses the profile: a NURBS
        // curve stays within the convex hull of its control points, and the
        // bounds above are the outer bounds of their enclosures.
        Ok(Self {
            frame,
            lo: Vector2::from_array([S::from_f64(lo[0]), S::from_f64(lo[1])]),
            size: Vector2::from_array([S::from_f64(hi[0] - lo[0]), S::from_f64(hi[1] - lo[1])]),
            swapped,
        })
    }

    /// `curve` in the cap's parameters.
    fn pcurve(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve2D<S>> {
        let control_points = curve
            .control_points
            .iter()
            .map(|cp| {
                Ok(Vector3::from_array([
                    cp[0].sub(cp[2].mul(self.lo[0])).div(self.size[0])?,
                    cp[1].sub(cp[2].mul(self.lo[1])).div(self.size[1])?,
                    cp[2],
                ]))
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let pcurve = NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())?;
        Ok(if self.swapped {
            pcurve.swap_xy()
        } else {
            pcurve
        })
    }

    fn surface(&self) -> GeopResult<NurbSurface3D<S>> {
        let corner = |i: usize, j: usize| {
            let pick = |k: usize, far: usize| {
                if far == 1 {
                    self.lo[k].add(self.size[k])
                } else {
                    self.lo[k]
                }
            };
            self.frame
                .point(&Vector2::from_array([pick(0, i), pick(1, j)]))
        };
        if self.swapped {
            bilinear(corner(0, 0), corner(0, 1), corner(1, 1), corner(1, 0))
        } else {
            bilinear(corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1))
        }
    }
}

/// A wall's sides in its own parameters (see the module docs): the curve at
/// the span's first station, run backwards along `u = 0`; the path of the
/// curve's start along `v = 0`; the curve at the span's last station along
/// `u = 1`; and the path of its end, backwards along `v = 1`.
fn wall_pcurves<S: Scalar>() -> GeopResult<[NurbCurve2D<S>; 4]> {
    let p = |u: f64, v: f64| Vector2::from_array([S::from_f64(u), S::from_f64(v)]);
    Ok([
        line2(p(0.0, 1.0), p(0.0, 0.0))?,
        line2(p(0.0, 0.0), p(1.0, 0.0))?,
        line2(p(1.0, 0.0), p(1.0, 1.0))?,
        line2(p(1.0, 1.0), p(0.0, 1.0))?,
    ])
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
pub fn sweep<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    path: &Path<S>,
    loops: &[SweepLoop<S>],
    solid: Option<&str>,
) -> GeopResult<BuiltBody> {
    let sections = vec![loops.to_vec(); path.stations.len()];
    skin(part, namer, path, &sections, solid)
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
pub fn skin<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    path: &Path<S>,
    sections: &[Vec<SweepLoop<S>>],
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
    let caps = solid.is_some() && !path.closed;
    let stations = path.stations.len();
    let spans = path.spans.len();

    let mut spec = BodySpec {
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: solid.is_some(),
    };
    let mut names = BodyNames {
        solid: solid.map(str::to_string),
        ..BodyNames::default()
    };
    let qualified = |name: &str, qualifier: Option<&str>| match qualifier {
        Some(q) => namer.name(&[name, q]),
        None => namer.name(&[name]),
    };

    // vertex[l][i][s], station_edge[l][i][s], lateral[l][i][j]: indices
    // into the spec, by loop, joint or curve, and station or span.
    let mut vertex: Vec<Vec<Vec<usize>>> = Vec::new();
    let mut station_edge: Vec<Vec<Vec<Option<usize>>>> = Vec::new();
    let mut lateral: Vec<Vec<Vec<usize>>> = Vec::new();
    for (l, lp) in loops.iter().enumerate() {
        let profile = &lp.profile;
        let section = |s: usize| &sections[s][l];
        let mut joints = Vec::new();
        for i in 0..lp.joints() {
            let name = &profile.joint_names[i];
            if lp.poles[i] {
                spec.vertices
                    .push(path.stations[0].point(&section(0).joint(i)?));
                names.vertices.push(namer.name(&[name]));
                joints.push(vec![spec.vertices.len() - 1; stations]);
            } else {
                joints.push(
                    (0..stations)
                        .map(|s| {
                            spec.vertices
                                .push(path.stations[s].point(&section(s).joint(i)?));
                            names
                                .vertices
                                .push(namer.name(&[name, &path.station_names[s]]));
                            Ok(spec.vertices.len() - 1)
                        })
                        .collect::<GeopResult<_>>()?,
                );
            }
        }

        let mut curves = Vec::new();
        for (i, name) in profile.curve_names.iter().enumerate() {
            let (start, end) = (&joints[i], &joints[lp.next(i)]);
            if lp.on_axis[i] {
                // One edge for every station — or none, without caps to
                // bound: no wall uses it.
                let edge = caps.then(|| -> GeopResult<usize> {
                    spec.edges.push(EdgeSpec {
                        curve: path.stations[0].curve(&section(0).profile.curves[i])?,
                        start: start[0],
                        end: end[0],
                    });
                    names.edges.push(namer.name(&[name]));
                    Ok(spec.edges.len() - 1)
                });
                let edge = edge.transpose()?;
                curves.push(vec![edge; stations]);
            } else {
                curves.push(
                    (0..stations)
                        .map(|s| {
                            spec.edges.push(EdgeSpec {
                                curve: path.stations[s].curve(&section(s).profile.curves[i])?,
                                start: start[s],
                                end: end[s],
                            });
                            names
                                .edges
                                .push(namer.name(&[name, &path.station_names[s]]));
                            Ok(Some(spec.edges.len() - 1))
                        })
                        .collect::<GeopResult<_>>()?,
                );
            }
        }

        let mut paths = Vec::new();
        for i in 0..lp.joints() {
            if lp.poles[i] {
                paths.push(Vec::new());
                continue;
            }
            paths.push(
                (0..spans)
                    .map(|j| {
                        let next = path.station(j + 1);
                        spec.edges.push(EdgeSpec {
                            curve: path.lateral(
                                j,
                                &section(j).joint(i)?,
                                &section(next).joint(i)?,
                            )?,
                            start: joints[i][j],
                            end: joints[i][next],
                        });
                        names.edges.push(qualified(
                            &profile.joint_names[i],
                            path.span_names[j].as_deref(),
                        ));
                        Ok(spec.edges.len() - 1)
                    })
                    .collect::<GeopResult<_>>()?,
            );
        }
        vertex.push(joints);
        station_edge.push(curves);
        lateral.push(paths);
    }

    let [first_back, start_side, last_forward, end_side] = wall_pcurves::<S>()?;
    for (l, lp) in loops.iter().enumerate() {
        let profile = &lp.profile;
        for (i, name) in profile.curve_names.iter().enumerate() {
            if lp.on_axis[i] {
                continue;
            }
            let next = lp.next(i);
            for j in 0..spans {
                let side = |joint: usize, sense: Sense, pcurve: &NurbCurve2D<S>| CoedgeSpec {
                    on: if lp.poles[joint] {
                        CoedgeOn::Vertex(vertex[l][joint][0])
                    } else {
                        CoedgeOn::Edge(lateral[l][joint][j], sense)
                    },
                    pcurve: pcurve.clone(),
                };
                let at = |s: usize, sense: Sense, pcurve: &NurbCurve2D<S>| CoedgeSpec {
                    on: CoedgeOn::Edge(
                        station_edge[l][i][s].expect("a curve off the axis has an edge"),
                        sense,
                    ),
                    pcurve: pcurve.clone(),
                };
                let last = path.station(j + 1);
                spec.faces.push(FaceSpec {
                    surface: path.wall(
                        j,
                        &sections[j][l].profile.curves[i],
                        &sections[last][l].profile.curves[i],
                    )?,
                    outer: vec![
                        at(j, Sense::Reversed, &first_back),
                        side(i, Sense::Forward, &start_side),
                        at(last, Sense::Forward, &last_forward),
                        side(next, Sense::Reversed, &end_side),
                    ],
                    holes: Vec::new(),
                });
                names
                    .faces
                    .push(qualified(name, path.span_names[j].as_deref()));
            }
        }
    }

    if caps {
        // The first station's cap runs every loop forwards, the last one's
        // backwards — the walls run them the other way — and each is
        // parametrized so that runs the outer loop counter-clockwise.
        for (s, forward, name) in [(0, true, "start"), (stations - 1, false, "end")] {
            let section = &sections[s];
            let cap = Cap::new(&path.stations[s], section, path.along_normal == forward)?;
            let mut boundaries = Vec::new();
            for (l, lp) in section.iter().enumerate() {
                let mut coedges = lp
                    .profile
                    .curves
                    .iter()
                    .enumerate()
                    .map(|(i, curve)| {
                        let edge = station_edge[l][i][s].expect("a cap's curves have edges");
                        Ok(if forward {
                            CoedgeSpec {
                                on: CoedgeOn::Edge(edge, Sense::Forward),
                                pcurve: cap.pcurve(curve)?,
                            }
                        } else {
                            CoedgeSpec {
                                on: CoedgeOn::Edge(edge, Sense::Reversed),
                                pcurve: cap.pcurve(curve)?.reverse(),
                            }
                        })
                    })
                    .collect::<GeopResult<Vec<_>>>()?;
                if !forward {
                    coedges.reverse();
                }
                boundaries.push(coedges);
            }
            let outer = boundaries.remove(0);
            spec.faces.push(FaceSpec {
                surface: cap.surface()?,
                outer,
                holes: boundaries,
            });
            names.faces.push(namer.name(&[name]));
        }
    }

    spec.shells = connected_faces(&spec);
    part.build_body(spec, names).with_context(ctx)
}

/// Checks that `sections` fit `path` and each other, as [`skin`] needs:
/// one per station, each a valid set of loops — closed ones, for a solid —
/// all with the same loops, curves and flags, the `i`-th curves of all of
/// them on one degree and one knot vector.
fn check_sections<S: Scalar>(
    path: &Path<S>,
    sections: &[Vec<SweepLoop<S>>],
    solid: bool,
) -> GeopResult<()> {
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
    let same = |a: &S, b: &S| a.is_subset_of(*b) && b.is_subset_of(*a);
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
fn connected_faces<S: Scalar>(spec: &BodySpec<S>) -> Vec<Vec<usize>> {
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
