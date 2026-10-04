//! Sweeping planar profiles along a [`Path`] — straight, for an extrude, or
//! around an axis, for a revolve — into a solid or a sheet, described whole
//! as a [`BodySpec`] and built in one go (see
//! [`geop_core_topology::build`]).
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
//! arc) and its curve (`v`, the curve's own degree and knots). So its normal
//! `∂u × ∂v` points out of the material exactly when the profile runs
//! counter-clockwise in its stations' `(e1, e2)` and the path runs along
//! `-(e1 × e2)` — or clockwise, along `e1 × e2` — see [`Path::along_normal`].
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
    /// Along circular arcs: a rational quadratic whose middle control row is
    /// the profile at `middle`, weighted `weight`.
    Arc { middle: Frame<S>, weight: S },
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

    /// The control rows of span `j`: each a station or middle frame, and its
    /// weight — none for a station's, whose weight is one: multiplying by
    /// it would only widen every enclosure by rounding.
    fn rows(&self, j: usize) -> Vec<(&Frame<S>, Option<S>)> {
        let (a, b) = (&self.stations[j], &self.stations[self.station(j + 1)]);
        match &self.spans[j] {
            Span::Line => vec![(a, None), (b, None)],
            Span::Arc { middle, weight } => vec![(a, None), (middle, Some(*weight)), (b, None)],
        }
    }

    fn knots(&self, j: usize) -> Vec<S> {
        let degree = self.rows(j).len() - 1;
        let mut knots = vec![S::ZERO; degree + 1];
        knots.extend(vec![S::ONE; degree + 1]);
        knots
    }

    /// The path of the profile point `p` along span `j`.
    fn lateral(&self, j: usize, p: &Vector2<S>) -> GeopResult<NurbCurve3D<S>> {
        let homogeneous = Vector3::from_array([p[0], p[1], S::ONE]);
        let rows = self.rows(j);
        let control_points = rows
            .iter()
            .map(|(frame, weight)| {
                weighted(
                    embed_point(&homogeneous, &frame.origin, &frame.e1, &frame.e2),
                    *weight,
                )
            })
            .collect();
        NurbCurve::try_new(rows.len() - 1, control_points, self.knots(j))
    }

    /// The wall `curve` sweeps through span `j`: `u` along the span, `v`
    /// along the curve.
    fn wall(&self, j: usize, curve: &NurbCurve2D<S>) -> GeopResult<NurbSurface3D<S>> {
        let rows = self.rows(j);
        let control_points = rows
            .iter()
            .flat_map(|(frame, weight)| {
                curve.control_points.iter().map(move |cp| {
                    weighted(
                        embed_point(cp, &frame.origin, &frame.e1, &frame.e2),
                        *weight,
                    )
                })
            })
            .collect();
        NurbSurface::try_new(
            rows.len() - 1,
            curve.degree,
            control_points,
            self.knots(j),
            curve.knot_vector.clone(),
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
    let ctx = with_context!(
        "sweep({}, {} loop(s), solid={solid:?})",
        namer.root(),
        loops.len()
    );
    path.check().with_context(ctx)?;
    if loops.is_empty() {
        return Err(GeopError::new("sweep: nothing to sweep")).with_context(ctx);
    }
    for lp in loops {
        lp.check().with_context(ctx)?;
        if solid.is_some() && !lp.profile.is_closed() {
            return Err(GeopError::new(
                "sweep: a solid is swept from closed loops only",
            ))
            .with_context(ctx);
        }
    }
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
    for lp in loops {
        let profile = &lp.profile;
        let mut joints = Vec::new();
        for i in 0..lp.joints() {
            let p = lp.joint(i)?;
            let name = &profile.joint_names[i];
            if lp.poles[i] {
                spec.vertices.push(path.stations[0].point(&p));
                names.vertices.push(namer.name(&[name]));
                joints.push(vec![spec.vertices.len() - 1; stations]);
            } else {
                joints.push(
                    (0..stations)
                        .map(|s| {
                            spec.vertices.push(path.stations[s].point(&p));
                            names
                                .vertices
                                .push(namer.name(&[name, &path.station_names[s]]));
                            spec.vertices.len() - 1
                        })
                        .collect(),
                );
            }
        }

        let mut curves = Vec::new();
        for (i, curve) in profile.curves.iter().enumerate() {
            let name = &profile.curve_names[i];
            let (start, end) = (&joints[i], &joints[lp.next(i)]);
            if lp.on_axis[i] {
                // One edge for every station — or none, without caps to
                // bound: no wall uses it.
                let edge = caps.then(|| -> GeopResult<usize> {
                    spec.edges.push(EdgeSpec {
                        curve: path.stations[0].curve(curve)?,
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
                                curve: path.stations[s].curve(curve)?,
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
            let p = lp.joint(i)?;
            paths.push(
                (0..spans)
                    .map(|j| {
                        spec.edges.push(EdgeSpec {
                            curve: path.lateral(j, &p)?,
                            start: joints[i][j],
                            end: joints[i][path.station(j + 1)],
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
        for (i, curve) in profile.curves.iter().enumerate() {
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
                spec.faces.push(FaceSpec {
                    surface: path.wall(j, curve)?,
                    outer: vec![
                        at(j, Sense::Reversed, &first_back),
                        side(i, Sense::Forward, &start_side),
                        at(path.station(j + 1), Sense::Forward, &last_forward),
                        side(next, Sense::Reversed, &end_side),
                    ],
                    holes: Vec::new(),
                });
                names.faces.push(qualified(
                    &profile.curve_names[i],
                    path.span_names[j].as_deref(),
                ));
            }
        }
    }

    if caps {
        // The first station's cap runs every loop forwards, the last one's
        // backwards — the walls run them the other way — and each is
        // parametrized so that runs the outer loop counter-clockwise.
        for (s, forward, name) in [(0, true, "start"), (stations - 1, false, "end")] {
            let cap = Cap::new(&path.stations[s], loops, path.along_normal == forward)?;
            let mut boundaries = Vec::new();
            for (l, lp) in loops.iter().enumerate() {
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
