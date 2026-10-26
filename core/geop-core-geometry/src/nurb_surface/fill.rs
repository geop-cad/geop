//! A closed loop of any number of curves filled smoothly
//! ([`NurbSurface3D::fill`]): a flat inner polygon and a four-sided patch
//! along every side of the loop, between the side, the spokes from its
//! ends to the polygon and the polygon's side, each patch tangent to its
//! neighbours across the spokes, to the polygon, and — where a face is
//! given along a side of the loop — to that face.
//!
//! **Across a spoke.** The two patches meeting along a spoke share its
//! tangent plane at every point of it exactly, by construction: the plane
//! is spanned by the spoke's tangent `T(v)` and a field `V(v)` across it,
//! both polynomials, and each patch's derivative across the spoke is
//! `α(v) V(v) + β(v) T(v)`, its own linear `α` and `β` — which is that
//! patch's row next to the spoke, a polynomial too (SMLib's cross-boundary
//! derivative `B(v) = p(v) D(v) + q(v) T(v)`). Where the row's end
//! weight is not one (a rational side), the row's weights run linearly
//! along it and the patch is one degree higher along the spoke than the
//! spoke, so the product still fits.
//!
//! **At the polygon.** Where two patches and the flat polygon meet, each
//! tangent to the other two, the corner has to be a flat point of both
//! patches: the shape operator of each vanishes along its polygon side and,
//! being symmetric and shared across the spoke, everywhere (the vertex
//! enclosure problem). So the spokes arrive *osculating* the polygon's
//! plane — their last three control points in it — the field `V` is
//! constant to first order there, and every control point of the patch
//! within two of that corner lies in the plane.
//!
//! **At the loop.** At a corner of the loop the two patches and the faces
//! along the two sides meet in one tangent plane: the plane of the two
//! sides there, which a face given along either must share. The spoke
//! leaves along the bisector of the corner in that plane, its first three
//! control points in it. Along a side with a flat face the patch's row
//! next to it lies in the face's plane, exactly; along a curved one it is
//! leant as [`NurbSurface3D::tangent_to`] does, around the points of it
//! the spokes' fields have set. Those set the twist at the corner, and it
//! has to agree with how the face's normal turns along the side: the
//! spoke's third control point and the field's second are moved off the
//! tangent plane so that the tilt rises from nothing at the corner as the
//! row's leaning has it further in (see [`twist`]). Where the patch still
//! strays from the face, the knot spans it strays in are halved, and near
//! a corner they shrink geometrically.
//!
//! Every position here is a free choice: the polygon's corners halfway to
//! the loop's centre, the spokes' and the field's control points, the
//! linear `α` and `β`. They are sharpened, so the patches share them
//! exactly.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::linear::solve_symmetric2,
    vector::{Vector3, Vector4},
};

use super::{
    NurbSurface3D,
    patch::{Neighbour, SIDES, TILT},
};
use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D, bezier_knots},
    shape::Plane,
};

/// The degree of a spoke: its first three control points in the tangent
/// plane at the loop's corner, its last three in the polygon's plane.
const SPOKE_DEGREE: usize = 5;

/// How often [`NurbSurface3D::fill`] splits the knot spans of its patches
/// where they stray from a curved neighbour, at most: how hard it tries to
/// follow the face's turning tangent plane, not what following it means.
/// Each round halves the spans where it strayed, so near a corner, where
/// the patch's twist is the spoke's and the tilt rises from nothing, the
/// spans shrink geometrically.
const MOST_SPLITS: usize = 16;

/// A closed loop filled by [`NurbSurface3D::fill`].
#[derive(Clone, Debug)]
pub struct Fill {
    /// Per side `k` of the loop, the patch along it, over `[0, 1]²`: side
    /// `k` at `v = 0` running with `u`, spoke `k + 1` at `u = 1` running
    /// with `v`, the inner polygon's side `k` at `v = 1` and spoke `k` at
    /// `u = 0` — round it as [`NurbSurface3D::coons`] numbers its sides.
    pub patches: Vec<NurbSurface3D>,
    /// Per corner `k` of the loop, where side `k` starts: the spoke from
    /// it to the inner polygon's corner `k`.
    pub spokes: Vec<NurbCurve3D>,
    /// The inner polygon's corners, running round as the loop does.
    pub inner: Vec<Vector3>,
    /// The plane the inner polygon lies in, its normal the side the loop
    /// runs counter-clockwise round, as every patch's does.
    pub plane: Plane,
}

impl NurbSurface3D {
    /// The closed loop `sides`, three or more curves each starting where
    /// the one before ends, filled smoothly (see the module docs): patches
    /// that run exactly along every side, meet each other and the flat
    /// inner polygon tangentially, and meet the face `neighbours[k]` names
    /// along side `k`, if any, tangentially too — exactly where that face
    /// is flat, leaving it at [`TILT`] towards its normal where it is
    /// curved, as [`NurbSurface3D::tangent_to`] does.
    ///
    /// Refused, naming the corner: a loop that turns back on itself, sides
    /// at a corner that do not lie in the tangent plane of a face given
    /// along either, and faces along two sides meeting there at an angle.
    pub fn fill(sides: &[NurbCurve3D], neighbours: &[Option<Neighbour<'_>>]) -> GeopResult<Fill> {
        let n = sides.len();
        if n < 3 || neighbours.len() != n {
            return Err(GeopError::new(format!(
                "NurbSurface::fill: {n} sides and {} neighbours: a loop of three sides at least, a neighbour or none along each",
                neighbours.len()
            )));
        }
        let curves = (sides.iter())
            .map(NurbCurve3D::with_unit_domain)
            .collect::<GeopResult<Vec<_>>>()?;
        let corners = (curves.iter())
            .map(NurbCurve3D::start)
            .collect::<GeopResult<Vec<_>>>()?;
        for k in 0..n {
            let end = curves[k].end()?;
            if !end.could_be_equal(&corners[(k + 1) % n]) {
                return Err(GeopError::new(format!(
                    "NurbSurface::fill: side {k} ends at {end:?}, but side {} starts at {:?}",
                    (k + 1) % n,
                    corners[(k + 1) % n]
                )));
            }
        }
        let (plane, inner) = inner_polygon(&corners)?;
        let frames = (0..n)
            .map(|k| Frame::at(k, &curves, neighbours, &plane, &inner))
            .collect::<GeopResult<Vec<_>>>()?;
        let curved = |k: usize| neighbours[k].as_ref().is_some_and(|s| !s.flat());
        // Per patch, the knots along its side added where it strayed.
        let mut added: Vec<Vec<Scalar>> = vec![Vec::new(); n];
        let mut strays = String::new();
        for _ in 0..=MOST_SPLITS {
            let (patches, spokes) = build(&curves, neighbours, &frames, &inner, &plane, &added)?;
            let mut again = false;
            for k in (0..n).filter(|&k| curved(k)) {
                for stray in patches[k].misses(&outer(neighbours, k))? {
                    // The knot span it strayed in, halved.
                    let knots = &patches[k].knot_vector_u;
                    let after = knots.iter().position(|t| t.definitely_greater(stray.at));
                    let Some(after) = after else { continue };
                    let half = knots[after - 1]
                        .add(knots[after])
                        .div(Scalar::TWO)?
                        .sharpen();
                    if !added[k].iter().any(|t| t.could_be_equal(half)) {
                        added[k].push(half);
                        again = true;
                    }
                    strays = format!("the patch along side {k} {}", stray.what);
                }
            }
            if !again {
                return Ok(Fill {
                    patches,
                    spokes,
                    inner,
                    plane,
                });
            }
        }
        Err(GeopError::new(format!(
            "NurbSurface::fill: split its patches {MOST_SPLITS} times where they strayed from their curved neighbours, the fill still does not leave them at {TILT:e} radians: {strays}"
        )))
    }
}

/// The neighbours of the patch along side `k`: the face along it, at its
/// side 0, and none elsewhere — the spokes and the polygon it is made
/// tangent to by construction.
fn outer<'a>(neighbours: &[Option<Neighbour<'a>>], k: usize) -> [Option<Neighbour<'a>>; SIDES] {
    [neighbours[k].clone(), None, None, None]
}

/// The inner polygon of a loop with `corners`: each corner halfway to
/// their average, moved onto the plane through that average square to the
/// loop's (Newell) normal — all free choices, sharpened — and that plane.
fn inner_polygon(corners: &[Vector3]) -> GeopResult<(Plane, Vec<Vector3>)> {
    let n = corners.len();
    let center = corners
        .iter()
        .fold(Vector3::zero(), |sum, p| sum.add(p))
        .prod_scalar(Scalar::ONE.div(Scalar::from_i64(n as i64))?)
        .sharpen();
    let mut normal = [0.0f64; 3];
    for k in 0..n {
        let (a, b) = (
            corners[k].to_f64_array(),
            corners[(k + 1) % n].to_f64_array(),
        );
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let length = (normal[0].powi(2) + normal[1].powi(2) + normal[2].powi(2)).sqrt();
    if length.is_nan() || length <= 0.0 {
        return Err(GeopError::new(format!(
            "NurbSurface::fill: the loop's corners {corners:?} enclose no area"
        )));
    }
    let normal = Vector3::from_array(normal.map(|x| Scalar::from_f64(x / length)));
    let plane = Plane {
        point: center,
        normal,
    };
    let half = Scalar::ONE.div(Scalar::TWO)?;
    let inner: Vec<Vector3> = corners
        .iter()
        .map(|p| {
            plane
                .project(&center.add(&p.sub(&center).prod_scalar(half)))
                .sharpen()
        })
        .collect();
    let mut area = Scalar::ZERO;
    for k in 0..n {
        area = area.add(
            inner[k]
                .sub(&center)
                .prod_cross(&inner[(k + 1) % n].sub(&center))
                .prod_dot(&normal),
        );
    }
    if !area.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "NurbSurface::fill: the inner polygon {inner:?} does not run round its normal {normal:?}"
        )));
    }
    Ok((plane, inner))
}

/// What a spoke is built from at its two ends: at the loop's corner `k`,
/// the tangent plane's normal there and the way into the hole along it;
/// at the polygon's corner, the way out of the polygon.
struct Frame {
    /// The tangent plane's unit normal at the loop's corner, on the side
    /// the loop runs counter-clockwise round.
    normal: Vector3,
    /// The unit bisector of the corner, in the tangent plane, into the
    /// hole.
    into: Vector3,
    /// The unit bisector of the polygon's outward normals at its corner.
    out: Vector3,
}

impl Frame {
    fn at(
        k: usize,
        curves: &[NurbCurve3D],
        neighbours: &[Option<Neighbour<'_>>],
        plane: &Plane,
        inner: &[Vector3],
    ) -> GeopResult<Self> {
        let n = curves.len();
        let before = (k + n - 1) % n;
        let e = curves[k].tangent(Scalar::ZERO)?.normalize()?;
        let f = curves[before].tangent(Scalar::ONE)?.neg().normalize()?;
        // Oriented to the loop's side.
        let oriented = |v: Vector3, what: &str| -> GeopResult<Vector3> {
            let along = v.prod_dot(&plane.normal);
            if along.definitely_greater(Scalar::ZERO) {
                Ok(v)
            } else if along.definitely_less(Scalar::ZERO) {
                Ok(v.neg())
            } else {
                Err(GeopError::new(format!(
                    "NurbSurface::fill: at corner {k} {what} {v:?} stands square to the loop's normal {:?}",
                    plane.normal
                )))
            }
        };
        let mut given: Vec<(usize, Vector3)> = Vec::new();
        let corner = curves[k].start()?;
        for (side, t) in [(k, Scalar::ZERO), (before, Scalar::ONE)] {
            if let Some(s) = &neighbours[side] {
                let (normal, _) = s.at(&corner, &curves[side].tangent(t)?)?;
                given.push((side, oriented(normal, "the normal of the face along side")?));
            }
        }
        let normal = match given.as_slice() {
            [(a, na), (b, nb)] => {
                if !na.prod_cross(nb).norm_sq().could_be_equal(Scalar::ZERO) {
                    return Err(GeopError::new(format!(
                        "NurbSurface::fill: the faces along sides {b} and {a} meet at an angle at corner {k}, normals {nb:?} and {na:?}: the fill cannot be tangent to both"
                    )));
                }
                *na
            }
            [(_, normal)] => *normal,
            _ => {
                let across = e.prod_cross(&f);
                if across.norm_sq().definitely_greater(Scalar::ZERO) {
                    oriented(across.normalize()?, "the plane of the sides")?
                } else {
                    let nu = plane.normal;
                    nu.sub(&e.prod_scalar(nu.prod_dot(&e))).normalize()?
                }
            }
        };
        for (side, tangent) in [(k, e), (before, f)] {
            if !tangent.prod_dot(&normal).could_be_equal(Scalar::ZERO) {
                return Err(GeopError::new(format!(
                    "NurbSurface::fill: side {side} leaves the tangent plane at corner {k}, normal {normal:?}, along {tangent:?}: the patches cannot be tangent there"
                )));
            }
        }
        let normal = normal.sharpen();
        let turn = e.sub(&f);
        if turn.norm_sq().could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "NurbSurface::fill: the loop turns back on itself at corner {k}"
            )));
        }
        let into = normal.prod_cross(&turn).normalize()?.sharpen();
        let outward = |a: usize, b: usize| -> GeopResult<Vector3> {
            inner[b]
                .sub(&inner[a])
                .prod_cross(&plane.normal)
                .normalize()
        };
        let out = outward(before, k)?
            .add(&outward(k, (k + 1) % n)?)
            .normalize()?
            .sharpen();
        Ok(Self { normal, into, out })
    }
}

/// A spoke's control points from the loop's corner `from` to the polygon's
/// corner `to`, leaving along `frame`'s bisector and arriving along its
/// way out of the polygon, a fifth of the way between them per control
/// point — free choices, sharpened — with its third control point lifted
/// off the tangent plane by `lift` (see [`twist`]).
fn spoke_points(from: Vector3, to: Vector3, frame: &Frame, lift: Scalar) -> Vec<Vector3> {
    let step = Scalar::from_f64(to.sub(&from).norm().to_f64() / SPOKE_DEGREE as f64);
    let along = |p: &Vector3, d: &Vector3, times: i64| {
        p.add(&d.prod_scalar(step.mul(Scalar::from_i64(times))))
            .sharpen()
    };
    vec![
        from,
        along(&from, &frame.into, 1),
        along(&from, &frame.into, 2)
            .add(&frame.normal.prod_scalar(lift))
            .sharpen(),
        along(&to, &frame.out, 2),
        along(&to, &frame.out, 1),
        to,
    ]
}

fn spoke_curve(points: &[Vector3]) -> GeopResult<NurbCurve3D> {
    NurbCurve::try_new(
        SPOKE_DEGREE,
        points.iter().map(|p| p.homogeneous(Scalar::ONE)).collect(),
        bezier_knots(SPOKE_DEGREE),
    )
}

/// The Bézier control points of `a(t) b(t)` over `[0, 1]`: `a` linear,
/// given by its values at either end, `b` by its control points — one
/// degree higher than `b`.
fn times_linear(a: [Scalar; 2], b: &[Vector3]) -> GeopResult<Vec<Vector3>> {
    let m = b.len() - 1;
    let q = Scalar::from_i64((m + 1) as i64);
    (0..=m + 1)
        .map(|j| {
            let mut out = Vector3::zero();
            if j <= m {
                let f = Scalar::from_i64((m + 1 - j) as i64).div(q)?;
                out = out.add(&b[j].prod_scalar(a[0].mul(f)));
            }
            if j >= 1 {
                let f = Scalar::from_i64(j as i64).div(q)?;
                out = out.add(&b[j - 1].prod_scalar(a[1].mul(f)));
            }
            Ok(out)
        })
        .collect()
}

/// A spoke's tangent plane field: its control points `V`, constant to
/// first order at both ends — across the spoke in the tangent plane at
/// the loop's corner, turned off it by `lean` at its second (see
/// [`twist`]), and across it in the polygon's plane at the other end.
fn field(frame: &Frame, plane: &Plane, lean: Scalar) -> Vec<Vector3> {
    let start = frame.normal.prod_cross(&frame.into).sharpen();
    let end = frame.out.prod_cross(&plane.normal).sharpen();
    vec![
        start,
        start.add(&frame.normal.prod_scalar(lean)).sharpen(),
        start,
        end,
        end,
        end,
    ]
}

/// One patch's derivative across a spoke, in the units of its control
/// points: `α V + β T`, `α` and `β` linear, at either end of the spoke.
#[derive(Clone, Copy)]
struct Across {
    alpha: [Scalar; 2],
    beta: [Scalar; 2],
}

impl Across {
    /// `α` and `β` such that `α V + β T` is `ends` at either end of the
    /// spoke — each decomposed in the plane of `V` and `T` there, a free
    /// choice, sharpened.
    fn of(ends: [Vector3; 2], v: [Vector3; 2], t: [Vector3; 2]) -> GeopResult<Self> {
        let mut alpha = [Scalar::ZERO; 2];
        let mut beta = [Scalar::ZERO; 2];
        for e in 0..2 {
            let (a, b) = solve_symmetric2(
                (
                    v[e].prod_dot(&v[e]),
                    v[e].prod_dot(&t[e]),
                    t[e].prod_dot(&t[e]),
                ),
                (v[e].prod_dot(&ends[e]), t[e].prod_dot(&ends[e])),
            )?;
            alpha[e] = a.sharpen();
            beta[e] = b.sharpen();
        }
        Ok(Self { alpha, beta })
    }

    /// Its control points along the spoke, of degree [`SPOKE_DEGREE`] + 1:
    /// `V` of [`SPOKE_DEGREE`] times a linear `α`, and the spoke's
    /// derivative of one degree less times a linear `β`, raised twice.
    fn points(&self, spoke: &[Vector3], v: &[Vector3]) -> GeopResult<Vec<Vector3>> {
        let p = Scalar::from_i64(SPOKE_DEGREE as i64);
        let t: Vec<Vector3> = spoke
            .windows(2)
            .map(|w| w[1].sub(&w[0]).prod_scalar(p))
            .collect();
        let tangent = times_linear([Scalar::ONE; 2], &times_linear(self.beta, &t)?)?;
        let across = times_linear(self.alpha, v)?;
        Ok(across.iter().zip(&tangent).map(|(a, b)| a.add(b)).collect())
    }
}

/// A patch's row of control points next to a spoke, of degree
/// [`SPOKE_DEGREE`] + 1 along it: the spoke times the row's weights, which
/// run linearly between `weights`, the row's own at its ends, plus or
/// minus (`into`: the spoke is at the patch's `u = 0`) the derivative
/// across `across`, homogeneous.
fn row_next_to(
    spoke: &[Vector3],
    across: &[Vector3],
    weights: [Scalar; 2],
    into: bool,
) -> GeopResult<Vec<Vector4>> {
    let weighted = times_linear(weights, spoke)?;
    let q = Scalar::from_i64((SPOKE_DEGREE + 1) as i64);
    (0..=SPOKE_DEGREE + 1)
        .map(|j| {
            let w = Scalar::interpolate(weights[0], weights[1], Scalar::from_i64(j as i64).div(q)?);
            let h = if into {
                weighted[j].add(&across[j])
            } else {
                weighted[j].sub(&across[j])
            };
            Ok(Vector4::from_array([h[0], h[1], h[2], w]))
        })
        .collect()
}

/// The point part and weight of a homogeneous control point.
fn split(cp: &Vector4) -> (Vector3, Scalar) {
    (Vector3::from_array([cp[0], cp[1], cp[2]]), cp[3])
}

/// The patch along side `k` before its rows next to the spokes, the
/// polygon and the side are set: the Coons patch of the side, the spokes
/// and the polygon's side, `SPOKE_DEGREE + 1` along the spokes, cubic at
/// least along the side, each span along it split in two until it has
/// `rows` control points along it at least, and the knots `added` put in.
///
/// Rows come from splitting rather than raising the degree: a row leant
/// onto a curved face is a collocation, and of a high degree it rings
/// where the tilt it follows rises from nothing at a corner.
fn coons_quad(
    side: &NurbCurve3D,
    spokes: [&[Vector3]; 2],
    inner: [Vector3; 2],
    rows: usize,
    added: &[Scalar],
) -> GeopResult<NurbSurface3D> {
    let start = spoke_curve(spokes[0])?;
    let end = spoke_curve(spokes[1])?;
    let back = NurbCurve3D::line(inner[1], inner[0])?;
    let mut patch = NurbSurface3D::coons([side, &end, &back, &start.reverse()])?;
    patch = patch.elevate_degree(false)?;
    while patch.degree_u < 3 {
        patch = patch.elevate_degree(true)?;
    }
    while patch.num_u < rows {
        patch = patch.refined([2, 1])?;
    }
    Ok(patch.with_knots_u(added))
}

/// The spokes and the patches of the fill (see the module docs), the
/// knots `added[k]` put into the patch along side `k`.
fn build(
    curves: &[NurbCurve3D],
    neighbours: &[Option<Neighbour<'_>>],
    frames: &[Frame],
    inner: &[Vector3],
    plane: &Plane,
    added: &[Vec<Scalar>],
) -> GeopResult<(Vec<NurbSurface3D>, Vec<NurbCurve3D>)> {
    let n = curves.len();
    let corner = |k: usize| curves[k % n].start();
    let rows = |k: usize| {
        if neighbours[k].as_ref().is_some_and(|s| !s.flat()) {
            5
        } else {
            4
        }
    };
    let quads = |spokes: &[Vec<Vector3>]| -> GeopResult<Vec<NurbSurface3D>> {
        (0..n)
            .map(|k| {
                coons_quad(
                    &curves[k],
                    [&spokes[k], &spokes[(k + 1) % n]],
                    [inner[k], inner[(k + 1) % n]],
                    rows(k),
                    &added[k],
                )
            })
            .collect()
    };
    // First with spokes flat at the loop's corners: their ends, which
    // decide the derivatives across them there, do not depend on the
    // twist.
    let flat: Vec<Vec<Vector3>> = (0..n)
        .map(|k| Ok(spoke_points(corner(k)?, inner[k], &frames[k], Scalar::ZERO)))
        .collect::<GeopResult<_>>()?;
    let first = quads(&flat)?;
    // Per spoke `k`: the derivative across it into the patch along side
    // `k`, at its `u = 0`, and out of the patch along side `k - 1`, at its
    // `u = 1`, both in the plane of the spoke and the field.
    let mut acrosses = Vec::with_capacity(n);
    for k in 0..n {
        let (after, before) = (&first[k], &first[(k + n - 1) % n]);
        let spoke = &flat[k];
        let v = field(&frames[k], plane, Scalar::ZERO);
        let p = SPOKE_DEGREE;
        let t = [
            spoke[1]
                .sub(&spoke[0])
                .prod_scalar(Scalar::from_i64(p as i64)),
            spoke[p]
                .sub(&spoke[p - 1])
                .prod_scalar(Scalar::from_i64(p as i64)),
        ];
        let ends = |patch: &NurbSurface3D, i: usize, into: bool| -> GeopResult<[Vector3; 2]> {
            let nv = patch.num_v;
            let mut out = [Vector3::zero(); 2];
            for (e, (j, on)) in [(0, spoke[0]), (nv - 1, spoke[p])].into_iter().enumerate() {
                let (h, w) = split(&patch.control_points[i * nv + j]);
                let d = h.sub(&on.prod_scalar(w));
                out[e] = if into { d } else { d.neg() };
            }
            Ok(out)
        };
        let x = Across::of(ends(after, 1, true)?, [v[0], v[p]], t)?;
        let y = Across::of(ends(before, before.num_u - 2, false)?, [v[0], v[p]], t)?;
        for e in 0..2 {
            if !x.alpha[e].mul(y.alpha[e]).definitely_greater(Scalar::ZERO) {
                return Err(GeopError::new(format!(
                    "NurbSurface::fill: the patches either side of spoke {k} fold over each other at its {} end: they leave it {:?} and {:?} across",
                    if e == 0 { "outer" } else { "inner" },
                    x.alpha[e],
                    y.alpha[e]
                )));
            }
        }
        acrosses.push((x, y));
    }
    // The twist at each loop's corner, and the spokes and fields with it.
    let mut spokes = Vec::with_capacity(n);
    let mut fields = Vec::with_capacity(n);
    for k in 0..n {
        let (lift, lean) = twist(k, neighbours, &frames[k], &first, &flat, &acrosses)
            .map_err(|e| e.with_context(format!("NurbSurface::fill: the twist at corner {k}")))?;
        spokes.push(spoke_points(corner(k)?, inner[k], &frames[k], lift));
        fields.push(field(&frames[k], plane, lean));
    }
    let mut patches = quads(&spokes)?;
    for (k, patch) in patches.iter_mut().enumerate() {
        set_rows(patch, k, &spokes, &fields, &acrosses, inner, plane)?;
        if neighbours[k].is_some() {
            let nu = patch.num_u;
            *patch = patch
                .tangent_rows(&outer(neighbours, k), &|i, j| {
                    j == 1 && (i == 1 || i == nu - 2)
                })
                .map_err(|e| {
                    e.with_context(format!("NurbSurface::fill: the patch along side {k}"))
                })?;
        }
    }
    let spokes = spokes
        .iter()
        .map(|s| spoke_curve(s))
        .collect::<GeopResult<Vec<_>>>()?;
    Ok((patches, spokes))
}

/// The rows of the patch along side `k` next to its spokes, from their
/// fields, and next to the polygon, in its plane — refused where it would
/// fold back over the polygon.
fn set_rows(
    patch: &mut NurbSurface3D,
    k: usize,
    spokes: &[Vec<Vector3>],
    fields: &[Vec<Vector3>],
    acrosses: &[(Across, Across)],
    inner: &[Vector3],
    plane: &Plane,
) -> GeopResult<()> {
    let n = spokes.len();
    let next = (k + 1) % n;
    let (nu, nv) = (patch.num_u, patch.num_v);
    let weights = |i: usize| {
        [
            patch.control_points[i * nv][3],
            patch.control_points[i * nv + nv - 1][3],
        ]
    };
    let start = row_next_to(
        &spokes[k],
        &acrosses[k].0.points(&spokes[k], &fields[k])?,
        weights(1),
        true,
    )?;
    let end = row_next_to(
        &spokes[next],
        &acrosses[next].1.points(&spokes[next], &fields[next])?,
        weights(nu - 2),
        false,
    )?;
    for j in 1..nv - 1 {
        patch.control_points[nv + j] = start[j].sharpen();
        patch.control_points[(nu - 2) * nv + j] = end[j].sharpen();
    }
    for i in 2..nu - 2 {
        let cp = patch.control_points[i * nv + nv - 2];
        let (point, w) = (cp.dehomogenized()?, cp[3]);
        patch.control_points[i * nv + nv - 2] = plane.project(&point).homogeneous(w);
    }
    let out = inner[next].sub(&inner[k]).prod_cross(&plane.normal);
    for i in 0..nu {
        let row = patch.control_points[i * nv + nv - 2].dehomogenized()?;
        let on = patch.control_points[i * nv + nv - 1].dehomogenized()?;
        if !row.sub(&on).prod_dot(&out).definitely_greater(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "NurbSurface::fill: the patch along side {k} folds back over the inner polygon at control point ({i}, {}), {row:?}",
                nv - 2
            )));
        }
    }
    patch.recompute_aabb();
    Ok(())
}

/// How far spoke `k`'s third control point is lifted off the tangent plane
/// at its loop's corner, and the field's second turned off it: the twist
/// there, which decides how the patches either side of the spoke start to
/// leave the faces along the sides meeting there.
///
/// Along side `k`, at the patch's `u = 0`, the patch leaves the face at
/// `f(u) = S_v(u, 0)·n(u)`, which is zero at the corner, where the spoke
/// lies in the face's tangent plane. Its slope there, `S_uv·n + S_v·n'`,
/// takes in how the face's normal turns along the side, `n'`, and the
/// twist `S_uv`: the derivative of the field's `α V + β T` along the
/// spoke, whose part along the normal is `5 α h_v + 20 β h_s` in the
/// units of the patch's control points, for the field's turn `h_v` and the
/// spoke's lift `h_s`. The slope is set to rise to [`TILT`] at the
/// patch's first Greville abscissa along the side — a free choice of how
/// a tilt that is nothing at the corner starts — along a curved face, and
/// to zero along a flat one; along side `k - 1`, at that patch's `u = 1`,
/// likewise, falling. Two faces give two conditions; one is met by the
/// field's turn alone.
fn twist(
    k: usize,
    neighbours: &[Option<Neighbour<'_>>],
    frame: &Frame,
    first: &[NurbSurface3D],
    flat: &[Vec<Vector3>],
    acrosses: &[(Across, Across)],
) -> GeopResult<(Scalar, Scalar)> {
    let n = neighbours.len();
    let before = (k + n - 1) % n;
    let curved = |s: usize| neighbours[s].as_ref().is_some_and(|s| !s.flat());
    if !curved(k) && !curved(before) {
        return Ok((Scalar::ZERO, Scalar::ZERO));
    }
    let spoke = &flat[k];
    let p = Scalar::from_i64(SPOKE_DEGREE as i64);
    // The spoke's derivative at the loop's corner.
    let leaving = spoke[1].sub(&spoke[0]).prod_scalar(p);
    let tilt = Scalar::from_f64(TILT).mul(leaving.norm());
    let mut a = Vec::with_capacity(2);
    let mut rhs = Vec::with_capacity(2);
    for (side, into) in [(k, true), (before, false)] {
        let Some(neighbour) = &neighbours[side] else {
            continue;
        };
        let patch = &first[side];
        let u = if into { Scalar::ZERO } else { Scalar::ONE };
        let (along, _) = patch.derivatives(u, Scalar::ZERO)?;
        let (normal, turn) = neighbour.turning(&spoke[0], &along).map_err(|e| {
            e.with_context(format!(
                "NurbSurface::fill: how the face along side {side} turns at corner {k}, {:?}, along {along:?}",
                spoke[0]
            ))
        })?;
        // The patch's derivative along `u` per unit of the field, at the
        // corner: its degree over the knot span there.
        let knots = &patch.knot_vector_u;
        let d = patch.degree_u;
        let (span, abscissa) = if into {
            (
                knots[d + 1].sub(knots[0]),
                knots[2..=d + 1].iter().fold(Scalar::ZERO, |s, &x| s.add(x)),
            )
        } else {
            let m = knots.len() - 1;
            (
                knots[m].sub(knots[m - d - 1]),
                knots[m - d - 1..m - 1]
                    .iter()
                    .fold(Scalar::ZERO, |s, &x| s.add(knots[m].sub(x))),
            )
        };
        // The lift and the turn move the patch along the frame's normal,
        // which is the face's or its reverse.
        let scale = Scalar::from_i64(d as i64)
            .div(span)?
            .mul(normal.prod_dot(&frame.normal));
        let across = if into { acrosses[k].0 } else { acrosses[k].1 };
        // The slope wanted, rising from the corner into the side: up to
        // the tilt at the first abscissa in from the corner.
        let slope = if neighbour.flat() {
            Scalar::ZERO
        } else {
            tilt.mul(Scalar::from_i64(d as i64)).div(abscissa)?
        };
        // `f'` along `u`: positive rising from `u = 0`, negative towards
        // `u = 1`.
        let wanted = if into { slope } else { slope.neg() };
        let by_lift = scale.mul(Scalar::from_i64(20)).mul(across.beta[0]);
        let by_lean = scale.mul(Scalar::from_i64(5)).mul(across.alpha[0]);
        a.push((by_lift, by_lean));
        rhs.push(wanted.sub(leaving.prod_dot(&turn)));
    }
    Ok(match (a.as_slice(), rhs.as_slice()) {
        ([(l0, t0), (l1, t1)], [r0, r1]) => {
            let det = l0.mul(*t1).sub(t0.mul(*l1));
            (
                r0.mul(*t1).sub(t0.mul(*r1)).div(det)?.sharpen(),
                l0.mul(*r1).sub(r0.mul(*l1)).div(det)?.sharpen(),
            )
        }
        ([(_, t0)], [r0]) => (Scalar::ZERO, r0.div(*t0)?.sharpen()),
        _ => (Scalar::ZERO, Scalar::ZERO),
    })
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use crate::nurb_curve::{NurbCurve3D, sqrt2_over_2};
    use crate::nurb_surface::NurbSurface3D;

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// The quarter circle round the origin from `a` to `b`, unit vectors
    /// square to each other.
    fn quarter(a: Vector3, b: Vector3) -> NurbCurve3D {
        NurbCurve3D::arc(a, a.add(&b), b, sqrt2_over_2()).unwrap()
    }

    /// Whether `a` and `b` are parallel to the rounding: the normals of a
    /// rational patch evaluated come out a few `1e-12` wide.
    fn parallel(a: &Vector3, b: &Vector3) -> bool {
        a.prod_cross(b).norm().to_f64() < 1e-10
    }

    /// The three quarter circles round a sphere's octant, rational sides
    /// meeting at right angles, filled: the patches run along the sides,
    /// share their normal across every spoke and with the inner triangle
    /// along its sides, all along them.
    #[test]
    fn an_octant_is_filled_smoothly() {
        let (x, y, z) = (v(1., 0., 0.), v(0., 1., 0.), v(0., 0., 1.));
        let sides = [quarter(x, y), quarter(y, z), quarter(z, x)];
        let fill = NurbSurface3D::fill(&sides, &[None, None, None]).unwrap();
        assert_eq!(fill.patches.len(), 3);
        let s = |i: i64| Scalar::from_ratio(i, 16).unwrap();
        for (k, patch) in fill.patches.iter().enumerate() {
            let before = &fill.patches[(k + 2) % 3];
            for i in 0..=16 {
                // Along the side.
                let on = patch.evaluate(s(i), Scalar::ZERO).unwrap();
                let side = sides[k]
                    .evaluate(sides[k].closest_point(&on).unwrap().0)
                    .unwrap();
                assert!(on.sub(&side).norm().to_f64() < 1e-12, "side {k} at {i}");
                // Across spoke `k`, at `u = 0` of this patch and `u = 1` of
                // the one before.
                if i < 16 {
                    let here = patch.normal(Scalar::ZERO, s(i)).unwrap();
                    let there = before.normal(Scalar::ONE, s(i)).unwrap();
                    assert!(
                        parallel(&here, &there),
                        "spoke {k} at {i}: {here:?} {there:?}"
                    );
                }
                // Along the inner triangle.
                if 0 < i && i < 16 {
                    let n = patch.normal(s(i), Scalar::ONE).unwrap();
                    assert!(
                        parallel(&n, &fill.plane.normal),
                        "inner side {k} at {i}: {n:?}"
                    );
                }
            }
        }
    }
}
