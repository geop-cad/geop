//! Where to look for the closed loops two surfaces meet in: points of
//! `S_a(u_a, v_a) = S_b(u_b, v_b)` at which the curve they meet in turns
//! back along a direction `d` ([`turning_points`]).
//!
//! # Why turning points
//!
//! A boolean finds the curves two faces meet in by tracing them from the
//! points where an edge of one pierces the other. A closed loop lying inside
//! both faces is pierced by no edge, and is never found that way: a bump of
//! one face poking through the other, two freeform faces meeting in an oval.
//!
//! Every closed loop has a point furthest along any direction `d`, and one
//! least far: there the curve's tangent, `n_a × n_b`, is square to `d`, so
//!
//! ```text
//! g = (n_a × n_b) · d = 0.
//! ```
//!
//! With the three equations of `S_a = S_b` that is a square system in the
//! four parameters, and every loop gives at least two of its solutions.
//! Where the surfaces cross transversally the solutions are isolated, so a
//! subdivision search finds every one ("Subdivide to isolate, then Newton
//! to refine", `AGENTS.md`): pairs of patches whose control-point boxes are
//! apart hold no point of both; pairs over which `g`, enclosed from the
//! patches' normals, is definitely not zero hold no turning point. What is
//! left at the handoff is a candidate, for the caller to refine onto the
//! curve.
//!
//! # What is not searched
//!
//! Where the two normals could be parallel the curve has no tangent to turn,
//! and `g` vanishes for every `d`: tangent contact, a face flush with
//! another. Those pairs are subdivided only down to a coarser handoff and
//! then set aside ([`TurningPoints::tangent`] counts them): a loop shrinking
//! into a tangency is the one case left to the caller to notice, as a face
//! the boolean cannot classify.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

use crate::{nearest::SurfacePatch, nurb_surface::NurbSurface3D};

/// How many times finer than the smaller surface a pair of patches is cut
/// before it is handed over as a candidate: where the search hands over to
/// Newton. Bounds effort, and how many candidates near one turning point the
/// caller refines; never what a candidate means.
const HANDOFF_PARTS: i64 = 64;

/// How many times finer than the smaller surface a pair of patches whose
/// normals could be parallel is cut before it is set aside (see the module
/// docs). Coarser than [`HANDOFF_PARTS`]: two faces flush with each other
/// are such a pair everywhere they overlap, and are set aside whole.
const TANGENT_PARTS: i64 = 8;

/// A patch of one of the two surfaces, single-span, with an enclosure of
/// the direction of its normal over all of it (see [`normal_directions`]).
#[derive(Clone)]
struct Piece {
    patch: SurfacePatch,
    normal: Vector3,
}

/// A box holding a positive multiple of the normal `S_u × S_v` at every
/// point of the single-span `patch`, from its control net.
///
/// With `A` the homogeneous points and `w` the weights, `w² S_u = w A_u -
/// w_u A`, so `w S_u = A_u - w_u S`: in the direction of `A_u - w_u S`.
/// Over the patch `A_u` and `w_u` are positive multiples of convex
/// combinations of the differences of the net along `u` (its hodograph,
/// all with one factor), and `S` lies in the hull of the Cartesian control
/// points; each is enclosed by its box, and likewise along `v`. For a
/// polynomial patch `w_u` vanishes, and the box is the hodographs' own.
/// Relative to a sharp point of the net, a free choice, so the box does
/// not grow with the patch's distance from the origin.
fn normal_directions(patch: &NurbSurface3D) -> Vector3 {
    let (nu, nv) = (patch.num_u, patch.num_v);
    let at = |i: usize, j: usize| patch.control_points[i * nv + j];
    let origin =
        Vector3::from_array([0, 1, 2].map(|k| at(0, 0)[k].div(at(0, 0)[3]).unwrap_or(at(0, 0)[k])))
            .sharpen();
    // The net relative to `origin`, homogeneous: `A - w origin`.
    let local = |i: usize, j: usize| -> [Scalar; 4] {
        let q = at(i, j);
        [
            q[0].sub(q[3].mul(origin[0])),
            q[1].sub(q[3].mul(origin[1])),
            q[2].sub(q[3].mul(origin[2])),
            q[3],
        ]
    };
    let union = |acc: Option<[Scalar; 4]>, x: [Scalar; 4]| -> Option<[Scalar; 4]> {
        Some(match acc {
            None => x,
            Some(a) => std::array::from_fn(|k| a[k].union(x[k])),
        })
    };
    let mut points: Option<[Scalar; 4]> = None;
    for i in 0..nu {
        for j in 0..nv {
            let q = local(i, j);
            let cartesian = [0, 1, 2].map(|k| q[k].div(q[3]).unwrap_or(Scalar::ENTIRE));
            points = union(
                points,
                [cartesian[0], cartesian[1], cartesian[2], Scalar::ZERO],
            );
        }
    }
    let points = points.expect("a net has a point");
    let hodograph = |along_u: bool| -> [Scalar; 4] {
        let mut acc = None;
        let (ni, nj) = if along_u { (nu - 1, nv) } else { (nu, nv - 1) };
        for i in 0..ni {
            for j in 0..nj {
                let (a, b) = if along_u {
                    (local(i, j), local(i + 1, j))
                } else {
                    (local(i, j), local(i, j + 1))
                };
                acc = union(acc, std::array::from_fn(|k| b[k].sub(a[k])));
            }
        }
        acc.unwrap_or([Scalar::ZERO; 4])
    };
    let direction = |d: [Scalar; 4]| {
        Vector3::from_array(std::array::from_fn(|k| d[k].sub(d[3].mul(points[k]))))
    };
    direction(hodograph(true)).prod_cross(&direction(hodograph(false)))
}

impl Piece {
    fn new(surface: NurbSurface3D) -> GeopResult<Self> {
        Ok(Self {
            normal: normal_directions(&surface),
            patch: SurfacePatch(surface),
        })
    }

    fn size(&self) -> GeopResult<Scalar> {
        self.patch.0.size()
    }

    fn halves(&self) -> GeopResult<[Piece; 2]> {
        let (a, b) = self.patch.halves()?;
        Ok([Piece::new(a.0)?, Piece::new(b.0)?])
    }

    /// The parameters of the middle of the patch, and their range over it.
    fn middle(&self) -> GeopResult<[(Scalar, Scalar); 2]> {
        let (u, v) = self.patch.middle()?;
        let over = |(lo, hi): (Scalar, Scalar)| lo.union(hi);
        Ok([
            (u, over(self.patch.0.domain_u())),
            (v, over(self.patch.0.domain_v())),
        ])
    }
}

/// `surface` cut at every knot, into its polynomial (or rational) pieces:
/// a normal enclosed over a parameter box holds only within one span.
fn spans(surface: &NurbSurface3D) -> GeopResult<Vec<Piece>> {
    let mut rows = vec![surface.clone()];
    for (along_u, breaks) in [
        (true, surface.breakpoints_u()),
        (false, surface.breakpoints_v()),
    ] {
        let inner = &breaks[1..breaks.len() - 1];
        let mut cut = Vec::new();
        for row in rows {
            let mut rest = row;
            for &t in inner {
                let (lo, hi) = if along_u {
                    rest.split_u(t)?
                } else {
                    rest.split_v(t)?
                };
                cut.push(lo);
                rest = hi;
            }
            cut.push(rest);
        }
        rows = cut;
    }
    rows.into_iter().map(Piece::new).collect()
}

/// What [`turning_points`] finds.
#[derive(Clone, Debug)]
pub struct TurningPoints {
    /// The direction the curves turn back along at the candidates: sharp,
    /// and a free choice (see [`direction`]).
    pub direction: Vector3,
    /// One candidate per cluster of touching boxes left at the handoff,
    /// where the two surfaces' normals are definitely not parallel: the
    /// sharp middle of one box of it, `(u_a, v_a, u_b, v_b)`, a seed for
    /// Newton onto the curve, and the box of the whole cluster. Several
    /// turning points closer than the handoff come out as one candidate.
    pub candidates: Vec<Candidate>,
    /// How many pairs of patches were set aside because the normals could
    /// be parallel over them (see the module docs).
    pub tangent: usize,
}

/// A candidate turning point (see [`TurningPoints::candidates`]).
#[derive(Clone, Debug)]
pub struct Candidate {
    /// A sharp seed in the cluster, `(u_a, v_a, u_b, v_b)`.
    pub seed: [Scalar; 4],
    /// The parameters' range over the whole cluster.
    pub cluster: [Scalar; 4],
}

/// The direction the loops' turning points are sought along, a free choice
/// (any `d` gives every loop two turning points), made to keep `g` away
/// from zero where it can: square to both surfaces' normals at their
/// middles, where `g` is then `|n_a × n_b|²` (for two planes it never
/// vanishes), unless those normals are nearly parallel, and square to one
/// of them otherwise, across the axis that normal runs least along. The
/// first is preferred unless it is less than half as long as the others.
fn direction(a: &NurbSurface3D, b: &NurbSurface3D) -> Vector3 {
    let normal = |s: &NurbSurface3D| -> Option<Vector3> {
        let (u, v) = SurfacePatch(s.clone()).middle().ok()?;
        s.normal(u, v).ok().map(|n| n.sharpen())
    };
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .map(|a| Vector3::from_array(a.map(Scalar::from_f64)));
    let across = |n: &Vector3| -> Vector3 {
        let least = axes
            .iter()
            .min_by(|x, y| {
                let (dx, dy) = (n.prod_dot(x).abs(), n.prod_dot(y).abs());
                dx.to_f64().total_cmp(&dy.to_f64())
            })
            .expect("three axes");
        n.prod_cross(least)
    };
    let (na, nb) = (normal(a), normal(b));
    let mut options: Vec<(f64, Vector3)> = Vec::new();
    if let (Some(na), Some(nb)) = (&na, &nb) {
        let d = na.prod_cross(nb);
        options.push((2.0 * d.norm().to_f64(), d));
    }
    for n in [&na, &nb].into_iter().flatten() {
        let d = across(n);
        options.push((d.norm().to_f64(), d));
    }
    options
        .into_iter()
        .max_by(|x, y| x.0.total_cmp(&y.0))
        .and_then(|(_, d)| d.normalize().ok())
        .map(|d| d.sharpen())
        .unwrap_or(axes[0])
}

/// Candidates for every point at which the curves `a` and `b` meet in turn
/// back along a direction chosen here — at least two on every closed loop
/// of them (see the module docs). Errors once `max_nodes` pairs of patches
/// have been searched with pairs left: a search that did not finish proves
/// nothing.
pub fn turning_points(
    a: &NurbSurface3D,
    b: &NurbSurface3D,
    max_nodes: usize,
) -> GeopResult<TurningPoints> {
    let ctx = |e: GeopError| e.with_context("turning_points(a, b)");
    let d = direction(a, b);
    let mut found = TurningPoints {
        direction: d,
        candidates: Vec::new(),
        tangent: 0,
    };
    if !a.could_overlap(b) {
        return Ok(found);
    }
    let size = a
        .size()
        .with_context(&ctx)?
        .min(b.size().with_context(&ctx)?);
    let parts = |n: i64| -> GeopResult<Scalar> { size.div(Scalar::from_i64(n)).map(|s| s.upper()) };
    let (handoff, tangent_handoff) = (
        parts(HANDOFF_PARTS).with_context(&ctx)?,
        parts(TANGENT_PARTS).with_context(&ctx)?,
    );

    let (pieces_a, pieces_b) = (spans(a).with_context(&ctx)?, spans(b).with_context(&ctx)?);
    let mut pending: Vec<(Piece, Piece)> = Vec::new();
    for pa in &pieces_a {
        for pb in &pieces_b {
            pending.push((pa.clone(), pb.clone()));
        }
    }
    let mut leaves: Vec<[Scalar; 4]> = Vec::new();
    let mut seeds: Vec<[Scalar; 4]> = Vec::new();
    let mut visited = 0usize;
    while let Some((pa, pb)) = pending.pop() {
        if !pa.patch.0.could_overlap(&pb.patch.0) {
            continue;
        }
        let cross = pa.normal.prod_cross(&pb.normal);
        if !cross.prod_dot(&d).could_be_equal(Scalar::ZERO) {
            continue;
        }
        if visited >= max_nodes {
            return Err(ctx(GeopError::new(format!(
                "searched {max_nodes} pairs of patches with {} left, {} candidates so far: the search did not finish",
                pending.len() + 1,
                leaves.len()
            ))));
        }
        visited += 1;
        let regular = (0..3).any(|k| !cross[k].could_be_equal(Scalar::ZERO));
        let limit = if regular { handoff } else { tangent_handoff };
        let (size_a, size_b) = (pa.size()?, pb.size()?);
        let (split_a, split_b) = (
            size_a.definitely_greater(limit),
            size_b.definitely_greater(limit),
        );
        if !split_a && !split_b {
            if !regular {
                found.tangent += 1;
                continue;
            }
            let ([(ua, ua_box), (va, va_box)], [(ub, ub_box), (vb, vb_box)]) =
                (pa.middle()?, pb.middle()?);
            seeds.push([ua, va, ub, vb]);
            leaves.push([ua_box, va_box, ub_box, vb_box]);
            continue;
        }
        let cut_a = split_a && (!split_b || size_a.to_f64() >= size_b.to_f64());
        if cut_a {
            for half in pa.halves().with_context(&ctx)? {
                pending.push((half, pb.clone()));
            }
        } else {
            for half in pb.halves().with_context(&ctx)? {
                pending.push((pa.clone(), half));
            }
        }
    }

    // Clusters of leaves whose boxes touch: one turning point found from
    // several sides, or several closer than the handoff.
    let mut cluster: Vec<usize> = (0..leaves.len()).collect();
    fn root(cluster: &mut [usize], mut i: usize) -> usize {
        while cluster[i] != i {
            cluster[i] = cluster[cluster[i]];
            i = cluster[i];
        }
        i
    }
    for i in 0..leaves.len() {
        for j in 0..i {
            let touch = (0..4).all(|k| {
                !leaves[i][k].definitely_less(leaves[j][k].lower())
                    && !leaves[i][k].definitely_greater(leaves[j][k].upper())
            });
            if touch {
                let (ri, rj) = (root(&mut cluster, i), root(&mut cluster, j));
                cluster[ri] = rj;
            }
        }
    }
    let mut groups: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for i in 0..leaves.len() {
        let r = root(&mut cluster, i);
        groups.entry(r).or_default().push(i);
    }
    for members in groups.into_values() {
        let whole: [Scalar; 4] = std::array::from_fn(|k| {
            members
                .iter()
                .map(|&i| leaves[i][k])
                .reduce(Scalar::union)
                .expect("a cluster has a member")
        });
        // The member nearest the middle of the cluster: a free choice of a
        // seed, taken where Newton has least far to go to any of it.
        let middle = whole.map(|x| x.midpoint().to_f64());
        let seed = members
            .iter()
            .map(|&i| seeds[i])
            .min_by(|x, y| {
                let dist = |s: &[Scalar; 4]| {
                    (0..4)
                        .map(|k| (s[k].to_f64() - middle[k]).powi(2))
                        .sum::<f64>()
                };
                dist(x).total_cmp(&dist(y))
            })
            .expect("a cluster has a member");
        found.candidates.push(Candidate {
            seed,
            cluster: whole,
        });
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{nurb_surface::NurbSurface, test_fixtures::knots};
    use geop_core_math::vector::Vector4;

    fn h(x: f64, y: f64, z: f64) -> Vector4 {
        Vector4::from_array([x, y, z, 1.0].map(Scalar::from_f64))
    }

    /// A biquadratic dome over `[0, 1]²`, its middle control point raised to
    /// `top`: the surface reaches a quarter of that.
    fn dome(top: f64) -> NurbSurface3D {
        let mut cps = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                let z = if i == 1 && j == 1 { top } else { 0.0 };
                cps.push(h(i as f64 / 2.0, j as f64 / 2.0, z));
            }
        }
        let k = knots(&[0., 0., 0., 1., 1., 1.]);
        NurbSurface::try_new(2, 2, cps, k.clone(), k).unwrap()
    }

    /// The plane `z = z` over `[-1, 2]²`.
    fn level(z: f64) -> NurbSurface3D {
        NurbSurface::try_new(
            1,
            1,
            vec![h(-1., -1., z), h(-1., 2., z), h(2., -1., z), h(2., 2., z)],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    /// A dome poking through a plane meets it in an oval inside both, which
    /// turns back at least twice along any direction: found, and every
    /// candidate a seed Newton can start from. Below the dome's top the
    /// plane meets nothing, and a plane above it neither.
    #[test]
    fn a_dome_through_a_plane_turns_twice() {
        let found = turning_points(&dome(2.4), &level(0.3), 20_000).unwrap();
        assert_eq!(found.candidates.len(), 2, "{found:?}");
        let found = turning_points(&dome(2.4), &level(0.9), 20_000).unwrap();
        assert!(found.candidates.is_empty(), "{found:?}");
    }

    /// Two planes crossing meet in a line, which turns nowhere.
    #[test]
    fn crossing_planes_turn_nowhere() {
        let tilted = NurbSurface::try_new(
            1,
            1,
            vec![h(0., 0., -1.), h(0., 1., -1.), h(1., 0., 1.), h(1., 1., 1.)],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let found = turning_points(&tilted, &level(0.0), 20_000).unwrap();
        assert!(found.candidates.is_empty(), "{found:?}");
        assert_eq!(found.tangent, 0);
    }

    /// A plane's normal over the whole of it is the one direction it has.
    #[test]
    fn a_planes_normal_is_sharp() {
        let piece = Piece::new(level(0.0)).unwrap();
        assert!(
            (0..3).all(|k| piece.normal[k].width().to_f64() < 1e-12)
                && piece.normal[2].definitely_greater(Scalar::ZERO),
            "{:?}",
            piece.normal
        );
    }
}
