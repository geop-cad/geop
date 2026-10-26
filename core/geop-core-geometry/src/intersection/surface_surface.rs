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
//! another, a node where two branches cross. Those pairs are subdivided only
//! down to a coarser handoff and then set aside ([`TurningPoints::tangent`]),
//! for [`refine_node`] to find the isolated nodes among them: a loop
//! shrinking into a tangency is the one case left to the caller to notice,
//! as a face the boolean cannot classify.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

use geop_core_math::{
    matrix::Matrix, solvers::linear::solve_linear_system, union_find::UnionFind, vector::Vector,
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
    /// Clusters of the pairs of patches set aside because the normals could
    /// be parallel over them (see the module docs): where the surfaces may
    /// touch, along a curve, over an area or at isolated points, the nodes
    /// where branches cross ([`refine_node`] tells those apart).
    pub tangent: Vec<Candidate>,
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
        tangent: Vec::new(),
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
    // Each leaf's seed and box, regular and tangent apart.
    let mut leaves: [Vec<([Scalar; 4], [Scalar; 4])>; 2] = [Vec::new(), Vec::new()];
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
                leaves[0].len()
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
            let ([(ua, ua_box), (va, va_box)], [(ub, ub_box), (vb, vb_box)]) =
                (pa.middle()?, pb.middle()?);
            leaves[usize::from(!regular)]
                .push(([ua, va, ub, vb], [ua_box, va_box, ub_box, vb_box]));
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

    let [regular, tangent] = leaves;
    found.candidates = clusters(&regular);
    found.tangent = clusters(&tangent);
    Ok(found)
}

/// Clusters of `leaves` (each a seed and a box) whose boxes touch: one
/// point found from several sides, or several closer than the handoff. Each
/// gets the seed of the member nearest the middle of the cluster: a free
/// choice, taken where Newton has least far to go to any of it.
fn clusters(leaves: &[([Scalar; 4], [Scalar; 4])]) -> Vec<Candidate> {
    let mut sets = UnionFind::new(leaves.len());
    for i in 0..leaves.len() {
        for j in 0..i {
            let touch = (0..4).all(|k| {
                !leaves[i].1[k].definitely_less(leaves[j].1[k].lower())
                    && !leaves[i].1[k].definitely_greater(leaves[j].1[k].upper())
            });
            if touch {
                sets.union(i, j);
            }
        }
    }
    let mut groups = sets.groups();
    for group in &mut groups {
        group.sort();
    }
    groups.sort_by_key(|g| g[0]);
    groups
        .into_iter()
        .map(|members| {
            let whole: [Scalar; 4] = std::array::from_fn(|k| {
                members
                    .iter()
                    .map(|&i| leaves[i].1[k])
                    .reduce(Scalar::union)
                    .expect("a cluster has a member")
            });
            let middle = whole.map(|x| x.midpoint().to_f64());
            let seed = members
                .iter()
                .map(|&i| leaves[i].0)
                .min_by(|x, y| {
                    let dist = |s: &[Scalar; 4]| {
                        (0..4)
                            .map(|k| (s[k].to_f64() - middle[k]).powi(2))
                            .sum::<f64>()
                    };
                    dist(x).total_cmp(&dist(y))
                })
                .expect("a cluster has a member");
            Candidate {
                seed,
                cluster: whole,
            }
        })
        .collect()
}

/// Newton steps [`refine_node`] takes at most.
const NODE_ITERATIONS: usize = 30;

/// The residual of the node system at `x = (u_a, v_a, u_b, v_b)` and its
/// Jacobian: `S_a - S_b`, and the cross product of the two (unnormalized)
/// normals `N = S_u × S_v` along two tangents `t` of `a`, which vanish
/// together where the surfaces touch.
fn node_system(
    a: &NurbSurface3D,
    b: &NurbSurface3D,
    [ua, va, ub, vb]: [Scalar; 4],
    t: &[Vector3; 2],
) -> GeopResult<([Scalar; 5], [[Scalar; 4]; 5])> {
    let (pa, pb) = (a.evaluate(ua, va)?, b.evaluate(ub, vb)?);
    let (sau, sav) = a.derivatives(ua, va)?;
    let (sbu, sbv) = b.derivatives(ub, vb)?;
    let [sauu, sauv, savv] = a.second_derivatives(ua, va)?;
    let [sbuu, sbuv, sbvv] = b.second_derivatives(ub, vb)?;
    let (na, nb) = (sau.prod_cross(&sav), sbu.prod_cross(&sbv));
    let na_u = sauu.prod_cross(&sav).add(&sau.prod_cross(&sauv));
    let na_v = sauv.prod_cross(&sav).add(&sau.prod_cross(&savv));
    let nb_u = sbuu.prod_cross(&sbv).add(&sbu.prod_cross(&sbuv));
    let nb_v = sbuv.prod_cross(&sbv).add(&sbu.prod_cross(&sbvv));
    let c = na.prod_cross(&nb);
    let dc = [
        na_u.prod_cross(&nb),
        na_v.prod_cross(&nb),
        na.prod_cross(&nb_u),
        na.prod_cross(&nb_v),
    ];
    let delta = pa.sub(&pb);
    let mut f = [Scalar::ZERO; 5];
    let mut j = [[Scalar::ZERO; 4]; 5];
    for k in 0..3 {
        f[k] = delta[k];
        j[k] = [sau[k], sav[k], sbu[k].neg(), sbv[k].neg()];
    }
    for (row, tangent) in t.iter().enumerate() {
        f[3 + row] = c.prod_dot(tangent);
        j[3 + row] = dc.map(|d| d.prod_dot(tangent));
    }
    Ok((f, j))
}

/// The Gauss-Newton step `-(J^T J)^-1 J^T f`, with `J^T J` taken sharp when
/// `sharp` (a preconditioner: the step's width is then the residual's) and
/// over the box otherwise, where solving it at all proves `J` of full rank
/// over the whole box.
fn node_step(f: &[Scalar; 5], j: &[[Scalar; 4]; 5], sharp: bool) -> GeopResult<[Scalar; 4]> {
    let mut jtj = [[Scalar::ZERO; 4]; 4];
    let mut jtf = [Scalar::ZERO; 4];
    for r in 0..4 {
        for c in 0..4 {
            let sum = (0..5).fold(Scalar::ZERO, |sum, k| sum.add(j[k][r].mul(j[k][c])));
            jtj[r][c] = if sharp { sum.sharpen() } else { sum };
        }
        jtf[r] = (0..5)
            .fold(Scalar::ZERO, |sum, k| sum.add(j[k][r].mul(f[k])))
            .neg();
    }
    let step = solve_linear_system(&Matrix::from_rows(jtj), &Vector::from_array(jtf))?;
    Ok(step.to_array())
}

/// The node near `seed` where `a` and `b` touch at an isolated point —
/// branches of the curve they meet in cross there, or they touch at that
/// point alone — as its parameters `(u_a, v_a, u_b, v_b)` and the point,
/// or `None` where Newton finds no such point, or one not isolated: where
/// the surfaces touch along a curve or over an area, the system is singular
/// and no node is claimed.
///
/// Gauss-Newton on `S_a = S_b` with the normals parallel, five equations in
/// four unknowns, each iterate sharpened as the seed of the next but the
/// last, whose width says how well the data pins the node down (see
/// `AGENTS.md`). The answer is then checked against every condition it was
/// asked to meet, over its box: both surfaces could be there, the normals
/// could be parallel, and the system is of full rank all over it.
pub fn refine_node(
    a: &NurbSurface3D,
    b: &NurbSurface3D,
    seed: [Scalar; 4],
) -> Option<([Scalar; 4], Vector3)> {
    let clamp = |x: [Scalar; 4]| -> [Scalar; 4] {
        let domains = [a.domain_u(), a.domain_v(), b.domain_u(), b.domain_v()];
        std::array::from_fn(|k| x[k].clamped(domains[k].0, domains[k].1))
    };
    let mut x = clamp(seed.map(Scalar::sharpen));
    let tangents = |x: &[Scalar; 4]| -> Option<[Vector3; 2]> {
        let (su, sv) = a.derivatives(x[0], x[1]).ok()?;
        Some([su.sharpen(), sv.sharpen()])
    };
    let t = tangents(&x)?;
    let mut answer = None;
    for _ in 0..NODE_ITERATIONS {
        let (f, j) = node_system(a, b, x, &t).ok()?;
        let step = node_step(&f, &j, true).ok()?;
        let honest = clamp(std::array::from_fn(|k| x[k].add(step[k])));
        if f.iter().all(|r| r.could_be_equal(Scalar::ZERO)) {
            answer = Some(honest);
            break;
        }
        x = clamp(honest.map(Scalar::sharpen));
    }
    let x = answer?;
    let (pa, pb) = (a.evaluate(x[0], x[1]).ok()?, b.evaluate(x[2], x[3]).ok()?);
    if !pa.could_be_equal(&pb) {
        return None;
    }
    let (f, j) = node_system(a, b, x, &t).ok()?;
    if !f.iter().all(|r| r.could_be_equal(Scalar::ZERO)) {
        return None;
    }
    node_step(&f, &j, false).ok()?;
    Some((x, pa.union(&pb)))
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
        assert!(found.tangent.is_empty(), "{found:?}");
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

    /// Two cylinders of radius one half, around `z` and around `x`, each
    /// turned over a half turn with the point `(0, 0.5, 0)` in the middle of
    /// it: their walls meet in two ellipses crossing there, the walls
    /// tangent. The search sets the node aside with the tangent pairs, and
    /// `refine_node` finds it there.
    #[test]
    fn crossing_cylinders_have_a_node() {
        use crate::shape::Axis;
        let v = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
        let pi = std::f64::consts::PI;
        let around_z = NurbSurface3D::revolve(
            &crate::test_fixtures::line([0.5, 0.0, -1.0], [0.5, 0.0, 1.0]),
            &Axis::try_new(v([0.0, 0.0, 0.0]), v([0.0, 0.0, 1.0])).unwrap(),
            pi / 4.0,
            3.0 * pi / 4.0,
        )
        .unwrap();
        let around_x = NurbSurface3D::revolve(
            &crate::test_fixtures::line([-1.0, 0.5, 0.0], [1.0, 0.5, 0.0]),
            &Axis::try_new(v([0.0, 0.0, 0.0]), v([1.0, 0.0, 0.0])).unwrap(),
            -pi / 4.0,
            pi / 4.0,
        )
        .unwrap();
        let found = turning_points(&around_z, &around_x, 20_000).unwrap();
        let nodes: Vec<Vector3> = found
            .tangent
            .iter()
            .filter_map(|c| refine_node(&around_z, &around_x, c.seed))
            .map(|(_, p)| p)
            .collect();
        assert_eq!(nodes.len(), 1, "{nodes:?} from {:?}", found.tangent);
        assert!(nodes[0].could_be_equal(&v([0.0, 0.5, 0.0])), "{nodes:?}");
        assert!(
            (0..3).all(|k| nodes[0][k].width().to_f64() < 1e-9),
            "{nodes:?}"
        );
    }
}
