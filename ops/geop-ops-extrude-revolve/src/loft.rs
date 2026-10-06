//! Lofting: a body through two or more planar profiles — sections — each in
//! a plane of its own, by skinning them (see [`crate::sweep::skin`]): a
//! square on one plane and a circle on another give a solid running from
//! one to the other. Two sections are joined by ruled walls, straight lines
//! between corresponding points of the two; three or more by walls running
//! smoothly through all of them, tangent across every section between (see
//! [`through`]).
//!
//! To be skinned, the sections are made to correspond:
//!
//! - **winding**: each is turned, if need be, to run the same way round
//!   seen along the loft — counter-clockwise looking back from the next
//!   section — mirroring its frame where its plane faces the other way;
//! - **matched points**: points the designer matched across the closed
//!   sections — the first of each section with each other, the second with
//!   each other, and so on — are lofted into each other: that is how one
//!   says which part of one profile goes where on the next. Every section
//!   matched has as many, running round it in the same order; it starts at
//!   its first;
//! - **pieces**: every section gets as many curves — between two matched
//!   points of a matched section as many as between the same two of every
//!   other — by halving its longest ones;
//! - **start**: a closed section that is not matched starts at whichever
//!   joint lines its joints up best with its neighbour towards the first
//!   matched section (or the section before, with none) — its shape about
//!   its centre — and an open one runs whichever way round lines its ends
//!   up;
//! - **curves**: the `i`-th curves of all sections are made compatible, one
//!   degree and one knot vector (see `NurbCurve::compatible`).
//!
//! Open chains loft into sheets: between two curves, the ruled surface
//! joining them.
//!
//! **Guide curves** shape the walls between the sections instead of letting
//! them run straight. A guide runs from a point of the first section through
//! every other to the last; where it meets each section is a matched point
//! (so guides and matching points do not mix). Between two sections, the
//! walls are the ruled surface plus a displacement that is affine across the
//! section and takes each guide's points on the sections along the guide:
//! the walls' edges there are the guides themselves (see `guided_span`).
//! One guide moves the sections along with it; two also turn and stretch
//! them; three, at most, fix any affine change of them.

use geop_core_geometry::{
    contains::curve::curve_could_contain,
    intersection::{curve_surface_intersect, refine_crossing},
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::build::BuiltBody;
use geop_ops::{Namer, Part, operation::Chain};

use crate::{
    common::{Profile, bilinear, end_point, start_point},
    plain::{Plain, V, add, scale, sub},
    sweep::{Frame, Path, Span, SweepLoop, skin},
};

/// One profile to loft through: a closed loop or an open chain, drawn in
/// `plane`'s `(u, v)` — a closed one counter-clockwise, as a sketch's outer
/// loop runs — and what its station is called. `matched`: the joints of a
/// closed loop matched with the other sections', in the order they are
/// matched (see [`mark`] and the module docs).
#[derive(Clone, Debug)]
pub struct Section<S: Scalar> {
    pub plane: CoordinateSystem<S>,
    pub profile: Profile<S>,
    pub name: String,
    pub matched: Vec<String>,
}

/// A section as it is skinned: its frame and its profile in it.
#[derive(Clone, Debug)]
struct Placed<S: Scalar> {
    frame: Frame<S>,
    profile: Profile<S>,
}

impl<S: Scalar> Placed<S> {
    /// The joints, in space, as plain numbers.
    fn joints(&self) -> GeopResult<Vec<[f64; 3]>> {
        let n = self.profile.joint_names.len();
        (0..n)
            .map(|i| {
                let curve = if i < self.profile.curves.len() {
                    self.profile.curves[i].clone()
                } else {
                    self.profile.curves[i - 1].reverse()
                };
                let cp = curve.control_points[0];
                let p = Vector2::from_array([cp[0].div(cp[2])?, cp[1].div(cp[2])?]);
                let q = self.frame.point(&p);
                Ok([q[0].to_f64(), q[1].to_f64(), q[2].to_f64()])
            })
            .collect()
    }

    /// The same section with its frame's `e2` turned round and its curves
    /// mirrored to match: the same curves in space, winding the other way
    /// in the frame.
    fn mirrored(&self) -> Self {
        let mirror = |c: &NurbCurve2D<S>| {
            let mut c = c.clone();
            for cp in &mut c.control_points {
                cp[1] = cp[1].neg();
            }
            c.recompute_aabb();
            c
        };
        Self {
            frame: Frame {
                e2: self.frame.e2.neg(),
                ..self.frame.clone()
            },
            profile: self.profile.map_curves(mirror),
        }
    }
}

fn centre(points: &[[f64; 3]]) -> [f64; 3] {
    let n = points.len() as f64;
    let mut c = [0.0; 3];
    for p in points {
        for k in 0..3 {
            c[k] += p[k] / n;
        }
    }
    c
}

fn distance_sq(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).powi(2)).sum()
}

/// The control polygon's length of `curve`: how long it is, near enough to
/// pick the longest curve to halve — a free choice.
fn polygon_length<S: Scalar>(curve: &NurbCurve2D<S>) -> f64 {
    let points: Vec<[f64; 2]> = curve
        .control_points
        .iter()
        .map(|cp| {
            let w = cp[2].to_f64();
            [cp[0].to_f64() / w, cp[1].to_f64() / w]
        })
        .collect();
    points
        .windows(2)
        .map(|w| ((w[1][0] - w[0][0]).powi(2) + (w[1][1] - w[0][1]).powi(2)).sqrt())
        .sum()
}

/// `profile` with its curve `i` halved, at the middle of its parameters:
/// the halves `X` and `X#half`, the joint between them `X@half`.
fn halved<S: Scalar>(profile: &Profile<S>, i: usize) -> GeopResult<Profile<S>> {
    let (a, b) = profile.curves[i].split_mid()?;
    let (a, b) = (
        a.with_unit_domain()?.with_unit_end_weights()?,
        b.with_unit_domain()?.with_unit_end_weights()?,
    );
    let mut out = profile.clone();
    let name = profile.curve_names[i].clone();
    out.curves.splice(i..=i, [a, b]);
    out.curve_names.insert(i + 1, format!("{name}#half"));
    out.joint_names.insert(i + 1, format!("{name}@half"));
    Ok(out)
}

/// The closed `profile` starting at its joint `k`.
pub fn starting_at<S: Scalar>(profile: &Profile<S>, k: usize) -> Profile<S> {
    let mut out = profile.clone();
    out.curves.rotate_left(k);
    out.curve_names.rotate_left(k);
    out.joint_names.rotate_left(k);
    out
}

/// The closed `profile` with a joint at `point`, and that joint's name:
/// the joint already there, or — where `point` lies along a curve — a new
/// one named `joint`, the curve split there, its second half `X#joint`.
/// Where exactly along the curve is a free choice within where `point`
/// could be: the halves are exact pieces of it either way. An error if
/// `point` is on no curve of the profile.
pub fn mark<S: Scalar>(
    profile: &Profile<S>,
    point: &Vector2<S>,
    joint: &str,
) -> GeopResult<(Profile<S>, String)> {
    const MAX_NODES: usize = 20_000;
    let size = S::from_f64(1e-7);
    let n = profile.curves.len();
    for (i, curve) in profile.curves.iter().enumerate() {
        let (t0, t1) = curve.domain();
        if curve.evaluate(t0)?.could_be_equal(point) {
            return Ok((profile.clone(), profile.joint_names[i].clone()));
        }
        let Some(t) = curve_could_contain(curve, point, MAX_NODES, size)? else {
            continue;
        };
        if t.could_be_equal(t1) {
            return Ok((profile.clone(), profile.joint_names[(i + 1) % n].clone()));
        }
        let (a, b) = curve.split(t.sharpen())?;
        let mut out = profile.clone();
        let name = profile.curve_names[i].clone();
        out.curves
            .splice(i..=i, [a.with_unit_domain()?, b.with_unit_domain()?]);
        out.curve_names.insert(i + 1, format!("{name}#{joint}"));
        out.joint_names.insert(i + 1, joint.to_string());
        return Ok((out, joint.to_string()));
    }
    Err(GeopError::new(format!(
        "the point at {point:?} is on none of the profile's curves"
    )))
}

/// Where the joints `matched` are along `profile`, which starts at the
/// first: their indices, increasing — an error if they do not run round it
/// in that order.
fn matched_indices<S: Scalar>(
    profile: &Profile<S>,
    matched: &[String],
    name: &str,
) -> GeopResult<Vec<usize>> {
    let indices = matched
        .iter()
        .map(|joint| {
            profile
                .joint_names
                .iter()
                .position(|j| j == joint)
                .ok_or_else(|| GeopError::new(format!("loft: no joint {joint} on {name}")))
        })
        .collect::<GeopResult<Vec<_>>>()?;
    if indices.windows(2).any(|w| w[0] >= w[1]) {
        return Err(GeopError::new(format!(
            "loft: the matching points of {name} do not run round it in the order they are matched with the other profiles'"
        )));
    }
    Ok(indices)
}

/// How many curves `profile` has between each two consecutive joints of
/// `indices` (see [`matched_indices`]), the last run back to the start.
fn runs<S: Scalar>(profile: &Profile<S>, indices: &[usize]) -> Vec<usize> {
    let n = profile.curves.len();
    (0..indices.len())
        .map(|j| indices.get(j + 1).copied().unwrap_or(n) - indices[j])
        .collect()
}

/// The longest curve of `profile` among `range`, by its control polygon.
fn longest_among<S: Scalar>(profile: &Profile<S>, range: std::ops::Range<usize>) -> usize {
    range
        .max_by(|&a, &b| {
            polygon_length(&profile.curves[a]).total_cmp(&polygon_length(&profile.curves[b]))
        })
        .expect("a run has curves")
}

/// The sections made to correspond, as the module docs say — closed ones
/// winding so that the loft runs against each frame's `e1 x e2`, which
/// makes the skinned walls face out.
fn correspond<S: Scalar>(sections: &[Section<S>]) -> GeopResult<Vec<Placed<S>>> {
    let closed = sections[0].profile.is_closed();
    if sections.iter().any(|s| s.profile.is_closed() != closed) {
        return Err(GeopError::new(
            "loft: the profiles are either all closed or all open",
        ));
    }
    let mut placed: Vec<Placed<S>> = sections
        .iter()
        .map(|s| Placed {
            frame: Frame {
                origin: *s.plane.origin(),
                e1: *s.plane.u(),
                e2: *s.plane.v(),
            },
            profile: s.profile.clone(),
        })
        .collect();
    let centres = placed
        .iter()
        .map(|p| Ok(centre(&p.joints()?)))
        .collect::<GeopResult<Vec<_>>>()?;

    // Winding: every closed section counter-clockwise in its frame, the
    // loft running against the frame's `e1 x e2`.
    if closed {
        let n = placed.len();
        for s in 0..n {
            let (from, to) = if s + 1 < n { (s, s + 1) } else { (s - 1, s) };
            let v = |a: [f64; 3]| Vector3::from_array(a.map(S::from_f64));
            let along = v(centres[to]).sub(&v(centres[from]));
            let frame = &placed[s].frame;
            let facing = frame.e1.prod_cross(&frame.e2).prod_dot(&along);
            if facing.definitely_greater(S::ZERO) {
                let reversed = placed[s].mirrored();
                placed[s] = Placed {
                    profile: reversed.profile.reversed(),
                    ..reversed
                };
            } else if !facing.definitely_less(S::ZERO) {
                return Err(GeopError::new(format!(
                    "loft: the loft could run along the plane of {}",
                    sections[s].name
                )));
            }
        }
    }

    // The matched points, in the order they run round the first matched
    // section as it now winds — every section's in the same order, so that
    // the `k`-th still meets the `k`-th — whatever order they were given in.
    let mut points: Vec<Vec<String>> = sections.iter().map(|s| s.matched.clone()).collect();
    if let Some(reference) = (0..sections.len()).find(|&s| !points[s].is_empty()) {
        let joints = &placed[reference].profile.joint_names;
        let mut order: Vec<usize> = (0..points[reference].len()).collect();
        order.sort_by_key(|&k| joints.iter().position(|j| *j == points[reference][k]));
        for list in points.iter_mut().filter(|l| l.len() == order.len()) {
            *list = order.iter().map(|&k| list[k].clone()).collect();
        }
    }

    // Matched points: each matched section starting at its first, the runs
    // between them as long in every matched section.
    let matched: Vec<usize> = (0..sections.len())
        .filter(|&s| !points[s].is_empty())
        .collect();
    let mut targets: Vec<usize> = Vec::new();
    if let Some(&first) = matched.first() {
        if !closed {
            return Err(GeopError::new(
                "loft: matching points are for closed profiles: open chains are lofted end to end",
            ));
        }
        let count = points[first].len();
        if let Some(&s) = matched.iter().find(|&&s| points[s].len() != count) {
            return Err(GeopError::new(format!(
                "loft: {} has {} matching points, {} has {}: every profile matched has as many",
                sections[s].name,
                points[s].len(),
                sections[first].name,
                count
            )));
        }
        targets = vec![0; count];
        for &s in &matched {
            let start = placed[s]
                .profile
                .joint_names
                .iter()
                .position(|j| *j == points[s][0])
                .ok_or_else(|| GeopError::new("loft: the first matching point is no joint"))?;
            placed[s].profile = starting_at(&placed[s].profile, start);
            let indices = matched_indices(&placed[s].profile, &points[s], &sections[s].name)?;
            for (t, run) in targets.iter_mut().zip(runs(&placed[s].profile, &indices)) {
                *t = (*t).max(run);
            }
        }
    }

    // Pieces: as many curves in every section — run by run in a matched one.
    let most = placed
        .iter()
        .map(|p| p.profile.curves.len())
        .max()
        .unwrap_or(0)
        .max(targets.iter().sum());
    // A section not matched with more curves than the runs add up to: the
    // last run takes the rest.
    let matched_total: usize = targets.iter().sum();
    if let Some(last) = targets.last_mut() {
        *last += most - matched_total;
    }
    for (s, p) in placed.iter_mut().enumerate() {
        if points[s].is_empty() {
            while p.profile.curves.len() < most {
                let longest = longest_among(&p.profile, 0..p.profile.curves.len());
                p.profile = halved(&p.profile, longest)?;
            }
            continue;
        }
        loop {
            let indices = matched_indices(&p.profile, &points[s], &sections[s].name)?;
            let short = runs(&p.profile, &indices)
                .into_iter()
                .zip(&targets)
                .position(|(run, &target)| run < target);
            let Some(j) = short else {
                break;
            };
            let end = indices
                .get(j + 1)
                .copied()
                .unwrap_or(p.profile.curves.len());
            let longest = longest_among(&p.profile, indices[j]..end);
            p.profile = halved(&p.profile, longest)?;
        }
    }

    // Start: a matched section where it is matched; the others lined up
    // with their neighbour towards the first matched one.
    let reference = matched.first().copied().unwrap_or(0);
    let order: Vec<(usize, usize)> = (reference + 1..placed.len())
        .map(|s| (s, s - 1))
        .chain((0..reference).rev().map(|s| (s, s + 1)))
        .collect();
    for (s, neighbour) in order {
        if !points[s].is_empty() {
            continue;
        }
        let before = placed[neighbour].joints()?;
        let candidates: Vec<Profile<S>> = if closed {
            (0..most)
                .map(|k| starting_at(&placed[s].profile, k))
                .collect()
        } else {
            vec![placed[s].profile.clone(), placed[s].profile.reversed()]
        };
        let mut best: Option<(f64, Profile<S>)> = None;
        for candidate in candidates {
            let joints = Placed {
                frame: placed[s].frame.clone(),
                profile: candidate.clone(),
            }
            .joints()?;
            // Closed: the shapes about their centres; open: where they are.
            let (ca, cb) = if closed {
                (centres[s], centres[neighbour])
            } else {
                ([0.0; 3], [0.0; 3])
            };
            let cost: f64 = joints
                .iter()
                .zip(&before)
                .map(|(a, b)| {
                    let a = [a[0] - ca[0], a[1] - ca[1], a[2] - ca[2]];
                    let b = [b[0] - cb[0], b[1] - cb[1], b[2] - cb[2]];
                    distance_sq(a, b)
                })
                .sum();
            if best.as_ref().is_none_or(|(c, _)| cost < *c) {
                best = Some((cost, candidate));
            }
        }
        placed[s].profile = best.expect("at least one candidate").1;
    }

    // Curves: compatible across the sections.
    for i in 0..most {
        let curves: Vec<NurbCurve2D<S>> = placed
            .iter()
            .map(|p| p.profile.curves[i].with_unit_end_weights())
            .collect::<GeopResult<_>>()?;
        for (p, curve) in placed.iter_mut().zip(NurbCurve::compatible(&curves)?) {
            p.profile.curves[i] = curve;
        }
    }
    Ok(placed)
}

// ── guide curves ────────────────────────────────────────────────────────────

/// A loft's guide, run from the first section to the last, and where it
/// crosses each section: a curve of its chain and a parameter along it.
struct Crossed<S: Scalar> {
    chain: Chain<S>,
    at: Vec<(usize, S)>,
}

/// Bounds of the search for where a guide crosses a section's plane.
const GUIDE_MAX_NODES: usize = 20_000;
const GUIDE_MIN_SUBDIVISION: f64 = 1e-7;

/// Where `guide` crosses each of `sections` (see [`Crossed`]): starting on
/// the first one's plane — run the other way if it ends there — ending on
/// the last one's, and crossing every plane between exactly once. An error,
/// naming the guide and the section, otherwise.
fn crossings<S: Scalar>(sections: &[Section<S>], guide: &Chain<S>) -> GeopResult<Crossed<S>> {
    let name = &guide.name;
    if guide.is_closed() {
        return Err(GeopError::new(format!(
            "loft: the guide {name} is a closed loop: a guide runs from the first profile to the last"
        )));
    }
    let (first, last) = (&sections[0], &sections[sections.len() - 1]);
    let on =
        |section: &Section<S>, p: &Vector3<S>| section.plane.to_uvw(p)[2].could_be_equal(S::ZERO);
    let ends = |chain: &Chain<S>| -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let joints = chain.joints()?;
        Ok((joints[0], joints[joints.len() - 1]))
    };
    let (start, _) = ends(guide)?;
    let chain = if on(first, &start) {
        guide.clone()
    } else {
        guide.reversed()
    };
    let (start, end) = ends(&chain)?;
    if !on(first, &start) || !on(last, &end) {
        return Err(GeopError::new(format!(
            "loft: the guide {name} has to run from the profile {} to the profile {}, starting and ending on them",
            first.name, last.name
        )));
    }
    let n = chain.curves.len();
    let mut at = vec![(0, S::ZERO)];
    for section in &sections[1..sections.len() - 1] {
        let ctx = with_context!(
            "where the guide {name} crosses the profile {}",
            section.name
        );
        let patch = plane_patch(&section.plane, &chain)?;
        let mut hits = Vec::new();
        for (i, curve) in chain.curves.iter().enumerate() {
            let found = curve_surface_intersect(
                curve,
                &patch,
                4,
                GUIDE_MAX_NODES,
                S::from_f64(GUIDE_MIN_SUBDIVISION),
            )
            .with_context(ctx)?;
            if found.is_coincident() {
                return Err(GeopError::new(format!(
                    "loft: the guide {name} runs along the plane of the profile {}",
                    section.name
                )));
            }
            for (t, uv) in found.into_vec() {
                // A crossing at a joint of the guide, found again at the
                // start of the curve after it.
                if i > 0 && t.could_be_equal(S::ZERO) {
                    continue;
                }
                let (t, _) = refine_crossing(curve, &patch, t, uv);
                hits.push((i, t));
            }
        }
        match hits.as_slice() {
            [hit] => at.push(*hit),
            _ => {
                return Err(GeopError::new(format!(
                    "loft: the guide {name} crosses the plane of the profile {} {} times, not once",
                    section.name,
                    hits.len()
                )));
            }
        }
    }
    at.push((n - 1, S::ONE));
    Ok(Crossed { chain, at })
}

/// A flat patch of `plane` that every crossing of `chain` with it lies on:
/// the box of the chain's control points seen in the plane, as wide again
/// on every side — a search domain, nothing more.
fn plane_patch<S: Scalar>(
    plane: &CoordinateSystem<S>,
    chain: &Chain<S>,
) -> GeopResult<geop_core_geometry::nurb_surface::NurbSurface3D<S>> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for cp in chain.curves.iter().flat_map(|c| &c.control_points) {
        let p = Vector3::from_array([cp[0].div(cp[3])?, cp[1].div(cp[3])?, cp[2].div(cp[3])?]);
        let uvw = plane.to_uvw(&p);
        for k in 0..2 {
            lo[k] = lo[k].min(uvw[k].to_f64());
            hi[k] = hi[k].max(uvw[k].to_f64());
        }
    }
    let margin = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1.0);
    let corner =
        |u: f64, v: f64| plane.uv_to_xyz(&Vector2::from_array([S::from_f64(u), S::from_f64(v)]));
    let (u0, u1, v0, v1) = (
        lo[0] - margin,
        hi[0] + margin,
        lo[1] - margin,
        hi[1] + margin,
    );
    bilinear(
        corner(u0, v0),
        corner(u1, v0),
        corner(u1, v1),
        corner(u0, v1),
    )
}

/// Sections with the guides' joints, and the guides as crossed (see [`guided`]).
type Guided<S> = (Vec<Section<S>>, Vec<Crossed<S>>);

/// `sections` with a joint wherever a guide crosses them, matched across
/// them in the order of the guides — the joint named `K,G` for the section
/// `K` and the guide `G`, unless one is there already — and the guides as
/// crossed. Guides take the place of matching points: the two do not mix.
fn guided<S: Scalar>(sections: &[Section<S>], guides: &[Chain<S>]) -> GeopResult<Guided<S>> {
    if guides.len() > MAX_GUIDES {
        return Err(GeopError::new(format!(
            "loft: {} guides given, but a loft follows at most {MAX_GUIDES}",
            guides.len()
        )));
    }
    if let Some(s) = sections.iter().find(|s| !s.matched.is_empty()) {
        return Err(GeopError::new(format!(
            "loft: the profile {} has matching points, but the guides say which points match: use one or the other",
            s.name
        )));
    }
    let crossed = guides
        .iter()
        .map(|g| crossings(sections, g))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut out = sections.to_vec();
    for (j, section) in out.iter_mut().enumerate() {
        for (guide, crossed) in guides.iter().zip(&crossed) {
            let (i, t) = crossed.at[j];
            let point = crossed.chain.curves[i].evaluate(t)?;
            let uvw = section.plane.to_uvw(&point);
            let (profile, joint) = mark(
                &section.profile,
                &Vector2::from_array([uvw[0], uvw[1]]),
                &format!("{},{}", section.name, guide.name),
            )
            .with_context(with_context!(
                "the guide {} on the profile {}",
                guide.name,
                section.name
            ))?;
            section.profile = profile;
            section.matched.push(joint);
        }
    }
    // Three guides that meet the first profile in a line cannot say how to
    // shape the sections across it.
    if guides.len() == 3 {
        let joint = |name: &String| -> GeopResult<Vector2<S>> {
            let profile = &out[0].profile;
            let i = profile
                .joint_names
                .iter()
                .position(|j| j == name)
                .expect("a guide's joint is on its profile");
            if i < profile.curves.len() {
                start_point(&profile.curves[i])
            } else {
                end_point(&profile.curves[i - 1])
            }
        };
        let q = out[0]
            .matched
            .iter()
            .map(joint)
            .collect::<GeopResult<Vec<_>>>()?;
        let (a, b) = (q[1].sub(&q[0]), q[2].sub(&q[0]));
        if a[0].mul(b[1]).sub(a[1].mul(b[0])).could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "loft: the guides {} could meet the profile {} in a line: they cannot shape it across that line",
                guides
                    .iter()
                    .map(|g| g.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
                out[0].name
            )));
        }
    }
    Ok((out, crossed))
}

/// At most this many guides: an affine map of each section is shaped by
/// them, which three points fix.
const MAX_GUIDES: usize = 3;

/// The piece of `crossed` from where it crosses section `j` to where it
/// crosses the next, as one curve on `[0, 1]`. Where it is cut is a free
/// choice within where it crosses — the section's joint is what the walls
/// meet — so the cuts are sharpened.
fn guide_piece<S: Scalar>(crossed: &Crossed<S>, j: usize) -> GeopResult<NurbCurve3D<S>> {
    let ((c0, t0), (c1, t1)) = (crossed.at[j], crossed.at[j + 1]);
    let (t0, t1) = (t0.sharpen(), t1.sharpen());
    let mut curves = Vec::new();
    for i in c0..=c1 {
        let curve = &crossed.chain.curves[i];
        let from = if i == c0 { t0 } else { S::ZERO };
        let to = if i == c1 { t1 } else { S::ONE };
        if from.could_be_equal(to) {
            continue;
        }
        curves.push(if i == c0 || i == c1 {
            curve.sub_curve(from, to)?
        } else {
            curve.clone()
        });
    }
    NurbCurve::join(&curves)
}

/// The span from the section `a` to the next, `b`, shaped by guides: one
/// piece of each (see [`guide_piece`]) and its joints `joints[k]` — the
/// names of the joint on `a` and on `b`.
///
/// The span takes the guides' degree, knots and weights, made compatible;
/// each inner row is the ruled surface between the two sections at the
/// row's Greville abscissa `ξ`, plus a displacement affine across the
/// section — an affine map of the blended profile point — that takes each
/// guide's joint to the guide's control point: the walls' edges along the
/// guides are the guides, and everything between is moved along with them.
/// Three guides fix the map; one moves the sections, two also turn and
/// stretch them along the line between their joints. The displacement is
/// worked out in plain numbers: a free choice, taken as sharp, the walls
/// all built from it.
fn guided_span<S: Scalar>(
    a: &Placed<S>,
    b: &Placed<S>,
    joints: &[(String, String)],
    pieces: &[NurbCurve3D<S>],
) -> GeopResult<Span<S>> {
    let pieces = NurbCurve::compatible(pieces)?;
    let first = &pieces[0];
    let (degree, knots) = (first.degree, first.knot_vector.clone());
    let rows = first.control_points.len();
    for (k, piece) in pieces.iter().enumerate().skip(1) {
        if !(0..rows).all(|r| piece.control_points[r][3].could_be_equal(first.control_points[r][3]))
        {
            return Err(GeopError::new(format!(
                "loft: the guides through {} and {} are rational curves of different weights: draw them alike, or as splines",
                joints[0].0, joints[k].0
            )));
        }
    }
    let joint_at = |placed: &Placed<S>, name: &str| -> GeopResult<[f64; 2]> {
        let profile = &placed.profile;
        let i = profile
            .joint_names
            .iter()
            .position(|j| j == name)
            .ok_or_else(|| GeopError::new(format!("loft: no joint {name} for a guide")))?;
        let p = if i < profile.curves.len() {
            start_point(&profile.curves[i])?
        } else {
            end_point(&profile.curves[i - 1])?
        };
        Ok([p[0].to_f64(), p[1].to_f64()])
    };
    let ends: Vec<([f64; 2], [f64; 2])> = joints
        .iter()
        .map(|(ja, jb)| Ok((joint_at(a, ja)?, joint_at(b, jb)?)))
        .collect::<GeopResult<_>>()?;
    let (fa, fb) = (Plain::of(&a.frame), Plain::of(&b.frame));
    let place = |f: &Plain, p: [f64; 2]| add(f.origin, add(scale(f.e1, p[0]), scale(f.e2, p[1])));
    let mut middle = Vec::with_capacity(rows.saturating_sub(2));
    for r in 1..rows - 1 {
        let xi = knots[r + 1..=r + degree]
            .iter()
            .map(|k| k.to_f64())
            .sum::<f64>()
            / degree as f64;
        let weight = first.control_points[r][3];
        // Each guide: where its joint lies blended across the section, and
        // how far the ruled surface there is from the guide's control point.
        let (q, d): (Vec<[f64; 2]>, Vec<V>) = ends
            .iter()
            .zip(&pieces)
            .map(|(&(ja, jb), piece)| {
                let q = [
                    (1.0 - xi) * ja[0] + xi * jb[0],
                    (1.0 - xi) * ja[1] + xi * jb[1],
                ];
                let ruled = add(scale(place(&fa, ja), 1.0 - xi), scale(place(&fb, jb), xi));
                let cp = piece.control_points[r];
                let w = cp[3].to_f64();
                let g = [cp[0].to_f64() / w, cp[1].to_f64() / w, cp[2].to_f64() / w];
                (q, sub(g, ruled))
            })
            .unzip();
        let (t, m) = affine_through(&q, &d);
        let moved = |f: &Plain| Plain {
            origin: add(f.origin, t),
            e1: add(f.e1, m[0]),
            e2: add(f.e2, m[1]),
        };
        middle.push([
            (moved(&fa).frame(), weight.mul(S::from_f64(1.0 - xi))),
            (moved(&fb).frame(), weight.mul(S::from_f64(xi))),
        ]);
    }
    Ok(Span::Blend {
        degree,
        knots,
        middle,
    })
}

/// The affine map `p -> t + m[0] p.x + m[1] p.y` taking each `q[k]` to
/// `d[k]`, for one to three points: a shift for one, the least stretch
/// along the line between them for two, exact for three not in a line.
fn affine_through(q: &[[f64; 2]], d: &[V]) -> (V, [V; 2]) {
    let m = match q.len() {
        1 => [[0.0; 3]; 2],
        2 => {
            let dq = [q[1][0] - q[0][0], q[1][1] - q[0][1]];
            let dd = sub(d[1], d[0]);
            let n = dq[0] * dq[0] + dq[1] * dq[1];
            [scale(dd, dq[0] / n), scale(dd, dq[1] / n)]
        }
        _ => {
            let (q1, q2) = (
                [q[1][0] - q[0][0], q[1][1] - q[0][1]],
                [q[2][0] - q[0][0], q[2][1] - q[0][1]],
            );
            let (d1, d2) = (sub(d[1], d[0]), sub(d[2], d[0]));
            // [d1 d2] [q1 q2]^-1, the points as columns.
            let det = q1[0] * q2[1] - q2[0] * q1[1];
            let inv = [[q2[1] / det, -q2[0] / det], [-q1[1] / det, q1[0] / det]];
            [
                add(scale(d1, inv[0][0]), scale(d2, inv[1][0])),
                add(scale(d1, inv[0][1]), scale(d2, inv[1][1])),
            ]
        }
    };
    let t = sub(d[0], add(scale(m[0], q[0][0]), scale(m[1], q[0][1])));
    (t, m)
}

/// Lofts through `sections` (see the module docs): closed loops into a solid
/// named `solid` — capped by the first and last sections — or, without one,
/// into sheets; open chains into sheets only.
///
/// Named after the first section's curves `X` and joints `P` and the
/// sections' names `K` (see [`skin`]): the walls `N(X)` — `N(X,K>L)` between
/// the sections `K` and `L` when there are more than two — the curves at the
/// sections `N(X,K)`, the edges between them `N(P)` or `N(P,K>L)`, the
/// vertices `N(P,K)`, the caps `N(start)` and `N(end)`.
/// The spans of a loft running smoothly through the sections `placed`, one
/// cubic between each two: its inner control rows each a section and a
/// third of its tangent there, so that the walls on either side of a
/// section meet it with one tangent plane.
///
/// The tangent at a section is Bessel's, from the sections either side,
/// each parametrized by how far apart their centres are; at the first and
/// the last, the parabola's through the three there. Where the sections
/// are spaced is a free choice of the loft's shape, as any spline through
/// them is; spacing by their distances keeps a section far from the others
/// from pulling the walls past it. Sections with their centres in one
/// place are spaced evenly.
fn through<S: Scalar>(placed: &[Placed<S>]) -> GeopResult<Vec<Span<S>>> {
    let n = placed.len() - 1;
    let centres = placed
        .iter()
        .map(|p| Ok(centre(&p.joints()?)))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut d: Vec<f64> = (0..n)
        .map(|j| distance_sq(centres[j], centres[j + 1]).sqrt())
        .collect();
    if d.iter().any(|&d| d <= 0.0) {
        d = vec![1.0; n];
    }
    // Rows as coefficients of the sections: `unit(s)` is section `s`.
    let unit = |s: usize| -> Vec<f64> { (0..=n).map(|k| if k == s { 1.0 } else { 0.0 }).collect() };
    let combine = |a: &[f64], x: f64, b: &[f64], y: f64| -> Vec<f64> {
        a.iter().zip(b).map(|(a, b)| x * a + y * b).collect()
    };
    // The chord of span `j`, per unit of its length.
    let chord = |j: usize| combine(&unit(j + 1), 1.0 / d[j], &unit(j), -1.0 / d[j]);
    let mut tangent: Vec<Vec<f64>> = vec![Vec::new(); n + 1];
    for j in 1..n {
        let w = d[j - 1] + d[j];
        tangent[j] = combine(&chord(j - 1), d[j] / w, &chord(j), d[j - 1] / w);
    }
    tangent[0] = combine(&chord(0), 2.0, &tangent[1], -1.0);
    tangent[n] = combine(&chord(n - 1), 2.0, &tangent[n - 1], -1.0);
    let row = |coefficients: Vec<f64>| -> Vec<(usize, S)> {
        coefficients
            .into_iter()
            .enumerate()
            .filter(|&(_, c)| c != 0.0)
            .map(|(s, c)| (s, S::from_f64(c)))
            .collect()
    };
    let (zero, one) = (S::ZERO, S::ONE);
    Ok((0..n)
        .map(|j| Span::Through {
            degree: 3,
            knots: vec![zero, zero, zero, zero, one, one, one, one],
            middle: vec![
                row(combine(&unit(j), 1.0, &tangent[j], d[j] / 3.0)),
                row(combine(&unit(j + 1), 1.0, &tangent[j + 1], -d[j] / 3.0)),
            ],
        })
        .collect())
}

pub fn loft<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: Option<&str>,
    sections: &[Section<S>],
    guides: &[Chain<S>],
) -> GeopResult<BuiltBody> {
    let ctx = with_context!(
        "loft({}, through {:?})",
        namer.root(),
        sections.iter().map(|s| &s.name).collect::<Vec<_>>()
    );
    if sections.len() < 2 {
        return Err(GeopError::new(format!(
            "loft: needs two profiles or more, not {}",
            sections.len()
        )))
        .with_context(ctx);
    }
    let (sections, crossed) = if guides.is_empty() {
        (sections.to_vec(), Vec::new())
    } else {
        guided(sections, guides).with_context(ctx)?
    };
    let sections = sections.as_slice();
    let placed = correspond(sections).with_context(ctx)?;
    let spans = sections.len() - 1;
    let through = if crossed.is_empty() && spans > 1 {
        Some(through(&placed).with_context(ctx)?)
    } else {
        None
    };
    let span_list = (0..spans)
        .map(|j| {
            if let Some(through) = &through {
                return Ok(through[j].clone());
            }
            if crossed.is_empty() {
                return Ok(Span::Line);
            }
            let joints: Vec<(String, String)> = (0..guides.len())
                .map(|k| {
                    (
                        sections[j].matched[k].clone(),
                        sections[j + 1].matched[k].clone(),
                    )
                })
                .collect();
            let pieces = crossed
                .iter()
                .map(|c| guide_piece(c, j))
                .collect::<GeopResult<Vec<_>>>()?;
            guided_span(&placed[j], &placed[j + 1], &joints, &pieces).with_context(with_context!(
                "between the profiles {} and {}",
                sections[j].name,
                sections[j + 1].name
            ))
        })
        .collect::<GeopResult<Vec<_>>>()
        .with_context(ctx)?;
    let path = Path {
        stations: placed.iter().map(|p| p.frame.clone()).collect(),
        spans: span_list,
        closed: false,
        along_normal: false,
        station_names: sections.iter().map(|s| s.name.clone()).collect(),
        span_names: if spans == 1 {
            vec![None]
        } else {
            sections
                .windows(2)
                .map(|w| Some(format!("{}>{}", w[0].name, w[1].name)))
                .collect()
        },
    };
    let loops: Vec<Vec<SweepLoop<S>>> = placed
        .into_iter()
        .map(|p| vec![SweepLoop::plain(p.profile)])
        .collect();
    skin(part, namer, &path, &loops, solid).with_context(ctx)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{arc2, line2, polygon, sqrt2_over_2};
    use geop_core_math::for_all_scalars;
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// The plane through `origin` spanned by `u` and `v`.
    fn plane<S: Scalar>(origin: Vector3<S>, u: Vector3<S>, v: Vector3<S>) -> CoordinateSystem<S> {
        CoordinateSystem::try_new(origin, u, v, u.prod_cross(&v)).unwrap()
    }

    /// The plane `z = height`, along `x` and `y`.
    fn level<S: Scalar>(height: f64) -> CoordinateSystem<S> {
        plane(v3(0., 0., height), v3(1., 0., 0.), v3(0., 1., 0.))
    }

    /// A circle of radius `r` around the origin, as four quarter arcs,
    /// counter-clockwise.
    fn circle<S: Scalar>(r: f64) -> Vec<NurbCurve2D<S>> {
        let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
        (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                arc2(
                    v2(a.0, a.1),
                    v2(a.0 + b.0, a.1 + b.1),
                    v2(b.0, b.1),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect()
    }

    /// The regular polygon of `n` corners on the circle of radius `r`,
    /// counter-clockwise, its first corner at angle `phase` (in turns).
    fn regular<S: Scalar>(n: usize, r: f64, phase: f64) -> Vec<NurbCurve2D<S>> {
        let corners: Vec<Vector2<S>> = (0..n)
            .map(|i| {
                let a = std::f64::consts::TAU * (phase + i as f64 / n as f64);
                v2(r * a.cos(), r * a.sin())
            })
            .collect();
        polygon(&corners).unwrap()
    }

    fn section<S: Scalar>(
        name: &str,
        plane: CoordinateSystem<S>,
        curves: Vec<NurbCurve2D<S>>,
    ) -> Section<S> {
        Section {
            plane,
            profile: Profile::closed(curves).with_prefix(&format!("{name},")),
            name: name.into(),
            matched: Vec::new(),
        }
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
    }

    /// Lofts `sections` into a solid in a fresh part, and checks it is valid.
    fn lofted<S: Scalar>(sections: &[Section<S>]) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("loft", "l").unwrap();
        loft(&mut part, &namer, Some(&namer.root()), sections, &[]).unwrap();
        part.check_names().unwrap();
        assert_valid(part.topology());
        part
    }

    /// A square below, a circle above: four ruled walls and two caps.
    fn check_loft_square_to_circle<S: Scalar>() {
        let part = lofted::<S>(&[
            section("a", level(0.0), regular(4, 1.0, 0.125)),
            section("b", level(2.0), circle(0.5)),
        ]);
        let model = part.topology();
        assert_eq!(model.faces.len(), 4 + 2);
        assert_eq!(model.vertices.len(), 8);
        // Every vertex of the top section lies on its circle.
        for v in model.vertices.values() {
            if v.point[2].could_be_equal(S::from_f64(2.0)) {
                let (x, y) = (v.point[0], v.point[1]);
                assert!(x.mul(x).add(y.mul(y)).could_be_equal(S::from_f64(0.25)));
            }
        }
    }
    #[test]
    fn loft_square_to_circle() {
        for_all_scalars!(check_loft_square_to_circle);
    }

    /// The upper plane tilted, and facing down: still a valid solid.
    fn check_loft_to_a_tilted_plane_facing_the_other_way<S: Scalar>() {
        let tilted = plane(v3::<S>(0.2, 0., 2.), v3(0.8, 0., 0.6), v3(0., -1., 0.));
        lofted::<S>(&[
            section("a", level(0.0), regular(4, 1.0, 0.0)),
            section("b", tilted, circle(0.5)),
        ]);
    }
    #[test]
    fn loft_to_a_tilted_plane_facing_the_other_way() {
        for_all_scalars!(check_loft_to_a_tilted_plane_facing_the_other_way);
    }

    /// Through three sections: two layers of walls, named after the spans
    /// between the sections.
    fn check_loft_through_three_sections<S: Scalar>() {
        let part = lofted::<S>(&[
            section("a", level(0.0), regular(4, 1.0, 0.125)),
            section("b", level(1.0), circle(0.5)),
            section("c", level(2.5), regular(4, 0.8, 0.0)),
        ]);
        assert_eq!(part.topology().faces.len(), 2 * 4 + 2);
        assert!(part.face_id("loft(l,a,c0,a>b)").is_ok());
        assert!(part.face_id("loft(l,a,c0,b>c)").is_ok());
        assert!(part.edge_id("loft(l,a,c0,c)").is_ok());
    }
    #[test]
    fn loft_through_three_sections() {
        for_all_scalars!(check_loft_through_three_sections);
    }

    /// Through three circles of different sizes, the walls either side of
    /// the middle one meet it with one tangent plane: the loft has no crease
    /// there, as ruled walls would.
    fn check_loft_through_three_sections_is_smooth<S: Scalar>() {
        let part = lofted::<S>(&[
            section("a", level(0.0), circle(1.0)),
            section("b", level(1.0), circle(1.5)),
            section("c", level(3.0), circle(0.8)),
        ]);
        let model = part.topology();
        let surface = |name: &str| &model.faces[&part.face_id(name).unwrap()].surface;
        let (below, above) = (surface("loft(l,a,c0,a>b)"), surface("loft(l,a,c0,b>c)"));
        for v in [0.1, 0.5, 0.9] {
            let normal = |surface: &geop_core_geometry::nurb_surface::NurbSurface3D<S>, u: S| {
                let (v0, v1) = surface.domain_v();
                let v = S::from_f64(v0.to_f64() + (v1.to_f64() - v0.to_f64()) * v);
                surface.normal(u, v).unwrap()
            };
            let (below, above) = (
                normal(below, below.domain_u().1),
                normal(above, above.domain_u().0),
            );
            assert!(
                below.could_be_equal(&above),
                "the walls meet the middle circle with the normals {below:?} below and {above:?} above"
            );
        }
    }
    #[test]
    fn loft_through_three_sections_is_smooth() {
        for_all_scalars!(check_loft_through_three_sections_is_smooth);
    }

    /// A triangle and a circle: the triangle's longest side is halved to
    /// match the circle's four quarters.
    fn check_loft_triangle_to_circle<S: Scalar>() {
        let part = lofted::<S>(&[
            section("a", level(0.0), regular(3, 1.0, 0.0)),
            section("b", level(1.5), circle(0.6)),
        ]);
        assert_eq!(part.topology().faces.len(), 4 + 2);
        // All three sides are as long: whichever was halved.
        assert!((0..3).any(|k| part.vertex_id(&format!("loft(l,a,c{k}@half,b)")).is_ok()));
    }
    #[test]
    fn loft_triangle_to_circle() {
        for_all_scalars!(check_loft_triangle_to_circle);
    }

    /// A hexagon and a circle: the circle's quarters are halved — rational
    /// arcs, brought back to end weights of one.
    fn check_loft_hexagon_to_circle<S: Scalar>() {
        let part = lofted::<S>(&[
            section("a", level(0.0), regular(6, 1.0, 0.0)),
            section("b", level(1.5), circle(0.6)),
        ]);
        assert_eq!(part.topology().faces.len(), 6 + 2);
    }
    #[test]
    fn loft_hexagon_to_circle() {
        for_all_scalars!(check_loft_hexagon_to_circle);
    }

    /// Two open curves loft into the ruled sheet between them.
    fn check_loft_between_two_curves_is_a_ruled_sheet<S: Scalar>() {
        let line = vec![line2(v2(-1., 0.), v2(1., 0.)).unwrap()];
        let arc = vec![arc2(v2(1., 0.), v2(1., 1.), v2(0., 1.), sqrt2_over_2()).unwrap()];
        let open = |name: &str, plane, curves| Section {
            plane,
            profile: Profile::open(curves).with_prefix(&format!("{name},")),
            name: name.to_string(),
            matched: Vec::new(),
        };
        let mut part = Part::<S>::new();
        let namer = Namer::new("loft", "l").unwrap();
        let built = loft(
            &mut part,
            &namer,
            None,
            &[open("a", level(0.0), line), open("b", level(1.0), arc)],
            &[],
        )
        .unwrap();
        assert!(built.solid.is_none());
        part.check_names().unwrap();
        if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
            panic!("{e:?}");
        }
        assert_eq!(part.topology().faces.len(), 1);
    }
    #[test]
    fn loft_between_two_curves_is_a_ruled_sheet() {
        for_all_scalars!(check_loft_between_two_curves_is_a_ruled_sheet);
    }

    /// One profile is not a loft.
    #[test]
    fn loft_of_one_profile_is_refused() {
        use geop_core_math::scalars::ScalInF64 as S;
        let mut part = Part::<S>::new();
        let namer = Namer::new("loft", "l").unwrap();
        let only = section("a", level(0.0), regular(4, 1.0, 0.0));
        assert!(loft(&mut part, &namer, Some("loft(l)"), &[only], &[]).is_err());
    }

    // ── guide curves ────────────────────────────────────────────────────────

    fn square<S: Scalar>(half: f64) -> Vec<NurbCurve2D<S>> {
        polygon(&[
            v2(-half, -half),
            v2(half, -half),
            v2(half, half),
            v2(-half, half),
        ])
        .unwrap()
    }

    /// The quadratic Bézier guide `name` through the control points `points`.
    fn guide<S: Scalar>(name: &str, points: [[f64; 3]; 3]) -> Chain<S> {
        let f = S::from_f64;
        let curve = NurbCurve::try_new(
            2,
            points
                .iter()
                .map(|p| {
                    geop_core_math::vector::Vector4::from_array([f(p[0]), f(p[1]), f(p[2]), f(1.0)])
                })
                .collect(),
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        Chain {
            name: name.into(),
            curves: vec![curve],
            curve_names: vec![format!("{name},c")],
            joint_names: vec![format!("{name},s"), format!("{name},e")],
        }
    }

    /// Lofts `sections` along `guides` into a solid in a fresh part, and
    /// checks it is valid.
    fn guided_loft<S: Scalar>(sections: &[Section<S>], guides: &[Chain<S>]) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("loft", "l").unwrap();
        loft(&mut part, &namer, Some(&namer.root()), sections, guides).unwrap();
        part.check_names().unwrap();
        assert_valid(part.topology());
        part
    }

    /// The edge `name` of `part` runs along `guide`: one point for one at
    /// every parameter — the walls' edge there is the guide itself.
    fn assert_follows<S: Scalar>(part: &Part<S>, name: &str, guide: &Chain<S>) {
        let edge = part.edge_id(name).unwrap();
        let curve = &part.topology().edges[&edge].curve;
        for i in 0..=8 {
            let t = S::from_f64(i as f64 / 8.0);
            let (a, b) = (
                curve.evaluate(t).unwrap(),
                guide.curves[0].evaluate(t).unwrap(),
            );
            // The edge's inner control points are the guide's, but for the
            // rounding of the shape between, a free choice in plain numbers.
            for k in 0..3 {
                assert!(
                    (a[k].to_f64() - b[k].to_f64()).abs() < 1e-12,
                    "at {t:?}: {a:?} vs {b:?}"
                );
            }
        }
    }

    /// Two squares, one guide bowing out from a corner of one to the same
    /// corner of the other: the walls bulge with it, their edge from that
    /// corner the guide.
    fn check_loft_with_one_guide<S: Scalar>() {
        let g = guide::<S>("g", [[1., 1., 0.], [2., 2., 1.], [1., 1., 2.]]);
        let part = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), square(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_eq!(part.topology().faces.len(), 4 + 2);
        assert_follows(&part, "loft(l,a,p2)", &g);
        // The other corners move along with it: half way up, the whole
        // section shifted by half the guide's bulge.
        let edge = part.edge_id("loft(l,a,p0)").unwrap();
        let middle = part.topology().edges[&edge]
            .curve
            .evaluate(S::from_f64(0.5))
            .unwrap();
        for (k, want) in [-0.5, -0.5, 1.0].into_iter().enumerate() {
            assert!((middle[k].to_f64() - want).abs() < 1e-12, "{middle:?}");
        }
    }
    #[test]
    fn loft_with_one_guide() {
        for_all_scalars!(check_loft_with_one_guide);
    }

    /// Two guides bowing out from opposite corners: the sections stretch
    /// along the diagonal between them, both edges the guides.
    fn check_loft_with_two_guides<S: Scalar>() {
        let g = guide::<S>("g", [[1., 1., 0.], [2., 2., 1.], [1., 1., 2.]]);
        let h = guide::<S>("h", [[-1., -1., 0.], [-2., -2., 1.], [-1., -1., 2.]]);
        let part = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), square(1.0)),
            ],
            &[h.clone(), g.clone()],
        );
        assert_follows(&part, "loft(l,a,p2)", &g);
        assert_follows(&part, "loft(l,a,p0)", &h);
    }
    #[test]
    fn loft_with_two_guides() {
        for_all_scalars!(check_loft_with_two_guides);
    }

    /// A guide meeting the profiles along their sides, not at a corner:
    /// each side split where the guide meets it.
    fn check_guide_splits_the_sides_it_meets<S: Scalar>() {
        let g = guide::<S>("g", [[1., 0., 0.], [1.5, 0., 1.], [1., 0., 2.]]);
        let part = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), circle(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_follows(&part, "loft(l,a,g)", &g);
    }
    #[test]
    fn guide_splits_the_sides_it_meets() {
        for_all_scalars!(check_guide_splits_the_sides_it_meets);
    }

    /// Through three profiles: the guide crosses the middle one at its
    /// corner, and is followed on either side of it.
    fn check_loft_with_a_guide_through_three_profiles<S: Scalar>() {
        let g = guide::<S>("g", [[1., 1., 0.], [1.4, 1.4, 1.], [1., 1., 2.]]);
        let part = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(1.0), square(1.2)),
                section("c", level(2.0), square(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_eq!(part.topology().faces.len(), 2 * 4 + 2);
        let corner = part.vertex_id("loft(l,a,p2,b)").unwrap();
        let p = part.topology().get_vertex(corner).unwrap().point;
        assert!(p.could_be_equal(&v3(1.2, 1.2, 1.0)), "{p:?}");
    }
    #[test]
    fn loft_with_a_guide_through_three_profiles() {
        for_all_scalars!(check_loft_with_a_guide_through_three_profiles);
    }

    /// What a guided loft refuses, by name.
    fn check_guided_loft_refuses_what_it_cannot_build<S: Scalar>() {
        let sections = [
            section("a", level(0.0), square::<S>(1.0)),
            section("b", level(2.0), square(1.0)),
        ];
        let refused = |sections: &[Section<S>], guides: &[Chain<S>], says: &str| {
            let mut part = Part::<S>::new();
            let namer = Namer::new("loft", "l").unwrap();
            let error = loft(&mut part, &namer, Some("loft(l)"), sections, guides).unwrap_err();
            assert!(error.root_message().contains(says), "{error:?}");
        };
        // Starting off the first profile's plane.
        refused(
            &sections,
            &[guide("off", [[1., 1., 0.5], [2., 2., 1.], [1., 1., 2.]])],
            "the guide off has to run from the profile a to the profile b",
        );
        // Starting on its plane, but off the profile.
        refused(
            &sections,
            &[guide("out", [[3., 1., 0.], [2., 2., 1.], [1., 1., 2.]])],
            "is on none of the profile's curves",
        );
        // With matching points too.
        let mut matched = sections.to_vec();
        matched[0].matched = vec!["a,p0".into()];
        matched[1].matched = vec!["b,p0".into()];
        refused(
            &matched,
            &[guide("g", [[1., 1., 0.], [2., 2., 1.], [1., 1., 2.]])],
            "use one or the other",
        );
        // Four guides.
        let four: Vec<Chain<S>> = (0..4)
            .map(|k| {
                let (x, y) = [(1., 1.), (-1., 1.), (-1., -1.), (1., -1.)][k];
                guide(
                    &format!("g{k}"),
                    [[x, y, 0.], [2. * x, 2. * y, 1.], [x, y, 2.]],
                )
            })
            .collect();
        refused(&sections, &four, "at most 3");
    }
    #[test]
    fn guided_loft_refuses_what_it_cannot_build() {
        for_all_scalars!(check_guided_loft_refuses_what_it_cannot_build);
    }
}
