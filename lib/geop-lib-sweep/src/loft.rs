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
//!   its first. Between two matched points, the sections' corners are
//!   matched too, wherever every matched section has as many there;
//! - **corners**: with no points matched, the sections' corners — the
//!   joints where the profile's tangent turns, which the designer sees;
//!   not the kernel's own split of an arc at a quarter turn, nor a sketch
//!   point where two curves meet tangentially (see `Profile::corners`) —
//!   are the matched points, in cyclic order, each section turned to line
//!   its corners up best with the previous section's that has any. Every
//!   section with corners has as many, or the loft is refused by name: a
//!   D of two corners into a triangle needs matched points or guides to
//!   say what goes where. A section without corners — a circle — is lined
//!   up as a whole, below;
//! - **runs**: between two matched points, the smooth run of each matched
//!   section is cut where any of the others has a joint, at the same
//!   fraction of the run's length (see `RunCuts::merged`): the flat bottom of a
//!   D meets the flat bottom of the next, whatever pieces their arcs come
//!   in. A section not matched gets as many curves as the others have, by
//!   halving its longest ones; where it has more, the others' longest
//!   stretches are cut in the middle;
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
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    plain::{self, P3, add, distance, dot, inverse2, scale, sub},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::build::BuiltBody;
use geop_ops_model::{Chain, NamedModel, Namer, Profile};

use crate::{
    plain::Plain,
    sweep::{Frame, Path, Span, SweepLoop, skin},
};

/// One profile to loft through: a closed loop or an open chain, drawn in
/// `plane`'s `(u, v)` — a closed one counter-clockwise, as a sketch's outer
/// loop runs — and what its station is called. `matched`: the joints of a
/// closed loop matched with the other sections', in the order they are
/// matched (see [`mark`] and the module docs).
#[derive(Clone, Debug)]
pub struct Section {
    pub plane: CoordinateSystem,
    pub profile: Profile,
    pub name: String,
    pub matched: Vec<String>,
}

/// A section as it is skinned: its frame and its profile in it.
#[derive(Clone, Debug)]
struct Placed {
    frame: Frame,
    profile: Profile,
}

impl Placed {
    /// The joints, in space, as plain numbers.
    fn joints(&self) -> GeopResult<Vec<[f64; 3]>> {
        let n = self.profile.joint_names.len();
        (0..n)
            .map(|i| {
                let q = self.frame.point(&self.profile.joint(i)?);
                Ok(q.to_f64_array())
            })
            .collect()
    }

    /// The same section with its frame's `e2` turned round and its curves
    /// mirrored to match: the same curves in space, winding the other way
    /// in the frame.
    fn mirrored(&self) -> Self {
        let mirror = |c: &NurbCurve2D| {
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

/// The mean of `points`: where a section or a profile is, to measure how
/// far apart sections are or which end of a path is nearer — free choices.
pub(crate) fn centre(points: &[[f64; 3]]) -> [f64; 3] {
    let n = points.len() as f64;
    let mut c = [0.0; 3];
    for p in points {
        for k in 0..3 {
            c[k] += p[k] / n;
        }
    }
    c
}

/// The control polygon's length of `curve`: how long it is, near enough to
/// pick the longest curve to halve — a free choice.
fn polygon_length(curve: &NurbCurve2D) -> f64 {
    let points: Vec<[f64; 2]> = curve
        .control_points
        .iter()
        .map(|cp| {
            let w = cp[2].to_f64();
            [cp[0].to_f64() / w, cp[1].to_f64() / w]
        })
        .collect();
    points.windows(2).map(|w| distance(w[1], w[0])).sum()
}

/// `profile` with its curve `X = curves[i]` cut at the fractions `at` of
/// its length, increasing and strictly inside `(0, 1)`: the pieces `X`,
/// `X#cut1`, `X#cut2`, ... and the joints between them `X@cut1`, `X@cut2`,
/// ... (a sketch names an arc's own quarter-turn pieces `X#1` and `X@1`).
/// Where along the curve is a free choice (see [`correspond`]), so the
/// cuts are sharp (see `NurbCurve::parameter_at_length_fraction`); the
/// pieces are exact pieces of it, each on `[0, 1]` with its end weights
/// one.
fn cut(profile: &Profile, i: usize, at: &[f64]) -> GeopResult<Profile> {
    let curve = &profile.curves[i];
    let (t0, t1) = curve.domain();
    let mut bounds = vec![t0];
    for &f in at {
        bounds.push(curve.parameter_at_length_fraction(f)?);
    }
    bounds.push(t1);
    let pieces = bounds
        .windows(2)
        .map(|w| {
            curve
                .sub_curve(w[0], w[1])?
                .with_unit_domain()?
                .with_unit_end_weights()
        })
        .collect::<GeopResult<Vec<_>>>()?;
    let mut out = profile.clone();
    let name = profile.curve_names[i].clone();
    let count = pieces.len();
    out.curves.splice(i..=i, pieces);
    out.curve_names
        .splice(i + 1..i + 1, (1..count).map(|k| format!("{name}#cut{k}")));
    out.joint_names
        .splice(i + 1..i + 1, (1..count).map(|k| format!("{name}@cut{k}")));
    Ok(out)
}

/// The closed `profile` with a joint at `point`, and that joint's name:
/// the joint already there, or — where `point` lies along a curve — a new
/// one named `joint`, the curve split there, its second half `X#joint`.
/// Where exactly along the curve is a free choice within where `point`
/// could be: the halves are exact pieces of it either way. An error if
/// `point` is on no curve of the profile.
pub fn mark(profile: &Profile, point: &Vector2, joint: &str) -> GeopResult<(Profile, String)> {
    let size = min_subdivision_size();
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
fn matched_indices(profile: &Profile, matched: &[String], name: &str) -> GeopResult<Vec<usize>> {
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

/// The curves of `profile` in each run between two consecutive joints of
/// `indices` (see [`matched_indices`]), the last run back to the start:
/// each run's range of curve indices.
fn runs(profile: &Profile, indices: &[usize]) -> Vec<std::ops::Range<usize>> {
    let n = profile.curves.len();
    (0..indices.len())
        .map(|j| indices[j]..indices.get(j + 1).copied().unwrap_or(n))
        .collect()
}

/// Where the joints inside the run `run` of `profile` are along it, as
/// fractions of the run's length — the curves' lengths, enclosed — and
/// that length: `k - 1` fractions for a run of `k` curves, increasing from
/// the run's start at 0 to its end at 1.
fn run_fractions(
    profile: &Profile,
    run: std::ops::Range<usize>,
) -> GeopResult<(Vec<Scalar>, Scalar)> {
    let lengths = run
        .map(|i| Ok(profile.curves[i].length()?.0))
        .collect::<GeopResult<Vec<Scalar>>>()?;
    let total = lengths.iter().fold(Scalar::ZERO, |sum, &l| sum.add(l));
    let mut fractions = Vec::with_capacity(lengths.len() - 1);
    let mut sum = Scalar::ZERO;
    for &l in &lengths[..lengths.len() - 1] {
        sum = sum.add(l);
        fractions.push(sum.div(total)?);
    }
    Ok((fractions, total))
}

/// The cuts of one run of several matched sections (see [`RunCuts::merged`]).
struct RunCuts {
    /// Where the cuts are, as fractions of the run's length: increasing,
    /// strictly inside `(0, 1)`, sharp.
    at: Vec<f64>,
    /// For each section, which of the cuts are its own joints, in order.
    own: Vec<Vec<usize>>,
}

impl RunCuts {
    /// The joints of corresponding runs of several sections, `fractions[s]`
    /// the run's of section `s` (see [`run_fractions`]), merged into one
    /// list of cuts along every one of them: joints of different sections
    /// that could lie at one fraction are one cut, which each of them
    /// keeps; every other joint is a cut the other sections get. Where
    /// exactly a cut lies is a free choice within what the joints enclose,
    /// so the cuts are sharp. `name(s)` names section `s` for an error: two
    /// joints of one section at one place along its run.
    fn merged(fractions: &[Vec<Scalar>], name: impl Fn(usize) -> String) -> GeopResult<Self> {
        let mut merged: Vec<Scalar> = Vec::new();
        let mut own: Vec<Vec<usize>> = vec![Vec::new(); fractions.len()];
        for (s, fractions) in fractions.iter().enumerate() {
            for &f in fractions {
                // After the cuts this section has claimed so far.
                let claimed = own[s].last().map_or(0, |&k| k + 1);
                let k = match merged[claimed..].iter().position(|m| m.could_be_equal(f)) {
                    Some(k) => {
                        merged[claimed + k] = merged[claimed + k].union(f);
                        claimed + k
                    }
                    None => {
                        let at = merged
                            .iter()
                            .position(|m| !m.definitely_less(f))
                            .unwrap_or(merged.len());
                        if at < claimed {
                            return Err(GeopError::new(format!(
                                "loft: two joints of the profile {} could lie at one place along it, {:?} of the way along the run between its matched points",
                                name(s),
                                f
                            )));
                        }
                        merged.insert(at, f);
                        for k in own.iter_mut().flatten().filter(|k| **k >= at) {
                            *k += 1;
                        }
                        at
                    }
                };
                own[s].push(k);
            }
        }
        let at: Vec<f64> = merged.iter().map(|m| m.to_f64()).collect();
        if !at.windows(2).all(|w| w[0] < w[1]) || at.iter().any(|&c| !(0.0 < c && c < 1.0)) {
            return Err(GeopError::new(format!(
                "loft: the joints along corresponding runs of the profiles cannot be told apart: {merged:?}"
            )));
        }
        Ok(Self { at, own })
    }

    /// One more cut, in the middle of the stretch between the cut `k - 1`
    /// and the cut `k` (the run's ends for the first and the last): no
    /// section's joint, a free choice.
    fn halve(&mut self, k: usize) {
        let bound = |i: usize| -> f64 {
            if i == 0 {
                0.0
            } else {
                self.at.get(i - 1).copied().unwrap_or(1.0)
            }
        };
        let middle = (bound(k) + bound(k + 1)) / 2.0;
        self.at.insert(k, middle);
        for i in self.own.iter_mut().flatten().filter(|i| **i >= k) {
            *i += 1;
        }
    }

    /// How many curves the run has once cut.
    fn pieces(&self) -> usize {
        self.at.len() + 1
    }

    /// The length of the stretch between the cut `k - 1` and the cut `k`,
    /// as a fraction of the run's.
    fn stretch(&self, k: usize) -> f64 {
        let bounds: Vec<f64> = [0.0]
            .into_iter()
            .chain(self.at.iter().copied())
            .chain([1.0])
            .collect();
        bounds[k + 1] - bounds[k]
    }
}

/// `profile`, each of whose runs `runs[j]` (see [`runs`]) is to be cut as
/// `cuts[j]` says (see [`RunCuts`]), with every cut that is not a joint of
/// section `s` made: the curve it falls in cut there, at the same fraction
/// of the curve's length as of its stretch between the cuts either side
/// (see [`cut`]).
fn cut_runs(
    profile: &Profile,
    runs: &[std::ops::Range<usize>],
    cuts: &[RunCuts],
    s: usize,
) -> GeopResult<Profile> {
    let mut out = profile.clone();
    // Runs from the last, so that cutting one leaves the indices of those
    // before it as they are.
    for j in (0..runs.len()).rev() {
        let run = &runs[j];
        let mut joints = cuts[j].own[s].iter().peekable();
        // For each curve of the run, the cuts within it, as fractions of
        // the curve's stretch between the cuts at its ends.
        let mut per_curve: Vec<Vec<f64>> = vec![Vec::new(); run.len()];
        let mut curve = 0;
        let mut start = 0.0;
        for (k, &c) in cuts[j].at.iter().enumerate() {
            if joints.next_if_eq(&&k).is_some() {
                // The joint after `curve`: the next curve starts here.
                for f in &mut per_curve[curve] {
                    *f = (*f - start) / (c - start);
                }
                curve += 1;
                start = c;
            } else {
                per_curve[curve].push(c);
            }
        }
        for f in &mut per_curve[curve] {
            *f = (*f - start) / (1.0 - start);
        }
        for (k, at) in per_curve.iter().enumerate().rev() {
            if !at.is_empty() {
                out = cut(&out, run.start + k, at)?;
            }
        }
    }
    Ok(out)
}

/// The corners of the closed sections `placed` as their matched points
/// (see [`correspond`]): every section with corners has as many, and
/// their corners are matched in cyclic order, each section's turned to line
/// its corners up best with the previous cornered section's, both about
/// their centres. A section without corners — a circle — gets none: it is
/// lined up with its neighbour as a whole. An error, naming the sections
/// and their counts, where the counts differ.
fn corner_anchors(
    sections: &[Section],
    placed: &[Placed],
    centres: &[[f64; 3]],
) -> GeopResult<Vec<Vec<String>>> {
    let corners = placed
        .iter()
        .map(|p| p.profile.corners())
        .collect::<GeopResult<Vec<Vec<usize>>>>()?;
    let mut points = vec![Vec::new(); placed.len()];
    let cornered: Vec<usize> = (0..placed.len())
        .filter(|&s| !corners[s].is_empty())
        .collect();
    let Some(&reference) = cornered.first() else {
        return Ok(points);
    };
    let count = corners[reference].len();
    if let Some(&s) = cornered.iter().find(|&&s| corners[s].len() != count) {
        return Err(GeopError::new(format!(
            "loft: the profile {} has {} corners, the profile {} has {count}: lofted as they are, the profiles' corners are matched one to one, so every profile with corners needs as many — match points, or add guides, to say which go where",
            sections[s].name,
            corners[s].len(),
            sections[reference].name
        )));
    }
    let about = |s: usize, k: usize| -> GeopResult<[f64; 3]> {
        let joint = placed[s]
            .frame
            .point(&placed[s].profile.joint(corners[s][k])?);
        Ok(sub(joint.to_f64_array(), centres[s]))
    };
    let mut shifts = vec![0; placed.len()];
    for w in cornered.windows(2) {
        let (previous, s) = (w[0], w[1]);
        let before = (0..count)
            .map(|k| about(previous, (k + shifts[previous]) % count))
            .collect::<GeopResult<Vec<_>>>()?;
        let mut best: Option<(f64, usize)> = None;
        for shift in 0..count {
            let mut cost = 0.0;
            for (k, &b) in before.iter().enumerate() {
                let d = sub(about(s, (k + shift) % count)?, b);
                cost += dot(d, d);
            }
            if best.is_none_or(|(c, _)| cost < c) {
                best = Some((cost, shift));
            }
        }
        shifts[s] = best.expect("a corner to start at").1;
    }
    for &s in &cornered {
        points[s] = (0..count)
            .map(|k| placed[s].profile.joint_names[corners[s][(k + shifts[s]) % count]].clone())
            .collect();
    }
    Ok(points)
}

/// The section `next` to an apex at `point`, collapsed into it: a frame at
/// the point, square as `next`'s, and every curve of `next` with all its
/// control points there — its degree, knots and weights kept, so that it
/// is compatible with `next`'s and the walls between are ruled into the
/// point. Named as `next` is.
fn collapsed(next: &Placed, point: &Vector3) -> GeopResult<Placed> {
    let profile = next.profile.map_curves(|c| {
        let mut c = c.clone();
        for cp in &mut c.control_points {
            cp[0] = Scalar::ZERO;
            cp[1] = Scalar::ZERO;
        }
        c.recompute_aabb();
        c
    });
    Ok(Placed {
        frame: Frame {
            origin: *point,
            ..next.frame.clone()
        },
        profile,
    })
}

/// The sections made to correspond, as the module docs say — closed ones
/// winding so that the loft runs against each frame's `e1 x e2`, which
/// makes the skinned walls face out. `beyond` are points the loft runs to
/// before the first section and after the last, if any: which way it runs
/// through a section next to one is towards it, or from it.
fn correspond(sections: &[Section], beyond: [Option<[f64; 3]>; 2]) -> GeopResult<Vec<Placed>> {
    let closed = sections[0].profile.is_closed();
    if sections.iter().any(|s| s.profile.is_closed() != closed) {
        return Err(GeopError::new(
            "loft: the profiles are either all closed or all open",
        ));
    }
    let mut placed: Vec<Placed> = sections
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
        let v = |a: [f64; 3]| Vector3::from_array(a.map(Scalar::from_f64));
        for s in 0..n {
            let (from, to) = match (s + 1 < n, s > 0, beyond) {
                (true, _, _) => (centres[s], centres[s + 1]),
                (false, _, [_, Some(after)]) => (centres[s], after),
                (false, true, _) => (centres[s - 1], centres[s]),
                (false, false, [Some(before), None]) => (before, centres[s]),
                (false, false, [None, None]) => {
                    return Err(GeopError::new("loft: one profile, and nothing to loft to"));
                }
            };
            let along = v(to).sub(&v(from));
            let frame = &placed[s].frame;
            let facing = frame.e1.prod_cross(&frame.e2).prod_dot(&along);
            if facing.definitely_greater(Scalar::ZERO) {
                let reversed = placed[s].mirrored();
                placed[s] = Placed {
                    profile: reversed.profile.reversed(),
                    ..reversed
                };
            } else if !facing.definitely_less(Scalar::ZERO) {
                return Err(GeopError::new(format!(
                    "loft: the loft could run along the plane of {}",
                    sections[s].name
                )));
            }
        }
    }

    // Matched points: the designer's, in the order they run round the first
    // matched section as it now winds — every section's in the same order,
    // so that the `k`-th still meets the `k`-th — whatever order they were
    // given in; or, with none given, the sections' corners.
    let mut points: Vec<Vec<String>> = sections.iter().map(|s| s.matched.clone()).collect();
    let explicit = points.iter().any(|p| !p.is_empty());
    if let Some(reference) = (0..sections.len()).find(|&s| !points[s].is_empty()) {
        let joints = &placed[reference].profile.joint_names;
        let mut order: Vec<usize> = (0..points[reference].len()).collect();
        order.sort_by_key(|&k| joints.iter().position(|j| *j == points[reference][k]));
        for list in points.iter_mut().filter(|l| l.len() == order.len()) {
            *list = order.iter().map(|&k| list[k].clone()).collect();
        }
    }
    if !explicit && closed {
        points = corner_anchors(sections, &placed, &centres)?;
    }

    // Each matched section starting at its first matched point, and the
    // runs between consecutive matched points cut alike in all of them.
    let matched: Vec<usize> = (0..sections.len())
        .filter(|&s| !points[s].is_empty())
        .collect();
    let mut most = placed
        .iter()
        .map(|p| p.profile.curves.len())
        .max()
        .unwrap_or(0);
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
        for &s in &matched {
            let start = placed[s]
                .profile
                .joint_names
                .iter()
                .position(|j| *j == points[s][0])
                .ok_or_else(|| GeopError::new("loft: the first matching point is no joint"))?;
            placed[s].profile = placed[s].profile.starting_at(start);
        }
        let indices = |points: &[Vec<String>]| -> GeopResult<Vec<Vec<usize>>> {
            matched
                .iter()
                .map(|&s| matched_indices(&placed[s].profile, &points[s], &sections[s].name))
                .collect()
        };

        // Corners between the designer's matched points are matched too,
        // between each two points every matched section has as many
        // corners between.
        if explicit {
            let inner: Vec<Vec<Vec<String>>> = matched
                .iter()
                .zip(indices(&points)?)
                .map(|(&s, indices)| {
                    let profile = &placed[s].profile;
                    let corners = profile.corners()?;
                    Ok(runs(profile, &indices)
                        .into_iter()
                        .map(|run| {
                            corners
                                .iter()
                                .filter(|&&c| run.start < c && c < run.end)
                                .map(|&c| profile.joint_names[c].clone())
                                .collect()
                        })
                        .collect())
                })
                .collect::<GeopResult<_>>()?;
            for (m, &s) in matched.iter().enumerate() {
                let mut all = Vec::new();
                for j in 0..count {
                    all.push(points[s][j].clone());
                    if inner.iter().all(|i| i[j].len() == inner[m][j].len()) {
                        all.extend(inner[m][j].iter().cloned());
                    }
                }
                points[s] = all;
            }
        }

        // Runs: every joint of any matched section's run is a cut of the
        // corresponding run of every other, at the same fraction of the
        // run's length (see `merged_cuts`).
        let run_list: Vec<Vec<std::ops::Range<usize>>> = matched
            .iter()
            .zip(indices(&points)?)
            .map(|(&s, indices)| runs(&placed[s].profile, &indices))
            .collect();
        let fractions: Vec<Vec<(Vec<Scalar>, Scalar)>> = matched
            .iter()
            .zip(&run_list)
            .map(|(&s, runs)| {
                runs.iter()
                    .map(|run| run_fractions(&placed[s].profile, run.clone()))
                    .collect()
            })
            .collect::<GeopResult<_>>()?;
        let count = run_list[0].len();
        let mut cuts: Vec<RunCuts> = (0..count)
            .map(|j| {
                let along: Vec<Vec<Scalar>> = fractions.iter().map(|f| f[j].0.clone()).collect();
                RunCuts::merged(&along, |m| sections[matched[m]].name.clone())
            })
            .collect::<GeopResult<_>>()?;
        // A section not matched with more curves than the runs add up to:
        // the longest stretch between two cuts of the first matched section
        // is cut in the middle, until the runs add up to as many — a free
        // choice, where no joint says where to cut.
        let total = |cuts: &[RunCuts]| cuts.iter().map(RunCuts::pieces).sum::<usize>();
        while total(&cuts) < most {
            let mut longest: Option<(f64, usize, usize)> = None;
            for (j, run) in cuts.iter().enumerate() {
                let length = fractions[0][j].1.to_f64();
                for k in 0..run.pieces() {
                    let stretch = run.stretch(k) * length;
                    if longest.is_none_or(|(l, _, _)| stretch > l) {
                        longest = Some((stretch, j, k));
                    }
                }
            }
            let (_, j, k) = longest.expect("a run has a stretch");
            cuts[j].halve(k);
        }
        most = total(&cuts);
        for (m, &s) in matched.iter().enumerate() {
            placed[s].profile = cut_runs(&placed[s].profile, &run_list[m], &cuts, m)
                .with_context(with_context!("cutting the profile {}", sections[s].name))?;
        }
    }

    // A section not matched: as many curves as the others, each halving its
    // longest one — a free choice, as no corner says where.
    for (s, p) in placed.iter_mut().enumerate() {
        if points[s].is_empty() {
            for _ in p.profile.curves.len()..most {
                let longest = (0..p.profile.curves.len())
                    .max_by(|&a, &b| {
                        polygon_length(&p.profile.curves[a])
                            .total_cmp(&polygon_length(&p.profile.curves[b]))
                    })
                    .expect("a profile has curves");
                p.profile = cut(&p.profile, longest, &[0.5])?;
            }
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
        let candidates: Vec<Profile> = if closed {
            (0..most)
                .map(|k| placed[s].profile.starting_at(k))
                .collect()
        } else {
            vec![placed[s].profile.clone(), placed[s].profile.reversed()]
        };
        let mut best: Option<(f64, Profile)> = None;
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
                .map(|(&a, &b)| {
                    let d = sub(sub(a, ca), sub(b, cb));
                    dot(d, d)
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
        let curves: Vec<NurbCurve2D> = placed
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
struct Crossed {
    chain: Chain<NurbCurve3D>,
    at: Vec<(usize, Scalar)>,
}

/// Where `guide` crosses each of `sections` (see [`Crossed`]): starting on
/// the first one's plane — run the other way if it ends there — ending on
/// the last one's, and crossing every plane between exactly once. An error,
/// naming the guide and the section, otherwise.
fn crossings(sections: &[Section], guide: &Chain<NurbCurve3D>) -> GeopResult<Crossed> {
    let name = &guide.name;
    if guide.is_closed() {
        return Err(GeopError::new(format!(
            "loft: the guide {name} is a closed loop: a guide runs from the first profile to the last"
        )));
    }
    let (first, last) = (&sections[0], &sections[sections.len() - 1]);
    let mut chain = guide.starting_on(&first.plane)?;
    if let Some(c) = &chain
        && !last.plane.to_uvw(&c.joint(c.curves.len())?)[2].could_be_equal(Scalar::ZERO)
    {
        chain = None;
    }
    let Some(chain) = chain else {
        return Err(GeopError::new(format!(
            "loft: the guide {name} has to run from the profile {} to the profile {}, starting and ending on them",
            first.name, last.name
        )));
    };
    let n = chain.curves.len();
    let mut at = vec![(0, Scalar::ZERO)];
    for section in &sections[1..sections.len() - 1] {
        let ctx = with_context!(
            "where the guide {name} crosses the profile {}",
            section.name
        );
        let patch = plane_patch(&section.plane, &chain)?;
        let mut hits = Vec::new();
        for (i, curve) in chain.curves.iter().enumerate() {
            let found =
                curve_surface_intersect(curve, &patch, 4, MAX_NODES, min_subdivision_size())
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
                if i > 0 && t.could_be_equal(Scalar::ZERO) {
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
    at.push((n - 1, Scalar::ONE));
    Ok(Crossed { chain, at })
}

/// A flat patch of `plane` that every crossing of `chain` with it lies on:
/// the box of the chain's control points seen in the plane, as wide again
/// on every side — a search domain, nothing more.
fn plane_patch(plane: &CoordinateSystem, chain: &Chain<NurbCurve3D>) -> GeopResult<NurbSurface3D> {
    let uvs = chain
        .curves
        .iter()
        .flat_map(|c| &c.control_points)
        .map(|cp| Ok(plane.to_uv(&cp.dehomogenized()?).to_f64_array()))
        .collect::<GeopResult<Vec<_>>>()?;
    let (lo, hi) =
        plain::bounds(uvs).ok_or_else(|| GeopError::new("the path has no control points"))?;
    let margin = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1.0);
    let corner = |u: f64, v: f64| {
        plane.uv_to_xyz(&Vector2::from_array([
            Scalar::from_f64(u),
            Scalar::from_f64(v),
        ]))
    };
    let (u0, u1, v0, v1) = (
        lo[0] - margin,
        hi[0] + margin,
        lo[1] - margin,
        hi[1] + margin,
    );
    NurbSurface3D::bilinear(
        corner(u0, v0),
        corner(u1, v0),
        corner(u1, v1),
        corner(u0, v1),
    )
}

/// Sections with the guides' joints, and the guides as crossed (see [`guided`]).
type Guided = (Vec<Section>, Vec<Crossed>);

/// `sections` with a joint wherever a guide crosses them, matched across
/// them in the order of the guides — the joint named `K,G` for the section
/// `K` and the guide `G`, unless one is there already — and the guides as
/// crossed. Guides take the place of matching points: the two do not mix.
fn guided(sections: &[Section], guides: &[Chain<NurbCurve3D>]) -> GeopResult<Guided> {
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
            let (profile, joint) = mark(
                &section.profile,
                &section.plane.to_uv(&point),
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
        let joint = |name: &String| -> GeopResult<Vector2> {
            let profile = &out[0].profile;
            let i = profile
                .joint_names
                .iter()
                .position(|j| j == name)
                .expect("a guide's joint is on its profile");
            profile.joint(i)
        };
        let q = out[0]
            .matched
            .iter()
            .map(joint)
            .collect::<GeopResult<Vec<_>>>()?;
        let (a, b) = (q[1].sub(&q[0]), q[2].sub(&q[0]));
        if a[0]
            .mul(b[1])
            .sub(a[1].mul(b[0]))
            .could_be_equal(Scalar::ZERO)
        {
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
fn guide_piece(crossed: &Crossed, j: usize) -> GeopResult<NurbCurve3D> {
    let ((c0, t0), (c1, t1)) = (crossed.at[j], crossed.at[j + 1]);
    let (t0, t1) = (t0.sharpen(), t1.sharpen());
    let mut curves = Vec::new();
    for i in c0..=c1 {
        let curve = &crossed.chain.curves[i];
        let from = if i == c0 { t0 } else { Scalar::ZERO };
        let to = if i == c1 { t1 } else { Scalar::ONE };
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
fn guided_span(
    a: &Placed,
    b: &Placed,
    joints: &[(String, String)],
    pieces: &[NurbCurve3D],
) -> GeopResult<Span> {
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
    let joint_at = |placed: &Placed, name: &str| -> GeopResult<[f64; 2]> {
        let profile = &placed.profile;
        let i = profile
            .joint_names
            .iter()
            .position(|j| j == name)
            .ok_or_else(|| GeopError::new(format!("loft: no joint {name} for a guide")))?;
        let p = profile.joint(i)?;
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
        let (q, d): (Vec<[f64; 2]>, Vec<P3>) = ends
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
            (moved(&fa).frame(), weight.mul(Scalar::from_f64(1.0 - xi))),
            (moved(&fb).frame(), weight.mul(Scalar::from_f64(xi))),
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
fn affine_through(q: &[[f64; 2]], d: &[P3]) -> (P3, [P3; 2]) {
    let m = match q.len() {
        1 => [[0.0; 3]; 2],
        2 => {
            let (dq, dd) = (sub(q[1], q[0]), sub(d[1], d[0]));
            let n = dot(dq, dq);
            [scale(dd, dq[0] / n), scale(dd, dq[1] / n)]
        }
        _ => {
            let (q1, q2) = (sub(q[1], q[0]), sub(q[2], q[0]));
            let (d1, d2) = (sub(d[1], d[0]), sub(d[2], d[0]));
            // [d1 d2] [q1 q2]^-1, the points as columns.
            let inv = inverse2([[q1[0], q2[0]], [q1[1], q2[1]]]);
            [
                add(scale(d1, inv[0][0]), scale(d2, inv[1][0])),
                add(scale(d1, inv[0][1]), scale(d2, inv[1][1])),
            ]
        }
    };
    let t = sub(d[0], add(scale(m[0], q[0][0]), scale(m[1], q[0][1])));
    (t, m)
}

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
fn through(placed: &[Placed], closed: bool) -> GeopResult<Vec<Span>> {
    // Spans; stations `0..=n`, the last the first again when `closed`.
    let n = if closed {
        placed.len()
    } else {
        placed.len() - 1
    };
    let station = |s: usize| s % placed.len();
    let centres = placed
        .iter()
        .map(|p| Ok(centre(&p.joints()?)))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut d: Vec<f64> = (0..n)
        .map(|j| distance(centres[j], centres[station(j + 1)]))
        .collect();
    if d.iter().any(|&d| d <= 0.0) {
        d = vec![1.0; n];
    }
    // Rows as coefficients of the sections: `unit(s)` is section `s`.
    let unit = |s: usize| -> Vec<f64> {
        (0..placed.len())
            .map(|k| if k == station(s) { 1.0 } else { 0.0 })
            .collect()
    };
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
    if closed {
        // Round the seam: the first section's tangent from the last span
        // and the first, as at any other.
        let w = d[n - 1] + d[0];
        tangent[0] = combine(&chord(n - 1), d[0] / w, &chord(0), d[n - 1] / w);
        tangent[n] = tangent[0].clone();
    } else {
        tangent[0] = combine(&chord(0), 2.0, &tangent[1], -1.0);
        tangent[n] = combine(&chord(n - 1), 2.0, &tangent[n - 1], -1.0);
    }
    let row = |coefficients: Vec<f64>| -> Vec<(usize, Scalar)> {
        coefficients
            .into_iter()
            .enumerate()
            .filter(|&(_, c)| c != 0.0)
            .map(|(s, c)| (s, Scalar::from_f64(c)))
            .collect()
    };
    let (zero, one) = (Scalar::ZERO, Scalar::ONE);
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

/// A point a loft comes to before its first section or after its last:
/// every curve of the profile next to it runs into it, as a cone into its
/// apex. Its station is named `name`.
#[derive(Clone, Debug)]
pub struct Apex {
    pub point: Vector3,
    pub name: String,
}

/// How a loft ends: before its first section and after its last, a cap
/// or a point it comes to (see [`Apex`]) — or nowhere, closing on itself
/// from its last section round to its first.
#[derive(Clone, Debug)]
pub enum Ends {
    Open([Option<Apex>; 2]),
    Closed,
}

impl Ends {
    /// Capped by the first and last sections.
    pub const CAPPED: Ends = Ends::Open([None, None]);
}

/// Lofts through `sections` (see the module docs): closed loops into a solid
/// named `solid` — capped by the first and last sections — or, without one,
/// into sheets; open chains into sheets only. Its `ends` may instead be
/// points the loft comes to before its first section and after its last:
/// there it closes in a point instead of a cap (see [`Apex`]). The tangent
/// of a smooth loft into an apex is a free choice, as at any end section:
/// from its neighbours (see [`through`]). Or the loft closes on itself
/// ([`Ends::Closed`]), running smoothly on from its last section round to
/// its first, through three sections or more, with no caps: a ring.
///
/// Named after the first section's curves `X` and joints `P` and the
/// sections' names `K` (see [`skin`]): the walls `N(X)` — `N(X,K>L)` between
/// the sections `K` and `L` when there are more than two — the curves at the
/// sections `N(X,K)`, the edges between them `N(P)` or `N(P,K>L)`, the
/// vertices `N(P,K)`, the caps `N(start)` and `N(end)`; an apex `A` is the
/// vertex `N(A)`.
pub fn loft(
    model: &mut NamedModel,
    namer: &Namer,
    solid: Option<&str>,
    sections: &[Section],
    ends: &Ends,
    guides: &[Chain<NurbCurve3D>],
) -> GeopResult<BuiltBody> {
    let (apexes, closed) = match ends {
        Ends::Open(apexes) => (apexes, false),
        Ends::Closed => (&[None, None], true),
    };
    let ctx = with_context!(
        "loft({}, through {:?}, apexes {:?})",
        namer.root(),
        sections.iter().map(|s| &s.name).collect::<Vec<_>>(),
        apexes.iter().flatten().map(|a| &a.name).collect::<Vec<_>>()
    );
    let ends_count = apexes.iter().flatten().count();
    let ends = ends_count;
    if sections.is_empty() || sections.len() + ends < 2 {
        return Err(GeopError::new(format!(
            "loft: needs two profiles or more, or a profile and a point, not {} profile(s) and {ends} point(s)",
            sections.len()
        )))
        .with_context(ctx);
    }
    if ends > 0 && !guides.is_empty() {
        return Err(GeopError::new(
            "loft: guides run from the first profile to the last, and a loft to a point has a point at an end: loft to the point without guides",
        ))
        .with_context(ctx);
    }
    if closed && (sections.len() < 3 || !guides.is_empty()) {
        return Err(GeopError::new(format!(
            "loft: a loft closing on itself runs through three profiles or more, without guides, not {} profile(s) and {} guide(s)",
            sections.len(),
            guides.len()
        )))
        .with_context(ctx);
    }
    if ends > 0 && !sections[0].profile.is_closed() {
        return Err(GeopError::new(
            "loft: only a closed profile comes to a point",
        ))
        .with_context(ctx);
    }
    let (sections, crossed) = if guides.is_empty() {
        (sections.to_vec(), Vec::new())
    } else {
        guided(sections, guides).with_context(ctx)?
    };
    let beyond = [0, 1].map(|k| apexes[k].as_ref().map(|a| a.point.to_f64_array()));
    let mut placed = correspond(&sections, beyond).with_context(ctx)?;
    let mut names: Vec<String> = sections.iter().map(|s| s.name.clone()).collect();
    let mut apex = vec![false; sections.len()];
    for (k, end) in apexes.iter().enumerate() {
        let Some(end) = end else { continue };
        let next = if k == 0 { 0 } else { placed.len() - 1 };
        let point = collapsed(&placed[next], &end.point).with_context(ctx)?;
        if k == 0 {
            placed.insert(0, point);
            names.insert(0, end.name.clone());
            apex.insert(0, true);
        } else {
            placed.push(point);
            names.push(end.name.clone());
            apex.push(true);
        }
    }
    let spans = if closed {
        placed.len()
    } else {
        placed.len() - 1
    };
    let through = if crossed.is_empty() && spans > 1 {
        Some(through(&placed, closed).with_context(ctx)?)
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
        closed,
        along_normal: false,
        span_names: if spans == 1 {
            vec![None]
        } else {
            (0..spans)
                .map(|j| Some(format!("{}>{}", names[j], names[(j + 1) % names.len()])))
                .collect()
        },
        station_names: names,
    };
    let loops: Vec<Vec<SweepLoop>> = placed
        .into_iter()
        .zip(apex)
        .map(|(p, apex)| {
            vec![SweepLoop {
                apex,
                ..SweepLoop::plain(p.profile)
            }]
        })
        .collect();
    skin(model, namer, &path, &loops, solid).with_context(ctx)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::polygon;
    use crate::shapes::{circle, plane, square, v2, v3};
    use geop_core_geometry::nurb_curve::{NurbCurve2D, sqrt2_over_2};
    use geop_ops_model::testing::{assert_model_valid, assert_model_valid_sheets};

    /// The plane `z = height`, along `x` and `y`.
    fn level(height: f64) -> CoordinateSystem {
        plane(v3(0., 0., height), v3(1., 0., 0.), v3(0., 1., 0.))
    }

    /// The regular polygon of `n` corners on the circle of radius `r`,
    /// counter-clockwise, its first corner at angle `phase` (in turns).
    fn regular(n: usize, r: f64, phase: f64) -> Vec<NurbCurve2D> {
        let corners: Vec<Vector2> = (0..n)
            .map(|i| {
                let a = std::f64::consts::TAU * (phase + i as f64 / n as f64);
                v2(r * a.cos(), r * a.sin())
            })
            .collect();
        polygon(&corners).unwrap()
    }

    fn section(name: &str, plane: CoordinateSystem, curves: Vec<NurbCurve2D>) -> Section {
        Section {
            plane,
            profile: Profile::closed(curves).with_prefix(&format!("{name},")),
            name: name.into(),
            matched: Vec::new(),
        }
    }

    /// Lofts `sections` into a solid in a fresh part, and checks it is valid.
    fn lofted(sections: &[Section]) -> NamedModel {
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        loft(
            &mut model,
            &namer,
            Some(&namer.root()),
            sections,
            &Ends::CAPPED,
            &[],
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        model
    }

    /// A square below, a circle above: four ruled walls and two caps.
    #[test]
    fn loft_square_to_circle() {
        let model = lofted(&[
            section("a", level(0.0), regular(4, 1.0, 0.125)),
            section("b", level(2.0), circle(0.5)),
        ]);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 + 2);
        assert_eq!(topology.vertices.len(), 8);
        // Every vertex of the top section lies on its circle.
        for v in topology.vertices.values() {
            if v.point[2].could_be_equal(Scalar::from_f64(2.0)) {
                let (x, y) = (v.point[0], v.point[1]);
                assert!(
                    x.mul(x)
                        .add(y.mul(y))
                        .could_be_equal(Scalar::from_f64(0.25))
                );
            }
        }
    }

    /// Lofts `sections` into a point `apex` at its start or end, as
    /// `at_end` says; checks the solid is valid and returns it.
    fn lofted_to(sections: &[Section], apex: Vector3, at_end: bool) -> NamedModel {
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        let apex = Apex {
            point: apex,
            name: "tip".into(),
        };
        let ends = if at_end {
            Ends::Open([None, Some(apex)])
        } else {
            Ends::Open([Some(apex), None])
        };
        loft(
            &mut model,
            &namer,
            Some(&namer.root()),
            sections,
            &ends,
            &[],
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        model
    }

    /// Circles in four planes round the `z` axis, lofted closed: a ring,
    /// smooth across every section — the seam too — with no caps.
    #[test]
    fn closed_loft_through_four_circles_is_a_ring() {
        let sections: Vec<Section> = (0..4)
            .map(|k| {
                let a = std::f64::consts::FRAC_PI_2 * k as f64;
                let (c, s) = (a.cos(), a.sin());
                let at = plane(v3(2.0 * c, 2.0 * s, 0.0), v3(c, s, 0.0), v3(0.0, 0.0, 1.0));
                section(&format!("s{k}"), at, circle(0.3))
            })
            .collect();
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        loft(
            &mut model,
            &namer,
            Some(&namer.root()),
            &sections,
            &Ends::Closed,
            &[],
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 * 4);
        assert!(model.face_id("loft(l,start)").is_err());
        assert!(model.face_id("loft(l,s0,c0,s3>s0)").is_ok());
        // The walls are smooth across the seam: the tangent along the loft
        // at the seam is the same from either side.
        let surface = |name: &str| {
            let face = model.face_id(name).unwrap();
            topology.get_face(face).unwrap().surface.clone()
        };
        let (before, after) = (
            surface("loft(l,s0,c0,s3>s0)"),
            surface("loft(l,s0,c0,s0>s1)"),
        );
        let half = Scalar::from_f64(0.5);
        let du = |s: &NurbSurface3D, u: f64| {
            let (du, _) = s.derivatives(Scalar::from_f64(u), half).unwrap();
            du.to_f64_array()
        };
        let (a, b) = (du(&before, 1.0), du(&after, 0.0));
        let unit = |v: [f64; 3]| plain::scale(v, 1.0 / plain::dot(v, v).sqrt());
        let (a, b) = (unit(a), unit(b));
        assert!(plain::distance(a, b) < 1e-9, "{a:?} {b:?}");
    }

    /// The volume of the solid `loft(l)` of `model`, from its faces.
    fn volume(model: &NamedModel) -> f64 {
        let solid = model.solid_id("loft(l)").unwrap();
        let mass = model.topology().mass_properties(solid).unwrap();
        mass.volume.to_f64()
    }

    /// A square into a point above: a pyramid, its four walls ruled into
    /// the apex, one cap — a third of the base times the height.
    #[test]
    fn loft_square_to_a_point_is_a_pyramid() {
        let model = lofted_to(
            &[section("a", level(0.0), square(1.0))],
            v3(0.0, 0.0, 3.0),
            true,
        );
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 + 1);
        assert_eq!(topology.vertices.len(), 5);
        assert!(model.vertex_id("loft(l,tip)").is_ok());
        assert!((volume(&model) - 4.0).abs() < 1e-9, "{}", volume(&model));
    }

    /// A point below a circle, off its axis: an oblique cone, from the
    /// point up — a third of the base times the height all the same.
    #[test]
    fn loft_from_a_point_to_a_circle_is_a_cone() {
        let model = lofted_to(
            &[section("a", level(2.0), circle(1.0))],
            v3(0.5, 0.0, 0.0),
            false,
        );
        let expected = std::f64::consts::PI * 2.0 / 3.0;
        assert!(
            (volume(&model) - expected).abs() < 1e-9,
            "{}",
            volume(&model)
        );
    }

    /// Through two sections into a point, smoothly: the walls are cubic
    /// through all three stations, the tangent at the point from its
    /// neighbours, as at any end section.
    #[test]
    fn loft_through_two_sections_into_a_point() {
        let model = lofted_to(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(1.0), circle(0.8)),
            ],
            v3(0.0, 0.0, 2.0),
            true,
        );
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 2 * 4 + 1);
    }

    /// The upper plane tilted, and facing down: still a valid solid.
    #[test]
    fn loft_to_a_tilted_plane_facing_the_other_way() {
        let tilted = plane(v3(0.2, 0., 2.), v3(0.8, 0., 0.6), v3(0., -1., 0.));
        lofted(&[
            section("a", level(0.0), regular(4, 1.0, 0.0)),
            section("b", tilted, circle(0.5)),
        ]);
    }

    /// Through three sections: two layers of walls, named after the spans
    /// between the sections.
    #[test]
    fn loft_through_three_sections() {
        let model = lofted(&[
            section("a", level(0.0), regular(4, 1.0, 0.125)),
            section("b", level(1.0), circle(0.5)),
            section("c", level(2.5), regular(4, 0.8, 0.0)),
        ]);
        assert_eq!(model.topology().faces.len(), 2 * 4 + 2);
        assert!(model.face_id("loft(l,a,c0,a>b)").is_ok());
        assert!(model.face_id("loft(l,a,c0,b>c)").is_ok());
        assert!(model.edge_id("loft(l,a,c0,c)").is_ok());
    }

    /// Through three circles of different sizes, the walls either side of
    /// the middle one meet it with one tangent plane: the loft has no crease
    /// there, as ruled walls would.
    #[test]
    fn loft_through_three_sections_is_smooth() {
        let model = lofted(&[
            section("a", level(0.0), circle(1.0)),
            section("b", level(1.0), circle(1.5)),
            section("c", level(3.0), circle(0.8)),
        ]);
        let topology = model.topology();
        let surface = |name: &str| &topology.faces[&model.face_id(name).unwrap()].surface;
        let (below, above) = (surface("loft(l,a,c0,a>b)"), surface("loft(l,a,c0,b>c)"));
        for v in [0.1, 0.5, 0.9] {
            let normal = |surface: &geop_core_geometry::nurb_surface::NurbSurface3D, u: Scalar| {
                let (v0, v1) = surface.domain_v();
                let v = Scalar::from_f64(v0.to_f64() + (v1.to_f64() - v0.to_f64()) * v);
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

    /// A triangle and a circle: the triangle's longest side is halved to
    /// match the circle's four quarters.
    #[test]
    fn loft_triangle_to_circle() {
        let model = lofted(&[
            section("a", level(0.0), regular(3, 1.0, 0.0)),
            section("b", level(1.5), circle(0.6)),
        ]);
        assert_eq!(model.topology().faces.len(), 4 + 2);
        // All three sides are as long: whichever was halved.
        assert!((0..3).any(|k| model.vertex_id(&format!("loft(l,a,c{k}@cut1,b)")).is_ok()));
    }

    /// A hexagon and a circle: the circle's quarters are halved — rational
    /// arcs, brought back to end weights of one.
    #[test]
    fn loft_hexagon_to_circle() {
        let model = lofted(&[
            section("a", level(0.0), regular(6, 1.0, 0.0)),
            section("b", level(1.5), circle(0.6)),
        ]);
        assert_eq!(model.topology().faces.len(), 6 + 2);
    }

    /// Two open curves loft into the ruled sheet between them.
    #[test]
    fn loft_between_two_curves_is_a_ruled_sheet() {
        let line = vec![NurbCurve2D::line(v2(-1., 0.), v2(1., 0.)).unwrap()];
        let arc =
            vec![NurbCurve2D::arc(v2(1., 0.), v2(1., 1.), v2(0., 1.), sqrt2_over_2()).unwrap()];
        let open = |name: &str, plane, curves| Section {
            plane,
            profile: Profile::open(curves).with_prefix(&format!("{name},")),
            name: name.to_string(),
            matched: Vec::new(),
        };
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        let built = loft(
            &mut model,
            &namer,
            None,
            &[open("a", level(0.0), line), open("b", level(1.0), arc)],
            &Ends::CAPPED,
            &[],
        )
        .unwrap();
        assert!(built.solid.is_none());
        model.check_names().unwrap();
        assert_model_valid_sheets(model.topology());
        assert_eq!(model.topology().faces.len(), 1);
    }

    /// One profile is not a loft.
    #[test]
    fn loft_of_one_profile_is_refused() {
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        let only = section("a", level(0.0), regular(4, 1.0, 0.0));
        assert!(
            loft(
                &mut model,
                &namer,
                Some("loft(l)"),
                &[only],
                &Ends::CAPPED,
                &[]
            )
            .is_err()
        );
    }

    // ── guide curves ────────────────────────────────────────────────────────

    /// The quadratic Bézier guide `name` through the control points `points`.
    fn guide(name: &str, points: [[f64; 3]; 3]) -> Chain<NurbCurve3D> {
        let f = Scalar::from_f64;
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
    fn guided_loft(sections: &[Section], guides: &[Chain<NurbCurve3D>]) -> NamedModel {
        let mut model = NamedModel::new();
        let namer = Namer::new("loft", "l").unwrap();
        loft(
            &mut model,
            &namer,
            Some(&namer.root()),
            sections,
            &Ends::CAPPED,
            guides,
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        model
    }

    /// The edge `name` of `model` runs along `guide`: one point for one at
    /// every parameter — the walls' edge there is the guide itself.
    fn assert_follows(model: &NamedModel, name: &str, guide: &Chain<NurbCurve3D>) {
        let edge = model.edge_id(name).unwrap();
        let curve = &model.topology().edges[&edge].curve;
        for i in 0..=8 {
            let t = Scalar::from_f64(i as f64 / 8.0);
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
    #[test]
    fn loft_with_one_guide() {
        let g = guide("g", [[1., 1., 0.], [2., 2., 1.], [1., 1., 2.]]);
        let model = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), square(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_eq!(model.topology().faces.len(), 4 + 2);
        assert_follows(&model, "loft(l,a,p2)", &g);
        // The other corners move along with it: half way up, the whole
        // section shifted by half the guide's bulge.
        let edge = model.edge_id("loft(l,a,p0)").unwrap();
        let middle = model.topology().edges[&edge]
            .curve
            .evaluate(Scalar::from_f64(0.5))
            .unwrap();
        for (k, want) in [-0.5, -0.5, 1.0].into_iter().enumerate() {
            assert!((middle[k].to_f64() - want).abs() < 1e-12, "{middle:?}");
        }
    }

    /// Two guides bowing out from opposite corners: the sections stretch
    /// along the diagonal between them, both edges the guides.
    #[test]
    fn loft_with_two_guides() {
        let g = guide("g", [[1., 1., 0.], [2., 2., 1.], [1., 1., 2.]]);
        let h = guide("h", [[-1., -1., 0.], [-2., -2., 1.], [-1., -1., 2.]]);
        let model = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), square(1.0)),
            ],
            &[h.clone(), g.clone()],
        );
        assert_follows(&model, "loft(l,a,p2)", &g);
        assert_follows(&model, "loft(l,a,p0)", &h);
    }

    /// A guide meeting the profiles along their sides, not at a corner:
    /// each side split where the guide meets it.
    #[test]
    fn guide_splits_the_sides_it_meets() {
        let g = guide("g", [[1., 0., 0.], [1.5, 0., 1.], [1., 0., 2.]]);
        let model = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(2.0), circle(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_follows(&model, "loft(l,a,g)", &g);
    }

    /// Through three profiles: the guide crosses the middle one at its
    /// corner, and is followed on either side of it.
    #[test]
    fn loft_with_a_guide_through_three_profiles() {
        let g = guide("g", [[1., 1., 0.], [1.4, 1.4, 1.], [1., 1., 2.]]);
        let model = guided_loft(
            &[
                section("a", level(0.0), square(1.0)),
                section("b", level(1.0), square(1.2)),
                section("c", level(2.0), square(1.0)),
            ],
            std::slice::from_ref(&g),
        );
        assert_eq!(model.topology().faces.len(), 2 * 4 + 2);
        let corner = model.vertex_id("loft(l,a,p2,b)").unwrap();
        let p = model.topology().get_vertex(corner).unwrap().point;
        assert!(p.could_be_equal(&v3(1.2, 1.2, 1.0)), "{p:?}");
    }

    /// What a guided loft refuses, by name.
    #[test]
    fn guided_loft_refuses_what_it_cannot_build() {
        let sections = [
            section("a", level(0.0), square(1.0)),
            section("b", level(2.0), square(1.0)),
        ];
        let refused = |sections: &[Section], guides: &[Chain<NurbCurve3D>], says: &str| {
            let mut model = NamedModel::new();
            let namer = Namer::new("loft", "l").unwrap();
            let error = loft(
                &mut model,
                &namer,
                Some("loft(l)"),
                sections,
                &Ends::CAPPED,
                guides,
            )
            .unwrap_err();
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
        let four: Vec<Chain<NurbCurve3D>> = (0..4)
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
}
