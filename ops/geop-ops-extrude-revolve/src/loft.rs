//! Lofting: a body through two or more planar profiles — sections — each in
//! a plane of its own, by skinning them (see [`crate::sweep::skin`]): a
//! square on one plane and a circle on another give a solid running from
//! one to the other. Each pair of consecutive sections is joined by ruled
//! walls, straight lines between corresponding points of the two.
//!
//! To be skinned, the sections are made to correspond:
//!
//! - **winding**: each is turned, if need be, to run the same way round
//!   seen along the loft — counter-clockwise looking back from the next
//!   section — mirroring its frame where its plane faces the other way;
//! - **pieces**: a section of fewer curves has its longest ones halved
//!   until it has as many as the others;
//! - **start**: a closed section starts at whichever joint lines its joints
//!   up best with the section before — its shape about its centre — and an
//!   open one runs whichever way round lines its ends up;
//! - **curves**: the `i`-th curves of all sections are made compatible, one
//!   degree and one knot vector (see `NurbCurve::compatible`).
//!
//! Open chains loft into sheets: between two curves, the ruled surface
//! joining them.

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::build::BuiltBody;
use geop_ops::{Namer, Part};

use crate::{
    common::Profile,
    sweep::{Frame, Path, Span, SweepLoop, skin},
};

/// One profile to loft through: a closed loop or an open chain, drawn in
/// `plane`'s `(u, v)` — a closed one counter-clockwise, as a sketch's outer
/// loop runs — and what its station is called.
#[derive(Clone, Debug)]
pub struct Section<S: Scalar> {
    pub plane: CoordinateSystem<S>,
    pub profile: Profile<S>,
    pub name: String,
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
fn starting_at<S: Scalar>(profile: &Profile<S>, k: usize) -> Profile<S> {
    let mut out = profile.clone();
    out.curves.rotate_left(k);
    out.curve_names.rotate_left(k);
    out.joint_names.rotate_left(k);
    out
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

    // Pieces: as many curves in every section.
    let most = placed
        .iter()
        .map(|p| p.profile.curves.len())
        .max()
        .unwrap_or(0);
    for p in &mut placed {
        while p.profile.curves.len() < most {
            let longest = (0..p.profile.curves.len())
                .max_by(|&a, &b| {
                    polygon_length(&p.profile.curves[a])
                        .total_cmp(&polygon_length(&p.profile.curves[b]))
                })
                .expect("a profile has curves");
            p.profile = halved(&p.profile, longest)?;
        }
    }

    // Start: line each section's joints up with the one before.
    for s in 1..placed.len() {
        let before = placed[s - 1].joints()?;
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
                (centres[s], centres[s - 1])
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

/// Lofts through `sections` (see the module docs): closed loops into a solid
/// named `solid` — capped by the first and last sections — or, without one,
/// into sheets; open chains into sheets only.
///
/// Named after the first section's curves `X` and joints `P` and the
/// sections' names `K` (see [`skin`]): the walls `N(X)` — `N(X,K>L)` between
/// the sections `K` and `L` when there are more than two — the curves at the
/// sections `N(X,K)`, the edges between them `N(P)` or `N(P,K>L)`, the
/// vertices `N(P,K)`, the caps `N(start)` and `N(end)`.
pub fn loft<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: Option<&str>,
    sections: &[Section<S>],
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
    let placed = correspond(sections).with_context(ctx)?;
    let spans = sections.len() - 1;
    let path = Path {
        stations: placed.iter().map(|p| p.frame.clone()).collect(),
        spans: vec![Span::Line; spans],
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
        loft(&mut part, &namer, Some(&namer.root()), sections).unwrap();
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
        };
        let mut part = Part::<S>::new();
        let namer = Namer::new("loft", "l").unwrap();
        let built = loft(
            &mut part,
            &namer,
            None,
            &[open("a", level(0.0), line), open("b", level(1.0), arc)],
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
        assert!(loft(&mut part, &namer, Some("loft(l)"), &[only]).is_err());
    }
}
