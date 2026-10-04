//! Revolving: sweeping a planar profile around an axis (see
//! [`crate::sweep`]) — a full turn, or any angle up to one.
//!
//! The profile is given in `(r, z)`: `r` away from the axis, `z` along it.
//! Every station is a half-plane through the axis, at its own angle; every
//! span between two of them, at most a quarter turn, sweeps each profile
//! point along an exact circular arc — a rational quadratic.
//!
//! A part of the profile on the axis stays put (see [`SweepLoop`]): a joint
//! there is a pole, a curve along it sweeps nothing. So a region touching
//! the axis along an edge revolves a full turn into a solid with no caps at
//! all, and through a partial turn into one whose two caps share that edge.

use crate::{
    common::{Profile, end_point, line2, sqrt2_over_2, start_point},
    sweep::{Frame, Path, Span, SweepLoop, sweep},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{SolidId, build::BuiltBody};
use geop_ops::{Namer, Part};

/// `(cos, sin)` of `degrees` — exact at every quarter turn, so that a
/// revolve through a right angle lands exactly on the plane it should.
fn cos_sin(degrees: f64) -> (f64, f64) {
    match degrees.rem_euclid(360.0) {
        0.0 => (1.0, 0.0),
        90.0 => (0.0, 1.0),
        180.0 => (-1.0, 0.0),
        270.0 => (0.0, -1.0),
        d => (d.to_radians().cos(), d.to_radians().sin()),
    }
}

/// The path around the axis `axes.w()` through `axes.origin()`, from the
/// angle `from` to `to` — either way round — in degrees, angles turning
/// from `axes.u()` towards `axes.v()`: a profile point `(r, z)` at angle `a`
/// lies at `origin + r (cos a u + sin a v) + z w`.
///
/// Its stations are `a0` (at `from`), `a1`, ... and the spans between them
/// `q0`, `q1`, ..., each at most a quarter turn, all equal. A full turn —
/// `to` a whole turn from `from`, either way — closes on itself, and always
/// runs backwards: `q3` from `a0` back to `a3`, then `q2`, `q1`, `q0`, so
/// that `a1` lies a quarter turn on from `a0`.
pub fn revolution<S: Scalar>(
    axes: &CoordinateSystem<S>,
    from: f64,
    to: f64,
) -> GeopResult<Path<S>> {
    let sweep = to - from;
    if !(sweep != 0.0 && sweep.abs() <= 360.0) {
        return Err(GeopError::new(format!(
            "revolution: cannot turn from {from} to {to} degrees: it must be more than none and at most a full turn"
        )));
    }
    let full = sweep.abs() == 360.0;
    let (u, v, w) = (axes.u(), axes.v(), axes.w());
    let direction = |degrees: f64| {
        let (c, s) = cos_sin(degrees);
        u.prod_scalar(S::from_f64(c))
            .add(&v.prod_scalar(S::from_f64(s)))
    };
    let frame = |e1: Vector3<S>| Frame {
        origin: *axes.origin(),
        e1,
        e2: *w,
    };
    // A full turn runs backwards, in quarter turns; any other forwards, in
    // as few equal spans as keep each at most a quarter turn.
    let (spans, step) = if full {
        (4, -90.0)
    } else {
        let spans = (sweep.abs() / 90.0).ceil() as usize;
        (spans, sweep / spans as f64)
    };
    let angles: Vec<f64> = (0..spans + usize::from(!full))
        .map(|k| from + k as f64 * step)
        .collect();
    let stations: Vec<Frame<S>> = angles.iter().map(|&a| frame(direction(a))).collect();
    // The middle control point of an arc through `step` lies where the
    // tangents at its ends meet, `1 / (1 + cos step)` along the sum of their
    // directions, weighted `cos(step / 2)`.
    let (cos_step, _) = cos_sin(step);
    let weight = if step.abs() == 90.0 {
        sqrt2_over_2()
    } else {
        S::from_f64((step / 2.0).to_radians().cos())
    };
    let spans_of = (0..spans)
        .map(|j| {
            let (a, b) = (&stations[j], &stations[(j + 1) % stations.len()]);
            Span::arc(
                frame(
                    a.e1.add(&b.e1)
                        .prod_scalar(S::from_f64(1.0 / (1.0 + cos_step))),
                ),
                weight,
            )
        })
        .collect();
    // Which way round the path runs, against the stations' `e1 x e2`: the
    // tangent at the first station, turning the way the angles do.
    let (c, s) = cos_sin(from);
    let tangent = v
        .prod_scalar(S::from_f64(c))
        .sub(&u.prod_scalar(S::from_f64(s)));
    let along = stations[0]
        .e1
        .prod_cross(w)
        .prod_dot(&tangent)
        .mul(S::from_f64(step));
    let along_normal = if along.definitely_greater(S::ZERO) {
        true
    } else if along.definitely_less(S::ZERO) {
        false
    } else {
        return Err(GeopError::new(format!(
            "revolution: the axes {axes} are degenerate"
        )));
    };
    let (station_names, span_names) = if full {
        (
            ["a0", "a3", "a2", "a1"].map(String::from).to_vec(),
            ["q3", "q2", "q1", "q0"]
                .map(|q| Some(q.to_string()))
                .to_vec(),
        )
    } else {
        (
            (0..angles.len()).map(|k| format!("a{k}")).collect(),
            (0..spans).map(|k| Some(format!("q{k}"))).collect(),
        )
    };
    Ok(Path {
        stations,
        spans: spans_of,
        closed: full,
        along_normal,
        station_names,
        span_names,
    })
}

/// Revolves `loops` — the first the outer loop, counter-clockwise in `(r,
/// z)`, the rest holes in it, clockwise — from `from` to `to` degrees
/// around `axes` (see [`revolution`]): into a solid named `solid`, its start
/// cap at `from`, or, without one, into sheets. Either way round, the solid
/// comes out with its faces pointing outwards.
///
/// Named after the profiles' curves `X` and joints `P` (see [`Profile`]
/// and [`sweep`]): `N(X,q)` the face `X` sweeps through span `q`, `N(X,a)`
/// the edge `X` is at station `a`, `N(P,q)` the arc `P` sweeps through `q`,
/// `N(P,a)` the vertex of `P` at `a` — `N(P)` for a pole — and for a partial
/// turn, the caps `N(start)` and `N(end)`.
pub fn revolve<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: Option<&str>,
    axes: &CoordinateSystem<S>,
    from: f64,
    to: f64,
    loops: &[SweepLoop<S>],
) -> GeopResult<BuiltBody> {
    let path = revolution(axes, from, to)?;
    let loops: Vec<SweepLoop<S>> = if path.along_normal {
        loops.iter().map(SweepLoop::reversed).collect()
    } else {
        loops.to_vec()
    };
    sweep(part, namer, &path, &loops, solid)
}

/// Revolves `profile` a full turn around `axes.w()` into a solid named
/// `solid` — the way the basic shapes are built: `profile` runs "top-down",
/// walked from its first point to its last the region it bounds together
/// with the axis lies on its right (e.g. `(0, h) -> (r, h) -> (r, 0) -> (0,
/// 0)` for a cylinder). An open chain starts and ends on the axis, and every
/// point of it there is a pole; a closed loop stays clear of the axis and
/// sweeps a ring.
pub fn revolve_at_oriented<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: &str,
    profile: &Profile<S>,
    axes: &CoordinateSystem<S>,
) -> GeopResult<SolidId> {
    profile.check_names()?;
    let lp = if profile.is_closed() {
        for (i, curve) in profile.curves.iter().enumerate() {
            if start_point(curve)?[0].could_be_equal(S::ZERO) {
                return Err(GeopError::new(format!(
                    "revolve: a closed profile must stay off the axis, but its point {i} could be on it"
                )));
            }
        }
        SweepLoop::plain(profile.clone())
    } else {
        // Closed along the axis, which sweeps nothing.
        let (first, last) = match (profile.curves.first(), profile.curves.last()) {
            (Some(first), Some(last)) => (first, last),
            _ => {
                return Err(GeopError::new(
                    "revolve: profile must have at least 1 curve",
                ));
            }
        };
        let mut closed = profile.clone();
        closed
            .curves
            .push(line2(end_point(last)?, start_point(first)?)?);
        closed.curve_names.push("axis".into());
        let poles = closed
            .curves
            .iter()
            .map(|c| Ok(start_point(c)?[0].could_be_equal(S::ZERO)))
            .collect::<GeopResult<Vec<bool>>>()?;
        if !(poles[0] && poles[profile.curves.len()]) {
            return Err(GeopError::new(
                "revolve: an open profile must start and end at r = 0 (a pole)",
            ));
        }
        let mut on_axis = vec![false; closed.curves.len()];
        on_axis[profile.curves.len()] = true;
        SweepLoop {
            profile: closed,
            poles,
            on_axis,
        }
    };
    // A full turn runs along its stations' `e1 x e2`, so it takes the
    // profile clockwise: top-down, as given.
    let built = sweep(
        part,
        namer,
        &revolution(axes, 0.0, 360.0)?,
        &[lp],
        Some(solid),
    )?;
    built
        .solid
        .ok_or_else(|| GeopError::new("revolve: built no solid"))
}

/// [`revolve_at_oriented`] around the axis parallel to `z` through
/// `origin`, with `a0` along `x`.
pub fn revolve_at<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    profile: &Profile<S>,
    origin: Vector3<S>,
) -> GeopResult<SolidId> {
    let axes = CoordinateSystem::try_new(
        origin,
        Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
        Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
        Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
    )?;
    revolve_at_oriented(part, namer, &namer.root(), profile, &axes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{arc2, polyline};
    use geop_core_geometry::nurb_curve::NurbCurve2D;
    use geop_core_math::for_all_scalars;
    use geop_core_math::vector::Vector2;
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    /// Revolve `curves` around the z-axis into a fresh part, as operation `r`.
    fn revolved<S: Scalar>(curves: Vec<NurbCurve2D<S>>) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("revolve", "r").unwrap();
        revolve_at(&mut part, &namer, &Profile::open(curves), Vector3::zero()).unwrap();
        part.check_names().unwrap();
        part
    }

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
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

    /// An exact sphere from two quarter arcs: curved profile edges become
    /// doubly curved quadrant patches.
    fn check_sphere_from_arcs_is_valid<S: Scalar>() {
        let w = sqrt2_over_2::<S>();
        let profile = vec![
            arc2(v2(0.0, 1.0), v2(1.0, 1.0), v2(1.0, 0.0), w).unwrap(),
            arc2(v2(1.0, 0.0), v2(1.0, -1.0), v2(0.0, -1.0), w).unwrap(),
        ];
        let part = revolved(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 8);
        // Two poles, and the equator's ring of four vertices named after the
        // joint between the arcs.
        for name in [
            "revolve(r,p0)",
            "revolve(r,p2)",
            "revolve(r,p1,a0)",
            "revolve(r,p1,a3)",
        ] {
            part.vertex_id(name).unwrap();
        }
        for name in ["revolve(r,c0,a0)", "revolve(r,c1,a2)", "revolve(r,p1,q3)"] {
            part.edge_id(name).unwrap();
        }
        part.face_id("revolve(r,c1,q3)").unwrap();
    }
    #[test]
    fn sphere_from_arcs_is_valid() {
        for_all_scalars!(check_sphere_from_arcs_is_valid);
    }

    /// A vase: a line up the side and a cubic spline bulging out, capped by
    /// lines back to the axis.
    fn check_vase_with_spline_is_valid<S: Scalar>() {
        let hom = |x: f64, y: f64| {
            geop_core_math::vector::Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE])
        };
        let spline = geop_core_geometry::nurb_curve::NurbCurve::try_new(
            3,
            vec![hom(0.5, 2.0), hom(1.5, 1.5), hom(0.2, 0.7), hom(1.0, 0.0)],
            vec![
                S::ZERO,
                S::ZERO,
                S::ZERO,
                S::ZERO,
                S::ONE,
                S::ONE,
                S::ONE,
                S::ONE,
            ],
        )
        .unwrap();
        let mut profile = polyline(&[v2(0.0, 2.0), v2(0.5, 2.0)]).unwrap();
        profile.push(spline);
        profile.extend(polyline(&[v2(1.0, 0.0), v2(0.0, 0.0)]).unwrap());
        let part = revolved(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 12);
    }
    #[test]
    fn vase_with_spline_is_valid() {
        for_all_scalars!(check_vase_with_spline_is_valid);
    }

    fn check_cone_is_valid<S: Scalar>() {
        let profile = polyline(&[v2::<S>(0.0, 1.0), v2(1.0, 0.0), v2(0.0, 0.0)]).unwrap();
        assert_valid(revolved(profile).topology());
    }
    #[test]
    fn cone_is_valid() {
        for_all_scalars!(check_cone_is_valid);
    }

    fn check_sphere_is_valid<S: Scalar>() {
        let n = 6;
        let points: Vec<Vector2<S>> = (0..=n)
            .map(|k| {
                if k == 0 || k == n {
                    return v2(0.0, if k == 0 { 1.0 } else { -1.0 });
                }
                let t = std::f64::consts::PI * (k as f64) / (n as f64);
                v2(t.sin(), t.cos())
            })
            .collect();
        assert_valid(revolved(polyline(&points).unwrap()).topology());
    }
    #[test]
    fn sphere_is_valid() {
        for_all_scalars!(check_sphere_is_valid);
    }

    /// Revolve the closed loop `curves` around the z-axis into a fresh part.
    fn revolved_ring<S: Scalar>(curves: Vec<NurbCurve2D<S>>) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("revolve", "r").unwrap();
        revolve_at(&mut part, &namer, &Profile::closed(curves), Vector3::zero()).unwrap();
        part.check_names().unwrap();
        part
    }

    /// A square off the axis revolves into a ring with a square cross
    /// section: genus one, four quadrant faces per side, no poles.
    fn check_square_ring_is_valid<S: Scalar>() {
        // Clockwise, so the square lies on the right of it walked along.
        let profile = polyline(&[
            v2::<S>(1.0, 1.0),
            v2(2.0, 1.0),
            v2(2.0, 0.0),
            v2(1.0, 0.0),
            v2(1.0, 1.0),
        ])
        .unwrap();
        let part = revolved_ring(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 16);
        assert_eq!(model.vertices.len(), 16);
        assert_eq!(model.edges.len(), 32);
        for name in ["revolve(r,c0,q0)", "revolve(r,c3,q3)", "revolve(r,p0,a3)"] {
            assert!(
                part.face_id(name).is_ok() || part.vertex_id(name).is_ok(),
                "{name}"
            );
        }
        part.edge_id("revolve(r,p0,q3)").unwrap();
        part.edge_id("revolve(r,c3,a0)").unwrap();
    }
    #[test]
    fn square_ring_is_valid() {
        for_all_scalars!(check_square_ring_is_valid);
    }

    /// A circle off the axis revolves into an exact torus: two half arcs,
    /// clockwise, swept into doubly curved quadrant patches.
    fn check_torus_is_valid<S: Scalar>() {
        let w = sqrt2_over_2::<S>();
        let profile = vec![
            arc2(v2(2.0, 1.0), v2(3.0, 1.0), v2(3.0, 0.0), w).unwrap(),
            arc2(v2(3.0, 0.0), v2(3.0, -1.0), v2(2.0, -1.0), w).unwrap(),
            arc2(v2(2.0, -1.0), v2(1.0, -1.0), v2(1.0, 0.0), w).unwrap(),
            arc2(v2(1.0, 0.0), v2(1.0, 1.0), v2(2.0, 1.0), w).unwrap(),
        ];
        let part = revolved_ring(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 16);
    }
    #[test]
    fn torus_is_valid() {
        for_all_scalars!(check_torus_is_valid);
    }

    /// A loop touching the axis cannot sweep a ring.
    #[test]
    fn rings_stay_off_the_axis() {
        type S = geop_core_math::scalars::ScalInF64;
        let profile =
            polyline(&[v2::<S>(0.0, 1.0), v2(1.0, 1.0), v2(1.0, 0.0), v2(0.0, 1.0)]).unwrap();
        let mut part = Part::<S>::new();
        let namer = Namer::new("revolve", "r").unwrap();
        assert!(
            revolve_at(
                &mut part,
                &namer,
                &Profile::closed(profile),
                Vector3::zero()
            )
            .is_err()
        );
    }

    /// The `(r, z)` square `[r0, r0 + 1] x [0, 1]`, counter-clockwise; with
    /// `r0 = 0` its side on the axis is a curve along it, between two poles.
    fn square_loop<S: Scalar>(r0: f64) -> SweepLoop<S> {
        let profile = Profile::closed(
            polyline(&[
                v2(r0, 0.0),
                v2(r0 + 1.0, 0.0),
                v2(r0 + 1.0, 1.0),
                v2(r0, 1.0),
                v2(r0, 0.0),
            ])
            .unwrap(),
        );
        let on = r0 == 0.0;
        SweepLoop {
            profile,
            poles: vec![on, false, false, on],
            on_axis: vec![false, false, false, on],
        }
    }

    fn z_axes<S: Scalar>() -> CoordinateSystem<S> {
        CoordinateSystem::world_at(Vector3::zero())
    }

    /// Revolves `loops` from `from` to `to` degrees around the z-axis into a
    /// solid of a fresh part, as operation `r`.
    fn revolved_by<S: Scalar>(loops: &[SweepLoop<S>], from: f64, to: f64) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("revolve", "r").unwrap();
        revolve(
            &mut part,
            &namer,
            Some("revolve(r)"),
            &z_axes(),
            from,
            to,
            loops,
        )
        .unwrap();
        part.check_names().unwrap();
        part
    }

    /// A partial turn of a region on the axis: walls for the three sides
    /// off it, one per span, and two caps sharing the side on the axis.
    fn check_partial_turns_are_capped<S: Scalar>() {
        for (from, to, spans) in [
            (0.0, 90.0, 1),
            (0.0, 270.0, 3),
            (0.0, -120.0, 2),
            (-30.0, 45.0, 1),
        ] {
            let part = revolved_by(&[square_loop::<S>(0.0)], from, to);
            let model = part.topology();
            assert_valid(model);
            assert_eq!(model.faces.len(), 3 * spans + 2, "{from} to {to}");
            let axis = part.edge_id("revolve(r,c3)").unwrap();
            let caps: Vec<_> = model
                .coedges_of_edge(axis)
                .into_iter()
                .map(|c| {
                    part.name_of(model.get_coedge(c).unwrap().face)
                        .unwrap()
                        .to_string()
                })
                .collect();
            assert_eq!(caps.len(), 2);
            assert!(caps.contains(&"revolve(r,start)".to_string()));
            assert!(caps.contains(&"revolve(r,end)".to_string()));
            // The end cap lies at `to`.
            let corner = model
                .get_vertex(part.vertex_id(&format!("revolve(r,p2,a{spans})")).unwrap())
                .unwrap()
                .point;
            let (c, s) = cos_sin(to);
            let expected = Vector3::from_array([S::from_f64(c), S::from_f64(s), S::ONE]);
            assert!(corner.could_be_equal(&expected), "{corner:?}");
        }
    }
    #[test]
    fn partial_turns_are_capped() {
        for_all_scalars!(check_partial_turns_are_capped);
    }

    /// A turn of a region clear of the axis, with a hole: a full one sweeps
    /// the hole into a void, a shell of its own; a partial one cuts a
    /// tunnel through the caps.
    fn check_holes_sweep_voids_and_tunnels<S: Scalar>() {
        let outer = SweepLoop::plain(Profile::closed(
            polyline(&[
                v2::<S>(1.0, 0.0),
                v2(4.0, 0.0),
                v2(4.0, 3.0),
                v2(1.0, 3.0),
                v2(1.0, 0.0),
            ])
            .unwrap(),
        ));
        let hole = SweepLoop::plain(
            Profile::closed(
                polyline(&[
                    v2::<S>(2.0, 1.0),
                    v2(2.0, 2.0),
                    v2(3.0, 2.0),
                    v2(3.0, 1.0),
                    v2(2.0, 1.0),
                ])
                .unwrap(),
            )
            .with_prefix("h"),
        );
        let loops = [outer, hole];
        let full = revolved_by(&loops, 0.0, 360.0);
        assert_valid(full.topology());
        let solid = full.solid_id("revolve(r)").unwrap();
        assert_eq!(full.topology().get_solid(solid).unwrap().shells.len(), 2);
        assert_eq!(full.topology().faces.len(), 32);

        let quarter = revolved_by(&loops, 0.0, 90.0);
        assert_valid(quarter.topology());
        assert_eq!(quarter.topology().faces.len(), 4 + 4 + 2);
        let cap = quarter.face_id("revolve(r,start)").unwrap();
        assert_eq!(quarter.topology().get_face(cap).unwrap().holes.len(), 1);
    }
    #[test]
    fn holes_sweep_voids_and_tunnels() {
        for_all_scalars!(check_holes_sweep_voids_and_tunnels);
    }

    /// Without a solid, a sweep builds a sheet of walls: an open chain off
    /// the axis turned fully is a tube with two free rims, one ending on
    /// the axis a disc closing over its pole.
    fn check_sheets_are_walls_alone<S: Scalar>() {
        let namer = Namer::new("revolve", "r").unwrap();
        for (chain, faces, free) in [
            (polyline(&[v2::<S>(1.0, 0.0), v2(1.0, 1.0)]).unwrap(), 4, 8),
            (polyline(&[v2(0.0, 0.0), v2(1.0, 0.0)]).unwrap(), 4, 4),
        ] {
            let mut part = Part::<S>::new();
            let profile = Profile::open(chain);
            let poles = profile
                .joint_names
                .iter()
                .enumerate()
                .map(|(i, _)| {
                    i == 0 && start_point(&profile.curves[0]).unwrap()[0].could_be_equal(S::ZERO)
                })
                .collect();
            let lp = SweepLoop {
                on_axis: vec![false],
                poles,
                profile,
            };
            let built = revolve(&mut part, &namer, None, &z_axes(), 0.0, 360.0, &[lp]).unwrap();
            part.check_names().unwrap();
            assert!(built.solid.is_none());
            let model = part.topology();
            if let Err(e) = validate(&ValidationParameters::default(), model) {
                panic!("{e:?}");
            }
            assert_eq!(model.faces.len(), faces);
            assert_eq!(part.sheet_face_names().len(), faces);
            let one_sided = model
                .edges
                .keys()
                .filter(|&&e| model.coedges_of_edge(e).len() == 1)
                .count();
            assert_eq!(one_sided, free);
        }
    }
    #[test]
    fn sheets_are_walls_alone() {
        for_all_scalars!(check_sheets_are_walls_alone);
    }
}
