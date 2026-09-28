//! Extrude a closed planar profile — an outer loop plus any number of holes,
//! each a closed chain of NURBS curves in the `(u, v)` plane of a coordinate
//! system — one unit along its `w` to produce a solid, entirely from euler
//! operations.
//!
//! **Orientation contract:** `outer` must wind counter-clockwise, each hole
//! clockwise, and `coordinate_system` must be **left-handed** (`u x v = -w`).
//! Every side wall is parametrized `(height, profile)`, so the normal it ends
//! up with is `w x d` for a wall along profile direction `d`; that is the
//! outward normal exactly under this combination. Hand it a right-handed
//! basis and the whole solid comes out inside-out — still structurally
//! valid, still renders, and rejected by
//! `validation::face_orientation::check_normals_point_outward`.
//! [`extrude_from_plane`] takes a right-handed plane instead and arranges
//! this itself.
//!
//! **Profile curves:** every curve must be clamped (start at its first
//! control point, end at its last), have the domain `[0, 1]`, and start
//! exactly where the previous one in its loop ends; each loop needs at least
//! two curves (every vertex of the solid sits at a curve joint). The curves
//! may lie anywhere in the plane: the flat caps are sized to the profile.
//!
//! [`grow_ring`] (`mvfs`/`mvr` + `mve_from_vertex` + `mve`) builds the outer
//! ring directly on a placeholder (`NurbSurface3D::everything()`) face;
//! `mef` (like `mer`, but the split-off ring is moved onto a brand new,
//! real-surfaced face instead of staying as a second boundary of the same
//! face) turns that ring into the bottom cap, leaving the *same* ring,
//! traced the other way, on the placeholder face. Each hole is grown the
//! same way, directly on the (now real) bottom cap instead — `mvr` gives it
//! a fresh bare-vertex boundary there, and `mer` (like `mef`, but the
//! split-off ring moves onto an already-existing face instead of a new one)
//! moves its "other side" onto the placeholder face too, alongside the
//! outer ring's own. [`build_side_walls`] then advances every one of these
//! rings straight up by `w`, closing off a side wall per curve — since the
//! pcurves `grow_ring`/`mer` leave it were only ever valid for whatever face
//! the ring was *grown* on, the very first thing it does is swap them all
//! out (`replace_pcurve`) for the side walls' own reusable convention — and
//! leaves their top rings on the placeholder face, so `replace_face` can
//! give it the real top surface (with the same holes) as the very last step.

use crate::common::{
    Profile, bilinear, embed_curve, embed_point, end_point, line2, line3, start_point,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_ops::{Namer, Part};
use geop_core_topology::{CoedgeId, FaceId, SolidId, VertexId};

/// Where the profile sits in `(u, v)`, and the pcurves and surfaces that
/// follow from that: the two flat caps span the profile's bounding box, so a
/// profile point maps to cap parameters by one affine map.
struct Caps<'a, S: Scalar> {
    coordinate_system: &'a CoordinateSystem<S>,
    lo: Vector2<S>,
    size: Vector2<S>,
}

impl<'a, S: Scalar> Caps<'a, S> {
    fn new(
        coordinate_system: &'a CoordinateSystem<S>,
        loops: impl Iterator<Item = &'a NurbCurve2D<S>>,
    ) -> GeopResult<Self> {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for curve in loops {
            for cp in &curve.control_points {
                for k in 0..2 {
                    let x = cp[k].div(cp[2])?;
                    lo[k] = lo[k].min(x.lower().to_f64());
                    hi[k] = hi[k].max(x.upper().to_f64());
                }
            }
        }
        // The caps span exactly this box. It encloses the profile: a NURBS
        // curve stays within the convex hull of its control points, and the
        // bounds above are the outer bounds of their enclosures.
        let size = Vector2::from_array([S::from_f64(hi[0] - lo[0]), S::from_f64(hi[1] - lo[1])]);
        let lo = Vector2::from_array([S::from_f64(lo[0]), S::from_f64(lo[1])]);
        Ok(Caps {
            coordinate_system,
            lo,
            size,
        })
    }

    /// `curve` in the bottom cap's parameters.
    fn bottom_pcurve(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve2D<S>> {
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
        NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())
    }

    /// `curve` in the top cap's parameters, which run along `(v, u)`.
    fn top_pcurve(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbCurve2D<S>> {
        Ok(self.bottom_pcurve(curve)?.swap_xy())
    }

    fn corner(&self, i: usize, j: usize, height: S) -> Vector3<S> {
        let pick = |k: usize, far: usize| {
            if far == 1 {
                self.lo[k].add(self.size[k])
            } else {
                self.lo[k]
            }
        };
        self.coordinate_system
            .to_xyz(&Vector3::from_array([pick(0, i), pick(1, j), height]))
    }

    fn bottom_surface(&self) -> GeopResult<NurbSurface3D<S>> {
        let h = S::ZERO;
        bilinear(
            self.corner(0, 0, h),
            self.corner(1, 0, h),
            self.corner(1, 1, h),
            self.corner(0, 1, h),
        )
    }

    fn top_surface(&self) -> GeopResult<NurbSurface3D<S>> {
        let h = S::ONE;
        bilinear(
            self.corner(0, 0, h),
            self.corner(0, 1, h),
            self.corner(1, 1, h),
            self.corner(1, 0, h),
        )
    }

    /// A profile point at `height` (0 = bottom cap, 1 = top cap).
    fn point(&self, p: &Vector2<S>, height: S) -> Vector3<S> {
        self.coordinate_system
            .to_xyz(&Vector3::from_array([p[0], p[1], height]))
    }

    /// A profile curve at `height`.
    fn curve(
        &self,
        curve: &NurbCurve2D<S>,
        height: S,
    ) -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve3D<S>> {
        let cs = self.coordinate_system;
        embed_curve(
            curve,
            &cs.origin().add(&cs.w().prod_scalar(height)),
            cs.u(),
            cs.v(),
        )
    }

    /// The side wall swept by `curve`: degree 1 in `u` (height 0 to 1),
    /// `curve`'s own degree and knots in `v`.
    fn wall(&self, curve: &NurbCurve2D<S>) -> GeopResult<NurbSurface3D<S>> {
        let cs = self.coordinate_system;
        let top = cs.origin().add(cs.w());
        let control_points = [cs.origin(), &top]
            .into_iter()
            .flat_map(|origin| {
                curve
                    .control_points
                    .iter()
                    .map(move |cp| embed_point(cp, origin, cs.u(), cs.v()))
            })
            .collect();
        NurbSurface::try_new(
            1,
            curve.degree,
            control_points,
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            curve.knot_vector.clone(),
        )
    }
}

/// The names one extrude gives to what it builds from one region, following
/// `geop_ops`'s scheme: `N` is the operation's [`Namer`], `X` a profile
/// curve's name and `P` a joint's (see [`Profile`]).
///
/// | entity | name |
/// |---|---|
/// | side face swept by curve `X` | `N(X)` |
/// | edge along `X` on the start / end cap | `N(X,start)` / `N(X,end)` |
/// | edge swept by joint `P` | `N(P)` |
/// | vertex at `P` on the start / end cap | `N(P,start)` / `N(P,end)` |
/// | start / end cap | `N(start)` / `N(end)`, or `N(start,R)` / `N(end,R)` |
///
/// The start cap lies on the profile's plane, the end cap `w` away from it.
/// `region` qualifies the caps (`R`) when one operation extrudes several
/// regions, each of which has caps of its own.
pub struct ExtrudeNames<'a> {
    pub namer: &'a Namer,
    pub region: Option<&'a str>,
    /// The name of the solid built.
    pub solid: String,
}

impl<'a> ExtrudeNames<'a> {
    /// The names for an operation that extrudes a single region: its solid
    /// is the operation's own name, its caps are unqualified.
    pub fn single(namer: &'a Namer) -> Self {
        Self {
            namer,
            region: None,
            solid: namer.root(),
        }
    }

    fn curve(&self, profile: &Profile<impl Scalar>, i: usize, role: &str) -> String {
        self.namer.name(&[&profile.curve_names[i], role])
    }
    fn joint(&self, profile: &Profile<impl Scalar>, i: usize, role: &str) -> String {
        self.namer.name(&[&profile.joint_names[i], role])
    }
    fn side_face(&self, profile: &Profile<impl Scalar>, i: usize) -> String {
        self.namer.name(&[&profile.curve_names[i]])
    }
    fn lateral_edge(&self, profile: &Profile<impl Scalar>, i: usize) -> String {
        self.namer.name(&[&profile.joint_names[i]])
    }
    fn cap(&self, role: &str) -> String {
        match self.region {
            None => self.namer.name(&[role]),
            Some(region) => self.namer.name(&[role, region]),
        }
    }
}

/// Grow a `curves.len() - 1`-edge chain from an existing bare-vertex
/// boundary `v0` of `face_id` (as added by `mvfs` or `mvr`), along
/// `curves[..n - 1]`, via `mve_from_vertex` then `mve` — both directions'
/// pcurves are the bottom cap's, valid against `face_id`'s own current
/// surface (be it still the generic placeholder, for the outer ring, or the
/// already-real bottom cap, for a hole).
/// Returns `(first_forward, first_mirror, last_forward, last_mirror)`.
fn grow_ring<S: Scalar>(
    part: &mut Part<S>,
    names: &ExtrudeNames,
    face_id: FaceId,
    v0: VertexId,
    caps: &Caps<S>,
    profile: &Profile<S>,
) -> GeopResult<(CoedgeId, CoedgeId, CoedgeId, CoedgeId)> {
    let curves = &profile.curves;
    let n = curves.len();
    let bottom = |i: usize| -> GeopResult<_> {
        let pcurve = caps.bottom_pcurve(&curves[i])?;
        Ok((
            caps.curve(&curves[i], S::ZERO)?,
            pcurve.clone(),
            pcurve.reverse(),
            caps.point(&end_point(&curves[i])?, S::ZERO),
        ))
    };

    let (curve, pcurve, pcurve_reversed, end) = bottom(0)?;
    let (_, first_forward, first_mirror, _) = part.mve_from_vertex(
        face_id,
        v0,
        curve,
        pcurve,
        pcurve_reversed,
        end,
        names.joint(profile, 1, "start"),
        names.curve(profile, 0, "start"),
    )?;

    let mut cursor = first_forward;
    let mut last_mirror = first_mirror;
    for i in 1..n - 1 {
        let (curve, pcurve, pcurve_reversed, end) = bottom(i)?;
        let (_, next_forward, next_mirror, _) = part.mve(
            cursor,
            curve,
            pcurve,
            pcurve_reversed,
            end,
            names.joint(profile, i + 1, "start"),
            names.curve(profile, i, "start"),
        )?;
        cursor = next_forward;
        last_mirror = next_mirror;
    }

    Ok((first_forward, first_mirror, cursor, last_mirror))
}

/// The reusable pcurve every "mirror" (base-level) coedge on the
/// placeholder face gets, whichever ring it belongs to: every side wall
/// shares the same `(height, profile)` layout, so its base-level edge is
/// always `(u, v) = (0, 1) -> (0, 0)`, regardless of which curve or which
/// ring it is.
fn base_level_pcurve<S: Scalar>() -> GeopResult<NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ZERO, S::ONE]),
        Vector2::from_array([S::ZERO, S::ZERO]),
    )
}

/// Sweep the ring anchored at `down_face_coedge` straight up by one unit in
/// `w`, one curve of `profile` at a time, closing off a side wall per curve.
/// `down_face_coedge` must be a coedge of a ring tracing `profile` the
/// *opposite* way (i.e. exactly what `mef`/`mer` leave behind on the face
/// they didn't peel the "real" ring off onto) — whatever pcurves it carries
/// in from `grow_ring` are only ever valid for the face it was *grown* on,
/// so the very first thing this function does is stamp every one of its
/// coedges with [`base_level_pcurve`] instead, the one every side wall it
/// goes on to build actually expects. Leaves the fully-swept (topmost) ring
/// in `down_face_coedge`'s ring's place, on whichever face it was already on.
fn build_side_walls<S: Scalar>(
    part: &mut Part<S>,
    names: &ExtrudeNames,
    caps: &Caps<S>,
    profile: &Profile<S>,
    mut down_face_coedge: CoedgeId,
) -> GeopResult<()> {
    let curves = &profile.curves;
    let n = curves.len();
    let points = curves
        .iter()
        .map(start_point)
        .collect::<GeopResult<Vec<_>>>()?;

    for coedge_id in part
        .topology()
        .iterate_loop_coedges(down_face_coedge)
        .collect::<Vec<_>>()
    {
        part.replace_pcurve(coedge_id, base_level_pcurve()?)
            .with_context(with_context!(
                "build_side_walls: replace_pcurve for coedge {coedge_id} failed"
            ))?;
    }

    // The straight edge up from profile vertex `i`, with its pcurves on the
    // walls before (`v = 1`) and after (`v = 0`) it.
    let upwards = |part: &mut Part<S>, at: CoedgeId, i: usize| {
        let top = caps.point(&points[i], S::ONE);
        part.mve(
            at,
            line3(caps.point(&points[i], S::ZERO), top)?,
            line2(
                Vector2::from_array([S::ZERO, S::ZERO]),
                Vector2::from_array([S::ONE, S::ZERO]),
            )?,
            line2(
                Vector2::from_array([S::ONE, S::ONE]),
                Vector2::from_array([S::ZERO, S::ONE]),
            )?,
            top,
            names.joint(profile, i, "end"),
            names.lateral_edge(profile, i),
        )
    };
    // Close the wall of `curves[i]` along its top edge.
    let close_wall = |part: &mut Part<S>, down: CoedgeId, up: CoedgeId, i: usize| {
        part.mef(
            down,
            up,
            caps.curve(&curves[i], S::ONE)?,
            line2(
                Vector2::from_array([S::ONE, S::ZERO]),
                Vector2::from_array([S::ONE, S::ONE]),
            )?,
            caps.top_pcurve(&curves[i])?.reverse(),
            caps.wall(&curves[i])?,
            names.curve(profile, i, "end"),
            names.side_face(profile, i),
        )
    };

    let (_, mut coedge_down, mut coedge_up, _) = upwards(part, down_face_coedge, 0)
        .with_context("build_side_walls: first upwards edge mve failed")?;

    let mut prev_coedge_down = coedge_down;
    let final_coedge_up = coedge_up;

    for i in 1..n {
        down_face_coedge = part.topology().get_coedge(down_face_coedge)?.prev;
        (_, coedge_down, coedge_up, _) = upwards(part, down_face_coedge, i).with_context(
            with_context!("build_side_walls: upwards edge mve failed (i={i})"),
        )?;
        close_wall(part, prev_coedge_down, coedge_up, i - 1).with_context(with_context!(
            "build_side_walls: closing side face mef failed (i={i})"
        ))?;
        prev_coedge_down = coedge_down;
    }

    // The last wall closes against the very first upwards edge.
    close_wall(part, prev_coedge_down, final_coedge_up, n - 1)
        .with_context("build_side_walls: closing mef for the last side face failed")?;

    Ok(())
}

/// Check that every loop is a closed chain of at least two clamped curves on
/// `[0, 1]`, with a name for every curve and joint.
fn validate_loop<S: Scalar>(profile: &Profile<S>) -> GeopResult<()> {
    profile.check_names()?;
    if !profile.is_closed() {
        return Err(GeopError::new(
            "extrude: every loop must be closed, but has a name for an end joint",
        ));
    }
    let curves = &profile.curves;
    if curves.len() < 2 {
        return Err(GeopError::new(
            "extrude: every loop needs at least 2 curves",
        ));
    }
    for (i, curve) in curves.iter().enumerate() {
        let (t0, t1) = curve.domain();
        if !(t0.could_be_equal(S::ZERO) && t1.could_be_equal(S::ONE)) {
            return Err(GeopError::new(format!(
                "extrude: curve {i} has domain ({t0:?}, {t1:?}), not [0, 1]"
            )));
        }
        let next = &curves[(i + 1) % curves.len()];
        let (end, start) = (end_point(curve)?, start_point(next)?);
        if !end.could_be_equal(&start) {
            return Err(GeopError::new(format!(
                "extrude: curve {i} ends at {end:?}, but the next one starts at {start:?}"
            )));
        }
    }
    Ok(())
}

/// Extrude `outer` (counter-clockwise) with `holes` (clockwise) one unit
/// along `coordinate_system`'s `w`, which must be left-handed (see the module
/// docs). Everything built is named after `outer`'s and the holes' curves
/// and joints, see [`ExtrudeNames`].
pub fn extrude<S: Scalar>(
    part: &mut Part<S>,
    names: &ExtrudeNames,
    coordinate_system: &CoordinateSystem<S>,
    outer: &Profile<S>,
    holes: &[Profile<S>],
) -> GeopResult<SolidId> {
    // Not a closure capturing `part` directly: that would hold an immutable
    // borrow of it alive for the whole function, conflicting with every
    // mutable `part.mve`/`part.mef` call below. Each `.with_context(&|e|
    // ctx(...))` call site instead builds a fresh, short-lived closure that
    // only borrows `part` for that one statement.
    fn ctx<S: Scalar>(
        coordinate_system: &CoordinateSystem<S>,
        outer: &Profile<S>,
        holes: &[Profile<S>],
        part: &Part<S>,
        e: GeopError,
    ) -> GeopError {
        e.with_context(format!(
            "extrude(
    coordinate_system={coordinate_system}
    outer={outer:?}
    holes={holes:?}
    model={}
)",
            part.topology()
        ))
    }

    validate_loop(outer)?;
    for hole in holes {
        validate_loop(hole)?;
    }
    let caps = Caps::new(
        coordinate_system,
        outer
            .curves
            .iter()
            .chain(holes.iter().flat_map(|h| h.curves.iter())),
    )?;
    let n = outer.curves.len();

    // The placeholder face becomes the end cap, via `replace_face` at the
    // very end.
    let (start_vertex, any_face, solid_id) = part.mvfs(
        caps.point(&start_point(&outer.curves[0])?, S::ZERO),
        names.joint(outer, 0, "start"),
        names.cap("end"),
        names.solid.clone(),
    )?;

    // Grow the outer ring directly on the placeholder face (the same
    // `grow_ring` helper each hole uses below).
    let (first_base_face_coedge, down_face_coedge, cursor_coedge_out, _) =
        grow_ring(part, names, any_face, start_vertex, &caps, outer)
            .with_context("extrude: growing the outer ring failed")
            .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;

    // last edge splits the bottom cap off, onto a new, real-surfaced face
    let (_, bottom_face_id, _, _) = part
        .mef(
            cursor_coedge_out,
            first_base_face_coedge,
            caps.curve(&outer.curves[n - 1], S::ZERO)?,
            caps.bottom_pcurve(&outer.curves[n - 1])?,
            base_level_pcurve()?,
            caps.bottom_surface()?,
            names.curve(outer, n - 1, "start"),
            names.cap("start"),
        )
        .with_context("extrude: closing mef for the bottom cap failed")
        .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;

    // Grow every hole directly on the (now real) bottom cap, then move its
    // mirror ring onto the placeholder face (alongside the outer ring's
    // own), ready for `build_side_walls` — collecting the placeholder-side
    // anchor for each so their side walls can be swept once every hole has
    // been attached.
    let mut hole_scaffolds: Vec<(&Profile<S>, CoedgeId)> = Vec::new();
    for hole in holes {
        let last = hole.curves.len() - 1;
        let hv0 = part
            .mvr(
                bottom_face_id,
                caps.point(&start_point(&hole.curves[0])?, S::ZERO),
                names.joint(hole, 0, "start"),
            )
            .with_context("extrude: mvr for a hole's starting vertex failed")
            .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;
        let (_, first_mirror, _, last_mirror) =
            grow_ring(part, names, bottom_face_id, hv0, &caps, hole)
                .with_context("extrude: growing a hole's ring failed")
                .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;

        // Close the hole's ring by moving its mirror sub-chain onto the
        // placeholder face — the complementary (forward) sub-chain, the
        // hole's own proper boundary, stays behind on the bottom cap.
        part.mer(
            first_mirror,
            last_mirror,
            caps.curve(&hole.curves[last], S::ZERO)?.reverse(),
            base_level_pcurve()?,
            caps.bottom_pcurve(&hole.curves[last])?,
            any_face,
            names.curve(hole, last, "start"),
        )
        .with_context("extrude: mer for a hole's ring failed")
        .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;

        hole_scaffolds.push((hole, first_mirror));
    }

    // Sweep the outer ring's own mirror chain, then each hole's, straight up.
    build_side_walls(part, names, &caps, outer, down_face_coedge)
        .with_context("extrude: building the outer ring's side walls failed")
        .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;
    for (hole, hole_down_face_coedge) in hole_scaffolds {
        build_side_walls(part, names, &caps, hole, hole_down_face_coedge)
            .with_context("extrude: building a hole's side walls failed")
            .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;
    }

    // Now close the top face using a replace_face operation
    part.replace_face(any_face, caps.top_surface()?)
        .with_context("extrude: replace_face for the top cap failed")
        .with_context(&|e| ctx(coordinate_system, outer, holes, part, e))?;

    Ok(solid_id)
}

/// Extrude a profile drawn on a right-handed `plane` (`u x v` along its
/// normal `w`) by `distance` along `w` — backwards for a negative distance.
/// `outer` winds counter-clockwise and the holes clockwise, as seen in the
/// plane's own `(u, v)`. The start cap lies on `plane`.
///
/// Translates to [`extrude`]'s left-handed contract: a negative distance
/// already gives a left-handed `(u, v, distance w)`; a positive one mirrors
/// the plane's `u` and `v` (and so every curve's `x` and `y`, then reverses
/// the loops to restore their winding — the names travel with their curves
/// and joints).
pub fn extrude_from_plane<S: Scalar>(
    part: &mut Part<S>,
    names: &ExtrudeNames,
    plane: &CoordinateSystem<S>,
    outer: &Profile<S>,
    holes: &[Profile<S>],
    distance: S,
) -> GeopResult<SolidId> {
    let w = plane.w().prod_scalar(distance);
    if distance.definitely_less(S::ZERO) {
        let cs = CoordinateSystem::try_new(*plane.origin(), *plane.u(), *plane.v(), w)?;
        extrude(part, names, &cs, outer, holes)
    } else if distance.definitely_greater(S::ZERO) {
        let cs = CoordinateSystem::try_new(*plane.origin(), *plane.v(), *plane.u(), w)?;
        let mirror = |profile: &Profile<S>| profile.map_curves(|c| c.swap_xy()).reversed();
        let holes: Vec<_> = holes.iter().map(mirror).collect();
        extrude(part, names, &cs, &mirror(outer), &holes)
    } else {
        Err(GeopError::new(format!(
            "extrude_from_plane: distance {distance:?} could be zero"
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{arc2, polygon, sqrt2_over_2};
    use geop_core_math::for_all_scalars;
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn unit_square<S: Scalar>() -> Vec<NurbCurve2D<S>> {
        polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0), v2(0.0, 1.0)]).unwrap()
    }

    /// A square hole (traced the *opposite* winding from `unit_square`, so
    /// the material — the region between the outer boundary and this hole
    /// — genuinely has the hole cut out of it), spanning `[x0, x0 + size] x
    /// [y0, y0 + size]`.
    fn square_hole<S: Scalar>(x0: f64, y0: f64, size: f64) -> Vec<NurbCurve2D<S>> {
        polygon(&[
            v2(x0, y0),
            v2(x0, y0 + size),
            v2(x0 + size, y0 + size),
            v2(x0 + size, y0),
        ])
        .unwrap()
    }

    /// A circle of radius `r` around `(cx, cy)` as four quarter arcs,
    /// counter-clockwise (reversed: clockwise, for a hole).
    fn circle<S: Scalar>(cx: f64, cy: f64, r: f64, clockwise: bool) -> Vec<NurbCurve2D<S>> {
        let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
        let mut arcs: Vec<NurbCurve2D<S>> = (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                arc2(
                    v2(cx + a.0, cy + a.1),
                    v2(cx + a.0 + b.0, cy + a.1 + b.1),
                    v2(cx + b.0, cy + b.1),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect();
        if clockwise {
            arcs = arcs.iter().rev().map(|c| c.reverse()).collect();
        }
        arcs
    }

    /// Left-handed (`u x v = -w`), which is what `extrude` requires of a CCW
    /// outer polygon for the resulting faces to point outward — see the
    /// module doc. `figure8_profile` builds its own the same way. A
    /// right-handed basis here produces a solid that is entirely inside-out:
    /// structurally perfect, and rejected by
    /// `validation::face_orientation::check_normals_point_outward`.
    fn axis_aligned_coordinate_system<S: Scalar>(origin: Vector3<S>) -> CoordinateSystem<S> {
        let u = Vector3::from_array([S::ONE, S::ZERO, S::ZERO]);
        let v = Vector3::from_array([S::ZERO, S::ONE, S::ZERO]);
        let w = Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(-1.0)]);
        CoordinateSystem::try_new(origin, u, v, w).unwrap()
    }

    /// Extrude into a fresh part, as the single-region operation `e`.
    fn extruded<S: Scalar>(
        cs: &CoordinateSystem<S>,
        outer: Vec<NurbCurve2D<S>>,
        holes: Vec<Vec<NurbCurve2D<S>>>,
    ) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("extrude", "e").unwrap();
        let holes: Vec<_> = holes
            .into_iter()
            .enumerate()
            .map(|(k, h)| Profile::closed(h).with_prefix(&format!("h{k}")))
            .collect();
        extrude(
            &mut part,
            &ExtrudeNames::single(&namer),
            cs,
            &Profile::closed(outer),
            &holes,
        )
        .unwrap();
        part.check_names().unwrap();
        part
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
        for edge_id in model.edges.keys() {
            assert_eq!(model.coedges_of_edge(*edge_id).len(), 2);
        }
    }

    fn check_extruded_square_is_valid<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let part = extruded(&cs, unit_square::<S>(), vec![]);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 6);
        assert_eq!(model.vertices.len(), 8);
        assert_eq!(model.edges.len(), 12);
    }
    #[test]
    fn extruded_square_is_valid() {
        for_all_scalars!(check_extruded_square_is_valid);
    }

    fn check_extruded_square_with_hole_is_valid<S: Scalar>() {
        let hole = square_hole::<S>(0.25, 0.25, 0.5);
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let part = extruded(&cs, unit_square::<S>(), vec![hole]);
        let model = part.topology();
        assert_valid(model);
        // 6 outer faces (4 sides + top + bottom) + 4 hole side walls; 8
        // outer vertices + 8 hole vertices (4 on the bottom cap, 4 on top).
        assert_eq!(model.faces.len(), 6 + 4);
        assert_eq!(model.vertices.len(), 8 + 8);
    }
    #[test]
    fn extruded_square_with_hole_is_valid() {
        for_all_scalars!(check_extruded_square_with_hole_is_valid);
    }

    fn check_extrude_offsets_footprint<S: Scalar>() {
        let origin = Vector3::from_array([S::from_f64(2.0), S::from_f64(-1.0), S::from_f64(0.5)]);
        let cs = axis_aligned_coordinate_system(origin);
        let part = extruded(&cs, unit_square::<S>(), vec![]);
        let model = part.topology();
        assert_valid(model);
        // `w` points along `-z` (see `axis_aligned_coordinate_system`), so
        // the profile sits at the origin's own `z` and the far face one unit
        // *below* it.
        for vertex in model.vertices.values() {
            assert!(
                vertex.point[2].could_be_equal(S::from_f64(0.5))
                    || vertex.point[2].could_be_equal(S::from_f64(-0.5)),
                "vertex at unexpected height: {:?}",
                vertex.point
            );
        }
    }
    #[test]
    fn extrude_offsets_footprint() {
        for_all_scalars!(check_extrude_offsets_footprint);
    }

    /// A profile anywhere in the plane, not just the unit square: the caps
    /// are sized to it.
    fn check_extrude_profile_away_from_origin<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let outer = polygon(&[v2(-3.0, 2.0), v2(-1.0, 2.0), v2(-2.0, 5.0)]).unwrap();
        let part = extruded(&cs, outer, vec![]);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 5);
    }
    #[test]
    fn extrude_profile_away_from_origin() {
        for_all_scalars!(check_extrude_profile_away_from_origin);
    }

    /// A disc with a round hole: every wall is a quarter of a cylinder.
    fn check_extruded_ring_is_valid<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let outer = circle::<S>(0.0, 0.0, 2.0, false);
        let hole = circle::<S>(0.3, 0.0, 1.0, true);
        let part = extruded(&cs, outer, vec![hole]);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 2 + 4 + 4);
    }
    #[test]
    fn extruded_ring_is_valid() {
        for_all_scalars!(check_extruded_ring_is_valid);
    }

    /// Both directions off a right-handed plane give valid solids on the
    /// expected side.
    fn check_extrude_from_plane_both_directions<S: Scalar>() {
        let plane = CoordinateSystem::try_new(
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
        )
        .unwrap();
        let mut outer = polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0)]).unwrap();
        outer.push(arc2(v2(1.0, 1.0), v2(0.0, 1.0), v2(0.0, 0.0), sqrt2_over_2()).unwrap());
        outer.remove(2);
        let namer = Namer::new("extrude", "e").unwrap();
        let outer = Profile::closed(outer);
        for (distance, far) in [(0.5, 1.5), (-0.5, 0.5)] {
            let mut part = Part::<S>::new();
            let names = ExtrudeNames::single(&namer);
            extrude_from_plane(
                &mut part,
                &names,
                &plane,
                &outer,
                &[],
                S::from_f64(distance),
            )
            .unwrap();
            part.check_names().unwrap();
            let model = part.topology();
            assert_valid(model);
            assert!(
                model
                    .vertices
                    .values()
                    .any(|v| v.point[2].could_be_equal(S::from_f64(far)))
            );
            // Whichever way the profile had to be mirrored to extrude it,
            // every name still sits where it says: `p0` at the profile's
            // first point, the end cap `distance` away from the plane.
            let p0_start = part.vertex_id("extrude(e,p0,start)").unwrap();
            let p0_end = part.vertex_id("extrude(e,p0,end)").unwrap();
            let point = |v| model.get_vertex(v).unwrap().point;
            assert!(point(p0_start).could_be_equal(&Vector3::from_array([
                S::ZERO,
                S::ZERO,
                S::ONE
            ])));
            assert!(point(p0_end)[2].could_be_equal(S::from_f64(far)));
            let end_cap = part.face_id("extrude(e,end)").unwrap();
            assert!(model.iterate_face_coedges(end_cap).all(|c| {
                model.coedge_start_vertex(c).unwrap().point[2].could_be_equal(S::from_f64(far))
            }));
        }
    }
    #[test]
    fn extrude_from_plane_both_directions() {
        for_all_scalars!(check_extrude_from_plane_both_directions);
    }
}

#[cfg(test)]
mod naming_tests {
    use super::*;
    use crate::common::polygon;
    use geop_core_math::scalars::ScalInF64 as S;

    /// Every entity of an extruded square is named after the profile element
    /// it was swept from, and the names hang together topologically: the
    /// side face of `c0` is bounded by `c0`'s two cap edges and the lateral
    /// edges of its two joints.
    #[test]
    fn extruded_square_names_follow_the_profile() {
        let square = polygon(&[
            Vector2::from_array([S::ZERO, S::ZERO]),
            Vector2::from_array([S::ONE, S::ZERO]),
            Vector2::from_array([S::ONE, S::ONE]),
            Vector2::from_array([S::ZERO, S::ONE]),
        ])
        .unwrap();
        let plane = CoordinateSystem::try_new(
            Vector3::zero(),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
        )
        .unwrap();
        let mut part = Part::<S>::new();
        let namer = Namer::new("extrude", "box").unwrap();
        extrude_from_plane(
            &mut part,
            &ExtrudeNames::single(&namer),
            &plane,
            &Profile::closed(square),
            &[],
            S::ONE,
        )
        .unwrap();
        part.check_names().unwrap();

        let description = geop_ops::PartDescription::of(&part).unwrap();
        assert_eq!(description.solids.len(), 1);
        assert_eq!(description.solids["extrude(box)"][0].len(), 6);
        let mut side = description.faces["extrude(box,c0)"].outer.clone();
        side.sort();
        let mut expected: Vec<String> = [
            "extrude(box,c0,end)",
            "extrude(box,c0,start)",
            "extrude(box,p0)",
            "extrude(box,p1)",
        ]
        .iter()
        .map(|e| e.to_string())
        .collect();
        expected.sort();
        let unsigned: Vec<String> = side.iter().map(|e| e[1..].to_string()).collect();
        let mut unsigned = unsigned;
        unsigned.sort();
        assert_eq!(unsigned, expected);
        assert_eq!(
            description.edges["extrude(box,p1)"].start,
            "extrude(box,p1,start)"
        );
        assert_eq!(
            description.edges["extrude(box,p1)"].end,
            "extrude(box,p1,end)"
        );
        let corner = part.vertex_id("extrude(box,p2,end)").unwrap();
        assert!(
            part.topology()
                .get_vertex(corner)
                .unwrap()
                .point
                .could_be_equal(&Vector3::from_array([S::ONE; 3]))
        );
    }
}
