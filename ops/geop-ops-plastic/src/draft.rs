//! [`draft`]: planar faces of a solid tilted about a neutral plane, so the
//! part comes out of its mould.
//!
//! **Why the faces are replaced, not cut.** A draft only changes the
//! planes some faces lie in; the solid keeps its topology — every face,
//! edge and vertex is still there, still between the same neighbours. So
//! the drafted solid is rebuilt face by face ([`geop_lib_shell::rebuilt`]):
//! each drafted face gets its tilted plane, each vertex of one moves to
//! where the surfaces around it now meet, each edge to where its two faces
//! do. The alternative, a boolean with a wedge per face, would have to put
//! the wedge's tilted face exactly through the face's edge on the neutral
//! plane — a cut along an edge that already exists, the one configuration
//! a boolean cannot stay tight through — and a wedge beside an inward
//! corner would cut into the neighbour. Replacing the geometry needs
//! neither, and keeps every name.
//!
//! **The tilt.** A face with outward normal `n` turns about the line where
//! its plane meets the neutral plane, by the draft angle, so that its normal
//! leans towards the pull direction `p`, the neutral plane's normal: past
//! the neutral plane, along `p`, the solid gets narrower, and it comes out
//! of a mould pulled that way. A negative angle leans the faces the other
//! way.
//!
//! What it does not do (yet): draft faces that are not planar, faces
//! parallel to the neutral plane (they have no line to turn about), or
//! faces that meet a neighbour tangentially (a fillet along an edge). An
//! angle so large that an edge turns around, or that a wall's two sides
//! cross, is refused.

use geop_core_geometry::shape::{Axis, Plane};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector3, Vector4},
    with_context,
};
use geop_core_topology::{FaceId, SolidId};
use geop_lib_shell::{Smooth, replace_faces};
use geop_ops_model::{NamedModel, Namer};

/// Tilts the planar faces `faces` of one solid by `angle` (in radians, a
/// quarter turn either way at most) about where each meets the plane
/// `neutral`, whose normal is the pull direction — see the module docs —
/// and returns the drafted solid, which replaces it and is named
/// `namer`'s root. Every face, edge and vertex keeps its name.
pub fn draft(
    part: &mut NamedModel,
    namer: &Namer,
    faces: &[FaceId],
    neutral: &Plane,
    angle: Scalar,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "draft({}, faces={faces:?}, neutral={neutral:?}, angle={angle:?})",
        namer.root()
    );
    let quarter = Scalar::from_f64(std::f64::consts::FRAC_PI_2);
    if !(angle.abs().definitely_greater(Scalar::ZERO) && angle.abs().definitely_less(quarter)) {
        return Err(GeopError::new(format!(
            "a draft angle is more than zero and less than a quarter turn, not {:?} degrees",
            angle.to_f64().to_degrees()
        )))
        .with_context(ctx);
    }
    let tilt = Tilt {
        neutral,
        cos: angle.cos(),
        sin: angle.sin(),
    };
    replace_faces(part, namer, faces, Smooth::Refuse, |name, surface| {
        let plane = surface.as_plane()?.ok_or_else(|| {
            GeopError::new(format!(
                "face {name} is not planar: only planar faces can be drafted"
            ))
        })?;
        let hinge = tilt.hinge(&plane).map_err(|e| {
            e.with_context(format!(
                "face {name} is parallel to the neutral plane: it has no line to be drafted about"
            ))
        })?;
        let mut turned = surface.clone();
        for cp in &mut turned.control_points {
            *cp = tilt.turn(&hinge, cp);
        }
        turned.recompute_aabb();
        Ok(turned)
    })
    .with_context(ctx)
}

/// How a drafted face turns: about where its plane meets `neutral`, by the
/// angle whose cosine and sine these are.
struct Tilt<'a> {
    neutral: &'a Plane,
    cos: Scalar,
    sin: Scalar,
}

impl Tilt<'_> {
    /// The line the face in `plane` turns about, along `n x p` — turning
    /// about it by a positive angle leans `n` towards `p`.
    fn hinge(&self, plane: &Plane) -> GeopResult<Axis> {
        plane.intersect_plane(self.neutral)
    }

    /// `p` turned about `hinge`, homogeneous: `(w x, w)` to `(w R(x - c) +
    /// w c, w)`, which needs no division by the weight.
    fn turn(&self, hinge: &Axis, p: &Vector4) -> Vector4 {
        let w = p[3];
        let c = hinge.point.prod_scalar(w);
        let v = Vector3::from_array([p[0], p[1], p[2]]).sub(&c);
        let turned = v
            .rotated_about(&hinge.direction, self.cos, self.sin)
            .add(&c);
        Vector4::from_array([turned[0], turned[1], turned[2], w])
    }
}
