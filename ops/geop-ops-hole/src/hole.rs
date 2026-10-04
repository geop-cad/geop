//! A hole as a solid to cut: its half section, revolved a full turn around
//! the hole's axis ([`hole_tool`]).
//!
//! The section is drawn in `(r, z)`, `r` away from the axis and `z` along
//! it, out of the face the hole is drilled into — so the face is `z = 0`
//! and the hole runs down to negative `z`. Its curves are named after what
//! they sweep, which names the faces the hole leaves (with `q` the quarter
//! turn, `q0` to `q3`):
//!
//! - `top`: the disc on the face, which the cut takes away;
//! - `counterbore` and `shoulder`: a counterbore's wall and its floor;
//! - `countersink`: a countersink's cone;
//! - `wall`: the drilled hole's wall;
//! - `bottom`: a flat bottom, or `point`: the cone a drill's point leaves.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
};
use geop_core_topology::SolidId;
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::{common::Profile, common::line2, revolve::revolve_at_oriented};

use crate::iso::MetricSize;

/// The included angle of a twist drill's point, in degrees.
pub const DRILL_POINT_ANGLE: f64 = 118.0;

/// What a hole sinks for a screw's head.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Head {
    None,
    /// A cylindrical recess `diameter` wide and `depth` deep.
    Counterbore {
        diameter: f64,
        depth: f64,
    },
    /// A 90° cone, `diameter` wide on the face.
    Countersink {
        diameter: f64,
    },
}

/// A hole's shape across: the drill, the head it sinks, the thread it is
/// tapped with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoleShape {
    pub diameter: f64,
    pub head: Head,
    pub thread: Option<&'static MetricSize>,
}

/// How deep a hole goes and how it ends there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoleDepth {
    /// From the face to where the drill's full diameter ends.
    pub depth: f64,
    /// Ends in a drill's point rather than flat.
    pub point: bool,
}

impl HoleShape {
    /// Checks that the shape can be drilled `depth` deep: every diameter
    /// positive, a head wider than the drill and shallower than the hole.
    pub fn check(&self, depth: f64) -> GeopResult<()> {
        let positive = |what: &str, v: f64| {
            if v.is_finite() && v > 0.0 {
                Ok(())
            } else {
                Err(GeopError::new(format!(
                    "the {what} {v} must be more than 0"
                )))
            }
        };
        positive("diameter", self.diameter)?;
        positive("depth", depth)?;
        let head_depth = match self.head {
            Head::None => 0.0,
            Head::Counterbore { diameter, depth } => {
                positive("counterbore diameter", diameter)?;
                positive("counterbore depth", depth)?;
                if diameter <= self.diameter {
                    return Err(GeopError::new(format!(
                        "the counterbore (Ø{diameter}) must be wider than the hole (Ø{})",
                        self.diameter
                    )));
                }
                depth
            }
            Head::Countersink { diameter } => {
                positive("countersink diameter", diameter)?;
                if diameter <= self.diameter {
                    return Err(GeopError::new(format!(
                        "the countersink (Ø{diameter}) must be wider than the hole (Ø{})",
                        self.diameter
                    )));
                }
                (diameter - self.diameter) / 2.0
            }
        };
        if head_depth >= depth {
            return Err(GeopError::new(format!(
                "the hole is {depth} deep, but its head alone takes {head_depth}"
            )));
        }
        Ok(())
    }

    /// The half section, top-down from the axis on the face (see the module
    /// docs), with its curves named and its joints `p0`, `p1`, ...
    fn section<S: Scalar>(&self, depth: HoleDepth) -> GeopResult<Profile<S>> {
        self.check(depth.depth)?;
        let r = self.diameter / 2.0;
        let mut corners = vec![(0.0, 0.0)];
        let mut names = vec!["top"];
        match self.head {
            Head::None => corners.push((r, 0.0)),
            Head::Counterbore { diameter, depth } => {
                corners.extend([(diameter / 2.0, 0.0), (diameter / 2.0, -depth), (r, -depth)]);
                names.extend(["counterbore", "shoulder"]);
            }
            Head::Countersink { diameter } => {
                let rs = diameter / 2.0;
                corners.extend([(rs, 0.0), (r, -(rs - r))]);
                names.push("countersink");
            }
        }
        corners.push((r, -depth.depth));
        names.push("wall");
        if depth.point {
            let tip = r / (DRILL_POINT_ANGLE / 2.0).to_radians().tan();
            corners.push((0.0, -depth.depth - tip));
            names.push("point");
        } else {
            corners.push((0.0, -depth.depth));
            names.push("bottom");
        }
        let p = |&(r, z): &(f64, f64)| Vector2::from_array([S::from_f64(r), S::from_f64(z)]);
        let curves = corners
            .windows(2)
            .map(|w| line2(p(&w[0]), p(&w[1])))
            .collect::<GeopResult<Vec<_>>>()?;
        Ok(Profile {
            curve_names: names.into_iter().map(String::from).collect(),
            joint_names: (0..corners.len()).map(|k| format!("p{k}")).collect(),
            curves,
        })
    }
}

/// The solid a hole of `shape` cuts, `depth` deep, drilled at `axes`'s
/// origin into the face whose outward normal is `axes.w()`: its section
/// revolved a full turn around `w`, into a solid named `solid`, its faces
/// named by `namer` (see the module docs).
pub fn hole_tool<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: &str,
    axes: &CoordinateSystem<S>,
    shape: &HoleShape,
    depth: HoleDepth,
) -> GeopResult<SolidId> {
    let section = shape.section(depth)?;
    revolve_at_oriented(part, namer, solid, &section, axes)
}
