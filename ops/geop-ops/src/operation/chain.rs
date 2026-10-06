//! [`Chain`]: curves in space, each on `[0, 1]` and starting where the one
//! before ends — what a path, a rail or a guide is, however it was picked.

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

/// A chain of 3-D curves, named like a profile: `joint_names[i]` is where
/// `curves[i]` starts. A closed chain, ending where it starts, has one joint
/// per curve, an open one also its end.
#[derive(Clone, Debug)]
pub struct Chain<S: Scalar> {
    /// What the chain goes by: its sketch's name, or its edge's.
    pub name: String,
    pub curves: Vec<NurbCurve3D<S>>,
    pub curve_names: Vec<String>,
    pub joint_names: Vec<String>,
}

/// The point a clamped curve starts at: its first control point.
pub fn start_of<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Vector3<S>> {
    let cp = curve.control_points[0];
    Ok(Vector3::from_array([
        cp[0].div(cp[3])?,
        cp[1].div(cp[3])?,
        cp[2].div(cp[3])?,
    ]))
}

/// The point a clamped curve ends at.
pub fn end_of<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Vector3<S>> {
    start_of(&curve.reverse())
}

/// The names of the pieces of a sketch loop or chain, and of its joints:
/// `sketch,c3` for a piece and `sketch,p2` for a joint, given each piece as
/// its own name and the names of its two joints. An open chain also names
/// its end joint.
pub fn piece_names(
    sketch: &str,
    pieces: &[(String, String, String)],
    closed: bool,
) -> (Vec<String>, Vec<String>) {
    let curve_names = pieces
        .iter()
        .map(|(name, ..)| format!("{sketch},{name}"))
        .collect();
    let mut joint_names: Vec<String> = pieces
        .iter()
        .map(|(_, start, _)| format!("{sketch},{start}"))
        .collect();
    if !closed && let Some((.., end)) = pieces.last() {
        joint_names.push(format!("{sketch},{end}"));
    }
    (curve_names, joint_names)
}

impl<S: Scalar> Chain<S> {
    pub fn is_closed(&self) -> bool {
        self.joint_names.len() == self.curves.len()
    }

    /// The open chain run the other way; every name stays with its curve
    /// or joint.
    pub fn reversed(&self) -> Self {
        Self {
            name: self.name.clone(),
            curves: self.curves.iter().rev().map(|c| c.reverse()).collect(),
            curve_names: self.curve_names.iter().rev().cloned().collect(),
            joint_names: self.joint_names.iter().rev().cloned().collect(),
        }
    }

    /// The closed chain starting at its joint `k`.
    pub fn starting_at(&self, k: usize) -> Self {
        let mut out = self.clone();
        out.curves.rotate_left(k);
        out.curve_names.rotate_left(k);
        out.joint_names.rotate_left(k);
        out
    }

    /// The joints, where they are.
    pub fn joints(&self) -> GeopResult<Vec<Vector3<S>>> {
        let mut joints = self
            .curves
            .iter()
            .map(start_of)
            .collect::<GeopResult<Vec<_>>>()?;
        if !self.is_closed() {
            joints.push(end_of(self.curves.last().expect("a chain has curves"))?);
        }
        Ok(joints)
    }
}
