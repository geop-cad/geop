//! [`Chain`]: curves each on `[0, 1]` and starting where the one before
//! ends, with a stable name for every curve and every joint — what a
//! profile, a path, a rail or a guide is, however it was drawn or picked.

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

/// A chain of curves, with a stable name for every curve and every joint
/// between them — what extrude, revolve and the sweeps build the names of
/// what they make from (see the crate docs). For a sketch these are its
/// element ids; for a shape built in code, positions in the chain
/// ([`Chain::closed`]).
///
/// `joint_names[i]` names the joint where `curves[i]` starts. A closed loop
/// has one joint per curve; an open chain one more, its end.
#[derive(Clone, Debug)]
pub struct Chain<C> {
    /// What the chain goes by: its sketch's name, or its edge's; empty for
    /// one built in code.
    pub name: String,
    pub curves: Vec<C>,
    pub curve_names: Vec<String>,
    pub joint_names: Vec<String>,
}

/// A chain in a plane: the profile extrude and revolve sweep.
pub type Profile<S> = Chain<NurbCurve2D<S>>;

/// A curve a [`Chain`] is made of: a clamped NURBS curve, in a plane or in
/// space, which starts at its first control point and ends at its last
/// (see [`NurbCurve::start`]).
pub trait ChainCurve: Clone {
    type Point;

    /// The point the curve starts at.
    fn start(&self) -> GeopResult<Self::Point>;

    /// The point the curve ends at.
    fn end(&self) -> GeopResult<Self::Point>;
}

impl<S: Scalar> ChainCurve for NurbCurve2D<S> {
    type Point = Vector2<S>;

    fn start(&self) -> GeopResult<Vector2<S>> {
        NurbCurve2D::start(self)
    }

    fn end(&self) -> GeopResult<Vector2<S>> {
        NurbCurve2D::end(self)
    }
}

impl<S: Scalar> ChainCurve for NurbCurve3D<S> {
    type Point = Vector3<S>;

    fn start(&self) -> GeopResult<Vector3<S>> {
        NurbCurve3D::start(self)
    }

    fn end(&self) -> GeopResult<Vector3<S>> {
        NurbCurve3D::end(self)
    }
}

impl<S: Scalar, const D: usize> Chain<NurbCurve<S, D>>
where
    NurbCurve<S, D>: ChainCurve,
{
    /// A closed loop whose curves are called `c0, c1, ...` and whose joints
    /// `p0, p1, ...`, `p0` where `c0` starts.
    pub fn closed(curves: Vec<NurbCurve<S, D>>) -> Self {
        let n = curves.len();
        Self::numbered(curves, n)
    }

    /// An open chain, named like [`Chain::closed`] plus its end joint.
    pub fn open(curves: Vec<NurbCurve<S, D>>) -> Self {
        let n = curves.len();
        Self::numbered(curves, n + 1)
    }

    fn numbered(curves: Vec<NurbCurve<S, D>>, joints: usize) -> Self {
        Self {
            name: String::new(),
            curve_names: (0..curves.len()).map(|i| format!("c{i}")).collect(),
            joint_names: (0..joints).map(|i| format!("p{i}")).collect(),
            curves,
        }
    }

    /// The same chain with `prefix` put in front of every name — to keep
    /// several [`Chain::closed`] loops of one extrude apart.
    pub fn with_prefix(mut self, prefix: &str) -> Self {
        for name in self.curve_names.iter_mut().chain(&mut self.joint_names) {
            *name = format!("{prefix}{name}");
        }
        self
    }

    pub fn is_closed(&self) -> bool {
        self.joint_names.len() == self.curves.len()
    }

    /// Checks that there is one name per curve and one per joint.
    pub fn check_names(&self) -> GeopResult<()> {
        let n = self.curves.len();
        if self.curve_names.len() != n
            || !(self.joint_names.len() == n || self.joint_names.len() == n + 1)
        {
            return Err(GeopError::new(format!(
                "chain of {n} curves has {} curve names and {} joint names",
                self.curve_names.len(),
                self.joint_names.len()
            )));
        }
        Ok(())
    }

    /// The same chain traversed the other way; every name stays with its
    /// curve or joint.
    pub fn reversed(&self) -> Self {
        let n = self.curves.len();
        let joint_names = if self.is_closed() {
            // Reversed curve `m` is old curve `n - 1 - m`, which now starts
            // where it used to end: at old joint `n - m`.
            (0..n)
                .map(|m| self.joint_names[(n - m) % n].clone())
                .collect()
        } else {
            self.joint_names.iter().rev().cloned().collect()
        };
        Self {
            name: self.name.clone(),
            curves: self.curves.iter().rev().map(|c| c.reverse()).collect(),
            curve_names: self.curve_names.iter().rev().cloned().collect(),
            joint_names,
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

    /// The same chain with `f` applied to every curve, keeping the names.
    pub fn map_curves(&self, f: impl Fn(&NurbCurve<S, D>) -> NurbCurve<S, D>) -> Self {
        Self {
            curves: self.curves.iter().map(f).collect(),
            ..self.clone()
        }
    }

    /// Joint `i`, where it is.
    pub fn joint(&self, i: usize) -> GeopResult<<NurbCurve<S, D> as ChainCurve>::Point> {
        if i < self.curves.len() {
            self.curves[i].start()
        } else {
            self.curves[i - 1].end()
        }
    }

    /// The joints, where they are.
    pub fn joints(&self) -> GeopResult<Vec<<NurbCurve<S, D> as ChainCurve>::Point>> {
        (0..self.joint_names.len()).map(|i| self.joint(i)).collect()
    }
}

impl<S: Scalar> Chain<NurbCurve3D<S>> {
    /// The open chain run from its end on `plane`: as it is if it starts
    /// there, the other way round if only its end could lie on it. `None`
    /// if neither end could.
    pub fn starting_on(&self, plane: &CoordinateSystem<S>) -> GeopResult<Option<Self>> {
        let on_plane = |p: &Vector3<S>| plane.to_uvw(p)[2].could_be_equal(S::ZERO);
        let joints = self.joints()?;
        Ok(if on_plane(&joints[0]) {
            Some(self.clone())
        } else if on_plane(&joints[joints.len() - 1]) {
            Some(self.reversed())
        } else {
            None
        })
    }
}
