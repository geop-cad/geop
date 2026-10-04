//! [`Placed`]: a body's pose during a solve, as a function of its
//! variables.

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
};

/// A body at its variables — a translation `dt` and a turn `w` about its
/// center `c` — applied to its pose when the solve started: turned by `R(w)`
/// about `c`, then moved by `dt` — the unit dual quaternion
/// `T(c + dt) R(w) T(-c) q0`, computed as the rotation and translation it
/// is: `R(w) r0`, and `c + dt + R(w) (t0 - c)`.
///
/// `R(w)` is the unit quaternion of the modified Rodrigues parameters
/// `σ = w / 4`: `((1 − |σ|²), 2σ) / (1 + |σ|²)`, turning by `4 atan(|w| /
/// 4)` about `w` — `|w|` radians to first order. It is rational, so smooth
/// everywhere and a unit quaternion by construction — no square root of
/// `|w|`, as the exponential needs — and every rotation short of a full turn
/// is one, a half turn at `|w| = 4` with the turn still changing half as
/// fast as `w`. The quaternion `(1, w / 2)` normalized, used before, reaches
/// a half turn only as `|w|` goes to infinity: a body that had to turn
/// nearly half way round in one drag ran its variables off to hundreds,
/// every step of the minimizer turning it by less, and the solve ran out of
/// steps. Turning about the body's own center rather than the world's
/// origin keeps a turn of a body far from the origin from also moving it.
#[derive(Clone, Debug)]
pub struct Placed<T: Scalar> {
    rotation: Quaternion<T>,
    /// The rotation, as a matrix's columns.
    axes: [Vector3<T>; 3],
    translation: Vector3<T>,
    /// The turn variables.
    pub w: Vector3<T>,
}

impl<T: Scalar> Placed<T> {
    /// The body at pose `start`, turning about `center` (in the world).
    pub fn new(
        start: &Pose<T>,
        center: &Vector3<T>,
        dt: Vector3<T>,
        w: Vector3<T>,
    ) -> GeopResult<Self> {
        let sigma = w.prod_scalar(T::ONE.div(T::from_i64(4))?);
        let s2 = sigma.prod_dot(&sigma);
        let inv = T::ONE.div(T::ONE.add(s2))?;
        let twice = T::TWO.mul(inv);
        let turn = Quaternion::new(
            T::ONE.sub(s2).mul(inv),
            sigma[0].mul(twice),
            sigma[1].mul(twice),
            sigma[2].mul(twice),
        );
        let turned = turn.rotation_columns()?;
        let by_turn = |v: &Vector3<T>| {
            turned[0]
                .prod_scalar(v[0])
                .add(&turned[1].prod_scalar(v[1]))
                .add(&turned[2].prod_scalar(v[2]))
        };
        let start_axes = start.rotation().rotation_columns()?;
        Ok(Placed {
            rotation: turn.mul(&start.rotation()),
            axes: start_axes.map(|a| by_turn(&a)),
            translation: center.add(&dt).add(&by_turn(&start.position().sub(center))),
            w,
        })
    }

    /// A body that does not move during the solve.
    pub fn fixed(pose: &Pose<T>) -> GeopResult<Self> {
        Self::new(pose, &Vector3::zero(), Vector3::zero(), Vector3::zero())
    }

    /// The same body in the scalar type `U`, every value mapped by `f` — a
    /// gradient embedded among more variables, say.
    pub fn map<U: Scalar>(&self, f: impl Fn(T) -> U) -> Placed<U> {
        Placed {
            rotation: self.rotation.map(&f),
            axes: self.axes.map(|a| a.map(&f)),
            translation: self.translation.map(&f),
            w: self.w.map(&f),
        }
    }

    /// The direction `local`, of the body, in the world.
    pub fn direction(&self, local: &Vector3<T>) -> Vector3<T> {
        let [x, y, z] = &self.axes;
        x.prod_scalar(local[0])
            .add(&y.prod_scalar(local[1]))
            .add(&z.prod_scalar(local[2]))
    }

    /// The point `local`, of the body, in the world.
    pub fn point(&self, local: &Vector3<T>) -> Vector3<T> {
        self.direction(local).add(&self.translation)
    }

    /// Where its origin is.
    pub fn translation(&self) -> &Vector3<T> {
        &self.translation
    }

    /// Its rotation, as a unit quaternion.
    pub fn rotation(&self) -> &Quaternion<T> {
        &self.rotation
    }

    /// Where its axes point.
    pub fn axes(&self) -> &[Vector3<T>; 3] {
        &self.axes
    }

    /// Its pose.
    pub fn pose(&self) -> GeopResult<Pose<T>> {
        Pose::new(self.translation, self.rotation)
    }
}
