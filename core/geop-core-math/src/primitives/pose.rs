//! [`Pose`]: where a rigid body sits, as a unit dual quaternion — built on
//! [`DualQuaternion`] and [`Quaternion`], in any scalar type.
//!
//! A unit dual quaternion `q = r + ε d` holds a rotation `r` (a unit
//! quaternion) and a translation `t` as `d = ½ t r`. It moves a point `p` by
//! `q (1 + ε p) q̄*`, composes by multiplication, and is undone by its
//! conjugate. Unlike angles, it has no singular configurations, and every
//! component is a polynomial of the rotation and translation — which is
//! what makes gradients through it well behaved, and its enclosures tight.

use serde::{Deserialize, Serialize};

use super::CoordinateSystem;
use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

/// A quaternion `w + x i + y j + z k`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion<T> {
    pub w: T,
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Scalar> Quaternion<T> {
    pub fn new(w: T, x: T, y: T, z: T) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self::new(T::ONE, T::ZERO, T::ZERO, T::ZERO)
    }

    /// Whether it is exactly the identity: every component sharp, `1` and
    /// three `0`s — no rotation at all, rather than one too small to tell.
    pub fn is_identity(&self) -> bool {
        let exactly = |c: T, value: f64| c.is_sharp() && c.to_f64() == value;
        exactly(self.w, 1.0) && exactly(self.x, 0.0) && exactly(self.y, 0.0) && exactly(self.z, 0.0)
    }

    /// `0 + v`: a vector as a pure quaternion.
    pub fn pure(v: &Vector3<T>) -> Self {
        Self::new(T::ZERO, v[0], v[1], v[2])
    }

    /// Its vector part.
    pub fn vector(&self) -> Vector3<T> {
        Vector3::from_array([self.x, self.y, self.z])
    }

    /// Its components `[w, x, y, z]`.
    pub fn components(&self) -> [T; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Every component mapped by `f`.
    pub fn map<U: Scalar>(&self, f: impl Fn(T) -> U) -> Quaternion<U> {
        Quaternion::new(f(self.w), f(self.x), f(self.y), f(self.z))
    }

    pub fn mul(&self, o: &Self) -> Self {
        Self {
            w: self
                .w
                .mul(o.w)
                .sub(self.x.mul(o.x))
                .sub(self.y.mul(o.y))
                .sub(self.z.mul(o.z)),
            x: self
                .w
                .mul(o.x)
                .add(self.x.mul(o.w))
                .add(self.y.mul(o.z))
                .sub(self.z.mul(o.y)),
            y: self
                .w
                .mul(o.y)
                .sub(self.x.mul(o.z))
                .add(self.y.mul(o.w))
                .add(self.z.mul(o.x)),
            z: self
                .w
                .mul(o.z)
                .add(self.x.mul(o.y))
                .sub(self.y.mul(o.x))
                .add(self.z.mul(o.w)),
        }
    }

    pub fn conj(&self) -> Self {
        Self::new(self.w, self.x.neg(), self.y.neg(), self.z.neg())
    }

    pub fn scale(&self, s: T) -> Self {
        self.map(|c| c.mul(s))
    }

    pub fn add(&self, o: &Self) -> Self {
        Self::new(
            self.w.add(o.w),
            self.x.add(o.x),
            self.y.add(o.y),
            self.z.add(o.z),
        )
    }

    pub fn dot(&self, o: &Self) -> T {
        self.w
            .mul(o.w)
            .add(self.x.mul(o.x))
            .add(self.y.mul(o.y))
            .add(self.z.mul(o.z))
    }

    pub fn norm_sq(&self) -> T {
        self.dot(self)
    }

    /// The unit quaternion of the same rotation; fails for one that could be
    /// zero, which stands for no rotation at all.
    pub fn normalized(&self) -> GeopResult<Self> {
        let n = self.norm_sq();
        if !n.definitely_greater(T::ZERO) {
            return Err(GeopError::new("a rotation cannot be the zero quaternion"));
        }
        Ok(self.scale(T::ONE.div(n.sqrt()?)?))
    }

    /// Turning by `degrees` about `x`, then `y`, then `z` — the world's axes.
    pub fn from_euler(degrees: [T; 3]) -> GeopResult<Self> {
        let half_radians = T::PI.div(T::from_i64(360))?;
        let [a, b, c] = degrees.map(|d| d.mul(half_radians));
        let qx = Self::new(a.cos(), a.sin(), T::ZERO, T::ZERO);
        let qy = Self::new(b.cos(), T::ZERO, b.sin(), T::ZERO);
        let qz = Self::new(c.cos(), T::ZERO, T::ZERO, c.sin());
        Ok(qz.mul(&qy.mul(&qx)))
    }

    /// The rotation it stands for, as a matrix's columns — where `x`, `y`
    /// and `z` turn to. It need not be unit: dividing by its norm squared
    /// makes the rotation of any non-zero quaternion, smoothly.
    pub fn rotation_columns(&self) -> GeopResult<[Vector3<T>; 3]> {
        let n = T::ONE.div(self.norm_sq())?;
        let two = T::TWO.mul(n);
        let Self { w, x, y, z } = *self;
        let (xx, yy, zz) = (x.mul(x), y.mul(y), z.mul(z));
        let (xy, xz, yz) = (x.mul(y), x.mul(z), y.mul(z));
        let (wx, wy, wz) = (w.mul(x), w.mul(y), w.mul(z));
        let one = T::ONE;
        let v = |a: T, b: T, c: T| Vector3::from_array([a, b, c]);
        Ok([
            v(
                one.sub(two.mul(yy.add(zz))),
                two.mul(xy.add(wz)),
                two.mul(xz.sub(wy)),
            ),
            v(
                two.mul(xy.sub(wz)),
                one.sub(two.mul(xx.add(zz))),
                two.mul(yz.add(wx)),
            ),
            v(
                two.mul(xz.add(wy)),
                two.mul(yz.sub(wx)),
                one.sub(two.mul(xx.add(yy))),
            ),
        ])
    }
}

/// A dual quaternion `real + ε dual` — a rigid motion when unit (see the
/// module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DualQuaternion<T> {
    pub real: Quaternion<T>,
    pub dual: Quaternion<T>,
}

impl<T: Scalar> DualQuaternion<T> {
    /// Turning by the unit quaternion `rotation`, then moving by
    /// `translation`.
    pub fn from_parts(rotation: Quaternion<T>, translation: &Vector3<T>) -> Self {
        let half = T::ONE.div(T::TWO).expect("2 is not zero");
        Self {
            real: rotation,
            dual: Quaternion::pure(translation).mul(&rotation).scale(half),
        }
    }

    pub fn translation_by(t: &Vector3<T>) -> Self {
        Self::from_parts(Quaternion::identity(), t)
    }

    /// Every component mapped by `f`.
    pub fn map<U: Scalar>(&self, f: impl Fn(T) -> U) -> DualQuaternion<U> {
        DualQuaternion {
            real: self.real.map(&f),
            dual: self.dual.map(&f),
        }
    }

    /// This motion after `inner`: a point is moved by `inner` first.
    pub fn mul(&self, inner: &Self) -> Self {
        Self {
            real: self.real.mul(&inner.real),
            dual: self.real.mul(&inner.dual).add(&self.dual.mul(&inner.real)),
        }
    }

    /// The motion undoing this one.
    pub fn conj(&self) -> Self {
        Self {
            real: self.real.conj(),
            dual: self.dual.conj(),
        }
    }

    /// Its translation: `2 d r*`.
    pub fn translation(&self) -> Vector3<T> {
        self.dual.mul(&self.real.conj()).scale(T::TWO).vector()
    }
}

/// A rigid motion — where a body sits: a point `p` of it is at
/// `R p + position`, with `R` the rotation of a unit quaternion — in the
/// scalar type `S`. Computed with as the unit dual quaternion of the two
/// ([`Pose::dual_quaternion`]); kept as the two themselves, so that what is
/// stored is exactly what was set, and a pose read and written again is
/// written exactly as it was read.
///
/// Serialized as `{"position": [x, y, z], "rotation": [w, x, y, z]}`, each
/// number its value's midpoint; a rotation read is normalized. Angles are
/// for showing a pose to people only ([`Pose::euler_degrees`],
/// [`Pose::from_euler`]).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(into = "PoseFile", try_from = "PoseFile", bound = "S: Scalar")]
pub struct Pose<S: Scalar> {
    rotation: Quaternion<S>,
    position: Vector3<S>,
}

/// A [`Pose`] as a file has it.
#[derive(Serialize, Deserialize)]
struct PoseFile {
    position: [f64; 3],
    rotation: [f64; 4],
}

impl<S: Scalar> From<Pose<S>> for PoseFile {
    fn from(pose: Pose<S>) -> Self {
        let p = pose.position();
        PoseFile {
            position: [0, 1, 2].map(|k| p[k].to_f64()),
            rotation: pose.rotation().components().map(|c| c.to_f64()),
        }
    }
}

impl<S: Scalar> TryFrom<PoseFile> for Pose<S> {
    type Error = GeopError;

    fn try_from(file: PoseFile) -> GeopResult<Self> {
        let [w, x, y, z] = file.rotation.map(S::from_f64);
        Pose::new(
            Vector3::from_array(file.position.map(S::from_f64)),
            Quaternion::new(w, x, y, z),
        )
    }
}

impl<S: Scalar> Default for Pose<S> {
    fn default() -> Self {
        Pose::identity()
    }
}

impl<S: Scalar> Pose<S> {
    pub fn identity() -> Self {
        Pose {
            rotation: Quaternion::identity(),
            position: Vector3::zero(),
        }
    }

    /// Turning by `rotation` — any quaternion definitely not zero,
    /// normalized here — then moving to `position`.
    pub fn new(position: Vector3<S>, rotation: Quaternion<S>) -> GeopResult<Self> {
        Ok(Pose {
            rotation: rotation.normalized()?,
            position,
        })
    }

    /// The pose a unit dual quaternion stands for.
    pub fn from_dual_quaternion(motion: &DualQuaternion<S>) -> Self {
        Pose {
            rotation: motion.real,
            position: motion.translation(),
        }
    }

    /// Moving to `position`, turned by `degrees` about `x`, then `y`, then
    /// `z` — the world's axes — as a dialog shows a pose.
    pub fn from_euler(position: Vector3<S>, degrees: [S; 3]) -> GeopResult<Self> {
        Pose::new(position, Quaternion::from_euler(degrees)?)
    }

    /// Where its body's origin is.
    pub fn position(&self) -> Vector3<S> {
        self.position
    }

    /// Its rotation, as a unit quaternion.
    pub fn rotation(&self) -> Quaternion<S> {
        self.rotation
    }

    /// The pose as a unit dual quaternion.
    pub fn dual_quaternion(&self) -> DualQuaternion<S> {
        DualQuaternion::from_parts(self.rotation, &self.position)
    }

    /// Every component mapped by `f` — into another scalar type, say.
    pub fn map<T: Scalar>(&self, f: impl Fn(S) -> T) -> Pose<T> {
        Pose {
            rotation: self.rotation.map(&f),
            position: self.position.map(&f),
        }
    }

    /// The same pose in the scalar type `T` (see [`Scalar::cast`]).
    pub fn cast<T: Scalar>(&self) -> Pose<T> {
        self.map(|c| c.cast())
    }

    /// Its rotation as angles in degrees about `x`, then `y`, then `z` (see
    /// [`Pose::from_euler`]) — for showing it to people, and so read off
    /// its midpoint. At gimbal lock (the second at ±90°) only the sum or
    /// difference of the other two is determined; the split is arbitrary
    /// but exact.
    pub fn euler_degrees(&self) -> [f64; 3] {
        let [w, x, y, z] = self.rotation().components().map(|c| c.to_f64());
        // The rotation matrix's first column and last row.
        let (r00, r10, r20) = (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        );
        let r21 = 2.0 * (y * z + w * x);
        let (r01, r11) = (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z));
        // `c` from the first column; whatever it comes out as — even from
        // rounding noise at gimbal lock — `b` and `a` are solved for the
        // rest of the matrix with it, so the triple reproduces the rotation.
        let c = r10.atan2(r00);
        let (sc, cc) = c.sin_cos();
        let b = (-r20).atan2(r00 * cc + r10 * sc);
        // Rx(a) = Ry(b)^T Rz(c)^T R: its (2,1) and (1,1) entries.
        let (sb, cb) = b.sin_cos();
        let m11 = -sc * r01 + cc * r11;
        let m21 = sb * (cc * r01 + sc * r11) + cb * r21;
        let a = m21.atan2(m11);
        [a, b, c].map(f64::to_degrees)
    }

    /// The same rotation, moved to `position`.
    pub fn with_position(&self, position: Vector3<S>) -> Pose<S> {
        Pose {
            rotation: self.rotation,
            position,
        }
    }

    /// This pose after `inner`: a point is moved by `inner` first.
    pub fn compose(&self, inner: &Pose<S>) -> Pose<S> {
        Pose {
            rotation: self.rotation.mul(&inner.rotation),
            position: self.apply(&inner.position),
        }
    }

    /// The pose that undoes this one.
    pub fn inverse(&self) -> Pose<S> {
        let rotation = self.rotation.conj();
        let turned = rotation
            .mul(&Quaternion::pure(&self.position))
            .mul(&self.rotation)
            .vector();
        Pose {
            rotation,
            position: turned.neg(),
        }
    }

    /// The point `p` moved by the pose.
    pub fn apply(&self, p: &Vector3<S>) -> Vector3<S> {
        let r = &self.rotation;
        r.mul(&Quaternion::pure(p))
            .mul(&r.conj())
            .vector()
            .add(&self.position)
    }

    /// Turning by `radians` about the line through `point` along the unit
    /// vector `direction` — counter-clockwise, looking against `direction`
    /// — as a pose: the rotation `(cos ½θ, sin ½θ d)`, and the position
    /// that keeps `point` where it is.
    pub fn rotation_about(
        point: &Vector3<S>,
        direction: &Vector3<S>,
        radians: S,
    ) -> GeopResult<Self> {
        let half = radians.div(S::TWO)?;
        let (c, s) = (half.cos(), half.sin());
        let turn = Pose::new(
            Vector3::zero(),
            Quaternion::new(
                c,
                s.mul(direction[0]),
                s.mul(direction[1]),
                s.mul(direction[2]),
            ),
        )?;
        Ok(turn.with_position(point.sub(&turn.apply(point))))
    }

    /// What the pose does to geometry: its rotation's columns and its
    /// position, enclosed once, so moving many points costs a matrix
    /// product each. A pose that does not turn at all — its rotation
    /// exactly the identity — moves geometry by additions alone.
    pub fn motion(&self) -> Motion<S> {
        let turns = !self.rotation.is_identity();
        Motion {
            axes: turns.then(|| {
                self.rotation
                    .rotation_columns()
                    .expect("a pose's rotation is a unit quaternion")
            }),
            position: self.position,
            mirrors: false,
        }
    }
}

/// An isometry as it moves geometry: a [`Pose`]'s rigid motion (see
/// [`Pose::motion`]), a plain translation, or a mirror in a plane — what a
/// body is moved, patterned or mirrored by.
///
/// A point `p` goes to `A p + position`, with `A` the matrix whose columns
/// are `axes`: a rotation's, or a mirror's reflection. Without axes `A` is
/// the identity and nothing is multiplied: a translation moves every point
/// by one addition per coordinate, as exactly as an interval addition can.
#[derive(Clone, Copy, Debug)]
pub struct Motion<S: Scalar> {
    axes: Option<[Vector3<S>; 3]>,
    position: Vector3<S>,
    mirrors: bool,
}

impl<S: Scalar> Motion<S> {
    /// Moving every point by `offset`.
    pub fn translation(offset: Vector3<S>) -> Self {
        Motion {
            axes: None,
            position: offset,
            mirrors: false,
        }
    }

    /// Mirroring in the plane through `point` with the normal `normal`,
    /// which need not be unit: with `n` the unit normal, `p` goes to
    /// `p - 2 ((p - point) . n) n` — `A = I - 2 n nᵀ`, and the position
    /// `2 (point . n) n`.
    pub fn mirror(point: &Vector3<S>, normal: &Vector3<S>) -> GeopResult<Self> {
        let n = normal.normalize()?;
        let column = |k: usize| {
            let mut c = n.prod_scalar(S::TWO.mul(n[k]).neg());
            c[k] = S::ONE.add(c[k]);
            c
        };
        Ok(Motion {
            axes: Some([column(0), column(1), column(2)]),
            position: n.prod_scalar(S::TWO.mul(point.prod_dot(&n))),
            mirrors: true,
        })
    }

    /// Whether it mirrors: turns a right-handed frame into a left-handed
    /// one, and with it the side of a surface its normal points to.
    pub fn mirrors(&self) -> bool {
        self.mirrors
    }

    /// Whether it turns or mirrors directions at all, rather than only
    /// moving points.
    pub fn turns(&self) -> bool {
        self.axes.is_some()
    }

    /// Where it moves the origin to: what it adds to every point it has
    /// turned.
    pub fn position(&self) -> Vector3<S> {
        self.position
    }

    /// The direction `d` turned — or mirrored; a translation leaves it as
    /// it is.
    pub fn rotate(&self, d: &Vector3<S>) -> Vector3<S> {
        match &self.axes {
            None => *d,
            Some([x, y, z]) => x
                .prod_scalar(d[0])
                .add(&y.prod_scalar(d[1]))
                .add(&z.prod_scalar(d[2])),
        }
    }

    /// The point `p` moved.
    pub fn apply(&self, p: &Vector3<S>) -> Vector3<S> {
        self.rotate(p).add(&self.position)
    }

    /// The frame `frame` moved. Fails for a mirror, which would leave it
    /// left-handed.
    pub fn apply_frame(&self, frame: &CoordinateSystem<S>) -> GeopResult<CoordinateSystem<S>> {
        if self.mirrors {
            return Err(GeopError::new(
                "Motion::apply_frame: a mirrored frame is left-handed",
            ));
        }
        CoordinateSystem::try_new(
            self.apply(frame.origin()),
            self.rotate(frame.u()),
            self.rotate(frame.v()),
            self.rotate(frame.w()),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalars::scal_in_f64::ScalInF64;

    type S = ScalInF64;

    fn v(p: [f64; 3]) -> Vector3<S> {
        Vector3::from_array(p.map(S::from_f64))
    }

    fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<S> {
        Pose::from_euler(v(position), degrees.map(S::from_f64)).unwrap()
    }

    /// `a` could be `b`, as an enclosure of it.
    fn encloses(a: &Vector3<S>, b: [f64; 3]) -> bool {
        (0..3).all(|k| a[k].could_be_equal(S::from_f64(b[k])))
    }

    fn close(a: &Vector3<S>, b: &Vector3<S>) -> bool {
        (0..3).all(|k| (a[k].to_f64() - b[k].to_f64()).abs() < 1e-12)
    }

    /// The angles turn about `x`, then `y`, then `z`: a quarter turn about
    /// `z` takes `x` to `y` — as an enclosure of the exact result.
    #[test]
    fn angles_turn_about_fixed_axes() {
        let turned = pose([1.0, 2.0, 3.0], [0.0, 0.0, 90.0]);
        assert!(encloses(
            &turned.apply(&v([1.0, 0.0, 0.0])),
            [1.0, 3.0, 3.0]
        ));
        assert!(encloses(
            &turned.motion().apply(&v([1.0, 0.0, 0.0])),
            [1.0, 3.0, 3.0]
        ));
        // About x first, then z: x stays x, then turns to y.
        let both = pose([0.0; 3], [90.0, 0.0, 90.0]);
        assert!(encloses(&both.apply(&v([1.0, 0.0, 0.0])), [0.0, 1.0, 0.0]));
        assert!(encloses(&both.apply(&v([0.0, 1.0, 0.0])), [0.0, 0.0, 1.0]));
    }

    /// Angles read back give the same motion, also at gimbal lock.
    #[test]
    fn angles_round_trip() {
        for degrees in [
            [10.0, 20.0, 30.0],
            [-170.0, 45.0, 100.0],
            [30.0, 90.0, 20.0],
        ] {
            let original = pose([0.5, -1.0, 2.0], degrees);
            let back = Pose::from_euler(
                original.position(),
                original.euler_degrees().map(S::from_f64),
            )
            .unwrap();
            for p in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.3, -0.2, 0.9]] {
                assert!(
                    close(&back.apply(&v(p)), &original.apply(&v(p))),
                    "{degrees:?}"
                );
            }
        }
    }

    #[test]
    fn inverse_and_compose_undo_each_other() {
        let a = pose([0.5, -1.0, 2.0], [10.0, -20.0, 70.0]);
        let p = [0.3, 0.4, 0.5];
        assert!(encloses(&a.compose(&a.inverse()).apply(&v(p)), p));
        let b = pose([1.0, 0.0, -1.0], [45.0, 30.0, 0.0]);
        let q = v([0.3, -0.7, 0.2]);
        let composed = a.compose(&b).apply(&q);
        let each = a.apply(&b.apply(&q));
        assert!((0..3).all(|k| composed[k].could_be_equal(each[k])));
    }

    /// Files hold a position and a quaternion; a rotation read is
    /// normalized, and the zero quaternion is refused.
    #[test]
    fn poses_read_and_write_position_and_quaternion() {
        let written = Pose::new(
            v([1.0, 2.0, 3.0]),
            Quaternion::new(S::ZERO, S::ZERO, S::ZERO, S::TWO),
        )
        .unwrap();
        let json = serde_json::to_value(written).unwrap();
        assert_eq!(
            json,
            serde_json::json!({"position": [1.0, 2.0, 3.0], "rotation": [0.0, 0.0, 0.0, 1.0]})
        );
        let read: Pose<S> = serde_json::from_value(json).unwrap();
        assert!(encloses(&read.apply(&v([1.0, 0.0, 0.0])), [0.0, 2.0, 3.0]));
        let zero =
            serde_json::json!({"position": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 0.0]});
        assert!(serde_json::from_value::<Pose<S>>(zero).is_err());
    }
}
