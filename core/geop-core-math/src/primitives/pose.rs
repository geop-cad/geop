//! [`Pose`]: where a rigid body sits, as a rotation — a unit
//! [`Quaternion`] — and a position, in any [`Number`].
//!
//! Unlike angles, a unit quaternion has no singular configurations, and
//! moving a point by it is a polynomial of its components — which is what
//! makes gradients through it well behaved, and its enclosures tight.

use serde::{Deserialize, Serialize};

use super::CoordinateSystem;
use crate::{
    dual::Dual,
    geop_error::{GeopError, GeopResult},
    scalars::{Number, Scalar},
    solvers::{least_squares::Rows, system::Variable},
    vector::{Vector, Vector3},
};

/// How many terms of each series [`Pose::exp`] sums near zero before it
/// bounds the rest: how tight the map is there, never whether it encloses.
const SERIES_TERMS: usize = 12;

/// A quaternion `w + x i + y j + z k`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion<T = Scalar> {
    pub w: T,
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Number> Quaternion<T> {
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
    pub fn map<U: Number>(&self, f: impl Fn(T) -> U) -> Quaternion<U> {
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

    /// The rotation whose matrix has the columns `columns` — where `x`, `y`
    /// and `z` turn to, orthonormal and right-handed: the inverse of
    /// [`Quaternion::rotation_columns`], up to sign. Of the four ways to
    /// read it off the matrix (Shepperd's), the one dividing by the largest
    /// of the trace and the diagonal: a free choice, and the
    /// well-conditioned one.
    pub fn from_rotation_columns(columns: [Vector3<T>; 3]) -> GeopResult<Self> {
        let m = |r: usize, c: usize| columns[c][r];
        let one = T::ONE;
        let trace = m(0, 0).add(m(1, 1)).add(m(2, 2));
        let size = |v: T| v.to_f64();
        let candidates = [trace, m(0, 0), m(1, 1), m(2, 2)];
        let largest = (1..4).fold(0, |best, k| {
            if size(candidates[k]) > size(candidates[best]) {
                k
            } else {
                best
            }
        });
        let quarter = T::ONE.div(T::from_i64(4))?;
        let q = match largest {
            0 => {
                let s = one.add(trace).sqrt()?.mul(T::TWO);
                let inv = one.div(s)?;
                Self::new(
                    s.mul(quarter),
                    m(2, 1).sub(m(1, 2)).mul(inv),
                    m(0, 2).sub(m(2, 0)).mul(inv),
                    m(1, 0).sub(m(0, 1)).mul(inv),
                )
            }
            1 => {
                let s = one
                    .add(m(0, 0))
                    .sub(m(1, 1))
                    .sub(m(2, 2))
                    .sqrt()?
                    .mul(T::TWO);
                let inv = one.div(s)?;
                Self::new(
                    m(2, 1).sub(m(1, 2)).mul(inv),
                    s.mul(quarter),
                    m(0, 1).add(m(1, 0)).mul(inv),
                    m(0, 2).add(m(2, 0)).mul(inv),
                )
            }
            2 => {
                let s = one
                    .add(m(1, 1))
                    .sub(m(0, 0))
                    .sub(m(2, 2))
                    .sqrt()?
                    .mul(T::TWO);
                let inv = one.div(s)?;
                Self::new(
                    m(0, 2).sub(m(2, 0)).mul(inv),
                    m(0, 1).add(m(1, 0)).mul(inv),
                    s.mul(quarter),
                    m(1, 2).add(m(2, 1)).mul(inv),
                )
            }
            _ => {
                let s = one
                    .add(m(2, 2))
                    .sub(m(0, 0))
                    .sub(m(1, 1))
                    .sqrt()?
                    .mul(T::TWO);
                let inv = one.div(s)?;
                Self::new(
                    m(1, 0).sub(m(0, 1)).mul(inv),
                    m(0, 2).add(m(2, 0)).mul(inv),
                    m(1, 2).add(m(2, 1)).mul(inv),
                    s.mul(quarter),
                )
            }
        };
        q.normalized()
            .map_err(|e| e.with_context(format!("from_rotation_columns({columns:?})")))
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

/// A rigid motion — where a body sits: a point `p` of it is at
/// `R p + position`, with `R` the rotation of a quaternion, which need not
/// be unit (see [`Pose::apply`]) — in the
/// number type `T`. Held as the two themselves, so that what is stored is
/// exactly what was set, and a pose read and written again is written
/// exactly as it was read.
///
/// Serialized as `{"position": [x, y, z], "rotation": [w, x, y, z]}`, each
/// number its value's midpoint; a rotation read is kept as read, sharp —
/// a hair off unit as a midpoint is, as a solver's step leaves one too —
/// so that reading what was written gives back what was read before
/// (see [`Pose::saved`]). Angles are
/// for showing a pose to people only ([`Pose::euler_degrees`],
/// [`Pose::from_euler`]).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(into = "PoseFile", try_from = "PoseFile", bound = "T: Number")]
pub struct Pose<T: Number = Scalar> {
    rotation: Quaternion<T>,
    position: Vector3<T>,
}

/// A [`Pose`] as a file has it.
#[derive(Serialize, Deserialize)]
struct PoseFile {
    position: [f64; 3],
    rotation: [f64; 4],
}

impl<T: Number> From<Pose<T>> for PoseFile {
    fn from(pose: Pose<T>) -> Self {
        let p = pose.position();
        PoseFile {
            position: p.to_f64_array(),
            rotation: pose.rotation().components().map(|c| c.to_f64()),
        }
    }
}

impl<T: Number> TryFrom<PoseFile> for Pose<T> {
    type Error = GeopError;

    /// The pose as read: normalizing the rotation would make it an
    /// interval no file holds, written as another midpoint, read as another
    /// interval — a pose that changes every time it is saved. The zero
    /// quaternion, which stands for no rotation, is refused.
    fn try_from(file: PoseFile) -> GeopResult<Self> {
        let [w, x, y, z] = file.rotation.map(T::from_f64);
        let rotation = Quaternion::new(w, x, y, z);
        if !rotation.norm_sq().definitely_greater(T::ZERO) {
            return Err(GeopError::new("a rotation cannot be the zero quaternion"));
        }
        Ok(Pose {
            rotation,
            position: Vector3::from_array(file.position.map(T::from_f64)),
        })
    }
}

/// Its position and its rotation's quaternion, each component exactly —
/// the quaternion as it is, not normalized: what a value kept to be read
/// again holds ([`crate::exact`]).
impl crate::exact::Exact for Pose {
    fn write<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let [w, x, y, z] = self.rotation.components();
        let rotation = Vector::from_array([w, x, y, z]);
        let written = (
            crate::exact::Written(&self.position),
            crate::exact::Written(&rotation),
        );
        written.serialize(serializer)
    }

    fn read<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let (crate::exact::Read(position), crate::exact::Read(rotation)): (
            crate::exact::Read<Vector3>,
            crate::exact::Read<Vector<4>>,
        ) = Deserialize::deserialize(deserializer)?;
        Ok(Pose {
            rotation: Quaternion::new(rotation[0], rotation[1], rotation[2], rotation[3]),
            position,
        })
    }
}

impl<T: Number> Default for Pose<T> {
    fn default() -> Self {
        Pose::identity()
    }
}

impl<T: Number> Pose<T> {
    pub fn identity() -> Self {
        Pose {
            rotation: Quaternion::identity(),
            position: Vector3::zero(),
        }
    }

    /// Turning by `rotation` — any quaternion definitely not zero,
    /// normalized here — then moving to `position`.
    pub fn new(position: Vector3<T>, rotation: Quaternion<T>) -> GeopResult<Self> {
        Ok(Pose {
            rotation: rotation.normalized()?,
            position,
        })
    }

    /// Moving to `position`, turned by `degrees` about `x`, then `y`, then
    /// `z` — the world's axes — as a dialog shows a pose.
    pub fn from_euler(position: Vector3<T>, degrees: [T; 3]) -> GeopResult<Self> {
        Pose::new(position, Quaternion::from_euler(degrees)?)
    }

    /// The pose a file holds of this one: written, and read back (see
    /// [`Pose`]'s serialization). Reading it and writing it again gives it
    /// back exactly.
    pub fn saved(&self) -> GeopResult<Self> {
        Pose::try_from(PoseFile::from(*self))
    }

    /// Where its body's origin is.
    pub fn position(&self) -> Vector3<T> {
        self.position
    }

    /// Its rotation's quaternion: unit as [`Pose::new`] makes it, as read
    /// from a file otherwise — a quaternion of any norm but zero stands for
    /// the rotation it divides out (see [`Pose::apply`]).
    pub fn rotation(&self) -> Quaternion<T> {
        self.rotation
    }

    /// Every component mapped by `f` — into dual numbers, say.
    pub fn map<U: Number>(&self, f: impl Fn(T) -> U) -> Pose<U> {
        Pose {
            rotation: self.rotation.map(&f),
            position: self.position.map(&f),
        }
    }

    /// Its rotation as angles in degrees about `x`, then `y`, then `z` (see
    /// [`Pose::from_euler`]) — for showing it to people, and so read off
    /// its midpoint. At gimbal lock (the second at ±90°) only the sum or
    /// difference of the other two is determined; the split is arbitrary
    /// but exact.
    pub fn euler_degrees(&self) -> [f64; 3] {
        // The unit quaternion of its rotation, which it need not be itself.
        let q = self.rotation().components().map(|c| c.to_f64());
        let norm = q.iter().map(|c| c * c).sum::<f64>().sqrt();
        let [w, x, y, z] = q.map(|c| c / norm);
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
    pub fn with_position(&self, position: Vector3<T>) -> Pose<T> {
        Pose {
            rotation: self.rotation,
            position,
        }
    }

    /// This pose after `inner`: a point is moved by `inner` first.
    pub fn compose(&self, inner: &Pose<T>) -> Pose<T> {
        Pose {
            rotation: self.rotation.mul(&inner.rotation),
            position: self.apply(&inner.position),
        }
    }

    /// The pose that undoes this one. Its rotation is the inverse
    /// quaternion, `q̄ / |q|²`, so that a pose composed with its inverse is
    /// the identity whatever `q`'s norm — and norms do not grow along a
    /// chain of poses undone and done again.
    pub fn inverse(&self) -> Pose<T> {
        let conj = self.rotation.conj();
        let turned = conj
            .mul(&Quaternion::pure(&self.position))
            .mul(&self.rotation);
        Pose {
            rotation: self.unturned(conj),
            position: self.unturned(turned).vector().neg(),
        }
    }

    /// The rigid motion of the twist `(ω, v)` — turning at `ω` while moving
    /// at `v`, for unit time: the exponential map of `ξ̂ = ω + ε v` as a
    /// unit dual quaternion, `exp(ξ̂ / 2)`. Its rotation turns by `|ω|`
    /// radians about `ω`; it is a screw motion along that axis. Zero is the
    /// identity, and to first order it is `(1, ω / 2) + ε (0, v / 2)`.
    ///
    /// With `φ = |ω| / 2`, it is `(cos φ, s ω) + ε (−s (ω·v) / 2, s v + c
    /// (ω·v) ω)`, where `s = sin φ / (2φ)` and `c = (cos φ − sin φ / φ) /
    /// (8φ²)`. Near `φ = 0` both are summed as their series in `φ²`, which
    /// are polynomials: smooth through zero, where the closed forms are
    /// `0 / 0` — so the map can be differentiated there with dual numbers,
    /// as a solver does to linearize a pose. The series are enclosed with
    /// what they leave out, so the map is an enclosure for a twist that is
    /// a box around zero too, and so are its derivatives.
    ///
    /// Its rotation is the real part; its position, the translation `2 d r*`
    /// of the dual part `d` and the rotation `r`, multiplied out:
    /// `2 s cos φ v + (s² + 2 c cos φ) (ω·v) ω + 2 s² ω × v`.
    pub fn exp(omega: &Vector3<T>, v: &Vector3<T>) -> GeopResult<Self> {
        let phi2 = omega.prod_dot(omega).div(T::from_i64(4))?;
        let (cos, s, c) = if phi2.definitely_less(T::ONE) {
            // Series in φ²: Σ (−φ²)^k / (2k)!, ½ Σ (−φ²)^k / (2k+1)! and
            // −Σ (−φ²)^k 2(k+1) / (8 (2k+3)!), each term made from the one
            // before — no factorial is formed, so they hold in any `Number`.
            let minus = phi2.neg();
            let (mut cos, mut s, mut c) = (T::ZERO, T::ZERO, T::ZERO);
            // The k-th terms, as k counts up (`d` the one subtracted from c).
            let mut cos_term = T::ONE;
            let mut s_term = T::ONE.div(T::TWO)?;
            let mut d = T::from_i64(2).div(T::from_i64(48))?;
            for k in 0..SERIES_TERMS {
                cos = cos.add(cos_term);
                s = s.add(s_term);
                c = c.sub(d);
                let k = k as i64;
                cos_term = cos_term
                    .mul(minus)
                    .div(T::from_i64((2 * k + 1) * (2 * k + 2)))?;
                s_term = s_term
                    .mul(minus)
                    .div(T::from_i64((2 * k + 2) * (2 * k + 3)))?;
                d = d
                    .mul(minus)
                    .mul(T::from_i64(k + 2))
                    .div(T::from_i64((k + 1) * (2 * k + 4) * (2 * k + 5)))?;
            }
            // Each series alternates, and for φ² < 1 its terms shrink from
            // the first one left out on — as do those of its derivative by
            // φ²: each sum is within that term of where it stops, and so,
            // for a dual number, is each derivative. A term times
            // `[-1, 1]` is that bound, part by part.
            let within = T::ONE.neg().union(T::ONE);
            (
                cos.add(cos_term.mul(within)),
                s.add(s_term.mul(within)),
                c.sub(d.mul(within)),
            )
        } else {
            let phi = phi2.sqrt()?;
            let (cos, sin) = (phi.cos(), phi.sin());
            let s = sin.div(T::TWO.mul(phi))?;
            let c = cos.sub(sin.div(phi)?).div(T::from_i64(8).mul(phi2))?;
            (cos, s, c)
        };
        let rotation = Quaternion::new(cos, s.mul(omega[0]), s.mul(omega[1]), s.mul(omega[2]));
        let (two_s_cos, s2) = (T::TWO.mul(s).mul(cos), s.mul(s));
        let along = s2.add(T::TWO.mul(c).mul(cos)).mul(omega.prod_dot(v));
        let position = (v.prod_scalar(two_s_cos))
            .add(&omega.prod_scalar(along))
            .add(&omega.prod_cross(v).prod_scalar(T::TWO.mul(s2)));
        Ok(Pose { rotation, position })
    }

    /// The point `p` moved by the pose: turned by the rotation its
    /// quaternion stands for, `q p q̄ / |q|²`, then moved.
    pub fn apply(&self, p: &Vector3<T>) -> Vector3<T> {
        let r = &self.rotation;
        let turned = self.unturned(r.mul(&Quaternion::pure(p)).mul(&r.conj()));
        turned.vector().add(&self.position)
    }

    /// `q` divided by the norm squared of the pose's quaternion: what a
    /// product with it and its conjugate turns by. A pose's quaternion
    /// stands for its rotation whatever its norm — a file keeps it as read,
    /// a solver's step leaves it a hair off unit — but a product with it
    /// scales by its norm squared. Exactly unit, nothing is divided, and
    /// nothing widened.
    fn unturned(&self, q: Quaternion<T>) -> Quaternion<T> {
        let n = self.rotation.norm_sq();
        if n.is_exactly(T::ONE) {
            return q;
        }
        // A pose's quaternion is not zero (see `Pose::new` and its reader).
        q.scale(
            T::ONE
                .div(n)
                .expect("a pose's rotation is not the zero quaternion"),
        )
    }

    /// Turning by `radians` about the line through `point` along the unit
    /// vector `direction` — counter-clockwise, looking against `direction`
    /// — as a pose: the rotation `(cos ½θ, sin ½θ d)`, and the position
    /// that keeps `point` where it is. That quaternion is unit already,
    /// within its enclosure, and kept as it is: normalized again, honestly,
    /// it came out four times as wide, and so did everything the pose turned.
    pub fn rotation_about(
        point: &Vector3<T>,
        direction: &Vector3<T>,
        radians: T,
    ) -> GeopResult<Self> {
        let half = radians.div(T::TWO)?;
        let (c, s) = (half.cos(), half.sin());
        let turn = Pose {
            rotation: Quaternion::new(
                c,
                s.mul(direction[0]),
                s.mul(direction[1]),
                s.mul(direction[2]),
            ),
            position: Vector3::zero(),
        };
        if !turn.rotation.norm_sq().definitely_greater(T::ZERO) {
            return Err(GeopError::new(format!(
                "Pose::rotation_about: the direction {direction:?} is no unit vector"
            )));
        }
        Ok(turn.with_position(point.sub(&turn.apply(point))))
    }

    /// What the pose does to geometry: its rotation's columns and its
    /// position, enclosed once, so moving many points costs a matrix
    /// product each. A pose that does not turn at all — its rotation
    /// exactly the identity — moves geometry by additions alone.
    pub fn motion(&self) -> Motion<T> {
        let turns = !self.rotation.is_identity();
        Motion {
            axes: turns.then(|| {
                self.rotation
                    .rotation_columns()
                    .expect("a pose's rotation is no zero quaternion")
            }),
            position: self.position,
            mirrors: false,
        }
    }
}

impl Pose {
    /// The differential of [`Pose::exp`] in the frame it leads to — the
    /// right Jacobian of the exponential map: the matrix `J` with
    /// `exp(ξ + dξ) = exp(ξ) ∘ exp(J dξ)` to first order, for the twist
    /// `ξ = (ω, v)`, rows and columns a turn's three numbers, then a move's.
    /// An enclosure for every twist in the boxes `omega` and `v`, made by
    /// differentiating the map — enclosed with what its series leave out —
    /// with dual numbers: column `k` is the twist `exp(ξ)⁻¹ ∂ₖ exp(ξ)`, its
    /// turn `2 r̄ ∂ₖr` and its move `Rᵀ ∂ₖt` for the rotation `r` and the
    /// position `t`.
    #[allow(clippy::needless_range_loop, reason = "a column of each of two blocks")]
    pub fn exp_differential(omega: &Vector3, v: &Vector3) -> GeopResult<[[Scalar; 6]; 6]> {
        let moved = Pose::exp_with_derivatives(omega, v)?;
        let r = moved.rotation();
        let at = r.map(|c| c.v);
        let [x, y, z] = at.rotation_columns()?;
        let mut jacobian = [[Scalar::ZERO; 6]; 6];
        for k in 0..6 {
            let dr = r.map(|c| c.d[k]);
            let turn = at.conj().mul(&dr).vector();
            let dt = Vector3::from_array([0, 1, 2].map(|i| moved.position()[i].d[k]));
            let along = [x.prod_dot(&dt), y.prod_dot(&dt), z.prod_dot(&dt)];
            for i in 0..3 {
                jacobian[i][k] = Scalar::TWO.mul(turn[i]);
                jacobian[3 + i][k] = along[i];
            }
        }
        Ok(jacobian)
    }
}

/// A pose steps by a twist `(ω, v)` in its own frame — turning about its
/// own origin, and moving along its own axes — by the exponential map:
/// `pose ∘ exp(ω, v)`.
impl Pose<Dual<6>> {
    /// The pose of the twist `(omega, v)` (see [`Pose::exp`]), with its
    /// derivatives by the twist's six numbers — a turn's three, then a
    /// move's: at a twist of zero, the step of a pose by its own six
    /// variables, in its own frame.
    pub fn exp_with_derivatives(omega: &Vector3, v: &Vector3) -> GeopResult<Self> {
        let var = |k: usize, x: Scalar| Dual::var(x, k);
        Pose::exp(
            &Vector3::from_array([var(0, omega[0]), var(1, omega[1]), var(2, omega[2])]),
            &Vector3::from_array([var(3, v[0]), var(4, v[1]), var(5, v[2])]),
        )
    }
}

impl Variable for Pose {
    fn dimension(&self) -> usize {
        6
    }

    fn retract(&self, step: &[Scalar]) -> GeopResult<Self> {
        let (omega, v) = twist(step)?;
        Ok(self.compose(&Pose::exp(&omega, &v)?).map(Scalar::sharpen))
    }

    /// Of a rigid motion: its rotation a unit quaternion. One that a step
    /// sharpened is a hair off unit, and turning a point by it scales it by
    /// its norm squared — a hair off every rigid motion, so off the true
    /// solution too. So it is made unit first, honestly: an enclosure of
    /// the rotation it stands for.
    fn retract_over(&self, step: &[Scalar]) -> GeopResult<Self> {
        let unit = Pose {
            rotation: self.rotation.normalized()?,
            position: self.position,
        };
        if step.iter().all(|d| d.is_zero() && d.is_sharp()) {
            return Ok(unit);
        }
        let (omega, v) = twist(step)?;
        Ok(unit.compose(&Pose::exp(&omega, &v)?))
    }

    /// The right Jacobian of the exponential map ([`Pose::exp_differential`]);
    /// the identity, exactly, for a step of exact zeros.
    fn differential(&self, step: &[Scalar]) -> GeopResult<Option<Vec<Vec<Scalar>>>> {
        if step.iter().all(|d| d.is_zero() && d.is_sharp()) {
            return Ok(None);
        }
        let (omega, v) = twist(step)?;
        let jacobian = Pose::exp_differential(&omega, &v)?;
        Ok(Some(jacobian.iter().map(|row| row.to_vec()).collect()))
    }

    /// Its position's difference from `at`'s, and its rotation's as the
    /// unit quaternions' difference — of the two quaternions of `at`'s
    /// rotation, the nearer — times `scale`, so that a turn counts as the
    /// distance it moves a point `scale` from its axis.
    fn stay(&self, at: &Self, scale: Scalar) -> GeopResult<Rows> {
        type D = crate::dual::Dual<6>;
        let step = Pose::exp_with_derivatives(&Vector3::zero(), &Vector3::zero())?;
        let moved = self.map(D::cst).compose(&step);
        let offset = moved.position().sub(&at.position().map(D::cst));
        let r = moved.rotation();
        let mut target = at.rotation();
        if r.map(|c| c.v).dot(&target).definitely_less(Scalar::ZERO) {
            target = target.scale(Scalar::ONE.neg());
        }
        let turned = (r.components().into_iter().zip(target.components()))
            .map(|(c, t)| c.sub(D::cst(t)).mul(D::cst(scale)));
        let mut rows = Rows::default();
        for residual in offset.to_array().into_iter().chain(turned) {
            rows.push(residual.v, residual.d.to_vec());
        }
        Ok(rows)
    }

    fn same(&self, other: &Self) -> bool {
        self == other
    }

    /// Moved as `from` moved to `to`: `to ∘ from⁻¹ ∘ self`.
    fn carried(&self, from: &Self, to: &Self) -> Option<Self> {
        Some(
            to.compose(&from.inverse())
                .compose(self)
                .map(Scalar::sharpen),
        )
    }
}

/// The twist `(ω, v)` a step of six numbers is.
fn twist(step: &[Scalar]) -> GeopResult<(Vector3, Vector3)> {
    match step {
        [w0, w1, w2, v0, v1, v2] => Ok((
            Vector3::from_array([*w0, *w1, *w2]),
            Vector3::from_array([*v0, *v1, *v2]),
        )),
        _ => Err(GeopError::new(format!(
            "a step of a pose is six numbers, not {}",
            step.len()
        ))),
    }
}

/// An affine map as it moves geometry: a [`Pose`]'s rigid motion (see
/// [`Pose::motion`]), a plain translation, a mirror in a plane, or a
/// scaling ([`Motion::scaling`], the one that is no isometry) — what a
/// body is moved, patterned, mirrored or scaled by.
///
/// A point `p` goes to `A p + position`, with `A` the matrix whose columns
/// are `axes`: a rotation's, a mirror's reflection or a scaling's diagonal.
/// Without axes `A` is the identity and nothing is multiplied: a
/// translation moves every point by one addition per coordinate, as
/// exactly as an interval addition can.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion<T: Number = Scalar> {
    axes: Option<[Vector3<T>; 3]>,
    position: Vector3<T>,
    mirrors: bool,
}

impl<T: Number> Motion<T> {
    /// Moving every point by `offset`.
    pub fn translation(offset: Vector3<T>) -> Self {
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
    pub fn mirror(point: &Vector3<T>, normal: &Vector3<T>) -> GeopResult<Self> {
        let n = normal.normalize()?;
        let column = |k: usize| {
            let mut c = n.prod_scalar(T::TWO.mul(n[k]).neg());
            c[k] = T::ONE.add(c[k]);
            c
        };
        Ok(Motion {
            axes: Some([column(0), column(1), column(2)]),
            position: n.prod_scalar(T::TWO.mul(point.prod_dot(&n))),
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
    pub fn position(&self) -> Vector3<T> {
        self.position
    }

    /// The direction `d` turned — or mirrored; a translation leaves it as
    /// it is.
    pub fn rotate(&self, d: &Vector3<T>) -> Vector3<T> {
        match &self.axes {
            None => *d,
            Some([x, y, z]) => x
                .prod_scalar(d[0])
                .add(&y.prod_scalar(d[1]))
                .add(&z.prod_scalar(d[2])),
        }
    }

    /// The point `p` moved.
    pub fn apply(&self, p: &Vector3<T>) -> Vector3<T> {
        self.rotate(p).add(&self.position)
    }
}

impl Motion {
    /// The affine map `p -> A p + position`, `A` the matrix whose columns
    /// are `columns`: what a body is scaled by, or carried by past the end
    /// of a deformation's zone (`geop_core_geometry::deform`). No isometry
    /// in general — lengths and angles change, directions it turns are no
    /// longer unit, and only a body's geometry is moved by it, whose NURBS
    /// map exactly (an affine map of their homogeneous control points). It
    /// mirrors where the determinant of `A` is negative, and fails where
    /// the determinant could be zero, which would flatten everything.
    pub fn affine(columns: [Vector3; 3], position: Vector3) -> GeopResult<Self> {
        let det = columns[0].prod_dot(&columns[1].prod_cross(&columns[2]));
        if det.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "Motion::affine: the columns {columns:?} flatten everything, their determinant being {det:?}"
            )));
        }
        Ok(Motion {
            axes: Some(columns),
            position,
            mirrors: det.definitely_less(Scalar::ZERO),
        })
    }

    /// Scaling about `center` by `factors` along `x`, `y` and `z`: `p` goes
    /// to `center + diag(factors) (p - center)` (see [`Motion::affine`]).
    /// An odd number of negative factors mirrors. Fails for a factor that
    /// could be zero.
    pub fn scaling(center: &Vector3, factors: [Scalar; 3]) -> GeopResult<Self> {
        if let Some(zero) = factors.iter().find(|f| f.could_be_equal(Scalar::ZERO)) {
            return Err(GeopError::new(format!(
                "Motion::scaling: a factor of {zero:?} flattens everything"
            )));
        }
        let column = |k: usize| {
            let mut c = Vector3::zero();
            c[k] = factors[k];
            c
        };
        let position = Vector3::from_array(std::array::from_fn(|k| {
            center[k].sub(factors[k].mul(center[k]))
        }));
        Self::affine([column(0), column(1), column(2)], position)
    }

    /// The frame `frame` moved. Fails for a mirror, which would leave it
    /// left-handed.
    pub fn apply_frame(&self, frame: &CoordinateSystem) -> GeopResult<CoordinateSystem> {
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

    fn v(p: [f64; 3]) -> Vector3 {
        Vector3::from_array(p.map(Scalar::from_f64))
    }

    fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose {
        Pose::from_euler(v(position), degrees.map(Scalar::from_f64)).unwrap()
    }

    /// `a` could be `b`, as an enclosure of it.
    fn encloses(a: &Vector3, b: [f64; 3]) -> bool {
        (0..3).all(|k| a[k].could_be_equal(Scalar::from_f64(b[k])))
    }

    fn close(a: &Vector3, b: &Vector3) -> bool {
        (0..3).all(|k| (a[k].to_f64() - b[k].to_f64()).abs() < 1e-12)
    }

    /// A rotation read back from its matrix is the same rotation, whichever
    /// of the trace and the diagonal is largest: no turn, half turns about
    /// each axis, and turns in between.
    #[test]
    fn rotations_are_read_back_from_their_matrices() {
        for degrees in [
            [0.0, 0.0, 0.0],
            [180.0, 0.0, 0.0],
            [0.0, 180.0, 0.0],
            [0.0, 0.0, 180.0],
            [30.0, -70.0, 125.0],
            [170.0, 10.0, -175.0],
        ] {
            let rotation = pose([0.0; 3], degrees).rotation();
            let columns = rotation.rotation_columns().unwrap();
            let back = Quaternion::from_rotation_columns(columns).unwrap();
            let again = back.rotation_columns().unwrap();
            for k in 0..3 {
                assert!(close(&columns[k], &again[k]), "{degrees:?}");
            }
        }
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
                original.euler_degrees().map(Scalar::from_f64),
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

    /// Files hold a position and a quaternion; the zero quaternion is
    /// refused.
    #[test]
    fn poses_read_and_write_position_and_quaternion() {
        let written = Pose::new(
            v([1.0, 2.0, 3.0]),
            Quaternion::new(Scalar::ZERO, Scalar::ZERO, Scalar::ZERO, Scalar::TWO),
        )
        .unwrap();
        let json = serde_json::to_value(written).unwrap();
        assert_eq!(
            json,
            serde_json::json!({"position": [1.0, 2.0, 3.0], "rotation": [0.0, 0.0, 0.0, 1.0]})
        );
        let read: Pose = serde_json::from_value(json).unwrap();
        assert!(encloses(&read.apply(&v([1.0, 0.0, 0.0])), [0.0, 2.0, 3.0]));
        let zero =
            serde_json::json!({"position": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 0.0]});
        assert!(serde_json::from_value::<Pose>(zero).is_err());
    }

    /// A pose read from a file is written back as it was, and read again
    /// is the same pose: saving a pose twice changes nothing.
    #[test]
    fn a_pose_read_is_written_back_as_it_was() {
        let file = serde_json::json!({
            "position": [34.18148346551969, 62.89507987346716, 5.871850811598488e-15],
            "rotation": [0.9731912888550477, -5.175907578430137e-17, 4.296936517018427e-17,
                         -0.22999720714967709],
        });
        let read: Pose = serde_json::from_value(file.clone()).unwrap();
        let written = serde_json::to_value(read).unwrap();
        assert_eq!(written, file);
        assert_eq!(serde_json::from_value::<Pose>(written).unwrap(), read);
        let turned = pose([1.0, -2.0, 0.5], [10.0, 20.0, 30.0]);
        let saved = turned.saved().unwrap();
        assert_eq!(saved.saved().unwrap(), saved);
    }

    /// A quaternion read sharp, a hair off unit, turns a point where the
    /// unit quaternion it stands for does: the two enclosures of where it
    /// goes overlap.
    #[test]
    fn a_rotation_read_off_unit_turns_as_its_unit_one() {
        let turned = pose([0.0; 3], [10.0, 20.0, 30.0]);
        let saved = turned.saved().unwrap();
        let unit = Pose::new(saved.position(), saved.rotation()).unwrap();
        let p = v([3.0, -1.0, 2.0]);
        let (a, b) = (saved.apply(&p), unit.apply(&p));
        assert!((0..3).all(|k| a[k].could_be_equal(b[k])), "{a:?} {b:?}");
    }

    /// A quarter turn about the line through (1, 1, 0) along `z` keeps that
    /// line where it is and takes (2, 1, 5) to (1, 2, 5).
    #[test]
    fn rotation_about_a_line_keeps_it() {
        let turn = Pose::rotation_about(
            &v([1.0, 1.0, 0.0]),
            &v([0.0, 0.0, 1.0]),
            Scalar::PI.div(Scalar::TWO).unwrap(),
        )
        .unwrap();
        assert!(encloses(&turn.apply(&v([1.0, 1.0, 7.0])), [1.0, 1.0, 7.0]));
        assert!(encloses(&turn.apply(&v([2.0, 1.0, 5.0])), [1.0, 2.0, 5.0]));
        let motion = turn.motion();
        assert!(motion.turns() && !motion.mirrors());
        assert!(encloses(
            &motion.apply(&v([2.0, 1.0, 5.0])),
            [1.0, 2.0, 5.0]
        ));
    }

    /// A pose that does not turn moves points by additions alone; a
    /// mirror in a slanted plane is undone by itself and reverses
    /// orientation.
    #[test]
    fn translations_and_mirrors() {
        let shift = Pose::identity().with_position(v([0.1, 0.2, 0.3])).motion();
        assert!(!shift.turns());
        let p = v([1.0, 2.0, 3.0]);
        assert_eq!(
            format!("{:?}", shift.apply(&p)),
            format!("{:?}", p.add(&v([0.1, 0.2, 0.3])))
        );
        let mirror = Motion::mirror(&v([1.0, 0.0, 2.0]), &v([1.0, -2.0, 0.5])).unwrap();
        assert!(mirror.mirrors());
        let q = v([0.3, -0.7, 4.0]);
        assert!(mirror.apply(&mirror.apply(&q)).could_be_equal(&q));
        assert!(encloses(
            &mirror.apply(&v([1.0, 0.0, 2.0])),
            [1.0, 0.0, 2.0]
        ));
        let (x, y, z) = (v([1.0, 0.0, 0.0]), v([0.0, 1.0, 0.0]), v([0.0, 0.0, 1.0]));
        let handedness = mirror
            .rotate(&x)
            .prod_cross(&mirror.rotate(&y))
            .prod_dot(&mirror.rotate(&z));
        assert!(
            handedness.could_be_equal(Scalar::ONE.neg()),
            "{handedness:?}"
        );
        assert!(mirror.apply_frame(&CoordinateSystem::world_at(q)).is_err());
    }

    /// The exponential of a twist: nothing for zero; a turn alone about the
    /// origin; a move alone; and a turn about a line through `q`, whose
    /// twist has `v = −ω × q`, the turn `rotation_about` gives.
    #[test]
    fn exp_is_the_motion_of_a_twist() {
        let zero = Pose::exp(&v([0.0; 3]), &v([0.0; 3])).unwrap();
        assert!(encloses(&zero.apply(&v([1.0, 2.0, 3.0])), [1.0, 2.0, 3.0]));

        let quarter = std::f64::consts::FRAC_PI_2;
        let turn = Pose::exp(&v([0.0, 0.0, quarter]), &v([0.0; 3])).unwrap();
        let p = turn.apply(&v([1.0, 0.0, 0.0]));
        assert!(
            (p[0].to_f64()).abs() < 1e-15 && (p[1].to_f64() - 1.0).abs() < 1e-15,
            "{p:?}"
        );

        let shift = Pose::exp(&v([0.0; 3]), &v([1.0, 2.0, 3.0])).unwrap();
        assert!(encloses(&shift.position(), [1.0, 2.0, 3.0]));

        for angle in [0.3, 1.9, 3.0, 5.5] {
            let (axis, q) = (v([0.0, 0.0, 1.0]), v([2.0, 1.0, 0.5]));
            let omega = axis.prod_scalar(Scalar::from_f64(angle));
            let screw = Pose::exp(&omega, &omega.prod_cross(&q).neg()).unwrap();
            let about = Pose::rotation_about(&q, &axis, Scalar::from_f64(angle)).unwrap();
            for point in [[0.0; 3], [1.0, -2.0, 4.0]] {
                let (a, b) = (screw.apply(&v(point)), about.apply(&v(point)));
                for k in 0..3 {
                    let gap = (a[k].to_f64() - b[k].to_f64()).abs();
                    assert!(gap < 1e-12, "{angle}: {a:?} vs {b:?}");
                }
            }
        }
    }

    /// The two ways the map is summed agree where they meet, at a turn of
    /// two radians: the series just below, the closed form just above.
    #[test]
    fn exp_is_continuous_where_its_series_ends() {
        let at = |angle: f64| {
            let omega = v([0.6, 0.0, 0.8]).prod_scalar(Scalar::from_f64(angle));
            Pose::exp(&omega, &v([0.3, -1.0, 2.0])).unwrap()
        };
        let (below, above) = (at(2.0 - 1e-12), at(2.0 + 1e-12));
        for point in [[0.0; 3], [1.0, 1.0, 1.0]] {
            let (a, b) = (below.apply(&v(point)), above.apply(&v(point)));
            for k in 0..3 {
                assert!(
                    (a[k].to_f64() - b[k].to_f64()).abs() < 1e-10,
                    "{a:?} vs {b:?}"
                );
            }
        }
    }

    /// Differentiated at zero with dual numbers, the map is its first order:
    /// a point `p` moves at `ω × p + v`.
    #[test]
    fn exp_differentiates_at_zero() {
        use crate::dual::Dual;
        type D = Dual<6>;
        let var = |k: usize| D::var(Scalar::ZERO, k);
        let omega = Vector3::from_array([var(0), var(1), var(2)]);
        let velocity = Vector3::from_array([var(3), var(4), var(5)]);
        let moved = Pose::<D>::exp(&omega, &velocity).unwrap();
        let p = [1.0, 2.0, 3.0];
        let at = moved.apply(&Vector3::from_array(p.map(|c| D::cst(Scalar::from_f64(c)))));
        // d/dω (ω × p) = −[p]×, d/dv v = I.
        let expected = [
            [0.0, p[2], -p[1], 1.0, 0.0, 0.0],
            [-p[2], 0.0, p[0], 0.0, 1.0, 0.0],
            [p[1], -p[0], 0.0, 0.0, 0.0, 1.0],
        ];
        for (k, row) in expected.iter().enumerate() {
            assert!(at[k].v.could_be_equal(Scalar::from_f64(p[k])));
            for (j, &exact) in row.iter().enumerate() {
                let d = at[k].d[j];
                assert!(
                    d.could_be_equal(Scalar::from_f64(exact)),
                    "∂{k}/∂{j} = {d:?}"
                );
            }
        }
    }

    /// A quaternion off unit — as a file keeps it — stands for the rotation
    /// of its unit one: a pose with it moves points as that one does, and
    /// composed with its inverse is the identity, its norm not compounded.
    #[test]
    fn a_rotation_off_unit_moves_points_as_its_unit_one() {
        let saved: Pose = serde_json::from_value(serde_json::json!({
            "position": [1.0, -7.25, 2.0],
            "rotation": [1.09, 0.2, -0.3, 0.1],
        }))
        .unwrap();
        let unit = Pose::new(saved.position(), saved.rotation()).unwrap();
        let p = [3.0, 4.0, -5.0];
        let moved = unit.apply(&v(p)).to_f64_array();
        assert!(
            encloses(&saved.apply(&v(p)), moved),
            "{:?}",
            saved.apply(&v(p))
        );
        let back = saved.inverse().compose(&saved);
        assert!(encloses(&back.apply(&v(p)), p), "{back:?}");
        assert!(back.rotation().norm_sq().could_be_equal(Scalar::ONE));
        assert!(encloses(&saved.inverse().apply(&saved.apply(&v(p))), p));
    }
}
