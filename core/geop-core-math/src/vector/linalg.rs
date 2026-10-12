use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3, Vector4},
};

impl<S: Scalar, const N: usize> Vector<S, N> {
    pub fn add(&self, other: &Self) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].add(other[i]);
        }
        out
    }

    pub fn sub(&self, other: &Self) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].sub(other[i]);
        }
        out
    }

    pub fn neg(&self) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].neg();
        }
        out
    }

    /// Componentwise [`Scalar::sharpen`] — see that method's own doc
    /// comment for when collapsing a value to a single representative
    /// point is a safe, free choice (you only ever needed *some* point
    /// within the interval) versus a silent loss of real uncertainty.
    pub fn sharpen(&self) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].sharpen();
        }
        out
    }

    /// Componentwise [`Scalar::interpolate`]: `a` at `alpha=0`, `b` at
    /// `alpha=1`. See that method's doc comment for why this beats a manual
    /// `a*(1-alpha) + b*alpha` when components of `a`/`b` may coincide.
    pub fn interpolate(a: &Self, b: &Self, alpha: S) -> Self {
        let mut out = Self::new();
        for i in 0..a.size() {
            out[i] = S::interpolate(a[i], b[i], alpha);
        }
        out
    }

    pub fn prod_scalar(&self, s: S) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].mul(s);
        }
        out
    }

    pub fn prod_dot(&self, other: &Self) -> S {
        let mut acc = S::ZERO;
        for i in 0..self.size() {
            acc = acc.add(self[i].mul(other[i]));
        }
        acc
    }

    pub fn norm_sq(&self) -> S {
        self.prod_dot(self)
    }

    pub fn norm(&self) -> S {
        self.norm_sq().sqrt().expect("Norm cannot be negative")
    }

    /// `|self|`, failing where [`Scalar::sqrt`] refuses the squared length
    /// instead of panicking like [`Self::norm`].
    pub fn try_norm(&self) -> GeopResult<S> {
        self.norm_sq().sqrt()
    }

    pub fn normalize(&self) -> GeopResult<Self> {
        let n = self.try_norm()?;
        if n.could_be_equal(S::ZERO) {
            return Err(GeopError::new("Cannot normalize zero-length vector"));
        }
        Ok(self.prod_scalar(S::ONE.div(n)?))
    }

    /// `N - 1` orthonormal vectors perpendicular to `self`, or an error for
    /// a zero `self`. They are the columns other than the first of the
    /// Householder reflection `H = I - 2 w wᵀ / (wᵀ w)`, `w = v̂ + σ e_0`,
    /// which swaps `v̂ = self / |self|` and `σ e_0`: `H` is orthogonal, so its
    /// columns are orthonormal, and all but the first are perpendicular to
    /// `v̂`. The sign `σ` of `v̂_0` keeps `wᵀ w = 2 (1 + |v̂_0|) >= 2` away
    /// from cancellation, so the result is well conditioned for every `v̂`.
    pub fn orthonormal_complement(&self) -> GeopResult<Vec<Self>> {
        let v = self.normalize()?;
        let sigma = if v[0].definitely_less(S::ZERO) {
            S::ONE.neg()
        } else {
            S::ONE
        };
        let w = v.add(&Self::axis(0).prod_scalar(sigma));
        let scale = S::from_i64(2).div(w.norm_sq())?;
        Ok((1..N)
            .map(|j| Self::axis(j).sub(&w.prod_scalar(scale.mul(w[j]))))
            .collect())
    }

    /// How far `self` is from the segment `a..b`.
    pub fn distance_to_segment(&self, a: &Self, b: &Self) -> S {
        let ab = b.sub(a);
        let along = self
            .sub(a)
            .prod_dot(&ab)
            .div(ab.norm_sq())
            .unwrap_or(S::ZERO);
        let along = if along.definitely_less(S::ZERO) {
            S::ZERO
        } else if along.definitely_greater(S::ONE) {
            S::ONE
        } else {
            along
        };
        self.sub(&a.add(&ab.prod_scalar(along))).norm()
    }

    /// True if every component `could_be_equal` `other`'s.
    pub fn could_be_equal(&self, other: &Self) -> bool {
        (0..self.size()).all(|i| self[i].could_be_equal(other[i]))
    }

    /// The componentwise [`Scalar::union`] of `self` and `other`.
    pub fn union(&self, other: &Self) -> Self {
        let mut out = Self::new();
        for i in 0..self.size() {
            out[i] = self[i].union(other[i]);
        }
        out
    }
}

impl<S: Scalar> Vector3<S> {
    /// Two unit directions square to the unit vector `self` and to each
    /// other: the first from whichever of `x`, `y`, `z` runs least along it
    /// — the first no other definitely runs less along, so that an axis a
    /// hair off one of them does not pick between two by noise. A free
    /// choice, made the same way wherever a direction across an axis is
    /// measured from: a joint's turns, a frame's `x`, a sketch's parallel.
    pub fn across(&self) -> GeopResult<[Self; 2]> {
        let size = |k: usize| self[k].abs();
        let least = (0..3)
            .find(|&k| !(0..3).any(|j| size(j).definitely_less(size(k))))
            .unwrap_or(0);
        let reference = Self::axis(least);
        let first = (reference.sub(&self.prod_scalar(self.prod_dot(&reference))))
            .normalize()
            .map_err(|e| e.with_context(format!("a direction square to {self:?}")))?;
        Ok([first, self.prod_cross(&first)])
    }

    pub fn prod_cross(&self, other: &Self) -> Self {
        let mut out = Self::new();
        out[0] = self[1].mul(other[2]).sub(self[2].mul(other[1]));
        out[1] = self[2].mul(other[0]).sub(self[0].mul(other[2]));
        out[2] = self[0].mul(other[1]).sub(self[1].mul(other[0]));
        out
    }

    /// Whether `self` and `other` could be parallel, in either sense: their
    /// cross product could be zero.
    pub fn could_be_parallel(&self, other: &Self) -> bool {
        self.prod_cross(other).norm_sq().could_be_equal(S::ZERO)
    }

    /// `self` turned about the unit vector `axis` by the angle with cosine
    /// `cos` and sine `sin` (Rodrigues' formula).
    pub fn rotated_about(&self, axis: &Self, cos: S, sin: S) -> Self {
        let along = axis.prod_scalar(axis.prod_dot(self).mul(S::ONE.sub(cos)));
        self.prod_scalar(cos)
            .add(&axis.prod_cross(self).prod_scalar(sin))
            .add(&along)
    }

    /// The point `self` with weight `w`, homogeneous: `(w x, w y, w z, w)`.
    /// Taken as it is where the weight is exactly one: multiplying by it
    /// would only widen it by rounding.
    pub fn homogeneous(&self, w: S) -> Vector4<S> {
        if w.is_exactly(S::ONE) {
            Vector4::from_array([self[0], self[1], self[2], w])
        } else {
            Vector4::from_array([self[0].mul(w), self[1].mul(w), self[2].mul(w), w])
        }
    }

    /// The point the homogeneous 2-D point `(w x, w y, w)` stands for,
    /// `(x, y)`: each coordinate divided by the weight (not multiplied by
    /// its inverse, which rounds twice).
    pub fn dehomogenized(&self) -> GeopResult<Vector2<S>> {
        Ok(Vector2::from_array([
            self[0].div(self[2])?,
            self[1].div(self[2])?,
        ]))
    }
}

impl<S: Scalar> Vector4<S> {
    /// The point the homogeneous 3-D point `(w x, w y, w z, w)` stands for,
    /// `(x, y, z)`: each coordinate divided by the weight (not multiplied by
    /// its inverse, which rounds twice).
    pub fn dehomogenized(&self) -> GeopResult<Vector3<S>> {
        Ok(Vector3::from_array([
            self[0].div(self[3])?,
            self[1].div(self[3])?,
            self[2].div(self[3])?,
        ]))
    }
}

impl<S: Scalar> Vector2<S> {
    /// The 2-D cross product `self.x * other.y - self.y * other.x`: the z
    /// component of the 3-D one, positive when `other` lies counter-clockwise
    /// of `self`.
    pub fn prod_cross(&self, other: &Self) -> S {
        self[0].mul(other[1]).sub(self[1].mul(other[0]))
    }

    /// The point `self` with weight `w`, homogeneous: `(w x, w y, w)` —
    /// taken as it is where the weight is exactly one (see
    /// [`Vector3::homogeneous`]).
    pub fn homogeneous(&self, w: S) -> Vector3<S> {
        if w.is_exactly(S::ONE) {
            Vector3::from_array([self[0], self[1], w])
        } else {
            Vector3::from_array([self[0].mul(w), self[1].mul(w), w])
        }
    }

    /// Rotated 90 degrees counter-clockwise — by `neg`, which is exact,
    /// where subtracting from zero would round.
    pub fn perp(&self) -> Self {
        Self::from_array([self[1].neg(), self[0]])
    }

    /// Rotated counter-clockwise by the angle with cosine `c` and sine `s`.
    pub fn rotate(&self, c: S, s: S) -> Self {
        Self::from_array([
            self[0].mul(c).sub(self[1].mul(s)),
            self[0].mul(s).add(self[1].mul(c)),
        ])
    }
}
