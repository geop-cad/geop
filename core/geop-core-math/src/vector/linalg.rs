use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
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

    pub fn normalize(&self) -> GeopResult<Self> {
        let n = self.norm();
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
    pub fn prod_cross(&self, other: &Self) -> Self {
        let mut out = Self::new();
        out[0] = self[1].mul(other[2]).sub(self[2].mul(other[1]));
        out[1] = self[2].mul(other[0]).sub(self[0].mul(other[2]));
        out[2] = self[0].mul(other[1]).sub(self[1].mul(other[0]));
        out
    }
}
