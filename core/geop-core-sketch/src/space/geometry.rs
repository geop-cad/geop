//! The geometry of a 3-D sketch's arcs, generic over [`Scalar`] so the same
//! formulas serve the residuals (differentiated as
//! [`geop_core_math::dual::Dual`]s) and building the NURBS.
//!
//! An arc is given by three points: from `s` through `m` to `e`. Everything
//! about it follows from the two legs `a = s - m` and `b = e - m` without a
//! single angle: the cosine of half its sweep is `-a.b / (|a| |b|)` and the
//! sine `|a x b| / (|a| |b|)` — the angle the arc makes at `m` is the
//! inscribed angle over the chord `se`.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

/// The circular arc from `s` through `m` to `e`.
#[derive(Clone, Copy, Debug)]
pub struct Arc3<T: Scalar> {
    pub s: Vector3<T>,
    pub m: Vector3<T>,
    pub e: Vector3<T>,
}

/// `|v|`, failing where it could be zero rather than differentiating a
/// square root there.
pub fn length<T: Scalar>(v: &Vector3<T>) -> GeopResult<T> {
    v.norm_sq().sqrt()
}

/// `v / |v|`.
pub fn unit<T: Scalar>(v: &Vector3<T>) -> GeopResult<Vector3<T>> {
    Ok(v.prod_scalar(T::ONE.div(length(v)?)?))
}

impl<T: Scalar> Arc3<T> {
    fn legs(&self) -> (Vector3<T>, Vector3<T>) {
        (self.s.sub(&self.m), self.e.sub(&self.m))
    }

    /// The unit normal of its plane, the way it turns: from `s` to `e` it
    /// runs counter-clockwise about it.
    pub fn normal(&self) -> GeopResult<Vector3<T>> {
        let (a, b) = self.legs();
        unit(&b.prod_cross(&a))
    }

    /// `cos` and `sin` of half its sweep.
    pub fn half_sweep(&self) -> GeopResult<(T, T)> {
        let (a, b) = self.legs();
        let ab = length(&a)?.mul(length(&b)?);
        Ok((
            a.prod_dot(&b).neg().div(ab)?,
            length(&a.prod_cross(&b))?.div(ab)?,
        ))
    }

    /// Its radius: `|a| |b| |e - s| / (2 |a x b|)`.
    pub fn radius(&self) -> GeopResult<T> {
        let (a, b) = self.legs();
        length(&a)?
            .mul(length(&b)?)
            .mul(length(&self.e.sub(&self.s))?)
            .div(T::TWO.mul(length(&a.prod_cross(&b))?))
    }

    /// Its center: `m + (|a|^2 b - |b|^2 a) x (a x b) / (2 |a x b|^2)`.
    pub fn center(&self) -> GeopResult<Vector3<T>> {
        let (a, b) = self.legs();
        let n = a.prod_cross(&b);
        let k = T::ONE.div(T::TWO.mul(n.norm_sq()))?;
        let v = b.prod_scalar(a.norm_sq()).sub(&a.prod_scalar(b.norm_sq()));
        Ok(self.m.add(&v.prod_cross(&n).prod_scalar(k)))
    }

    /// Its unit tangent at `end`, in its own direction — from `s` to `e`:
    /// the chord's direction turned back (at the start) or on (at the end)
    /// by half the sweep, in the arc's plane.
    pub fn tangent(&self, at_end: bool) -> GeopResult<Vector3<T>> {
        let chord = unit(&self.e.sub(&self.s))?;
        let side = self.normal()?.prod_cross(&chord);
        let (cos, sin) = self.half_sweep()?;
        let sin = if at_end { sin } else { sin.neg() };
        Ok(chord.prod_scalar(cos).add(&side.prod_scalar(sin)))
    }
}

/// `(u, v)`: two unit vectors that span, with `direction`, all of space —
/// to measure how far another vector is from parallel to it along. Which
/// two is a free choice: worked out in plain numbers, sharp.
pub fn across<S: Scalar>(direction: &Vector3<S>) -> GeopResult<[Vector3<S>; 2]> {
    let d = direction.map(|c| S::from_f64(c.to_f64()));
    let complement = d.orthonormal_complement()?;
    let sharp = |v: &Vector3<S>| v.map(|c| S::from_f64(c.to_f64()));
    Ok([sharp(&complement[0]), sharp(&complement[1])])
}
