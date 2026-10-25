pub mod linalg;

use std::{
    fmt::Display,
    ops::{Index, IndexMut},
};

use crate::scalars::{Number, Scalar};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<const N: usize, T = Scalar> {
    data: [T; N],
}

pub type Vector4<T = Scalar> = Vector<4, T>;
pub type Vector3<T = Scalar> = Vector<3, T>;
pub type Vector2<T = Scalar> = Vector<2, T>;

impl<T: Default + Copy, const N: usize> Default for Vector<N, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Default + Copy, const N: usize> Vector<N, T> {
    pub fn new() -> Self {
        Self {
            data: [T::default(); N],
        }
    }

    pub fn size(&self) -> usize {
        N
    }
}

impl<T: Number, const N: usize> Vector<N, T> {
    pub fn from_array(data: [T; N]) -> Self {
        Self { data }
    }

    /// Its components, as an array.
    pub fn to_array(&self) -> [T; N] {
        self.data
    }

    /// Its components as plain numbers ([`Scalar::to_f64`], the midpoint):
    /// for output and drawing, never for construction.
    pub fn to_f64_array(&self) -> [f64; N] {
        self.data.map(|x| x.to_f64())
    }

    /// Every component mapped by `f` — into dual numbers, say.
    pub fn map<U: Number>(&self, f: impl Fn(T) -> U) -> Vector<N, U> {
        Vector::from_array(self.data.map(f))
    }

    /// Element access using a multi-index slice; only the first index is used
    /// (vectors are 1-D).  Panics if `idx` is empty.
    pub fn get(&self, idx: &[usize]) -> T {
        self.data[idx[0]]
    }
}

impl<T, const N: usize> Index<usize> for Vector<N, T> {
    type Output = T;

    fn index(&self, idx: usize) -> &Self::Output {
        &self.data[idx]
    }
}

impl<T: Default + Copy, const N: usize> IndexMut<usize> for Vector<N, T> {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        &mut self.data[idx]
    }
}

// swap xy
impl<T: Number> Vector<2, T> {
    pub fn swap_xy(&self) -> Self {
        Self::from_array([self[1], self[0]])
    }
}

// zero
impl<T: Number, const N: usize> Vector<N, T> {
    pub fn zero() -> Self {
        Self { data: [T::ZERO; N] }
    }

    /// The `k`-th coordinate axis: one at `k`, zero elsewhere.
    pub fn axis(k: usize) -> Self {
        let mut out = Self::zero();
        out[k] = T::ONE;
        out
    }

    /// The first `M` components — e.g. the Cartesian part `H` of a
    /// homogeneous point `(H, w)`, which is *not* its position `H / w`.
    pub fn head<const M: usize>(&self) -> Vector<M, T> {
        Vector::from_array(std::array::from_fn(|k| self.data[k]))
    }
}

impl<const N: usize> Vector<N> {
    /// Every component set to [`Scalar::ENTIRE`] — `could_be_equal`s any
    /// other vector of the same dimension componentwise.
    pub fn everything() -> Self {
        Self {
            data: [Scalar::ENTIRE; N],
        }
    }

    /// Componentwise [`Scalar::interpolate`]: `a` at `alpha=0`, `b` at
    /// `alpha=1`. See that method's doc comment for why this beats a manual
    /// `a*(1-alpha) + b*alpha` when components of `a`/`b` may coincide.
    pub fn interpolate(a: &Self, b: &Self, alpha: Scalar) -> Self {
        let mut out = Self::new();
        for i in 0..a.size() {
            out[i] = Scalar::interpolate(a[i], b[i], alpha);
        }
        out
    }
}

impl<T: Number, const N: usize> Display for Vector<N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Vector{}(", N)?;
        for i in 0..N {
            write!(f, "{}", self.data[i])?;
            if i < N - 1 {
                write!(f, ", ")?;
            }
        }
        write!(f, ")")
    }
}

/// Format a slice of vectors as `[v0, v1, ...]` — shared by the `Display`
/// impls below (a plain `&[Vector<N, T>]` and a slice of those, e.g. a list
/// of polygons/holes).
fn fmt_vector_slice<T: Display>(items: &[T], f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[")?;
    for (i, v) in items.iter().enumerate() {
        write!(f, "{v}")?;
        if i + 1 < items.len() {
            write!(f, ", ")?;
        }
    }
    write!(f, "]")
}

/// A `Display`-only wrapper around `&[Vector<N, T>]` (or `&[Vec<Vector<T,
/// N>>]`, via `VectorSlice(&Vec::from(...))`-style nesting) — `Display`
/// can't be implemented directly on a foreign slice/`Vec` type, so callers
/// wanting to print a slice of vectors (e.g. a boundary loop or polygon)
/// go through this instead: `format!("{}", VectorSlice(&points))`.
pub struct VectorSlice<'a, T>(pub &'a [T]);

impl<'a, T: Number, const N: usize> Display for VectorSlice<'a, Vector<N, T>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt_vector_slice(self.0, f)
    }
}

impl<'a, T: Number, const N: usize> Display for VectorSlice<'a, Vec<Vector<N, T>>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[")?;
        for (i, v) in self.0.iter().enumerate() {
            fmt_vector_slice(v, f)?;
            if i + 1 < self.0.len() {
                write!(f, ", ")?;
            }
        }
        write!(f, "]")
    }
}

/// A vector serializes as the array of its components' midpoints — how it
/// travels to a viewer, which draws points, not enclosures — and reads back
/// from an array of plain numbers, each a sharp scalar.
impl<T: Number, const N: usize> serde::Serialize for Vector<N, T> {
    fn serialize<Ser: serde::Serializer>(&self, serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
        use serde::ser::SerializeTuple;
        let mut tuple = serializer.serialize_tuple(N)?;
        for x in &self.data {
            tuple.serialize_element(&x.to_f64())?;
        }
        tuple.end()
    }
}

impl<'de, T: Number, const N: usize> serde::Deserialize<'de> for Vector<N, T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Components<T, const N: usize>(std::marker::PhantomData<T>);

        impl<'de, T: Number, const N: usize> serde::de::Visitor<'de> for Components<T, N> {
            type Value = Vector<N, T>;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(f, "an array of {N} numbers")
            }

            fn visit_seq<A: serde::de::SeqAccess<'de>>(
                self,
                mut seq: A,
            ) -> Result<Self::Value, A::Error> {
                let mut out = Vector::zero();
                for k in 0..N {
                    let x: f64 = seq
                        .next_element()?
                        .ok_or_else(|| serde::de::Error::invalid_length(k, &self))?;
                    out[k] = T::from_f64(x);
                }
                if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                    return Err(serde::de::Error::invalid_length(N + 1, &self));
                }
                Ok(out)
            }
        }

        deserializer.deserialize_tuple(N, Components(std::marker::PhantomData))
    }
}

// tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector() {
        let mut v = Vector::<3>::new();
        v[0] = Scalar::from_i64(1);
        v[1] = Scalar::from_i64(2);
        v[2] = Scalar::from_i64(3);

        assert!(v[0].could_be_equal(Scalar::from_i64(1)));
        assert!(v[1].could_be_equal(Scalar::from_i64(2)));
        assert!(v[2].could_be_equal(Scalar::from_i64(3)));
    }

    #[test]
    fn test_vector_everything() {
        let v = Vector::<3>::everything();
        assert!(v[0].could_be_equal(Scalar::from_f64(42.0)));
        assert!(v[1].could_be_equal(Scalar::from_f64(-1e300)));
        assert!(v[2].could_be_equal(Scalar::from_i64(0)));
    }
}
