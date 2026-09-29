pub mod linalg;

use std::{
    fmt::Display,
    ops::{Index, IndexMut},
};

use crate::scalars::Scalar;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector<S, const N: usize> {
    data: [S; N],
}

pub type Vector4<S> = Vector<S, 4>;
pub type Vector3<S> = Vector<S, 3>;
pub type Vector2<S> = Vector<S, 2>;

/// Type alias for backwards compatibility with older code.
pub type VecN<S, const N: usize> = Vector<S, N>;

impl<S: Default + Copy, const N: usize> Vector<S, N> {
    pub fn new() -> Self {
        Self {
            data: [S::default(); N],
        }
    }

    pub fn size(&self) -> usize {
        N
    }
}

impl<S: Scalar, const N: usize> Vector<S, N> {
    pub fn from_array(data: [S; N]) -> Self {
        Self { data }
    }

    /// Element access using a multi-index slice; only the first index is used
    /// (vectors are 1-D).  Panics if `idx` is empty.
    pub fn get(&self, idx: &[usize]) -> S {
        self.data[idx[0]]
    }
}

impl<S, const N: usize> Index<usize> for Vector<S, N> {
    type Output = S;

    fn index(&self, idx: usize) -> &Self::Output {
        &self.data[idx]
    }
}

impl<S: Default + Copy, const N: usize> IndexMut<usize> for Vector<S, N> {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        &mut self.data[idx]
    }
}

// swap xy
impl<S: Scalar> Vector<S, 2> {
    pub fn swap_xy(&self) -> Self {
        Self::from_array([self[1], self[0]])
    }
}

// zero
impl<S: Scalar, const N: usize> Vector<S, N> {
    pub fn zero() -> Self {
        Self { data: [S::ZERO; N] }
    }

    /// Every component set to [`Scalar::ENTIRE`] — `could_be_equal`s any
    /// other vector of the same dimension componentwise.
    pub fn everything() -> Self {
        Self {
            data: [S::ENTIRE; N],
        }
    }

    /// The `k`-th coordinate axis: one at `k`, zero elsewhere.
    pub fn axis(k: usize) -> Self {
        let mut out = Self::zero();
        out[k] = S::ONE;
        out
    }

    /// The first `M` components — e.g. the Cartesian part `H` of a
    /// homogeneous point `(H, w)`, which is *not* its position `H / w`.
    pub fn head<const M: usize>(&self) -> Vector<S, M> {
        Vector::from_array(std::array::from_fn(|k| self.data[k]))
    }
}

impl<S: Scalar, const N: usize> Display for Vector<S, N> {
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
/// impls below (a plain `&[Vector<S, N>]` and a slice of those, e.g. a list
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

/// A `Display`-only wrapper around `&[Vector<S, N>]` (or `&[Vec<Vector<S,
/// N>>]`, via `VectorSlice(&Vec::from(...))`-style nesting) — `Display`
/// can't be implemented directly on a foreign slice/`Vec` type, so callers
/// wanting to print a slice of vectors (e.g. a boundary loop or polygon)
/// go through this instead: `format!("{}", VectorSlice(&points))`.
pub struct VectorSlice<'a, T>(pub &'a [T]);

impl<'a, S: Scalar, const N: usize> Display for VectorSlice<'a, Vector<S, N>> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt_vector_slice(self.0, f)
    }
}

impl<'a, S: Scalar, const N: usize> Display for VectorSlice<'a, Vec<Vector<S, N>>> {
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

// tests
#[cfg(test)]
mod tests {
    use crate::scalars::{ScalInF64, Scalar};

    use super::*;

    #[test]
    fn test_vector() {
        let mut v = Vector::<ScalInF64, 3>::new();
        v[0] = 1.into();
        v[1] = 2.into();
        v[2] = 3.into();

        assert!(v[0].could_be_equal(1.into()));
        assert!(v[1].could_be_equal(2.into()));
        assert!(v[2].could_be_equal(3.into()));
    }

    #[test]
    fn test_vector_everything() {
        let v = Vector::<ScalInF64, 3>::everything();
        assert!(v[0].could_be_equal(42.0.into()));
        assert!(v[1].could_be_equal((-1e300).into()));
        assert!(v[2].could_be_equal(0.into()));
    }
}

/// A vector serializes as the array of its components' midpoints — how it
/// travels to a viewer, which draws points, not enclosures — and reads back
/// from an array of plain numbers, each a sharp scalar.
impl<S: Scalar, const N: usize> serde::Serialize for Vector<S, N> {
    fn serialize<Ser: serde::Serializer>(&self, serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
        use serde::ser::SerializeTuple;
        let mut tuple = serializer.serialize_tuple(N)?;
        for x in &self.data {
            tuple.serialize_element(&x.to_f64())?;
        }
        tuple.end()
    }
}

impl<'de, S: Scalar, const N: usize> serde::Deserialize<'de> for Vector<S, N> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Components<S, const N: usize>(std::marker::PhantomData<S>);

        impl<'de, S: Scalar, const N: usize> serde::de::Visitor<'de> for Components<S, N> {
            type Value = Vector<S, N>;

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
                    out[k] = S::from_f64(x);
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
