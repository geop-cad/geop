//! [`Exact`]: values written as the kernel holds them — every enclosure's
//! two bounds, to the bit — and read back the same, for what is kept to be
//! read again: a cache of what a file was made into
//! (`geop_core_program::cache`).
//!
//! The plain serde of a [`Scalar`] or a [`Vector`] is its midpoint
//! ([`crate::scalars::as_f64`]), which is what a viewer draws. Read back,
//! a midpoint is another value: a part built from it is not the part that
//! was kept. Whatever is written to be read again as itself is written
//! with [`Exact`], a field of it `#[serde(with = "geop_core_math::exact")]`.

use serde::{
    Deserialize, Deserializer, Serialize, Serializer,
    de::{Error, SeqAccess, Visitor},
    ser::{SerializeSeq, SerializeTuple},
};

use crate::{scalars::Scalar, vector::Vector};

/// A value written exactly ([`Exact::write`]) and read back as written
/// ([`Exact::read`]).
pub trait Exact: Sized {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error>;
    fn read<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error>;
}

/// `#[serde(with = "geop_core_math::exact")]`: the field written exactly.
pub fn serialize<T: Exact, S: Serializer>(value: &T, serializer: S) -> Result<S::Ok, S::Error> {
    value.write(serializer)
}

/// `#[serde(with = "geop_core_math::exact")]`: the field read as written.
pub fn deserialize<'de, T: Exact, D: Deserializer<'de>>(deserializer: D) -> Result<T, D::Error> {
    T::read(deserializer)
}

/// `value`, to be serialized exactly as an element of something else.
pub struct Written<'a, T>(pub &'a T);

impl<T: Exact> Serialize for Written<'_, T> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.0.write(serializer)
    }
}

/// A value read as written, as an element of something else.
pub struct Read<T>(pub T);

impl<'de, T: Exact> Deserialize<'de> for Read<T> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        T::read(deserializer).map(Read)
    }
}

/// A sharp scalar as its number, any other as `[lo, hi]`. Sharp to the
/// bit: `[-0, 0]` is not sharp, and is written as both.
impl Exact for Scalar {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        if self.lo.to_bits() == self.hi.to_bits() {
            return serializer.serialize_f64(self.lo);
        }
        let mut pair = serializer.serialize_tuple(2)?;
        pair.serialize_element(&self.lo)?;
        pair.serialize_element(&self.hi)?;
        pair.end()
    }

    fn read<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Bounds;

        impl<'de> Visitor<'de> for Bounds {
            type Value = Scalar;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(f, "a number, or its bounds [lo, hi]")
            }

            fn visit_f64<E: Error>(self, x: f64) -> Result<Scalar, E> {
                Ok(Scalar { lo: x, hi: x })
            }

            #[expect(
                clippy::cast_precision_loss,
                reason = "a whole number written for an f64 was that f64"
            )]
            fn visit_i64<E: Error>(self, x: i64) -> Result<Scalar, E> {
                self.visit_f64(x as f64)
            }

            #[expect(
                clippy::cast_precision_loss,
                reason = "a whole number written for an f64 was that f64"
            )]
            fn visit_u64<E: Error>(self, x: u64) -> Result<Scalar, E> {
                self.visit_f64(x as f64)
            }

            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Scalar, A::Error> {
                let lo: f64 = seq
                    .next_element()?
                    .ok_or_else(|| A::Error::invalid_length(0, &self))?;
                let hi: f64 = seq
                    .next_element()?
                    .ok_or_else(|| A::Error::invalid_length(1, &self))?;
                if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                    return Err(A::Error::invalid_length(3, &self));
                }
                // Bounds out of order, or not numbers at all.
                if lo.partial_cmp(&hi).is_none_or(std::cmp::Ordering::is_gt) {
                    return Err(A::Error::custom(format!("[{lo}, {hi}] is no interval")));
                }
                Ok(Scalar { lo, hi })
            }
        }

        deserializer.deserialize_any(Bounds)
    }
}

/// Its components, each exactly.
impl<const N: usize> Exact for Vector<N> {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut tuple = serializer.serialize_tuple(N)?;
        for k in 0..N {
            tuple.serialize_element(&Written(&self[k]))?;
        }
        tuple.end()
    }

    fn read<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Components<const N: usize>;

        impl<'de, const N: usize> Visitor<'de> for Components<N> {
            type Value = Vector<N>;

            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(f, "an array of {N} scalars")
            }

            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Vector<N>, A::Error> {
                let mut out = Vector::zero();
                for k in 0..N {
                    let Read(x) = seq
                        .next_element()?
                        .ok_or_else(|| A::Error::invalid_length(k, &self))?;
                    out[k] = x;
                }
                if seq.next_element::<serde::de::IgnoredAny>()?.is_some() {
                    return Err(A::Error::invalid_length(N + 1, &self));
                }
                Ok(out)
            }
        }

        deserializer.deserialize_tuple(N, Components::<N>)
    }
}

/// Its elements, each exactly.
impl<T: Exact> Exact for Vec<T> {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(Some(self.len()))?;
        for x in self {
            seq.serialize_element(&Written(x))?;
        }
        seq.end()
    }

    fn read<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let read = Vec::<Read<T>>::deserialize(deserializer)?;
        Ok(read.into_iter().map(|Read(x)| x).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct Kept {
        #[serde(with = "crate::exact")]
        points: Vec<Vector<2>>,
    }

    /// Sharp scalars, wide ones, and a zero of two signs read back to the
    /// bit, a sharp one written as a plain number.
    #[test]
    fn scalars_read_back_as_written() {
        let third = Scalar::from_f64(1.0).div(Scalar::from_f64(3.0)).unwrap();
        let kept = Kept {
            points: vec![
                Vector::from_array([Scalar::from_f64(0.1), third]),
                Vector::from_array([Scalar { lo: -0.0, hi: 0.0 }, Scalar::from_f64(-2.0)]),
            ],
        };
        let text = serde_json::to_string(&kept).unwrap();
        assert!(text.starts_with("{\"points\":[[0.1,["), "{text}");
        let back: Kept = serde_json::from_str(&text).unwrap();
        let bits = |k: &Kept| -> Vec<(u64, u64)> {
            (k.points.iter())
                .flat_map(|p| (0..2).map(move |i| (p[i].lo.to_bits(), p[i].hi.to_bits())))
                .collect()
        };
        assert_eq!(bits(&back), bits(&kept));
    }

    #[test]
    fn bounds_out_of_order_are_refused() {
        assert!(serde_json::from_str::<Kept>("{\"points\":[[[2.0,1.0],0]]}").is_err());
    }
}
