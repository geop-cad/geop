//! Vectors and frames in plain numbers, for the choices a sweep or a loft
//! is free to make — sections along a spline, twists, guide shapes: worked
//! out in `f64`, and taken as sharp scalars once made, the way a
//! subdivision point is.

use geop_core_math::{scalars::Scalar, vector::Vector3};

use crate::sweep::Frame;

pub(crate) type V = [f64; 3];

pub(crate) fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub(crate) fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn scale(a: V, k: f64) -> V {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub(crate) fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub(crate) fn unit(a: V) -> V {
    scale(a, 1.0 / dot(a, a).sqrt())
}
pub(crate) fn plain<S: Scalar>(v: &Vector3<S>) -> V {
    [v[0].to_f64(), v[1].to_f64(), v[2].to_f64()]
}

/// A [`Frame`] in plain numbers, or a frame's derivative.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Plain {
    pub origin: V,
    pub e1: V,
    pub e2: V,
}

impl Plain {
    pub(crate) fn of<S: Scalar>(frame: &Frame<S>) -> Self {
        Plain {
            origin: plain(&frame.origin),
            e1: plain(&frame.e1),
            e2: plain(&frame.e2),
        }
    }

    /// The frame, taken as sharp: a free choice made in plain numbers.
    pub(crate) fn frame<S: Scalar>(&self) -> Frame<S> {
        let v = |a: V| Vector3::from_array(a.map(S::from_f64));
        Frame {
            origin: v(self.origin),
            e1: v(self.e1),
            e2: v(self.e2),
        }
    }

    /// `self + h derivative`.
    pub(crate) fn step(&self, derivative: &Plain, h: f64) -> Self {
        Plain {
            origin: add(self.origin, scale(derivative.origin, h)),
            e1: add(self.e1, scale(derivative.e1, h)),
            e2: add(self.e2, scale(derivative.e2, h)),
        }
    }

    /// `self - other`: how far `other` is from `self`, as a derivative.
    pub(crate) fn minus(&self, other: &Plain) -> Self {
        Plain {
            origin: sub(self.origin, other.origin),
            e1: sub(self.e1, other.e1),
            e2: sub(self.e2, other.e2),
        }
    }
}
