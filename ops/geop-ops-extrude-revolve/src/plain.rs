//! Frames in plain numbers, for the choices a sweep or a loft is free to
//! make — sections along a spline, twists, guide shapes: worked out in
//! `f64` (see [`geop_core_math::plain`]), and taken as sharp scalars once
//! made, the way a subdivision point is.

use geop_core_math::{
    plain::{P3, add, scale, sub},
    scalars::Scalar,
    vector::Vector3,
};

use crate::sweep::Frame;

/// A [`Frame`] in plain numbers, or a frame's derivative.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Plain {
    pub origin: P3,
    pub e1: P3,
    pub e2: P3,
}

impl Plain {
    pub(crate) fn of<S: Scalar>(frame: &Frame<S>) -> Self {
        Plain {
            origin: frame.origin.to_f64_array(),
            e1: frame.e1.to_f64_array(),
            e2: frame.e2.to_f64_array(),
        }
    }

    /// The frame, taken as sharp: a free choice made in plain numbers.
    pub(crate) fn frame<S: Scalar>(&self) -> Frame<S> {
        let v = |a: P3| Vector3::from_array(a.map(S::from_f64));
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
