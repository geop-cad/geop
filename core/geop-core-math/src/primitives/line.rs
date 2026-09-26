use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

pub struct Line<S: Scalar> {
    start: Vector3<S>,
    end: Vector3<S>,
}

impl<S: Scalar> core::fmt::Debug for Line<S> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Line({:?} → {:?})", self.start, self.end)
    }
}

impl<S: Scalar> Line<S> {
    /// Fails if `start` and `end` could be equal (zero-length segment uncertain).
    pub fn try_new(start: Vector3<S>, end: Vector3<S>) -> GeopResult<Self> {
        let d = end.sub(&start);
        if d.norm_sq().could_be_equal(S::ZERO) {
            return Err(GeopError::new(
                "Line::try_new: start and end could be equal (degenerate segment)",
            ));
        }
        Ok(Self { start, end })
    }

    pub fn start(&self) -> &Vector3<S> {
        &self.start
    }
    pub fn end(&self) -> &Vector3<S> {
        &self.end
    }

    pub fn direction(&self) -> Vector3<S> {
        self.end.sub(&self.start)
    }

    pub fn length_sq(&self) -> S {
        let d = self.direction();
        d.norm_sq()
    }
}
