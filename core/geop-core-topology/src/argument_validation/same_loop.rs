use crate::{CoedgeId, Model};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::walk::{Walked, walk},
};

/// Validates that `coedge2` lies on the loop traced by following `next` from
/// `coedge1` (i.e. they bound the same face boundary loop).
pub fn validate_same_loop<S: Scalar>(
    model: &Model<S>,
    coedge1: CoedgeId,
    coedge2: CoedgeId,
) -> GeopResult<()> {
    // Round the loop from `coedge1`, ending where `coedge2` is.
    let around = walk(coedge1, |&c| {
        Ok::<_, GeopError>(if c == coedge2 {
            None
        } else {
            Some(model.get_coedge(c)?.next)
        })
    })?;
    match around.end {
        Walked::Open => return Ok(()),
        Walked::Closed => {}
        Walked::Revisited(c) => {
            return Err(GeopError::new(format!(
                "the loop of coedge1 runs into itself at coedge {c}"
            )));
        }
    }
    Err(GeopError::new(
        "coedge1 and coedge2 must belong to the same loop",
    ))
}
