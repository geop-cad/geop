use crate::{CoedgeId, Model, argument_validation::same_loop::validate_same_loop};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// Validates that `coedge1` and `coedge2` lie on different boundary loops
/// (the inverse precondition of [`validate_same_loop`]).
pub fn validate_different_loop<S: Scalar>(
    model: &Model<S>,
    coedge1: CoedgeId,
    coedge2: CoedgeId,
) -> GeopResult<()> {
    if validate_same_loop(model, coedge1, coedge2).is_ok() {
        return Err(GeopError::new(
            "coedge1 and coedge2 must belong to different loops",
        ));
    }
    Ok(())
}
