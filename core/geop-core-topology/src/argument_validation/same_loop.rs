use crate::{CoedgeId, Model};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// Validates that `coedge2` lies on the loop traced by following `next` from
/// `coedge1` (i.e. they bound the same face boundary loop).
pub fn validate_same_loop<S: Scalar>(
    model: &Model<S>,
    coedge1: CoedgeId,
    coedge2: CoedgeId,
) -> GeopResult<()> {
    let mut cursor = coedge1;
    loop {
        if cursor == coedge2 {
            return Ok(());
        }
        cursor = model.get_coedge(cursor)?.next;
        if cursor == coedge1 {
            break;
        }
    }
    Err(GeopError::new(
        "coedge1 and coedge2 must belong to the same loop",
    ))
}
