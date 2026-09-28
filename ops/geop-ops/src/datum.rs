//! A [`Part`]'s datums (see [`Datum`]): reference geometry the part is
//! built *with*, not *of*. A sketch is placed on a datum plane, a datum
//! axis is picked as a direction, a datum is built from another one. Named
//! like any other entity.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Datum,
    scalars::Scalar,
};

use crate::ids::DatumId;
use crate::part::Part;

impl<S: Scalar> Part<S> {
    /// Adds `datum` to the part under `name`. Fails, leaving the part
    /// unchanged, if `name` is already taken.
    pub fn add_datum(&mut self, datum: Datum<S>, name: impl Into<String>) -> GeopResult<DatumId> {
        let id = DatumId(self.fresh_id());
        self.names.insert(id, name)?;
        self.datums.insert(id, datum);
        Ok(id)
    }

    pub fn datum(&self, id: DatumId) -> GeopResult<&Datum<S>> {
        self.datums
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("Part has no datum {id}")))
    }

    /// Every datum, in the order they were added.
    pub fn datums(&self) -> impl Iterator<Item = (DatumId, &Datum<S>)> {
        self.datums.iter().map(|(&id, d)| (id, d))
    }
}
