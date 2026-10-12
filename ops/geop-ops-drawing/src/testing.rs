//! What the drawing's tests share besides `geop_ops::testing`: parts to
//! place.

use std::{collections::BTreeSet, rc::Rc};

use geop_ops::{Design, NamedModel, parameters::Parameters, placed::Component, testing::built};

/// A part's model, as the tests build it.
pub(crate) type Part<S> = NamedModel<S>;

/// `bodies` as the part of the program file `file`, to place.
pub(crate) fn component(file: &str, bodies: &NamedModel<Design>) -> Rc<Component> {
    Rc::new(Component::new(
        file.into(),
        BTreeSet::from([file.to_string()]),
        file.into(),
        Parameters::default(),
        built(bodies),
    ))
}
