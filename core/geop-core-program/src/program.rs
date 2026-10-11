use crate::operation::Operation;
use std::any::Any;

pub struct ProgramStep {
    pub id: String,
    pub operation: &'static str,
    pub args: Box<dyn Any>,
}

impl ProgramStep {
    pub fn new<O: Operation>(id: impl Into<String>, args: O::Args) -> Self {
        Self {
            id: id.into(),
            operation: O::NAME,
            args: Box::new(args),
        }
    }
}

pub struct Program {
    pub steps: Vec<ProgramStep>,
}

impl Program {
    pub fn new(steps: Vec<ProgramStep>) -> Self {
        Self { steps }
    }
}
