pub mod add_two_frame;
pub mod middle_frame;

#[derive(Debug)]
pub struct Frame {
    pub data: [f64; 3],
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use crate::{
        operation::Registered,
        program::{Program, ProgramRunner, ProgramStep},
        test::{
            add_two_frame::{AddTwoFrameArgs, AddTwoFrameOperation},
            middle_frame::{MiddleFrameArgs, MiddleFrameOperation},
        },
    };

    use super::*;

    #[test]
    fn test_frame_creation() -> Result<(), Box<dyn std::error::Error>> {
        let frame1 = Frame {
            data: [0.0, 0.0, 0.0],
        };
        let frame2 = Frame {
            data: [0.0, 0.0, 1.0],
        };

        let add_two_frame = Registered::<AddTwoFrameOperation>::boxed();
        let middle_frame = Registered::<MiddleFrameOperation>::boxed();
        // TODO: Operations that depend on other Operations should register them in the constructor so they are definetly available

        let mut runner = ProgramRunner::new(BTreeMap::from([
            ("add_two_frame".to_string(), add_two_frame),
            ("middle_frame".to_string(), middle_frame),
        ]));

        // TODO: Make it typesafe so that program steps can only accept the correct argument types for each operation
        let program = Program::new(vec![
            ProgramStep {
                id: "step1".to_string(),
                operation_name: "add_two_frame".to_string(),
                args: Box::new(AddTwoFrameArgs {
                    data1: frame1.data,
                    data2: frame2.data,
                }), // Replace with actual arguments
            },
            ProgramStep {
                id: "step2".to_string(),
                operation_name: "middle_frame".to_string(),
                args: Box::new(MiddleFrameArgs {
                    frame1ref: "step1_1".to_string(),
                    frame2ref: "step1_2".to_string(),
                }), // Replace with actual arguments for middle_frame
            },
        ]);

        println!("First run");
        runner.run(&program, &BTreeMap::new(), &BTreeMap::new())?;

        println!("Second run");
        runner.run(&program, &BTreeMap::new(), &BTreeMap::new())?;

        // change frame1
        let frame1 = Frame {
            data: [1.0, 0.0, 0.0],
        };
        let program = Program::new(vec![
            ProgramStep {
                id: "step1".to_string(),
                operation_name: "add_two_frame".to_string(),
                args: Box::new(AddTwoFrameArgs {
                    data1: frame1.data,
                    data2: frame2.data,
                }), // Replace with actual arguments
            },
            ProgramStep {
                id: "step2".to_string(),
                operation_name: "middle_frame".to_string(),
                args: Box::new(MiddleFrameArgs {
                    frame1ref: "step1_1".to_string(),
                    frame2ref: "step1_2".to_string(),
                }), // Replace with actual arguments for middle_frame
            },
        ]);

        println!("Third run after changing frame1");
        runner.run(&program, &BTreeMap::new(), &BTreeMap::new())?;

        Ok(())
    }
}
