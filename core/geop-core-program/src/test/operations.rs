//! The tests' toy operations. A [`Frame`] is a point, a target of its own
//! read whole: defined by one step ([`AddTwoFrameOperation`],
//! [`InputFrameOperation`], [`StateFrameOperation`],
//! [`MiddleFrameOperation`]) and revised by the steps after it
//! ([`ShiftFrameOperation`]). The [`Datums`] are the one target of their
//! kind a part has, which its steps revise one after another
//! ([`AddTwoDatums`], [`MiddleDatum`]), and a step builds on a datum by an
//! [`ElementReference`] into it.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{
    element::{ElementReference, HasElements, Kind},
    kept::Keeper,
    numeric::{ParameterValue, parameter},
    operation::Operation,
    part::StepContext,
    target::{Target, TargetReference},
    test::{label, record_built},
    values::{Setting, StateValue},
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    #[serde(with = "geop_core_math::exact")]
    pub data: Vector3,
}

/// Defined by one step, shifted by the ones after it; kept between
/// processes, written exactly. (The datums are not: what a kept frame reads
/// of them is made again in every process.)
impl Target for Frame {
    const CHAIN: bool = true;
    const KEEPER: Option<Keeper<Self>> = Some(Keeper::SERDE);
}

/// A frame is read whole: a kind of its own.
impl Kind for Frame {
    const NAME: &'static str = "frame";
}

impl HasElements<Frame> for Frame {
    type Element<'a> = &'a Frame;

    fn element(&self, name: &str) -> GeopResult<&Frame> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!("a frame has no element `{name}`"))),
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AddTwoFrameArgs {
    pub data1: Vector3,
    pub data2: Vector3,
}

/// Defines the targets `<step>_1` and `<step>_2` as frames at the two points.
pub struct AddTwoFrameOperation;

impl Operation for AddTwoFrameOperation {
    const KIND: &'static str = "add_two_frame";
    type Args = AddTwoFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        for (suffix, data) in [("_1", &args.data1), ("_2", &args.data2)] {
            let reference = TargetReference::<Frame>::new(step.to_string() + suffix);
            let args = (label(reference.name(), step), *data);
            ctx.produce(&reference, &args, |_, (label, data)| {
                record_built(label);
                Ok(Frame { data: *data })
            })?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MiddleFrameArgs {
    pub frame1: ElementReference<Frame, Frame>,
    pub frame2: ElementReference<Frame, Frame>,
}

/// Defines the step's target as the frame halfway between two others, as
/// each is in its own part.
pub struct MiddleFrameOperation;

impl Operation for MiddleFrameOperation {
    const KIND: &'static str = "middle_frame";
    type Args = MiddleFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label(step, step), args.clone());
        ctx.produce(
            &TargetReference::<Frame>::new(step),
            &args,
            |part, (label, args)| {
                record_built(label);
                let frame1 = part.retrieve_placed(&args.frame1)?.element;
                let frame2 = part.retrieve_placed(&args.frame2)?.element;
                let half = Scalar::from_ratio(1, 2)?;
                Ok(Frame {
                    data: frame1.data.add(&frame2.data).prod_scalar(half),
                })
            },
        )
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ShiftFrameArgs {
    pub targets: Vec<String>,
    pub by: Vector3,
}

/// Updates each of the targets `targets` by moving its frame `by`.
pub struct ShiftFrameOperation;

impl Operation for ShiftFrameOperation {
    const KIND: &'static str = "shift_frame";
    type Args = ShiftFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        for target in &args.targets {
            let reference = TargetReference::<Frame>::new(target);
            let args = (label(target, step), args.by);
            ctx.produce(&reference, &args, |reader, (label, by)| {
                record_built(label);
                let frame: Frame = reader.previous()?;
                Ok(Frame {
                    data: frame.data.add(by),
                })
            })?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct InputFrameArgs {
    pub target: String,
    pub input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
pub struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const KIND: &'static str = "input_frame";
    type Args = InputFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let reference = TargetReference::<Frame>::new(&args.target);
        let args = (label(&args.target, step), args.clone());
        ctx.produce(&reference, &args, |part, (label, args)| {
            record_built(label);
            let x = match part.retrieve_input(&args.input) {
                Some(Setting::Number(x)) => Scalar::from_f64(x),
                _ => {
                    return Err(GeopError::new(format!(
                        "input `{}` is not given",
                        args.input
                    )));
                }
            };
            Ok(Frame {
                data: Vector3::from_array([x, Scalar::ZERO, Scalar::ZERO]),
            })
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NumberArgs {
    pub name: String,
    pub value: f64,
}

/// Defines the parameter `name` (see [`crate::numeric::parameter`]): the
/// run's input of its name, given a number, else `value`.
pub struct NumberOperation;

impl Operation for NumberOperation {
    const KIND: &'static str = "number";
    type Args = NumberArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        ctx.produce(&parameter(&args.name), args, |part, args| {
            Ok(ParameterValue::Number(
                match part.retrieve_input(&args.name) {
                    Some(Setting::Number(value)) => value,
                    _ => args.value,
                },
            ))
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct StateFrameArgs {
    pub state: String,
}

/// Defines the step's target as the frame at the first three components of
/// the state `state`.
pub struct StateFrameOperation;

impl Operation for StateFrameOperation {
    const KIND: &'static str = "state_frame";
    type Args = StateFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label(step, step), args.clone());
        ctx.produce(
            &TargetReference::<Frame>::new(step),
            &args,
            |part, (label, args)| {
                record_built(label);
                let data = match part.retrieve_state(&args.state) {
                    StateValue::Pose(pose) => pose.position(),
                    StateValue::Number(_) => Vector3::zero(),
                };
                Ok(Frame { data })
            },
        )
    }
}

/// The datums of a part, by name: the one target of its kind a part has,
/// revised by every step that adds one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Datums(pub BTreeMap<String, Frame>);

impl Target for Datums {
    const CHAIN: bool = true;
}

/// The part's datums.
pub fn datums() -> TargetReference<Datums> {
    TargetReference::new("datums")
}

/// A datum, as an element of [`Datums`].
pub struct Datum;

impl Kind for Datum {
    const NAME: &'static str = "datum";
}

impl HasElements<Datum> for Datums {
    type Element<'a> = &'a Frame;
    const SINGLETON: Option<&'static str> = Some("datums");

    fn element(&self, name: &str) -> GeopResult<&Frame> {
        (self.0.get(name)).ok_or_else(|| GeopError::new(format!("there is no datum `{name}`")))
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AddTwoDatumsArgs {
    pub data1: Vector3,
    pub data2: Vector3,
}

/// Adds the datums `<step>_1` and `<step>_2` at the two points.
pub struct AddTwoDatums;

impl Operation for AddTwoDatums {
    const KIND: &'static str = "add_two_datums";
    type Args = AddTwoDatumsArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label("datums", step), step.to_string(), args.clone());
        ctx.produce(&datums(), &args, |reader, (label, step, args)| {
            record_built(label);
            let mut datums: Datums = reader.previous()?;
            for (suffix, data) in [("_1", args.data1), ("_2", args.data2)] {
                datums.0.insert(format!("{step}{suffix}"), Frame { data });
            }
            Ok(datums)
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MiddleDatumArgs {
    pub a: ElementReference<Datums, Datum>,
    pub b: ElementReference<Datums, Datum>,
}

/// Adds the datum `<step>` halfway between the datums `a` and `b`.
pub struct MiddleDatum;

impl Operation for MiddleDatum {
    const KIND: &'static str = "middle_datum";
    type Args = MiddleDatumArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label("datums", step), step.to_string(), args.clone());
        ctx.produce(&datums(), &args, |reader, (label, step, args)| {
            record_built(label);
            let mut datums: Datums = reader.previous()?;
            // Read through the reader, as of the steps before: the datums
            // this step revises.
            let a = reader.retrieve_element(&args.a)?.data;
            let b = reader.retrieve_element(&args.b)?.data;
            let half = Scalar::from_ratio(1, 2)?;
            let data = a.add(&b).prod_scalar(half);
            datums.0.insert(step.clone(), Frame { data });
            Ok(datums)
        })
    }
}
