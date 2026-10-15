//! What the sheet-metal steps record of the bodies they build — the
//! part's target [`sheets`], which every one of them revises — and how a
//! step of them runs ([`run`]): it works out what it makes ([`made`]),
//! records it, and builds it into the model.
//!
//! A body's [`Sheet`] says which faces are flat and which are bends, and
//! how they join: so the next flange and the flat pattern work from what
//! the body is made of, never from guessing it from its geometry.

use geop_core_editor::part_operation::PartOperation;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    target::TargetReference,
};
use geop_ops_model::{
    Design, ModelOperation, NamedModel, Namer, Records, SolidNotes, build_model, solid_notes,
};
use geop_ops_parameters::{BuildPart, PartReader, Setting, StateValue};
use geop_ops_sketch::{PlacedSketch, sketch};

use crate::{edge_flange::EdgeRef, flat_pattern::FlatPatternData, sheet::Sheet};

/// What a sheet-metal body is recorded as.
#[derive(Clone, Debug, PartialEq)]
pub enum SheetBody {
    /// A body as it is bent: its sheet.
    Bent(Sheet<Design>),
    /// The flat pattern of one: what a drawing and cutting read of it.
    Flat(FlatPatternData),
}

impl SheetBody {
    /// Its sheet, if it is a body as it is bent.
    pub fn bent(&self) -> Option<&Sheet<Design>> {
        match self {
            SheetBody::Bent(sheet) => Some(sheet),
            SheetBody::Flat(_) => None,
        }
    }

    /// How thick its sheet is.
    pub fn thickness(&self) -> f64 {
        match self {
            SheetBody::Bent(sheet) => sheet.rules.thickness,
            SheetBody::Flat(flat) => flat.thickness,
        }
    }
}

/// The part's sheet-metal bodies, by the names of their solids: the value
/// of its target [`sheets`].
pub type Sheets = Records<SheetBody>;

/// The part's sheet-metal bodies.
pub fn sheets() -> TargetReference<Sheets> {
    TargetReference::new("sheets")
}

/// A sheet-metal body, by its solid's name: an element of [`sheets`].
impl Kind for SheetBody {
    const NAME: &'static str = "sheet_body";
}

impl HasElements<SheetBody> for Sheets {
    type Element<'a> = &'a SheetBody;
    const SINGLETON: Option<&'static str> = Some("sheets");

    fn element(&self, name: &str) -> GeopResult<&SheetBody> {
        self.get(name).ok_or_else(|| {
            GeopError::new(format!(
                "{name:?} is no sheet-metal body: none was recorded for it"
            ))
        })
    }
}

/// The sheet-metal body of the solid `solid` in the part `reader` reads,
/// if it is one: none before a step built one.
pub fn sheet_body<'a>(reader: &PartReader<'a>, solid: &str) -> Option<&'a SheetBody> {
    reader.defines(&sheets()).then_some(())?;
    reader
        .retrieve_element(&ElementReference::new(sheets(), solid))
        .ok()
}

/// A sheet-metal body found by an edge: its solid, its sheet, and the
/// edge as [`Sheet::edge_named`] finds it.
pub(crate) type WithEdge = (String, Sheet<Design>, (usize, usize, bool));

/// The sheet-metal body of `model`, in the part `reader` reads, with the
/// edge `edge` on a flat's outline: its solid, its sheet, and the edge as
/// [`Sheet::edge_named`] finds it. `what` is for the error: what is bent
/// from such an edge.
pub(crate) fn with_edge(
    model: &NamedModel<Design>,
    reader: &PartReader,
    edge: Option<&EdgeRef>,
    what: &str,
) -> GeopResult<WithEdge> {
    let edge = edge
        .ok_or_else(|| GeopError::new(format!("no edge is picked: {what} is bent from one")))?
        .name();
    (model.solid_names().into_iter())
        .find_map(|solid| {
            let sheet = sheet_body(reader, &solid)?.bent()?;
            let at = sheet.edge_named(edge)?;
            Some((solid, sheet.clone(), at))
        })
        .ok_or_else(|| {
            GeopError::new(format!(
                "{edge:?} is no edge of a sheet-metal body's flat face on its outline: {what} is bent from one of those"
            ))
        })
}

/// What a sheet-metal step builds.
#[derive(Clone, Debug, PartialEq)]
pub enum Making {
    /// A body as it is bent, from its sheet.
    Bent(Sheet<Design>),
    /// The flat pattern of a body.
    Flat(Box<Unfolded>),
}

/// The flat pattern of the body of `sheet`: the sheet unfolded, the sketch
/// of its bend lines, and what it records.
#[derive(Clone, Debug, PartialEq)]
pub struct Unfolded {
    pub sheet: Sheet<Design>,
    pub bend_lines: PlacedSketch<Design>,
    pub data: FlatPatternData,
}

/// What a sheet-metal step makes: the body `making`, the solid `solid`, in
/// place of the sheet-metal body `consumes`, if any.
#[derive(Clone, Debug, PartialEq)]
pub struct Made {
    pub consumes: Option<String>,
    pub solid: String,
    pub making: Making,
}

/// What the step `step` makes.
pub fn made(step: &str) -> TargetReference<Made> {
    TargetReference::new(format!("sheet_metal:{step}"))
}

/// Read whole.
impl Kind for Made {
    const NAME: &'static str = "sheet_metal";
}

impl HasElements<Made> for Made {
    type Element<'a> = &'a Made;

    fn element(&self, name: &str) -> GeopResult<&Made> {
        whole(self, name)
    }
}

/// What the step `step` made, as `reader` reads it.
fn made_by<'a>(reader: &PartReader<'a>, step: &str) -> GeopResult<&'a Made> {
    reader.retrieve_element(&ElementReference::whole(made(step)))
}

/// A sheet-metal operation: it works out what it makes from the part read
/// — the sketches, the sheet-metal bodies there are — and is then built
/// alike ([`run`]).
pub(crate) trait SheetStep:
    PartOperation<Input = Setting, State = StateValue> + ModelOperation
{
    /// What the step `step` with `args` makes, as `reader` reads the part.
    fn plan(reader: &PartReader, step: &str, args: &Self::Args) -> GeopResult<Made>;
}

/// The generator of [`made`].
fn planned<O: SheetStep>(
    reader: &PartReader,
    (step, args): &(String, O::Args),
) -> GeopResult<Made> {
    O::plan(reader, step, args)
}

/// Records what the step `step` made: its body in place of the one it
/// consumed.
// A generator's arguments are a type of their own: the step's id, a `String`.
#[allow(clippy::ptr_arg)]
fn record(reader: &PartReader, step: &String) -> GeopResult<Sheets> {
    let mut sheets: Sheets = reader.previous()?;
    let made = made_by(reader, step)?;
    if let Some(consumed) = &made.consumes {
        sheets.remove(consumed);
    }
    let body = match &made.making {
        Making::Bent(sheet) => SheetBody::Bent(sheet.clone()),
        Making::Flat(unfolded) => SheetBody::Flat(unfolded.data.clone()),
    };
    sheets.insert(made.solid.clone(), body);
    Ok(sheets)
}

/// Notes the thickness of what the step `step` made, in place of the body
/// it consumed (see [`geop_ops_model::solid_notes`]).
#[allow(clippy::ptr_arg)]
fn note(reader: &PartReader, step: &String) -> GeopResult<SolidNotes> {
    let mut notes: SolidNotes = reader.previous()?;
    let made = made_by(reader, step)?;
    if let Some(consumed) = &made.consumes {
        notes.remove(consumed);
    }
    let thickness = match &made.making {
        Making::Bent(sheet) => sheet.rules.thickness,
        Making::Flat(unfolded) => unfolded.data.thickness,
    };
    notes.note_mut(made.solid.clone()).thickness = Some(thickness);
    Ok(notes)
}

/// Runs the step `step` of `O`: works out what it makes, records it, and
/// builds it into the model.
pub(crate) fn run<O: SheetStep>(
    step: &str,
    part: &mut BuildPart,
    args: &O::Args,
) -> GeopResult<()> {
    part.define_target(&made(step), &(step.to_string(), args.clone()), planned::<O>)?;
    part.revise_target(&sheets(), &step.to_string(), record)?;
    part.revise_target(&solid_notes(), &step.to_string(), note)?;
    build_model::<O>(part, step, args)
}

/// The sketch of the bend lines of the flat pattern the step `step` made.
#[allow(clippy::ptr_arg)]
fn bend_lines(reader: &PartReader, step: &String) -> GeopResult<PlacedSketch<Design>> {
    match &made_by(reader, step)?.making {
        Making::Flat(unfolded) => Ok(unfolded.bend_lines.clone()),
        Making::Bent(_) => Err(GeopError::new(format!(
            "step {step:?} made no flat pattern to draw the bend lines of"
        ))),
    }
}

/// Adds the sketch `name` of the bend lines of the flat pattern the step
/// `step` made.
pub(crate) fn add_bend_lines(part: &mut BuildPart, step: &str, name: &str) -> GeopResult<()> {
    part.define_target(&sketch(name), &step.to_string(), bend_lines)
}

/// Builds what the step `step` made into `model` ([`ModelOperation::build`]
/// of every sheet-metal operation).
pub(crate) fn build(
    model: &mut NamedModel<Design>,
    reader: &PartReader,
    step: &str,
) -> GeopResult<()> {
    let ctx = with_context!("building the sheet metal of step {step}");
    let made = made_by(reader, step).with_context(ctx)?;
    match &made.making {
        Making::Bent(sheet) => sheet
            .build(model, made.consumes.as_deref(), &made.solid)
            .with_context(ctx),
        Making::Flat(unfolded) => {
            let namer = Namer::new("flat_pattern", step)?;
            crate::flat_pattern::build(model, &unfolded.sheet, made.consumes.as_deref(), &namer)
                .with_context(ctx)
        }
    }
}
