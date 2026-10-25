//! [`AddSketch`]: place a sketch in the part.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::CanvasEvent,
    part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader};
use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    with_context,
};
use geop_core_sketch::ConstraintId;
use geop_ops_model::sketch::{PlacedSketch, Sketch, SketchPlane, sketch};
use serde::{Deserialize, Serialize};

use crate::{
    constraints::{Pick, set_value},
    editor::{self, SketchSession},
    references::{Reference, Source},
};

/// Adds a sketch on a plane to the part, named by the operation's id: a
/// planar face, a datum plane or a frame's plane (see [`SketchPlane`]). The plane is
/// resolved when the sketch is added: a sketch on a face stays where the
/// face was, whatever later operations do to the face.
///
/// What the sketch projects of the part (see [`crate::references`]) and
/// the formulas its dimensions are given by are brought up to date with
/// the part every time it is built, so the sketch follows both.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketch;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketchArgs {
    /// The plane to sketch on: a planar face, a datum plane or a frame's
    /// plane; none yet, for a new sketch waiting for one to be picked.
    #[serde(default)]
    pub plane: Option<SketchPlane>,
    /// The sketch as drawn; solve it first for its constraints to hold.
    pub sketch: Sketch,
    /// Its reference geometry: its own origin and axes, and what it
    /// projects of the part.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub references: Vec<Reference>,
    /// The dimensions given by a formula of the part's parameters rather
    /// than a number, by constraint — an angle's in degrees.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub formulas: BTreeMap<ConstraintId, NumericInput>,
    /// Where each dimension placed by the designer shows its value: an
    /// offset in the plane from what it measures, so it moves with the
    /// geometry. Only what the editor draws; nothing built reads it.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub labels: BTreeMap<ConstraintId, [f64; 2]>,
}

impl AddSketchArgs {
    /// A new sketch on `plane`, with nothing in it but its own origin and
    /// axes.
    pub fn new(plane: Option<SketchPlane>) -> Self {
        let mut args = AddSketchArgs {
            plane,
            sketch: Sketch::new(),
            references: Vec::new(),
            formulas: BTreeMap::new(),
            labels: BTreeMap::new(),
        };
        args.ensure_frame();
        args
    }

    /// `sketch`, drawn in code, on `plane` — without its own origin and
    /// axes, which [`AddSketchArgs::ensure_frame`] adds once it is edited.
    pub fn on(plane: SketchPlane, sketch: Sketch) -> Self {
        Self {
            plane: Some(plane),
            sketch,
            ..Default::default()
        }
    }

    /// The reference of the sketch's own origin and axes, added if the
    /// sketch has none yet: one put together from its parts, as a program
    /// written in code does, rather than by [`AddSketchArgs::new`].
    pub fn ensure_frame(&mut self) {
        if !self.references.iter().any(|r| r.source == Source::Frame) {
            let frame = Reference::frame(&mut self.sketch);
            self.references.insert(0, frame);
        }
    }

    /// The reference `pick` belongs to, by index, if any.
    pub fn reference_of(&self, pick: Pick) -> Option<usize> {
        self.references.iter().position(|r| match pick {
            Pick::Point(p) => r.has_point(p),
            Pick::Curve(c) => r.has_curve(c),
        })
    }

    /// Brings what the sketch projects up to date with the part `reader`
    /// reads, the sketch lying in `plane` (see [`Reference::update`]).
    pub fn update_references(
        &mut self,
        reader: &Reader,
        plane: &CoordinateSystem,
    ) -> GeopResult<()> {
        for reference in &mut self.references {
            reference.update(&mut self.sketch, reader, plane)?;
        }
        Ok(())
    }

    /// Gives every dimension with a formula its value, the parameters read
    /// through `reader` — and forgets the formulas and labels of
    /// constraints gone.
    pub fn apply_formulas(&mut self, reader: &Reader) -> GeopResult<()> {
        let sketch = &mut self.sketch;
        self.formulas
            .retain(|k, _| sketch.constraints.contains_key(k));
        self.labels
            .retain(|k, _| sketch.constraints.contains_key(k));
        for (k, formula) in &self.formulas {
            let ctx = with_context!("the formula {formula} of constraint {k}");
            let c = (sketch.constraints.get_mut(k))
                .ok_or_else(|| GeopError::new("retained"))
                .with_context(ctx)?;
            set_value(c, formula.resolve(reader).with_context(ctx)?);
        }
        Ok(())
    }
}

/// What a sketch step keeps while it is edited: the drawing's session, and
/// the keys of the visuals selected.
pub type SketchEditing = FormState<SketchSession>;

/// The sketch the step `step` draws with `args`: on its plane as the part
/// before it has it, with what it projects and its formulas brought up to
/// date, solved anew if either changed what was drawn.
fn draw(reader: &Reader, (step, args): &(String, AddSketchArgs)) -> GeopResult<PlacedSketch> {
    let ctx = with_context!("add_sketch({step}, plane={:?})", args.plane);
    let Some(plane) = &args.plane else {
        return Err(GeopError::new("pick a plane to sketch on")).with_context(ctx);
    };
    let plane = plane.resolve(reader).with_context(ctx)?;
    let mut drawn = args.clone();
    drawn.update_references(reader, &plane).with_context(ctx)?;
    drawn.apply_formulas(reader).with_context(ctx)?;
    drawn.sketch.validate().with_context(ctx)?;
    // What it is given, or the values of its dimensions, changed since
    // it was drawn: solved anew, for its constraints to hold. A sketch
    // as it was drawn is as it was solved.
    if drawn.sketch != args.sketch {
        drawn.sketch.solve().with_context(ctx)?;
    }
    Ok(PlacedSketch {
        name: step.clone(),
        plane,
        sketch: drawn.sketch,
    })
}

part_operation!(AddSketch(AddSketchArgs));

impl PartOperation for AddSketch {
    const KIND: &'static str = "add_sketch";
    const INFO: OperationInfo = OperationInfo::new(
        "Sketch",
        "Draw a sketch on a plane of a coordinate system, a datum plane or a planar face.",
        OperationGroup::Sketch,
        OperationTier::Big,
    )
    .icon(crate::icons::ADD_SKETCH);
    type Args = AddSketchArgs;
    type Session = SketchSession;

    /// An empty sketch, on no plane yet: where to sketch is the first thing
    /// a new sketch asks for.
    fn new_args(_: &Context) -> AddSketchArgs {
        AddSketchArgs::new(None)
    }

    /// Defines the sketch named after the step.
    fn run(ctx: &mut StepContext, args: &AddSketchArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&sketch(step), &(step.to_string(), args.clone()), draw)
    }

    /// The plane, picked, and the sketch drawn in it: see
    /// [`crate::editor`].
    fn form<'a>(
        context: &'a Context<'a>,
        args: &AddSketchArgs,
        editing: &SketchEditing,
    ) -> EditForm<'a, AddSketchArgs, SketchSession> {
        editor::form(&context.before, args, &editing.session, &editing.selection)
    }

    fn canvas(
        context: &Context,
        edit: Edit<'_, AddSketchArgs, SketchSession>,
        event: &CanvasEvent,
    ) {
        editor::event(&context.before, edit, event)
    }
}
