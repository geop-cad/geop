//! [`AddSketch`]: place a sketch in the part.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::ConstraintId;
use geop_ops::{
    Context, Library, Part, PlacedSketch,
    operation::{EntityRef, Operation},
    ui::{CanvasEvent, Edit, Form},
};
use serde::{Deserialize, Serialize};

use crate::{
    Constraint, Sketch,
    constraints::{Pick, set_value},
    editor::{self, SketchSession},
    references::{Reference, Source},
};

/// Adds a sketch on a plane to the part, named by the operation's id: a
/// planar face, a datum plane or a frame's plane (see [`EntityRef`]). The plane is
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
    pub plane: Option<EntityRef>,
    /// The sketch as drawn; solve it first for its constraints to hold.
    pub sketch: Sketch,
    /// Its reference geometry: its own origin and axes, and what it
    /// projects of the part.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub references: Vec<Reference>,
    /// The dimensions given by a formula of the part's parameters (see
    /// [`geop_ops::parameters`]) rather than a number, by constraint —
    /// an angle's in degrees.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub formulas: BTreeMap<ConstraintId, String>,
    /// Where each dimension placed by the designer shows its value: an
    /// offset in the plane from what it measures, so it moves with the
    /// geometry. Only what the editor draws; nothing built reads it.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub labels: BTreeMap<ConstraintId, [f64; 2]>,
}

impl AddSketchArgs {
    /// A new sketch on `plane`, with nothing in it but its own origin and
    /// axes.
    pub fn new(plane: Option<EntityRef>) -> Self {
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

    /// The reference of the sketch's own origin and axes, added if the
    /// sketch has none yet — a sketch from before sketches had them.
    pub fn ensure_frame(&mut self) -> &Reference {
        let at = match self
            .references
            .iter()
            .position(|r| r.source == Source::Frame)
        {
            Some(at) => at,
            None => {
                self.references
                    .insert(0, Reference::frame(&mut self.sketch));
                0
            }
        };
        &self.references[at]
    }

    /// The reference `point` or `curve` belongs to, by index, if any.
    pub fn reference_of(&self, pick: Pick) -> Option<usize> {
        self.references.iter().position(|r| match pick {
            Pick::Point(p) => r.has_point(p),
            Pick::Curve(c) => r.has_curve(c),
        })
    }

    /// Brings what the sketch projects up to date with `part`, the sketch
    /// lying in `plane` (see [`Reference::update`]).
    pub fn update_references<S: Scalar>(
        &mut self,
        part: &Part<S>,
        plane: &CoordinateSystem<S>,
    ) -> GeopResult<()> {
        for reference in &mut self.references {
            reference.update(&mut self.sketch, part, plane)?;
        }
        Ok(())
    }

    /// Gives every dimension with a formula its value, `evaluate` reading
    /// the parameters — and forgets the formulas and labels of constraints
    /// gone.
    pub fn apply_formulas(
        &mut self,
        mut evaluate: impl FnMut(&str) -> GeopResult<f64>,
    ) -> GeopResult<()> {
        self.formulas
            .retain(|k, _| self.sketch.constraints.contains_key(k));
        self.labels
            .retain(|k, _| self.sketch.constraints.contains_key(k));
        for (k, formula) in &self.formulas {
            let c = self.sketch.constraints.get_mut(k).expect("retained");
            let ctx = with_context!("the formula {formula:?} of constraint {k}");
            let v = evaluate(formula).with_context(ctx)?;
            set_value(
                c,
                if matches!(c, Constraint::Angle { .. }) {
                    v.to_radians()
                } else {
                    v
                },
            );
        }
        Ok(())
    }
}

impl Operation for AddSketch {
    type Args = AddSketchArgs;
    type Session = SketchSession;

    fn formulas<'a>(&self, args: &'a mut AddSketchArgs) -> Vec<&'a mut String> {
        args.formulas.values_mut().collect()
    }

    /// An empty sketch, on no plane yet: where to sketch is the first thing
    /// a new sketch asks for.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> AddSketchArgs {
        AddSketchArgs::new(None)
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &AddSketchArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("add_sketch({operation_id}, plane={:?})", args.plane);
        let Some(plane) = &args.plane else {
            return Err(GeopError::new("pick a plane to sketch on")).with_context(ctx);
        };
        let plane = plane.resolve_plane(&part).with_context(ctx)?;
        let drawn = &args.sketch;
        let mut args = args.clone();
        args.update_references(&part, &plane).with_context(ctx)?;
        args.apply_formulas(|formula| part.evaluate(formula))
            .with_context(ctx)?;
        args.sketch.validate().with_context(ctx)?;
        // What it is given, or the values of its dimensions, changed since
        // it was drawn: solved anew, for its constraints to hold. A sketch
        // as it was drawn is as it was solved.
        if args.sketch != *drawn {
            args.sketch.solve().with_context(ctx)?;
        }
        let placed = PlacedSketch {
            plane,
            sketch: args.sketch,
        };
        part.add_sketch(placed, operation_id).with_context(ctx)?;
        Ok(part)
    }

    /// The plane, picked, and the sketch drawn in it: see
    /// [`crate::editor`].
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &AddSketchArgs,
        s: &SketchSession,
        selection: &[String],
    ) -> Form<'a, S, AddSketchArgs, SketchSession> {
        editor::form(context.before, args, s, selection)
    }

    fn event<S: Scalar>(
        &self,
        context: Context<'_, S>,
        edit: Edit<'_, AddSketchArgs, SketchSession>,
        event: &CanvasEvent<S>,
    ) {
        editor::event(context.before, edit, event);
    }
}
