//! [`AddSketch`]: place a sketch in the part.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Part, PlacedSketch,
    operation::{EntityRef, Operation},
    ui::{CanvasEvent, Edit, Form},
};
use serde::{Deserialize, Serialize};

use crate::{
    Sketch,
    editor::{self, SketchSession},
};

/// Adds a sketch on a plane to the part, named by the operation's id: a
/// planar face, a datum plane or a frame's plane (see [`EntityRef`]). The plane is
/// resolved when the sketch is added: a sketch on a face stays where the
/// face was, whatever later operations do to the face.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketch;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AddSketchArgs {
    /// The plane to sketch on: a planar face, a datum plane or a frame's
    /// plane; none yet, for a new sketch waiting for one to be picked.
    #[serde(default)]
    pub plane: Option<EntityRef>,
    /// The sketch as drawn; solve it first for its constraints to hold.
    pub sketch: Sketch,
}

impl Operation for AddSketch {
    type Args = AddSketchArgs;
    type Session = SketchSession;

    /// An empty sketch, on no plane yet: where to sketch is the first thing
    /// a new sketch asks for.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> AddSketchArgs {
        AddSketchArgs {
            plane: None,
            sketch: Sketch::new(),
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &AddSketchArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("add_sketch({operation_id}, plane={:?})", args.plane);
        args.sketch.validate().with_context(ctx)?;
        let Some(plane) = &args.plane else {
            return Err(GeopError::new("pick a plane to sketch on")).with_context(ctx);
        };
        let plane = plane.resolve_plane(&part).with_context(ctx)?;
        let placed = PlacedSketch {
            plane,
            sketch: args.sketch.clone(),
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
        let before = context.before;
        editor::form(before, args, s, selection)
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
