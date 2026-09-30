//! [`AddSketch`]: place a sketch in the part.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::Sketch;
use geop_ops::{
    Part, PlacedSketch,
    operation::{EntityRef, Operation},
    ui::{CanvasEvent, Edit, Form},
};
use serde::{Deserialize, Serialize};

use crate::editor::{self, SketchSession};

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
        before: &'a Part<S>,
        args: &AddSketchArgs,
        s: &SketchSession,
        selection: &[String],
    ) -> Form<'a, S, AddSketchArgs, SketchSession> {
        editor::form(before, args, s, selection)
    }

    fn event<S: Scalar>(
        &self,
        before: &Part<S>,
        args: &mut AddSketchArgs,
        session: &mut SketchSession,
        selection: &mut Vec<String>,
        event: &CanvasEvent<S>,
    ) {
        let edit = Edit {
            args,
            session,
            selection,
        };
        editor::event(before, edit, event);
    }
}
