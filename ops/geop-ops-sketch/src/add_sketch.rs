//! [`AddSketch`]: place a sketch in the part.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::{DatumComponent, FrameAxis},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::Sketch;
use geop_ops::{
    EditContext, Edited, ORIGIN, Part, PlacedSketch,
    operation::{EntityRef, Operation, resolve_plane},
    ui::Event,
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
    /// plane.
    pub plane: EntityRef,
    /// The sketch as drawn; solve it first for its constraints to hold.
    pub sketch: Sketch,
}

impl Operation for AddSketch {
    type Args = AddSketchArgs;
    type Session = SketchSession;

    /// An empty sketch on the origin's `xy` plane.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> AddSketchArgs {
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
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
        let plane = resolve_plane(&part, &args.plane).with_context(ctx)?;
        let placed = PlacedSketch {
            plane,
            sketch: args.sketch.clone(),
        };
        part.add_sketch(placed, operation_id).with_context(ctx)?;
        Ok(part)
    }

    /// Choosing the plane, then drawing in it: see [`crate::editor`].
    fn edit<S: Scalar>(
        &self,
        ctx: &EditContext<S>,
        args: AddSketchArgs,
        session: SketchSession,
        event: Option<&Event<S>>,
    ) -> Edited<AddSketchArgs, SketchSession, S> {
        editor::edit(ctx, args, session, event)
    }

    fn summary(&self, args: &AddSketchArgs) -> String {
        let n = args.sketch.curves.len();
        format!(
            "plane={}, {n} curve{}",
            args.plane.label(),
            if n == 1 { "" } else { "s" }
        )
    }

    fn references(&self, args: &AddSketchArgs) -> Vec<EntityRef> {
        vec![args.plane.clone()]
    }
}
