//! [`AddSketch`]: place a sketch in the part.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_sketch::Sketch;
use serde::{Deserialize, Serialize};

use geop_ops::{
    Part, PlacedSketch,
    operation::{
        EntityRef, Handle, HandleGroup, HandleMotion, Operation, OperationArgs, arg_path,
        resolve_plane, to_f64,
    },
};

/// Adds a sketch on a plane to the part, named by the operation's id: a base
/// plane, a planar face or a datum plane (see [`EntityRef`]). The plane is
/// resolved when the sketch is added: a sketch on a face stays where the
/// face was, whatever later operations do to the face.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddSketch;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, OperationArgs)]
pub struct AddSketchArgs {
    /// The plane to sketch on: a base plane, a planar face or a datum plane.
    #[arg(Plane)]
    pub plane: EntityRef,
    /// The sketch as drawn; solve it first for its constraints to hold.
    #[arg(Drawing { plane: "plane" })]
    pub sketch: Sketch,
}

impl<S: Scalar> Operation<S> for AddSketch {
    type Args = AddSketchArgs;

    fn apply(
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

    /// Every point, as a handle that slides in the sketch plane.
    fn handles(&self, before: &Part<S>, args: &AddSketchArgs) -> GeopResult<Vec<Handle>> {
        let plane = resolve_plane(before, &args.plane)?;
        let (u, v) = (to_f64(plane.u()), to_f64(plane.v()));
        Ok(args
            .sketch
            .points
            .iter()
            .map(|(id, p)| {
                let at = plane.uv_to_xyz(&Vector2::from_array([p.x, p.y].map(S::from_f64)));
                let path = |c: &str| arg_path(&["sketch", "points", &id.0.to_string(), c]);
                Handle {
                    label: id.to_string(),
                    group: HandleGroup::Sketch,
                    position: to_f64(&at),
                    motion: HandleMotion::Planar {
                        u,
                        v,
                        x: path("x"),
                        y: path("y"),
                        value: [p.x, p.y],
                    },
                }
            })
            .collect())
    }
}
