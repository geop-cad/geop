//! [`Primitive`]: a solid at given coordinates, kept as a new body or
//! combined with a solid of the part — what a test builds on instead of a
//! sketch and an extrusion.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, FrameAxis},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_core_topology::SolidId;
use geop_lib_booleans::combine_tools;
use geop_lib_sweep::{SweepLoop, circle_at, extrude, polygon, revolve_at_oriented};
use geop_ops_model::{Combine, ModelOperation, NamedModel, Namer, Profile, Tool, build_model};
use serde::{Deserialize, Serialize};

/// A solid given by its coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Shape {
    /// The box spanning `[min, max]`, its faces towards each axis.
    Block { min: [f64; 3], max: [f64; 3] },
    /// The polygon `corners` in the `xy` plane, counter-clockwise, swept
    /// from `z0` up to `z1`.
    Prism {
        corners: Vec<[f64; 2]>,
        z0: f64,
        z1: f64,
    },
    /// The cylinder of `radius` standing on the centre `base` of its
    /// bottom cap, `height` along `axis`.
    Cylinder {
        base: [f64; 3],
        axis: FrameAxis,
        radius: f64,
        height: f64,
    },
    /// The sphere of `radius` around `center`, a half circle turned round
    /// the `z` axis through it.
    Sphere { center: [f64; 3], radius: f64 },
}

impl Shape {
    /// The operation its solid is named as: `block(step)`, `prism(step)`,
    /// `cylinder(step)` or `sphere(step)`.
    pub fn kind(&self) -> &'static str {
        match self {
            Shape::Block { .. } => "block",
            Shape::Prism { .. } => "prism",
            Shape::Cylinder { .. } => "cylinder",
            Shape::Sphere { .. } => "sphere",
        }
    }
}

/// A [`Shape`], kept as a new body or combined with a solid of the part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrimitiveArgs {
    pub shape: Shape,
    pub combine: Combine,
}

impl PrimitiveArgs {
    /// `shape` as a new body.
    pub fn new(shape: Shape) -> Self {
        PrimitiveArgs {
            shape,
            combine: Combine::NewBody,
        }
    }

    /// The box spanning `[min, max]`, as a new body.
    pub fn block(min: [f64; 3], max: [f64; 3]) -> Self {
        Self::new(Shape::Block { min, max })
    }

    /// The polygon `corners`, counter-clockwise in the `xy` plane, swept
    /// from `z0` up to `z1`, as a new body.
    pub fn prism(corners: &[[f64; 2]], z0: f64, z1: f64) -> Self {
        Self::new(Shape::Prism {
            corners: corners.to_vec(),
            z0,
            z1,
        })
    }

    /// The cylinder of `radius` standing on `base`, `height` along
    /// `axis`, as a new body.
    pub fn cylinder(base: [f64; 3], axis: FrameAxis, radius: f64, height: f64) -> Self {
        Self::new(Shape::Cylinder {
            base,
            axis,
            radius,
            height,
        })
    }

    /// The cylinder of `radius` standing on `base`, `height` up along
    /// `z`, as a new body.
    pub fn cylinder_z(base: [f64; 3], radius: f64, height: f64) -> Self {
        Self::cylinder(base, FrameAxis::Z, radius, height)
    }

    /// The sphere of `radius` around `center`, as a new body.
    pub fn sphere(center: [f64; 3], radius: f64) -> Self {
        Self::new(Shape::Sphere { center, radius })
    }

    /// The same shape, combined as `combine` says.
    pub fn combined(self, combine: Combine) -> Self {
        PrimitiveArgs { combine, ..self }
    }

    /// The same shape, cut from the solid named `target`.
    pub fn cut(self, target: &str) -> Self {
        self.combined(Combine::cut(target))
    }

    /// The same shape, joined to the solid named `target`.
    pub fn join(self, target: &str) -> Self {
        self.combined(Combine::join(target))
    }
}

/// Adds the solid `args.shape` describes to the part, combined as
/// `args.combine` says and named after the step by the shape's kind:
/// `block(B)`, `cylinder(B)`, ... — its faces as the sweep that built it
/// names them (`block(B,end)` is the top of a block, `cylinder(B,end)` of
/// a cylinder, `cylinder(B,c0)` to `c3` its side's quarters). Only for tests: an editor never offers it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Primitive;

part_operation!(Primitive(PrimitiveArgs));

impl PartOperation for Primitive {
    const KIND: &'static str = "primitive";
    const INFO: OperationInfo = OperationInfo::new(
        "Primitive",
        "A block, a prism, a cylinder or a sphere at given coordinates (tests only).",
        OperationGroup::Solid,
        OperationTier::Menu,
    );
    type Args = PrimitiveArgs;
    type Session = ();

    /// A unit block.
    fn new_args(_: &Context) -> PrimitiveArgs {
        PrimitiveArgs::block([0.0; 3], [1.0; 3])
    }

    fn run(ctx: &mut StepContext, args: &PrimitiveArgs) -> GeopResult<()> {
        build_model::<Self>(ctx, args)
    }

    /// Only how it is combined: the shape is written, not edited.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &PrimitiveArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, PrimitiveArgs> {
        let mut f = EditForm::<PrimitiveArgs>::default();
        args.combine
            .fields(&mut f, &context.before, |args, combine| {
                args.combine = combine
            });
        f
    }
}

impl ModelOperation for Primitive {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &PrimitiveArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("primitive({step}, {args:?})");
        let namer = Namer::new(args.shape.kind(), step).with_context(ctx)?;
        let solid = args.combine.built_name(&namer);
        let solid = build_shape(model, &namer, &solid, &args.shape).with_context(ctx)?;
        combine_tools(model, &namer, step, &[Tool::whole(solid)], &args.combine)
            .with_context(ctx)?
            .stopped()
            .with_context(ctx)?;
        Ok(())
    }
}

/// The point `p`, as sharp scalars.
fn point(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// The point `p`, as sharp scalars.
fn point2(p: [f64; 2]) -> Vector2 {
    Vector2::from_array(p.map(Scalar::from_f64))
}

/// `shape` built into `model` as the solid named `solid`, its entities
/// named by `namer`.
fn build_shape(
    model: &mut NamedModel,
    namer: &Namer,
    solid: &str,
    shape: &Shape,
) -> GeopResult<SolidId> {
    match shape {
        Shape::Block { min, max } => {
            let corners = [
                [min[0], min[1]],
                [max[0], min[1]],
                [max[0], max[1]],
                [min[0], max[1]],
            ];
            prism(model, namer, solid, &corners, min[2], max[2])
        }
        Shape::Prism { corners, z0, z1 } => prism(model, namer, solid, corners, *z0, *z1),
        Shape::Cylinder {
            base,
            axis,
            radius,
            height,
        } => {
            // A circle extruded along the axis, as a sketched disc is: planar
            // caps (`start` at the base, `end` at the top) and four side
            // quarters (`c0`..`c3`). Each orientation a cyclic permutation of
            // the world's axes, so `u x v = w` holds and the solid comes out
            // with its normals outwards.
            let [x, y, z] = [0, 1, 2].map(Vector3::axis);
            let (u, v, w) = match axis {
                FrameAxis::X => (y, z, x),
                FrameAxis::Y => (z, x, y),
                FrameAxis::Z => (x, y, z),
            };
            let (r, h) = (Scalar::from_f64(*radius), Scalar::from_f64(*height));
            let plane = CoordinateSystem::try_new(
                point(*base),
                u.prod_scalar(r),
                v.prod_scalar(r),
                w.prod_scalar(h),
            )?;
            let loops = [SweepLoop::plain(Profile::closed(circle_at(0.0, 0.0, 1.0)?))];
            let built = extrude(
                model,
                namer,
                Some(solid),
                &plane,
                &[Scalar::ZERO, Scalar::ONE],
                &loops,
            )?;
            built
                .solid
                .ok_or_else(|| GeopError::new("extruded as a solid, but none came out"))
        }
        Shape::Sphere { center, radius } => {
            let axes = CoordinateSystem::world_at(point(*center));
            // The half circle from the north pole over `(r, 0)` to the south
            // pole: the first quarter arc reversed, then the last reversed.
            let quarters = circle_at(0.0, 0.0, *radius)?;
            let profile = vec![quarters[0].reverse(), quarters[3].reverse()];
            revolve_at_oriented(model, namer, solid, &Profile::open(profile), &axes)
        }
    }
}

/// The polygon `corners` swept from `z0` up to `z1`, into `model` as the
/// solid named `solid`.
fn prism(
    model: &mut NamedModel,
    namer: &Namer,
    solid: &str,
    corners: &[[f64; 2]],
    z0: f64,
    z1: f64,
) -> GeopResult<SolidId> {
    if corners.len() < 3 {
        return Err(GeopError::new(format!(
            "a prism needs at least three corners, not {corners:?}"
        )));
    }
    let [x, y, z] = [0, 1, 2].map(Vector3::axis);
    let plane = CoordinateSystem::try_new(point([0.0, 0.0, z0]), x, y, z)?;
    let outline: Vec<Vector2> = corners.iter().map(|&c| point2(c)).collect();
    let loops = [SweepLoop::plain(Profile::closed(polygon(&outline)?))];
    let built = extrude(
        model,
        namer,
        Some(solid),
        &plane,
        &[Scalar::ZERO, Scalar::from_f64(z1 - z0)],
        &loops,
    )?;
    built
        .solid
        .ok_or_else(|| GeopError::new("extruded as a solid, but none came out"))
}
