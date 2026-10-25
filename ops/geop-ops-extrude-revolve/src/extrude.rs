//! [`Extrude`]: sweep a sketch's area along its plane's normal, and how a
//! sketch becomes the named [`Profile`](geop_ops_model::Profile)s extrude and revolve sweep.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Track, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader};
use geop_core_sketch::{Sketch, profile::curve_polyline};
use geop_core_topology::build::BuiltBody;
use geop_ops_model::sketch::{PlacedSketch, SketchRef, shape_profiles};
use geop_ops_model::{
    Extent, ModelOperation, NamedModel, Namer, build_model, reach_past, read_model,
};
use serde::{Deserialize, Serialize};

use geop_lib_sweep::{SweepLoop, extrude};

use crate::sweeping::{Extents, Output, Sweeping, build_extents};

/// Extrudes the one area of a sketch along the sketch plane's normal, into
/// a solid named `extrude(E)` for the operation `E` — kept as a new body,
/// or combined with another solid, see [`Combine`](geop_ops_model::Combine); the result is
/// `extrude(E)` either way. Or, as faces ([`Output::Faces`]), sweeps the
/// sketch's curves into faces standing on their own — the area's outline,
/// or a chain of curves enclosing nothing.
///
/// How far it goes is an [`Extents`]: a length, or up to the next face of
/// the solid it is joined to or cut from, on one side of the plane, both
/// alike, or each its own way.
///
/// The faces, edges and vertices are named after the sketch elements they
/// are swept from (see [`geop_lib_sweep::extrude`]), with `X` a piece of a
/// sketch curve (`c3`, or `c3#1` for the second piece of an arc or circle
/// split into several) and `P` a joint (`p2` for a sketch point, `c3@1`
/// where a curve was split) of the sketch `K`:
///
/// - `extrude(E,start)` / `extrude(E,end)`: the caps, at either end;
/// - `extrude(E,K,X)`: the side face swept by `X`; `extrude(E,K,X,start)` /
///   `extrude(E,K,X,end)` its edges on the two caps;
/// - `extrude(E,K,P)`: the edge swept by `P`; `extrude(E,K,P,start)` /
///   `extrude(E,K,P,end)` its vertices.
///
/// A side going up to the next face is built on its own, from the sketch's
/// plane — its caps `start` on the plane — and the second of two sides
/// built so is named within `side2`: `extrude(E,side2,K,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Extrude;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtrudeArgs {
    /// The sketch to extrude.
    pub sketch: SketchRef,
    /// How far, along the sketch plane's normal — backwards for a negative
    /// length.
    pub extent: Extents,
    /// A solid, kept as a new body or combined with another, or faces.
    #[serde(default)]
    pub output: Output,
}

/// Where a side's length is dragged: at the centre of the sketch, that far
/// along the sketch plane's normal — backwards for the second side. None
/// while the sketch cannot be found.
fn distance_handle(
    reader: &Reader,
    args: &ExtrudeArgs,
    length: f64,
    second: bool,
) -> Option<Track> {
    let placed = reader.retrieve_element(&args.sketch).ok()?;
    let plane = &placed.plane;
    let center = plane.uv_to_xyz(&sketch_center(&placed.sketch)?);
    // A symmetric extrude goes half its length either way; the second side
    // the other way.
    let scale = match (args.extent.symmetric, second) {
        (true, _) => 0.5,
        (false, false) => 1.0,
        (false, true) => -1.0,
    };
    let normal = plane.w();
    Some(Track {
        at: center.add(&normal.prod_scalar(Scalar::from_f64(length * scale))),
        direction: normal.prod_scalar(Scalar::from_f64(scale)),
    })
}

part_operation!(Extrude(ExtrudeArgs));

impl PartOperation for Extrude {
    const KIND: &'static str = "extrude";
    const INFO: OperationInfo = OperationInfo::new(
        "Extrude",
        "Sweep a sketch's area along its plane's normal into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Big,
    )
    .icon(crate::icons::EXTRUDE);
    type Args = ExtrudeArgs;
    type Session = ();

    /// The newest sketch, 10 up — a size that reads in millimetres —
    /// joined to the newest solid if there is one.
    fn new_args(context: &Context) -> ExtrudeArgs {
        let before = &context.before;
        let newest = before.targets_of::<PlacedSketch>().pop();
        ExtrudeArgs {
            sketch: SketchRef::whole(newest.unwrap_or_else(|| geop_ops_model::sketch::sketch(""))),
            extent: Extents::blind(10.0),
            output: Output::new_for(&read_model(before).unwrap_or_default()),
        }
    }

    fn run(ctx: &mut StepContext, args: &ExtrudeArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sketch, picked; how far, each length typed or dragged as a
    /// handle; whether a face; and how to combine. A length typed or
    /// dragged across the sketch plane turns a join into a cut, and back
    /// (see [`Combine::follow_sign`](geop_ops_model::Combine::follow_sign));
    /// a formula is followed, not steered by.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ExtrudeArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ExtrudeArgs> {
        let before = &context.before;
        let mut f = EditForm::<ExtrudeArgs>::default();
        f.pick_one(
            "sketch",
            "sketch",
            before,
            Some(&args.sketch),
            |edit, sketch| edit.args.sketch = sketch,
        );
        let amount = |length: &NumericInput, second| {
            let label = if second { "distance 2" } else { "distance" };
            let number = Number::numeric(length, label, before, Unit::Length);
            let at = distance_handle(before, args, number.value, second);
            number.range(-10.0, 10.0).handle(at)
        };
        args.extent.fields(
            &mut f,
            ("distance", 1.0),
            amount,
            |args| &mut args.extent,
            |edit, length| {
                let to = length.plain();
                let from = std::mem::replace(&mut edit.args.extent.side1, Extent::Blind(length));
                if let (Extent::Blind(NumericInput::Plain(from)), Some(to)) = (from, to)
                    && let Output::Solid(combine) = &mut edit.args.output
                {
                    combine.follow_sign(from, to);
                }
            },
        );
        let reaches = args.extent.reaches_target();
        let output = |args: &mut ExtrudeArgs, output| args.output = output;
        (args.output).fields(&mut f, before, reaches, output);
        f
    }
}

impl ModelOperation for Extrude {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &ExtrudeArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("extrude", step)?;
        let placed = reader.retrieve_element(&args.sketch)?;
        let sketch = &placed.sketch;
        let geometry = sketch.enclose()?;
        let shape = args.output.shape(sketch)?;
        let loops: Vec<_> = (shape_profiles(&placed.name, sketch, &geometry, shape)?.into_iter())
            .map(SweepLoop::plain)
            .collect();
        let extruding = Extruding {
            plane: &placed.plane,
            loops: &loops,
        };
        let how = (&args.extent, &args.output);
        build_extents(model, reader, &namer, step, how, &extruding)
    }
}

/// An extrude's loops, swept along the normal of their sketch's `plane`
/// (see [`Sweeping`]): its measure is the distance from the plane.
struct Extruding<'a> {
    plane: &'a CoordinateSystem,
    loops: &'a [SweepLoop],
}

impl Sweeping for Extruding<'_> {
    fn loops(&self) -> &[SweepLoop] {
        self.loops
    }

    fn sweep(
        &self,
        model: &mut NamedModel,
        namer: &Namer,
        name: Option<&str>,
        from: f64,
        to: f64,
    ) -> GeopResult<BuiltBody> {
        let s = Scalar::from_f64;
        extrude(model, namer, name, self.plane, s(from), s(to), self.loops)
    }

    fn past(&self, hull: &[[f64; 3]], sign: f64) -> GeopResult<f64> {
        reach_past(hull, self.plane, sign)
    }

    fn ends(&self, _: f64) -> GeopResult<(String, String)> {
        Ok(("start".into(), "end".into()))
    }

    fn all_the_way(&self, far: f64) -> f64 {
        far
    }

    fn measure(&self, sign: f64, p: &Vector3) -> Option<f64> {
        let along = (p.sub(self.plane.origin()))
            .prod_dot(self.plane.w())
            .mul(Scalar::from_f64(sign));
        along
            .definitely_greater(Scalar::ZERO)
            .then(|| along.to_f64())
    }
}

/// The centre of the box around what `sketch` draws — its profile curves,
/// or its points if it has none — in sketch coordinates.
fn sketch_center(sketch: &Sketch) -> Option<Vector2> {
    let mut drawn: Vec<Vector2> = sketch
        .curves
        .iter()
        .filter(|(_, c)| !c.construction)
        .flat_map(|(&id, _)| curve_polyline(sketch, id).unwrap_or_default())
        .collect();
    if drawn.is_empty() {
        drawn = sketch.positions().into_values().collect();
    }
    let hull = drawn.into_iter().reduce(|a, b| a.union(&b))?;
    // The middle of the box is a free choice: sharp.
    Some(hull.map(Scalar::midpoint))
}
