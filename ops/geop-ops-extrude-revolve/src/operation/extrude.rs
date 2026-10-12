//! [`Extrude`]: sweep a sketch's area along its plane's normal, and how a
//! sketch becomes the named [`Profile`]s extrude and revolve sweep.

use geop_core_math::scalars::Ring;
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{Enclosure, Region, Shape, Sketch, profile::curve_polyline};
use geop_core_topology::build::BuiltBody;
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, EntityRef, NamedModel, Namer,
    operation::{BuildReader, Lookup, PartOperation, Scope, Seen, StepContext},
    parameters::Formula,
    part_operation,
    ui::{EditForm, FormState, Track, Unit, Value, formula_number, sketch_field},
};
use geop_ops_booleans::{Extent, extent::reach_past};
use serde::{Deserialize, Serialize};

use super::{Extents, Output, Sweeping, build_extents};
use crate::{common::Profile, extrude::extrude, sweep::SweepLoop};

/// Extrudes the one area of a sketch along the sketch plane's normal, into
/// a solid named `extrude(E)` for the operation `E` — kept as a new body,
/// or combined with another solid, see [`Combine`](geop_ops_booleans::Combine); the result is
/// `extrude(E)` either way. Or, as faces ([`Output::Faces`]), sweeps the
/// sketch's curves into faces standing on their own — the area's outline,
/// or a chain of curves enclosing nothing.
///
/// How far it goes is an [`Extents`]: a length, or up to the next face of
/// the solid it is joined to or cut from, on one side of the plane, both
/// alike, or each its own way.
///
/// The faces, edges and vertices are named after the sketch elements they
/// are swept from (see [`crate::extrude::extrude`]), with `X` a piece of a
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
    pub sketch: String,
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
fn distance_handle<S: Scalar>(
    lookup: &mut (impl Lookup<S> + ?Sized),
    args: &ExtrudeArgs,
    length: f64,
    second: bool,
) -> Option<Track<S>> {
    let placed = lookup.sketch(&args.sketch).ok()?;
    let plane = &placed.plane;
    let center = plane.uv_to_xyz(&sketch_center(&placed.sketch)?);
    // A symmetric extrude goes half its length either way; the second side
    // the other way.
    let scale = match (args.extent.symmetric, second) {
        (true, _) => 0.5,
        (false, false) => 1.0,
        (false, true) => -1.0,
    };
    let normal = *plane.w();
    Some(Track {
        at: center.add(&normal.prod_scalar(S::from_f64(length * scale))),
        direction: normal.prod_scalar(S::from_f64(scale)),
    })
}

part_operation!(Extrude);

impl PartOperation for Extrude {
    const INFO: OperationInfo = OperationInfo::new(
        "extrude",
        "Extrude",
        "Sweep a sketch's area along its plane's normal into a solid, or its curves into faces.",
        OperationGroup::Solid,
        OperationTier::Big,
    )
    .icon(crate::icons::EXTRUDE);
    type Args = ExtrudeArgs;
    type Session = ();

    fn formulas(args: &mut ExtrudeArgs) -> Vec<&mut String> {
        args.extent.formulas()
    }

    /// The newest sketch, a unit up, joined to the newest solid if there is
    /// one.
    fn new_args(context: &StepContext) -> ExtrudeArgs {
        let seen = Seen::new(context.before);
        ExtrudeArgs {
            sketch: seen.sketch_names().pop().unwrap_or_default(),
            extent: Extents::blind(1.0),
            output: Output::new_for(seen.model()),
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &ExtrudeArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("extrude", step)?;
        let placed = (Scope {
            model,
            reader: &mut *reader,
        })
        .sketch(&args.sketch)?
        .clone();
        let sketch = &placed.sketch;
        let geometry = sketch.enclose::<Design>()?;
        let shape = args.output.shape(sketch)?;
        let loops = shape_loops(&args.sketch, sketch, &geometry, shape)?;
        let extruding = Extruding {
            plane: &placed.plane,
            loops: &loops,
        };
        let how = (&args.extent, &args.output);
        build_extents(model, reader, &namer, step, how, &extruding)
    }

    /// The sketch, picked; how far, each length typed or dragged as a
    /// handle; whether a face; and how to combine. A length typed or
    /// dragged across the sketch plane turns a join into a cut, and back
    /// (see [`Combine::follow_sign`](geop_ops_booleans::Combine::follow_sign));
    /// a formula is followed, not steered by.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &ExtrudeArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ExtrudeArgs> {
        let mut seen = Seen::new(context.before);
        let mut f = EditForm::<Design, ExtrudeArgs>::default();
        let sketch = std::slice::from_ref(&args.sketch);
        sketch_field(&mut f.form.dialog, &context.before, "sketch", sketch, false);
        f.on("sketch", |edit, value| {
            if let Value::Entities(picked) = value {
                edit.args.sketch =
                    EntityRef::last_name(EntityRef::sketch, &picked).unwrap_or_default();
            }
        });
        let amount = |length: &Formula, second| {
            let label = if second { "distance 2" } else { "distance" };
            let number = formula_number(label, length, &mut seen, Unit::Length);
            let at = distance_handle(&mut seen, args, number.value, second);
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
                if let (Extent::Blind(Formula::Plain(from)), Some(to)) = (from, to)
                    && let Output::Solid(combine) = &mut edit.args.output
                {
                    combine.follow_sign(from, to);
                }
            },
        );
        let reaches = args.extent.reaches_target();
        let output = |args: &mut ExtrudeArgs, output| args.output = output;
        (args.output).fields(&mut f, Seen::new(context.before).model(), reaches, output);
        f
    }
}

/// An extrude's loops, swept along the normal of their sketch's `plane`
/// (see [`Sweeping`]): its measure is the distance from the plane.
struct Extruding<'a> {
    plane: &'a CoordinateSystem<Design>,
    loops: &'a [SweepLoop<Design>],
}

impl Sweeping for Extruding<'_> {
    fn loops(&self) -> &[SweepLoop<Design>] {
        self.loops
    }

    fn sweep(
        &self,
        model: &mut NamedModel<Design>,
        namer: &Namer,
        name: Option<&str>,
        from: f64,
        to: f64,
    ) -> GeopResult<BuiltBody> {
        let s = Design::from_f64;
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

    fn measure(&self, sign: f64, p: &Vector3<Design>) -> Option<f64> {
        let along = (p.sub(self.plane.origin()))
            .prod_dot(self.plane.w())
            .mul(Design::from_f64(sign));
        along
            .definitely_greater(Design::ZERO)
            .then(|| along.to_f64())
    }
}

/// The loops of the sketch `name`'s `region` to sweep, outer loop first,
/// with the sketch's points at `geometry`.
fn region_loops<S: Scalar>(
    name: &str,
    sketch: &Sketch<Design>,
    geometry: &Enclosure<S>,
    region: &Region,
) -> GeopResult<Vec<SweepLoop<S>>> {
    std::iter::once(&region.outer)
        .chain(&region.holes)
        .map(|lp| {
            Ok(SweepLoop::plain(Profile::of_pieces(
                name,
                lp.to_nurbs(sketch, geometry)?,
                true,
            )))
        })
        .collect()
}

/// The loops of `shape` to sweep into a sheet: the region's, or the one
/// open chain.
pub fn shape_loops<S: Scalar>(
    name: &str,
    sketch: &Sketch<Design>,
    geometry: &Enclosure<S>,
    shape: Shape,
) -> GeopResult<Vec<SweepLoop<S>>> {
    match shape {
        Shape::Region(region) => region_loops(name, sketch, geometry, &region),
        Shape::Chain(chain) => Ok(vec![SweepLoop::plain(Profile::of_pieces(
            name,
            chain.to_nurbs(sketch, geometry)?,
            false,
        ))]),
    }
}

/// The centre of the box around what `sketch` draws — its profile curves,
/// or its points if it has none — in sketch coordinates.
fn sketch_center<S: Scalar>(sketch: &Sketch<Design>) -> Option<Vector2<S>> {
    let mut drawn: Vec<Vector2<Design>> = sketch
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
    Some(hull.map(|c| c.cast::<S>().midpoint()))
}
