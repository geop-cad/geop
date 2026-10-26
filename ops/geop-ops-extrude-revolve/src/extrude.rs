//! [`Extrude`]: sweep a sketch's area along its plane's normal, and how a
//! sketch becomes the named [`Profile`](geop_ops_model::Profile)s extrude and revolve sweep.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Track, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader};
use geop_core_sketch::{Shape, Sketch, profile::curve_polyline};
use geop_core_topology::{Body, build::BuiltBody};
use geop_lib_shell::{rebuilt, room};
use geop_ops_model::sketch::{PlacedSketch, SketchRef, shape_profiles};
use geop_ops_model::{
    BodyNames, Extent, ModelOperation, NamedModel, Namer, build_model, reach_past, read_model,
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
/// alike, or each its own way. With a draft angle its walls lean in away
/// from the plane, each by the angle (see [`drafted`]); both ways from the
/// plane, a ridge there.
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
    /// The draft angle of the walls, in degrees: positive narrows the
    /// area away from the sketch's plane, on either side of it, negative
    /// widens it (see [`drafted`]).
    #[serde(default, skip_serializing_if = "is_zero")]
    pub draft: f64,
}

fn is_zero(angle: &f64) -> bool {
    *angle == 0.0
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
            draft: 0.0,
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
        let draft = Number::new("draft", args.draft, Unit::Angle).range(-45.0, 45.0);
        f.number("draft", draft, |edit, v| edit.args.draft = v);
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
        let slope = match args.draft {
            0.0 => None,
            angle => Some(draft_slope(angle, &args.output, &shape)?),
        };
        let loops: Vec<_> = (shape_profiles(&placed.name, sketch, &geometry, shape)?.into_iter())
            .map(SweepLoop::plain)
            .collect();
        let extruding = Extruding {
            plane: &placed.plane,
            loops: &loops,
            slope,
        };
        let how = (&args.extent, &args.output);
        build_extents(model, reader, &namer, step, how, &extruding)
    }
}

/// The tangent of the draft `angle`, in degrees, by which a wall leans in
/// per unit of height: refused, saying why, where `output` and `shape`
/// are no solid with an area to narrow, or the angle no lean.
fn draft_slope(angle: f64, output: &Output, shape: &Shape) -> GeopResult<Scalar> {
    if matches!(shape, Shape::Chain(_)) {
        return Err(GeopError::new(
            "a draft narrows an area: an open chain of curves encloses none",
        ));
    }
    if matches!(output, Output::Faces { .. }) {
        return Err(GeopError::new(
            "a draft tapers a solid's walls between its caps: extrude faces without one",
        ));
    }
    if angle.is_nan() || angle.abs() >= 90.0 {
        return Err(GeopError::new(format!(
            "a draft of {angle} degrees: walls drafted by less than 90 degrees only"
        )));
    }
    let radians = Scalar::from_f64(angle).degrees_to_radians()?;
    radians.sin().div(radians.cos())
}

/// An extrude's loops, swept along the normal of their sketch's `plane`
/// (see [`Sweeping`]): its measure is the distance from the plane.
struct Extruding<'a> {
    plane: &'a CoordinateSystem,
    loops: &'a [SweepLoop],
    /// How far its walls lean in per unit of height, if it is drafted (see
    /// [`drafted`]).
    slope: Option<Scalar>,
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
        // Drafted through the sketch's plane, the walls lean in away from
        // it either way: a ridge there.
        let heights = match self.slope {
            Some(_) if from * to < 0.0 => vec![s(from), Scalar::ZERO, s(to)],
            _ => vec![s(from), s(to)],
        };
        let built = extrude(model, namer, name, self.plane, &heights, self.loops)?;
        match self.slope {
            Some(slope) => drafted(model, namer, &built, self.plane, slope),
            None => Ok(built),
        }
    }

    fn past(&self, hull: &[Vector3], sign: f64) -> GeopResult<f64> {
        reach_past(hull, self.plane, sign)
    }

    fn ends(&self, _: f64) -> GeopResult<(String, String)> {
        Ok(("start".into(), "end".into()))
    }

    fn all_the_way(&self, far: f64) -> f64 {
        far
    }

    fn full_turn(&self) -> Option<f64> {
        None
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

/// The solid `built`, extruded straight along the normal of `plane` by
/// `namer`, drafted: every wall leaning in by `slope` per unit of its
/// distance from the plane, on either side of it — the outer loop's walls
/// in, the holes' out — each on its own surface offset by that distance
/// times `slope` (see [`drafted_wall`]): a line's wall a plane at the
/// draft angle, an arc's a cone about the arc's axis. The caps stay where
/// they are, and the solid is rebuilt around the drafted walls
/// ([`rebuilt`]): each corner where the walls and a cap now meet, each edge
/// where its two faces do — a hyperbola where an arc's cone meets a line's
/// plane at a sharp corner. Every entity keeps its name.
///
/// Refused, naming the face, where a wall is drafted past its axis — an
/// arc narrower than the draft narrows it — and, naming the edge, where
/// the draft narrows a wall away between its neighbours.
fn drafted(
    model: &mut NamedModel,
    namer: &Namer,
    built: &BuiltBody,
    plane: &CoordinateSystem,
    slope: Scalar,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!("drafted({}, slope={slope:?})", namer.root());
    let solid = built
        .solid
        .ok_or_else(|| GeopError::new("only a solid is drafted"))
        .with_context(ctx)?;
    let (mut spec, sources) = model
        .topology()
        .body_spec(&built.faces, true)
        .with_context(ctx)?;
    let names = BodyNames {
        solid: Some(model.name(solid)?.to_string()),
        ..model.source_names(&sources)?
    };
    let caps = [namer.name(&["start"]), namer.name(&["end"])];
    let walls: Vec<bool> = names.faces.iter().map(|f| !caps.contains(f)).collect();
    for ((face, name), &wall) in spec.faces.iter_mut().zip(&names.faces).zip(&walls) {
        if wall {
            face.surface = drafted_wall(&face.surface, plane, slope).map_err(|e| {
                GeopError::new(format!(
                    "the draft cannot lean face {name} in that far: {}",
                    e.root_message()
                ))
            })?;
        }
    }
    let reseat = vec![false; spec.vertices.len()];
    let spec = rebuilt(&spec, &names, &walls, &reseat).with_context(ctx)?;
    model
        .delete_bodies(&[Body::Solid(solid)])
        .with_context(ctx)?;
    model.build_body(spec, names).with_context(ctx)
}

/// `wall`, a surface standing square to `plane` on one side of it, with
/// its rows of control points each at one height, leant in by `slope` per
/// unit of height away from the plane: given room first ([`room`], which
/// the rebuild asks of a new surface — a corner leant out moves past where
/// the wall ended), each control point where the wall's offset against its
/// normal by its height times `slope` has it ([`NurbSurface3D::offset`],
/// which keeps the parametrization), the height counted the way the wall
/// lies from the plane, so the room on the other side carries the drafted
/// wall on. The rows' offsets are what the wall is at their heights, and
/// between them it is ruled, so the result is the drafted wall exactly
/// wherever the offset is exact.
fn drafted_wall(
    wall: &NurbSurface3D,
    plane: &CoordinateSystem,
    slope: Scalar,
) -> GeopResult<NurbSurface3D> {
    let height = |point: &Vector4| -> GeopResult<Scalar> {
        Ok(point
            .dehomogenized()?
            .sub(plane.origin())
            .prod_dot(plane.w()))
    };
    let mut side = None;
    for point in &wall.control_points {
        side = side.or(height(point)?.definite_sign());
    }
    let slope = match side {
        Some(true) => slope,
        Some(false) => slope.neg(),
        None => return Err(GeopError::new("a wall of no height")),
    };
    let wall = room(wall.clone())?;
    let mut drafted = wall.clone();
    for (k, point) in drafted.control_points.iter_mut().enumerate() {
        let height = height(point)?;
        if height.is_zero() {
            continue;
        }
        // Where the offset has it, at the weight it had: offsetting a wall
        // square to the plane keeps its weights, and a row's weights kept
        // the same numbers keep the wall a polynomial along its height.
        let weight = point[3];
        let offset = wall
            .offset(height.mul(slope).neg())?
            .ok_or_else(|| GeopError::new("the draft makes a wall collapse at this height"))?;
        // The offset is on the wall's own net where it can be (a plane's, a
        // cylinder's); a free-form wall's is fitted on a finer one, and has
        // no control point of the wall's to take.
        if (offset.degree_u, offset.degree_v, offset.num_u, offset.num_v)
            != (wall.degree_u, wall.degree_v, wall.num_u, wall.num_v)
        {
            return Err(GeopError::new(
                "a wall whose offset is not on its own net, a free-form one, is not drafted",
            ));
        }
        let moved = offset.control_points[k];
        let at = moved.dehomogenized()?.prod_scalar(weight);
        *point = Vector4::from_array([at[0], at[1], at[2], weight]);
    }
    drafted.recompute_aabb();
    Ok(drafted)
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
