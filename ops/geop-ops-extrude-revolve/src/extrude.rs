//! [`Extrude`]: sweep a sketch's area along its plane's normal, and how a
//! sketch becomes the named [`Profile`](geop_ops_model::Profile)s extrude and revolve sweep.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Track, Unit},
    part_operation,
    part_operation::PartOperation,
};
use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader};
use geop_core_sketch::{
    CurveId, PointId, ProfileEdge, ProfileJoint, ProfileLoop, ProfilePiece, Region, Shape, Sketch,
    offset::Corners, profile::curve_polyline,
};
use geop_core_topology::build::BuiltBody;
use geop_ops_model::sketch::{PlacedSketch, SketchRef, chain_of_pieces, shape_profiles};
use geop_ops_model::{
    Extent, ModelOperation, NamedModel, Namer, build_model, reach_past, read_model,
};
use serde::{Deserialize, Serialize};

use geop_lib_sweep::{Level, SweepLoop, drafted, extrude};

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
/// alike, or each its own way. With a draft angle its walls taper: the
/// area at each end is the sketch's offset inwards by the distance from
/// the plane times the angle's tangent, the walls ruled between (see
/// [`draft_levels`]); both ways from the plane, a ridge there.
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
    /// widens it (see [`draft_levels`]).
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
        let region = match (&shape, args.draft) {
            (_, 0.0) => None,
            (Shape::Region(region), _) => Some(region.clone()),
            (Shape::Chain(_), _) => {
                return Err(GeopError::new(
                    "a draft narrows an area: an open chain of curves encloses none",
                ));
            }
        };
        let loops: Vec<_> = (shape_profiles(&placed.name, sketch, &geometry, shape)?.into_iter())
            .map(SweepLoop::plain)
            .collect();
        let extruding = Extruding {
            plane: &placed.plane,
            loops: &loops,
            draft: region.as_ref().map(|region| (&*placed, region, args.draft)),
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
    /// The sketch, its area and the draft angle, for a drafted extrude.
    draft: Option<(&'a PlacedSketch, &'a Region, f64)>,
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
        let Some((placed, region, angle)) = self.draft else {
            return extrude(model, namer, name, self.plane, s(from), s(to), self.loops);
        };
        // Through the sketch's plane, the area narrows away from it either
        // way: a ridge there.
        let heights = if from * to < 0.0 {
            vec![from, 0.0, to]
        } else {
            vec![from, to]
        };
        let levels = draft_levels(placed, region, angle, &heights)?;
        drafted(model, namer, name, self.plane, &levels)
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

/// The loops of `region`, of the sketch `placed`, at each of `heights`
/// along its plane's normal, drafted by `angle` degrees: each offset in
/// the plane (see [`Sketch::offset`]) by `|h| tan(angle)`, inwards for a
/// positive angle — the outer loop shrinking, the holes growing — its
/// corners extended to meet, each offset curve's joints matched with its
/// own curve's. A wall is ruled from a curve to its offsets, so it is the
/// drafted plane or cone wherever the corners at its ends move straight
/// with the height: between lines, and where a curve runs on tangentially.
/// Refused, by name, where an offset is: a curve too short for its
/// corners, an arc offset past its centre, a spline.
pub(crate) fn draft_levels(
    placed: &PlacedSketch,
    region: &Region,
    angle: f64,
    heights: &[f64],
) -> GeopResult<Vec<Level>> {
    if angle.is_nan() || angle.abs() >= 90.0 {
        return Err(GeopError::new(format!(
            "a draft of {angle} degrees: walls drafted by less than 90 degrees only"
        )));
    }
    let slope = angle.to_radians().tan();
    let mut sketch = placed.sketch.clone();
    let loops: Vec<&ProfileLoop> = std::iter::once(&region.outer)
        .chain(&region.holes)
        .collect();
    // Per height, per loop: the loop drawn there, and each of its curves
    // mapped to the sketch's own curve it is the offset of.
    let mut drawn: Vec<Vec<(ProfileLoop, BTreeMap<CurveId, CurveId>)>> = Vec::new();
    for &h in heights {
        let mut level = Vec::new();
        for lp in &loops {
            if h == 0.0 {
                level.push(((*lp).clone(), BTreeMap::new()));
                continue;
            }
            let curves: Vec<CurveId> = lp.edges.iter().map(|e| e.curve).collect();
            let chain = sketch.chain(&curves)?;
            let first = &chain.edges[0];
            let along = lp.edges.iter().find(|e| e.curve == first.curve);
            // The offset goes to the chain's left: inwards of a loop run as
            // the area runs it, which the chain may run the other way.
            let side = if along.is_some_and(|e| e.reversed == first.reversed) {
                1.0
            } else {
                -1.0
            };
            let d = h.abs() * slope * side;
            let ctx = with_context!("drafting by {angle} degrees: offsetting by {d} at height {h}");
            let offset = sketch
                .offset(&curves, Scalar::from_f64(d), Corners::Extend)
                .with_context(ctx)?;
            let of: BTreeMap<CurveId, CurveId> = (chain.edges.iter().map(|e| e.curve))
                .zip(offset.curves)
                .collect();
            let edges = lp
                .edges
                .iter()
                .map(|e| ProfileEdge {
                    curve: of[&e.curve],
                    reversed: e.reversed,
                })
                .collect();
            let back = of.into_iter().map(|(base, off)| (off, base)).collect();
            level.push((ProfileLoop { edges }, back));
        }
        drawn.push(level);
    }
    let geometry = sketch.enclose()?;
    let start = |sketch: &Sketch, e: &ProfileEdge| -> Option<PointId> {
        let (s, t) = sketch.curves[&e.curve].endpoints()?;
        Some(if e.reversed { t } else { s })
    };
    heights
        .iter()
        .zip(drawn)
        .map(|(&h, level)| {
            let mut profiles = Vec::new();
            let mut matched = Vec::new();
            for ((lp, back), base) in level.iter().zip(&loops) {
                // Named after the sketch's own curves and points.
                let points: BTreeMap<PointId, PointId> = lp
                    .edges
                    .iter()
                    .zip(&base.edges)
                    .filter_map(|(e, b)| Some((start(&sketch, e)?, start(&sketch, b)?)))
                    .collect();
                let curve = |c: CurveId| back.get(&c).copied().unwrap_or(c);
                let joint = |j: ProfileJoint| match j {
                    ProfileJoint::Point(p) => ProfileJoint::Point(*points.get(&p).unwrap_or(&p)),
                    ProfileJoint::Split { curve: c, index } => ProfileJoint::Split {
                        curve: curve(c),
                        index,
                    },
                };
                let pieces: Vec<ProfilePiece> = lp
                    .to_nurbs(&sketch, &geometry)?
                    .into_iter()
                    .map(|p| ProfilePiece {
                        source: curve(p.source),
                        start: joint(p.start),
                        end: joint(p.end),
                        ..p
                    })
                    .collect();
                // Matched: where one of the sketch's curves meets the next,
                // or the seam of a circle.
                let mut at: Vec<usize> = (0..pieces.len())
                    .filter(|&i| matches!(pieces[i].start, ProfileJoint::Point(_)))
                    .collect();
                if at.is_empty() {
                    at.push(0);
                }
                let profile = chain_of_pieces(&placed.name, pieces, true);
                matched.push(at.iter().map(|&i| profile.joint_names[i].clone()).collect());
                profiles.push(profile);
            }
            Ok(Level {
                height: Scalar::from_f64(h),
                loops: profiles,
                matched,
            })
        })
        .collect()
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
