//! [`AddDatum`]: reference geometry — a point, an axis, a plane or a
//! coordinate system — built from entities picked in the part, in one of the
//! ways CAD systems commonly offer (see [`Construction`]).
//!
//! Which constructions can be chosen depends on what is selected, and on its
//! shape, not only its kind: a straight edge is a line, a circular one has a
//! center and an axis, a flat face is a plane (see [`Shape`]).
//! [`fitting_constructions`] tells an editor which constructions fit a
//! selection, by exactly the matching a step applies.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{CanvasEvent, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumKind},
    scalars::{Field, Ring, Scalar},
    vector::Vector3,
    with_context,
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader, values::Design};
use serde::{Deserialize, Serialize};

use geop_ops_model::datums::{Datums, datums};
use geop_ops_model::frame_along;

use crate::{
    editor,
    selection::{Role, Selected, Shape},
};

/// What kind of value a construction takes besides its selection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamKind {
    /// A number, or a formula of the part's parameters (see
    /// [`NumericInput`]);
    /// `min`/`max` bound what a slider offers, not what is valid.
    Number {
        default: f64,
        min: f64,
        max: f64,
        unit: Unit,
    },
    Bool {
        default: bool,
    },
}

/// A value a construction takes besides its selection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Param {
    pub name: &'static str,
    pub doc: &'static str,
    pub kind: ParamKind,
}

/// One way to build a datum (see [`Construction`]): what it builds, what it
/// needs selected — one entity per input, in any order — and the values it
/// takes besides.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstructionSchema {
    /// How the construction is spelled: `offset`.
    pub method: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    pub result: DatumKind,
    pub inputs: &'static [Role],
    pub params: &'static [Param],
}

/// Declares [`Construction`] and [`CONSTRUCTIONS`] from one table, so the
/// two can't disagree: each construction's variant, method name, label,
/// the inputs it needs selected, what it builds, what it does, and the
/// values it takes besides — each with its kind.
macro_rules! constructions {
    ($(
        $variant:ident $method:literal $label:literal [$($role:ident),*] -> $result:ident,
        $doc:literal {
            $($param:ident: $pty:ty = $pkind:ident { $($kfield:ident: $kval:expr),* }, $pdoc:literal;)*
        }
    )*) => {
        /// How a datum is built from its selection: one variant per way,
        /// with the values it takes besides. Serialized with its method:
        /// `{"method": "offset", "distance": 1.0}`.
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(tag = "method")]
        pub enum Construction {
            $(
                #[doc = $doc]
                #[serde(rename = $method)]
                $variant { $(#[doc = $pdoc] $param: $pty),* },
            )*
        }

        /// Every [`Construction`], described for an editor.
        pub const CONSTRUCTIONS: &[ConstructionSchema] = &[$(
            ConstructionSchema {
                method: $method,
                label: $label,
                doc: $doc,
                result: DatumKind::$result,
                inputs: &[$(Role::$role),*],
                params: &[$(Param {
                    name: stringify!($param),
                    doc: $pdoc,
                    kind: ParamKind::$pkind { $($kfield: $kval),* },
                }),*],
            },
        )*];

        impl Construction {
            /// How it is described among [`CONSTRUCTIONS`].
            pub fn schema(&self) -> &'static ConstructionSchema {
                let method = match self {
                    $(Construction::$variant { .. } => $method,)*
                };
                CONSTRUCTIONS
                    .iter()
                    .find(|c| c.method == method)
                    .expect("every construction is described")
            }
        }
    };
}

constructions! {
    // ── points ──
    Point "point" "Point" [Point] -> Point,
    "A point offset from the selected one: along its own axes if it is a datum or the origin, along the world's otherwise. Its frame is that point's, moved." {
        x: NumericInput = Number { default: 0.0, min: -10.0, max: 10.0, unit: Unit::Length }, "How far along x.";
        y: NumericInput = Number { default: 0.0, min: -10.0, max: 10.0, unit: Unit::Length }, "How far along y.";
        z: NumericInput = Number { default: 0.0, min: -10.0, max: 10.0, unit: Unit::Length }, "How far along z.";
    }
    Midpoint "midpoint" "Midpoint" [Point, Point] -> Point,
    "The point halfway between two points." {}
    EdgePoint "edge_point" "Point on edge" [Curve] -> Point,
    "A point along an edge or a sketch's curve, its z axis along it." {
        position: NumericInput = Number { default: 0.5, min: 0.0, max: 1.0, unit: Unit::Fraction }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }
    Center "center" "Center" [Circle] -> Point,
    "The center of a circular edge or a sketch's arc or circle, its z axis the one it turns around." {}
    ProjectOnPlane "project_on_plane" "Projection onto plane" [Point, Plane] -> Point,
    "The foot of the perpendicular dropped from a point onto a plane." {}
    ProjectOnLine "project_on_line" "Projection onto line" [Point, Line] -> Point,
    "The foot of the perpendicular dropped from a point onto a line." {}
    LinePlane "line_plane" "Line meets plane" [Line, Plane] -> Point,
    "Where a line pierces a plane." {}
    LineLine "line_line" "Lines meet" [Line, Line] -> Point,
    "Where two lines cross — or, if they miss each other, halfway between where they come closest. Its z axis is normal to both." {}
    ThreePlanes "three_planes" "Three planes meet" [Plane, Plane, Plane] -> Point,
    "The one point three planes share." {}

    // ── axes ──
    TwoPoints "two_points" "Line through points" [Point, Point] -> Axis,
    "The line from one point through another." {}
    AlongLine "along_line" "Along line" [Line] -> Axis,
    "The line a straight edge or an axis runs along." {}
    AxisOf "axis_of" "Axis of arc or cylinder" [Round] -> Axis,
    "The axis a circular edge, or a cylindrical, conical or spherical face, turns around." {}
    PlanePlane "plane_plane" "Two planes meet" [Plane, Plane] -> Axis,
    "The line two planes meet in." {}
    Perpendicular "perpendicular" "Perpendicular to plane" [Point, Plane] -> Axis,
    "The perpendicular dropped from a point onto a plane: the line through the point along the plane's normal." {}
    Parallel "parallel" "Parallel through point" [Point, Line] -> Axis,
    "The line through a point parallel to a line." {}
    PerpendicularToLine "perpendicular_to_line" "Perpendicular to line" [Point, Line] -> Axis,
    "The perpendicular dropped from a point onto a line: from the point to its foot on the line." {}
    Bisector "bisector" "Angle bisector" [Line, Line] -> Axis,
    "The line halving the angle between two crossing lines, through where they cross — or, between parallel lines, the line halfway between them." {
        other: bool = Bool { default: false }, "Halve the other angle: the one between the first line and the second one reversed.";
    }
    Tangent "tangent" "Tangent to edge" [Curve] -> Axis,
    "The tangent to an edge or a sketch's curve at a point along it." {
        position: NumericInput = Number { default: 0.5, min: 0.0, max: 1.0, unit: Unit::Fraction }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }

    // ── planes ──
    Offset "offset" "Offset plane" [Plane] -> Plane,
    "A plane parallel to the selected one, a distance along its normal." {
        distance: NumericInput = Number { default: 1.0, min: -10.0, max: 10.0, unit: Unit::Length }, "How far along the plane's normal; backwards if negative.";
    }
    Midplane "midplane" "Midplane" [Plane, Plane] -> Plane,
    "The plane halfway between two parallel planes, or halving the angle between two that meet." {
        other: bool = Bool { default: false }, "For planes that meet: halve the other angle between them.";
    }
    ThreePoints "three_points" "Plane through points" [Point, Point, Point] -> Plane,
    "The plane through three points." {}
    Angle "angle" "Plane at angle" [Plane, Line] -> Plane,
    "The plane through a line at an angle to a plane: turned around the line from the plane through it most nearly parallel to the selected one — which, for a line parallel to that plane, is parallel to it." {
        angle: NumericInput = Number { default: 45.0, min: -180.0, max: 180.0, unit: Unit::Angle }, "How far to turn, in degrees, right-handed about the line's direction.";
    }
    LinePoint "line_point" "Plane through line and point" [Line, Point] -> Plane,
    "The plane through a line and a point off it." {}
    TwoLines "two_lines" "Plane through lines" [Line, Line] -> Plane,
    "The plane two crossing or parallel lines lie in — for lines that miss each other, the plane through the first parallel to the second." {}
    ParallelPlane "parallel_plane" "Parallel plane through point" [Plane, Point] -> Plane,
    "The plane through a point parallel to a plane." {}
    NormalToLine "normal_to_line" "Plane normal to line" [Line, Point] -> Plane,
    "The plane through a point perpendicular to a line." {}
    NormalToEdge "normal_to_edge" "Plane normal to edge" [Curve] -> Plane,
    "The plane perpendicular to an edge or a sketch's curve at a point along it." {
        position: NumericInput = Number { default: 0.5, min: 0.0, max: 1.0, unit: Unit::Fraction }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }

    // ── coordinate systems ──
    FrameThreePoints "frame_three_points" "Coordinate system through points" [Point, Point, Point] -> Frame,
    "The coordinate system at the first point, its x axis towards the second and its xy plane through the third." {}
}

/// Adds a datum to the part, named by the operation's id: a point, an
/// axis, a plane or a coordinate system built from the selected entities by
/// one of the
/// [`Construction`]s. Like a sketch's plane, it is built when the step runs
/// and stays where it was, whatever later steps do to what it was built
/// from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddDatum;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AddDatumArgs {
    /// What it is built from: points, edges, faces and other references,
    /// picked in the viewport.
    pub selection: Vec<Selected>,
    /// How it is built from them; only the ways that fit what is selected
    /// can be chosen.
    pub construction: Construction,
}

/// Which selected entity fills each of `inputs`, in order: one entity per
/// input, each able to fill its role, and of all such assignments the first
/// in selection order. `None` if there is none.
fn assign(inputs: &[Role], selection: &[Shape]) -> Option<Vec<usize>> {
    fn extend(inputs: &[Role], selection: &[Shape], chosen: &mut Vec<usize>) -> bool {
        let Some(role) = inputs.get(chosen.len()) else {
            return true;
        };
        for (i, geometry) in selection.iter().enumerate() {
            if !chosen.contains(&i) && geometry.fits(*role) {
                chosen.push(i);
                if extend(inputs, selection, chosen) {
                    return true;
                }
                chosen.pop();
            }
        }
        false
    }
    let mut chosen = Vec::new();
    (inputs.len() == selection.len() && extend(inputs, selection, &mut chosen)).then_some(chosen)
}

/// The method of every construction that fits `selection` as `reader`
/// reads the part — by the same matching a step applies. None, if the part
/// lacks an element of it.
pub fn fitting_constructions(reader: &Reader, selection: &[Selected]) -> Vec<&'static str> {
    let Ok(resolved) = selection
        .iter()
        .map(|e| e.shape(reader))
        .collect::<GeopResult<Vec<_>>>()
    else {
        return Vec::new();
    };
    CONSTRUCTIONS
        .iter()
        .filter(|c| assign(c.inputs, &resolved).is_some())
        .map(|c| c.method)
        .collect()
}

impl AddDatumArgs {
    /// The selection resolved as `reader` reads the part, and ordered as
    /// the construction takes it.
    pub(crate) fn inputs(&self, reader: &Reader) -> GeopResult<Vec<Shape>> {
        let schema = self.construction.schema();
        let resolved = self
            .selection
            .iter()
            .map(|e| e.shape(reader))
            .collect::<GeopResult<Vec<_>>>()?;
        let Some(order) = assign(schema.inputs, &resolved) else {
            let needs: Vec<&str> = schema.inputs.iter().map(|&r| r.describe()).collect();
            return Err(GeopError::new(format!(
                "{} needs {} selected, one each, and nothing else",
                schema.label,
                needs.join(" and ")
            )));
        };
        Ok(order.into_iter().map(|i| resolved[i].clone()).collect())
    }
}

/// The plane of a frame whose `w` is its normal.
fn plane_of<S: Scalar>(frame: &CoordinateSystem<S>) -> Plane<S> {
    Plane {
        point: *frame.origin(),
        normal: *frame.w(),
    }
}

/// `frame`'s axes, at `origin`.
fn moved<S: Scalar>(
    frame: &CoordinateSystem<S>,
    origin: Vector3<S>,
) -> GeopResult<CoordinateSystem<S>> {
    CoordinateSystem::try_new(origin, *frame.u(), *frame.v(), *frame.w())
}

/// The point `position` of the way along the edge `edge` (see
/// [`Construction::EdgePoint`]), and the unit tangent there.
fn along_edge(edge: &Shape, position: f64) -> GeopResult<(Vector3<Design>, Vector3<Design>)> {
    type S = Design;
    if !(0.0..=1.0).contains(&position) {
        return Err(GeopError::new(format!(
            "position {position} is not along the edge: it must be from 0 to 1"
        )));
    }
    let curve = edge.curve.as_ref().expect("an edge has a curve");
    let (t0, t1) = curve.domain();
    let fraction = S::from_f64(position);
    if let Some(line) = &edge.line {
        let (a, b) = (curve.evaluate(t0)?, curve.evaluate(t1)?);
        return Ok((Vector3::interpolate(&a, &b, fraction), line.direction));
    }
    if let Some(arc) = &edge.arc {
        let p = arc.point_at(position)?;
        return Ok((p, arc.tangent_at(&p)?));
    }
    let t = S::interpolate(t0, t1, fraction);
    Ok((curve.evaluate(t)?, curve.tangent(t)?.normalize()?))
}

/// `x` degrees, in radians.
fn radians<S: Scalar>(degrees: f64) -> S {
    S::from_f64(degrees.to_radians())
}

impl Construction {
    /// The construction `schema` describes, every value at its default.
    pub fn default_of(schema: &ConstructionSchema) -> Self {
        let mut json = serde_json::json!({ "method": schema.method });
        for param in schema.params {
            json[param.name] = match param.kind {
                ParamKind::Number { default, .. } => default.into(),
                ParamKind::Bool { default } => default.into(),
            };
        }
        serde_json::from_value(json).expect("every construction reads from its schema")
    }

    /// The value it takes named `name`, as it serializes.
    pub fn param(&self, name: &str) -> Option<serde_json::Value> {
        serde_json::to_value(self).ok()?.get(name).cloned()
    }

    /// It with the value named `name` set to `value` — `None` if it takes
    /// no such value, or not of that kind.
    pub fn with_param(&self, name: &str, value: serde_json::Value) -> Option<Self> {
        let mut json = serde_json::to_value(self).ok()?;
        json.get(name)?;
        json[name] = value;
        serde_json::from_value(json).ok()
    }

    /// The frame it builds from `inputs`, ordered as it takes them (see
    /// [`AddDatumArgs::inputs`]), `value` giving the values of its numbers
    /// — as a step reads them, or as a form shows them.
    pub(crate) fn build(
        &self,
        inputs: &[Shape],
        mut value: impl FnMut(&NumericInput) -> GeopResult<f64>,
    ) -> GeopResult<CoordinateSystem<Design>> {
        type S = Design;
        let point = |i: usize| inputs[i].point.expect("assigned a point");
        let line = |i: usize| inputs[i].line.clone().expect("assigned a line");
        let frame = |i: usize| inputs[i].plane.clone().expect("assigned a plane");
        let plane = |i: usize| plane_of(&frame(i));
        let half = S::ONE.div(S::TWO)?;
        match self {
            Construction::Point { x, y, z } => {
                let p = point(0);
                let base = match &inputs[0].frame {
                    Some(frame) => frame.clone(),
                    None => CoordinateSystem::world_at(p),
                };
                let xyz = [value(x)?, value(y)?, value(z)?];
                let offset = base.to_xyz(&Vector3::from_array(xyz.map(S::from_f64)));
                moved(&base, offset)
            }
            Construction::Midpoint {} => Ok(CoordinateSystem::world_at(Vector3::interpolate(
                &point(0),
                &point(1),
                half,
            ))),
            Construction::EdgePoint { position } => {
                let (p, tangent) = along_edge(&inputs[0], value(position)?)?;
                frame_along(p, &tangent)
            }
            Construction::Center {} => {
                let arc = inputs[0].arc.clone().expect("assigned an arc");
                let c = arc.circle;
                let u = arc.start.sub(&c.center).normalize()?;
                CoordinateSystem::try_from_xz(c.center, u, c.normal)
            }
            Construction::ProjectOnPlane {} => {
                let foot = plane(1).project(&point(0));
                moved(&frame(1), foot)
            }
            Construction::ProjectOnLine {} => {
                let l = line(1);
                frame_along(l.project(&point(0)), &l.direction)
            }
            Construction::LinePlane {} => {
                let at = plane(1).intersect_axis(&line(0))?;
                moved(&frame(1), at)
            }
            Construction::LineLine {} => {
                let (a, b) = (line(0), line(1));
                let at = a.nearest(&b)?;
                let w = a.direction.prod_cross(&b.direction).normalize()?;
                CoordinateSystem::try_from_xz(at, a.direction, w)
            }
            Construction::ThreePlanes {} => {
                let meet = plane(0).intersect_plane(&plane(1))?;
                Ok(CoordinateSystem::world_at(plane(2).intersect_axis(&meet)?))
            }
            Construction::TwoPoints {} => {
                let (a, b) = (point(0), point(1));
                let d = b.sub(&a);
                if d.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the points coincide: no line runs through both",
                    ));
                }
                frame_along(a, &d)
            }
            Construction::AlongLine {} => {
                let l = line(0);
                frame_along(l.point, &l.direction)
            }
            Construction::AxisOf {} => {
                let axis = inputs[0].round.clone().expect("assigned something round");
                frame_along(axis.point, &axis.direction)
            }
            Construction::PlanePlane {} => {
                let meet = plane(0).intersect_plane(&plane(1))?;
                frame_along(meet.point, &meet.direction)
            }
            Construction::Perpendicular {} => frame_along(point(0), &plane(1).normal),
            Construction::Parallel {} => frame_along(point(0), &line(1).direction),
            Construction::PerpendicularToLine {} => {
                let p = point(0);
                let d = line(1).project(&p).sub(&p);
                if d.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the point lies on the line: there is no perpendicular to drop",
                    ));
                }
                frame_along(p, &d)
            }
            Construction::Bisector { other } => {
                let (a, b) = (line(0), line(1));
                if a.could_be_parallel(&b) {
                    let mid = Vector3::interpolate(&a.point, &b.project(&a.point), half);
                    return frame_along(mid, &a.direction);
                }
                let at = a.nearest(&b)?;
                let second = if *other {
                    b.direction.neg()
                } else {
                    b.direction
                };
                frame_along(at, &a.direction.add(&second))
            }
            Construction::Tangent { position } => {
                let (p, tangent) = along_edge(&inputs[0], value(position)?)?;
                frame_along(p, &tangent)
            }
            Construction::Offset { distance } => {
                let f = frame(0);
                moved(
                    &f,
                    f.origin()
                        .add(&f.w().prod_scalar(S::from_f64(value(distance)?))),
                )
            }
            Construction::Midplane { other } => {
                // Every point as far in front of one plane as behind the
                // other (`inner`: halving the angle between them, or the
                // gap between parallel planes facing apart), or as far in
                // front of both (`outer`): with `d_k = n_k . p_k`, the
                // planes `(n1 -+ n2) . x = d1 -+ d2`. Neither needs the line
                // the planes meet in, which runs off to infinity as they
                // turn parallel.
                let (p, q) = (plane(0), plane(1));
                let (d1, d2) = (p.point.prod_dot(&p.normal), q.point.prod_dot(&q.normal));
                let inner = (p.normal.sub(&q.normal), d1.sub(d2));
                let outer = (p.normal.add(&q.normal), d1.add(d2));
                let (first, second) = if *other {
                    (outer, inner)
                } else {
                    (inner, outer)
                };
                // One of the two is only degenerate for parallel planes,
                // where the other is the plane halfway between them.
                let (normal, offset) = if first.0.norm_sq().could_be_equal(S::ZERO) {
                    second
                } else {
                    first
                };
                let n2 = normal.norm_sq();
                let midplane = Plane::try_new(normal.prod_scalar(offset.div(n2)?), normal)?;
                let near = Vector3::interpolate(frame(0).origin(), frame(1).origin(), half);
                frame_along(midplane.project(&near), &midplane.normal)
            }
            Construction::ThreePoints {} => {
                let (a, b, c) = (point(0), point(1), point(2));
                let n = b.sub(&a).prod_cross(&c.sub(&a));
                if n.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the points lie on one line: every plane through that line runs through them",
                    ));
                }
                frame_along(a, &n)
            }
            Construction::Angle { angle } => {
                let (n, l) = (plane(0).normal, line(1));
                // The selected plane's normal, turned square to the line: the
                // normal of the plane through the line most nearly parallel
                // to the selected one.
                let square = n.sub(&l.direction.prod_scalar(l.direction.prod_dot(&n)));
                if square.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the line is perpendicular to the plane: every plane through it is at right angles to it",
                    ));
                }
                let a: S = radians(value(angle)?);
                let normal = square
                    .prod_scalar(a.cos())
                    .add(&l.direction.prod_cross(&square).prod_scalar(a.sin()));
                frame_along(l.point, &normal)
            }
            Construction::LinePoint {} => {
                let l = line(0);
                let n = l.direction.prod_cross(&point(1).sub(&l.point));
                if n.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the point lies on the line: every plane through the line runs through it",
                    ));
                }
                frame_along(l.point, &n)
            }
            Construction::TwoLines {} => {
                let (a, b) = (line(0), line(1));
                let n = if a.could_be_parallel(&b) {
                    let n = a.direction.prod_cross(&b.point.sub(&a.point));
                    if n.norm_sq().could_be_equal(S::ZERO) {
                        return Err(GeopError::new(
                            "the lines coincide: every plane through one runs through the other",
                        ));
                    }
                    n
                } else {
                    a.direction.prod_cross(&b.direction)
                };
                frame_along(a.point, &n)
            }
            Construction::ParallelPlane {} => {
                let f = frame(0);
                let d = plane(0).signed_distance(&point(1));
                moved(&f, f.origin().add(&f.w().prod_scalar(d)))
            }
            Construction::NormalToLine {} => frame_along(point(1), &line(0).direction),
            Construction::NormalToEdge { position } => {
                let (p, tangent) = along_edge(&inputs[0], value(position)?)?;
                frame_along(p, &tangent)
            }
            Construction::FrameThreePoints {} => {
                let (a, b, c) = (point(0), point(1), point(2));
                let x = b.sub(&a);
                let z = x.prod_cross(&c.sub(&a));
                if z.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(
                        "the points lie on one line: they pick out no plane for x and y",
                    ));
                }
                let (u, w) = (x.normalize()?, z.normalize()?);
                CoordinateSystem::try_from_xz(a, u, w)
            }
        }
    }
}

/// The datums with the one the step `step` adds with `args`, built from its
/// selection as the part before it has it.
fn add(reader: &Reader, (step, args): &(String, AddDatumArgs)) -> GeopResult<Datums> {
    let ctx = with_context!("add_datum({step}, {args:?})");
    let inputs = args.inputs(reader).with_context(ctx)?;
    let frame = (args.construction)
        .build(&inputs, |f| f.resolve(reader))
        .with_context(ctx)?;
    let mut datums: Datums = reader.previous()?;
    let datum = Datum {
        kind: args.construction.schema().result,
        frame,
    };
    datums.insert(step, datum).with_context(ctx)?;
    Ok(datums)
}

part_operation!(AddDatum(AddDatumArgs));

impl PartOperation for AddDatum {
    const KIND: &'static str = "add_datum";
    const INFO: OperationInfo = OperationInfo::new(
        "Reference",
        "Add a point, an axis, a plane or a coordinate system to build with, made from what is picked.",
        OperationGroup::Sketch,
        OperationTier::Small,
    )
    .icon(crate::icons::ADD_DATUM);
    type Args = AddDatumArgs;
    type Session = ();

    /// Nothing selected yet, and the first construction.
    fn new_args(_: &Context) -> AddDatumArgs {
        AddDatumArgs {
            selection: Vec::new(),
            construction: Construction::default_of(&CONSTRUCTIONS[0]),
        }
    }

    /// Adds the datum named after the step to the part's datums.
    fn run(ctx: &mut StepContext, args: &AddDatumArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&datums(), &(step.to_string(), args.clone()), add)
    }

    /// Picking the selection, choosing among the constructions that fit
    /// it, and offsets as handles and a gizmo: see [`crate::editor`].
    fn form<'a>(
        context: &'a Context<'a>,
        args: &AddDatumArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, AddDatumArgs> {
        editor::form(&context.before, args)
    }

    fn canvas(context: &Context, edit: Edit<'_, AddDatumArgs, ()>, event: &CanvasEvent<Design>) {
        editor::event(&context.before, edit.args, event);
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        primitives::{DatumComponent, FrameAxis},
        scalars::ScalInF64 as S,
    };
    use geop_core_program::{element::ElementReference, operation::Operation};
    use geop_ops_model::datums::{AnyDatum, ORIGIN, component_name, read_datums};
    use geop_ops_model::model;

    use super::*;

    fn datum(name: impl Into<String>) -> Selected {
        Selected::Datum(ElementReference::<Datums, AnyDatum>::new(datums(), name))
    }
    fn origin() -> Selected {
        datum(ORIGIN)
    }
    fn base(normal: FrameAxis) -> Selected {
        datum(component_name(ORIGIN, DatumComponent::Plane(normal)))
    }
    fn axis(along: FrameAxis) -> Selected {
        datum(component_name(ORIGIN, DatumComponent::Axis(along)))
    }
    fn v(p: [f64; 3]) -> Vector3<S> {
        Vector3::from_array(p.map(S::from_f64))
    }

    /// The datum step `d` with `args` adds to a part of nothing but the
    /// origin, or why it fails.
    fn added(args: &AddDatumArgs) -> GeopResult<Datum<S>> {
        let mut part = geop_core_program::part::Part::default();
        let runner = geop_core_program::runner::Runner::new(Default::default());
        runner.run_step(&mut part, "d", |ctx| {
            <AddDatum as Operation>::run(ctx, args)
        })?;
        let datums = read_datums(&part.current())?.into_owned();
        datums.get("d")
    }

    /// Every construction serializes under its own method, with exactly the
    /// values its schema lists, and back.
    #[test]
    fn constructions_serialize_as_described() {
        for schema in CONSTRUCTIONS {
            let mut json = serde_json::json!({ "method": schema.method });
            for param in schema.params {
                json[param.name] = match param.kind {
                    ParamKind::Number { default, .. } => default.into(),
                    ParamKind::Bool { default } => default.into(),
                };
            }
            let construction: Construction = serde_json::from_value(json.clone())
                .unwrap_or_else(|e| panic!("{}: {e}", schema.method));
            assert_eq!(construction.schema(), schema);
            assert_eq!(serde_json::to_value(&construction).unwrap(), json);
            assert_eq!(Construction::default_of(schema), construction);
        }
    }

    /// A datum is added to the part's datums: built where its selection
    /// puts it.
    #[test]
    fn an_offset_plane_is_added() {
        let args = AddDatumArgs {
            selection: vec![base(FrameAxis::Z)],
            construction: Construction::Offset {
                distance: 2.0.into(),
            },
        };
        let datum = added(&args).unwrap();
        assert_eq!(datum.kind, DatumKind::Plane);
        assert!(datum.frame.origin().could_be_equal(&v([0.0, 0.0, 2.0])));
        assert!(datum.frame.w().could_be_equal(&v([0.0, 0.0, 1.0])));
        assert_eq!(
            serde_json::to_value(&args.selection).unwrap(),
            serde_json::json!([{"datum": "origin:xy"}])
        );
    }

    #[test]
    fn a_selection_that_does_not_fit_says_what_it_needs() {
        let args = AddDatumArgs {
            selection: vec![origin()],
            construction: Construction::Offset {
                distance: 1.0.into(),
            },
        };
        let Err(e) = added(&args) else {
            panic!("an offset plane from a point");
        };
        assert!(e.to_string().contains("needs a plane selected"), "{e}");
    }

    /// Degenerate selections are refused, saying why.
    #[test]
    fn degenerate_selections_fail() {
        let err = |selection: Vec<Selected>, construction: Construction| {
            let args = AddDatumArgs {
                selection,
                construction,
            };
            match added(&args) {
                Ok(_) => panic!("{args:?} built a datum"),
                Err(e) => format!("{e:?}"),
            }
        };
        assert!(err(vec![origin(), origin()], Construction::TwoPoints {}).contains("coincide"));
        assert!(
            err(
                vec![base(FrameAxis::Z), base(FrameAxis::Z)],
                Construction::PlanePlane {}
            )
            .contains("parallel")
        );
        assert!(
            err(
                vec![origin(), axis(FrameAxis::X)],
                Construction::PerpendicularToLine {}
            )
            .contains("lies on the line")
        );
        assert!(
            err(
                vec![base(FrameAxis::Z), axis(FrameAxis::Z)],
                Construction::Angle { angle: 10.0.into() }
            )
            .contains("perpendicular")
        );
        let nowhere = Selected::Edge(ElementReference::new(model(), "nowhere"));
        assert!(err(vec![nowhere], Construction::AlongLine {}).contains("nowhere"));
    }
}
