//! [`AddDatum`]: reference geometry — a point, an axis or a plane — built
//! from entities picked in the part, in one of the ways CAD systems commonly
//! offer (see [`Construction`]).
//!
//! Which constructions can be chosen depends on what is selected, and on its
//! shape, not only its kind: a straight edge is a line, a circular one has a
//! center and an axis, a flat face is a plane (see [`Geometry`]).
//! [`inspect_selection`] tells an editor which constructions fit a
//! selection, by exactly the matching a step applies.

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_part::{Datum, DatumKind, Part};
use geop_ops_parts_derive::OperationArgs;
use serde::{Deserialize, Serialize};

use super::{
    ArgKind, ArgSchema, EntityRef, Handle, HandleGroup, HandleMotion, Operation, arg_path,
    entity::{Geometry, Role, frame_along, world_frame},
    extrude::to_f64,
    schema::ConstructionSchema,
};

/// Declares [`Construction`] and [`CONSTRUCTIONS`] from one table, so the
/// two can't disagree: each construction's variant, method name, label,
/// the inputs it needs selected, what it builds, what it does, and the
/// values it takes besides — each with its kind, as an argument's.
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
                params: &[$(ArgSchema {
                    name: stringify!($param),
                    doc: $pdoc,
                    kind: ArgKind::$pkind { $($kfield: $kval),* },
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
        x: f64 = Number { default: 0.0, min: -10.0, max: 10.0 }, "How far along x.";
        y: f64 = Number { default: 0.0, min: -10.0, max: 10.0 }, "How far along y.";
        z: f64 = Number { default: 0.0, min: -10.0, max: 10.0 }, "How far along z.";
    }
    Midpoint "midpoint" "Midpoint" [Point, Point] -> Point,
    "The point halfway between two points." {}
    EdgePoint "edge_point" "Point on edge" [Edge] -> Point,
    "A point along an edge, its z axis along the edge." {
        position: f64 = Number { default: 0.5, min: 0.0, max: 1.0 }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }
    Center "center" "Center" [Circle] -> Point,
    "The center of a circular edge, its z axis the one the arc turns around." {}
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
    Tangent "tangent" "Tangent to edge" [Edge] -> Axis,
    "The tangent to an edge at a point along it." {
        position: f64 = Number { default: 0.5, min: 0.0, max: 1.0 }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }

    // ── planes ──
    Offset "offset" "Offset plane" [Plane] -> Plane,
    "A plane parallel to the selected one, a distance along its normal." {
        distance: f64 = Number { default: 1.0, min: -10.0, max: 10.0 }, "How far along the plane's normal; backwards if negative.";
    }
    Midplane "midplane" "Midplane" [Plane, Plane] -> Plane,
    "The plane halfway between two parallel planes, or halving the angle between two that meet." {
        other: bool = Bool { default: false }, "For planes that meet: halve the other angle between them.";
    }
    ThreePoints "three_points" "Plane through points" [Point, Point, Point] -> Plane,
    "The plane through three points." {}
    Angle "angle" "Plane at angle" [Plane, Line] -> Plane,
    "The plane through a line at an angle to a plane: turned around the line from the plane through it most nearly parallel to the selected one — which, for a line parallel to that plane, is parallel to it." {
        angle: f64 = Number { default: 45.0, min: -180.0, max: 180.0 }, "How far to turn, in degrees, right-handed about the line's direction.";
    }
    LinePoint "line_point" "Plane through line and point" [Line, Point] -> Plane,
    "The plane through a line and a point off it." {}
    TwoLines "two_lines" "Plane through lines" [Line, Line] -> Plane,
    "The plane two crossing or parallel lines lie in — for lines that miss each other, the plane through the first parallel to the second." {}
    ParallelPlane "parallel_plane" "Parallel plane through point" [Plane, Point] -> Plane,
    "The plane through a point parallel to a plane." {}
    NormalToLine "normal_to_line" "Plane normal to line" [Line, Point] -> Plane,
    "The plane through a point perpendicular to a line." {}
    NormalToEdge "normal_to_edge" "Plane normal to edge" [Edge] -> Plane,
    "The plane perpendicular to an edge at a point along it." {
        position: f64 = Number { default: 0.5, min: 0.0, max: 1.0 }, "Where along the edge, from its start (0) to its end (1): by length on a straight or circular edge, by parameter on any other.";
    }
}

/// Adds a datum to the part, named by the operation's id: a point, an axis
/// or a plane built from the selected entities by one of the
/// [`Construction`]s. Like a sketch's plane, it is built when the step runs
/// and stays where it was, whatever later steps do to what it was built
/// from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddDatum;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, OperationArgs)]
pub struct AddDatumArgs {
    /// What it is built from: points, edges, faces and other references,
    /// picked in the viewport.
    #[arg(Selection)]
    pub selection: Vec<EntityRef>,
    /// How it is built from them; only the ways that fit what is selected
    /// can be chosen.
    #[arg(Construction { selection: "selection", options: CONSTRUCTIONS })]
    pub construction: Construction,
}

/// Which selected entity fills each of `inputs`, in order: one entity per
/// input, each able to fill its role, and of all such assignments the first
/// in selection order. `None` if there is none.
fn assign<S: Scalar>(inputs: &[Role], selection: &[Geometry<S>]) -> Option<Vec<usize>> {
    fn extend<S: Scalar>(
        inputs: &[Role],
        selection: &[Geometry<S>],
        chosen: &mut Vec<usize>,
    ) -> bool {
        let Some(role) = inputs.get(chosen.len()) else {
            return true;
        };
        for (i, geometry) in selection.iter().enumerate() {
            if !chosen.contains(&i) && role.fits(geometry) {
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

/// What an editor offers for a selection: what each entity can be used as,
/// and which constructions fit — by the same matching a step applies.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SelectionFit {
    /// Per selected entity; none for one the part does not have.
    pub roles: Vec<Vec<Role>>,
    /// The method of every construction that fits.
    pub fits: Vec<&'static str>,
}

/// Which constructions fit `selection` in `part` (see [`SelectionFit`]).
pub fn inspect_selection<S: Scalar>(part: &Part<S>, selection: &[EntityRef]) -> SelectionFit {
    let resolved: Option<Vec<Geometry<S>>> =
        selection.iter().map(|e| e.resolve(part).ok()).collect();
    let roles = selection
        .iter()
        .map(|e| e.resolve(part).map(|g| g.roles()).unwrap_or_default())
        .collect();
    let fits = match &resolved {
        Some(resolved) => CONSTRUCTIONS
            .iter()
            .filter(|c| assign(c.inputs, resolved).is_some())
            .map(|c| c.method)
            .collect(),
        None => Vec::new(),
    };
    SelectionFit { roles, fits }
}

/// A role as a construction's requirement reads: `a point`.
fn describe_role(role: Role) -> &'static str {
    match role {
        Role::Point => "a point",
        Role::Line => "a line",
        Role::Plane => "a plane",
        Role::Edge => "an edge",
        Role::Circle => "a circular edge",
        Role::Round => "a circular edge or a round face",
    }
}

impl AddDatumArgs {
    /// The selection resolved in `part` and ordered as the construction
    /// takes it.
    fn inputs<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Vec<Geometry<S>>> {
        let schema = self.construction.schema();
        let resolved = self
            .selection
            .iter()
            .map(|e| e.resolve(part))
            .collect::<GeopResult<Vec<_>>>()?;
        let Some(order) = assign(schema.inputs, &resolved) else {
            let needs: Vec<&str> = schema.inputs.iter().map(|&r| describe_role(r)).collect();
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
fn along_edge<S: Scalar>(
    edge: &Geometry<S>,
    position: f64,
) -> GeopResult<(Vector3<S>, Vector3<S>)> {
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
    /// The frame it builds from `inputs`, ordered as it takes them (see
    /// [`AddDatumArgs::inputs`]).
    fn build<S: Scalar>(&self, inputs: &[Geometry<S>]) -> GeopResult<CoordinateSystem<S>> {
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
                    None => world_frame(p)?,
                };
                let offset = base.to_xyz(&Vector3::from_array([*x, *y, *z].map(S::from_f64)));
                moved(&base, offset)
            }
            Construction::Midpoint {} => {
                world_frame(Vector3::interpolate(&point(0), &point(1), half))
            }
            Construction::EdgePoint { position } => {
                let (p, tangent) = along_edge(&inputs[0], *position)?;
                frame_along(p, &tangent)
            }
            Construction::Center {} => {
                let arc = inputs[0].arc.clone().expect("assigned an arc");
                let c = arc.circle;
                let u = arc.start.sub(&c.center).normalize()?;
                CoordinateSystem::try_new(c.center, u, c.normal.prod_cross(&u), c.normal)
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
                CoordinateSystem::try_new(at, a.direction, w.prod_cross(&a.direction), w)
            }
            Construction::ThreePlanes {} => {
                let meet = plane(0).intersect_plane(&plane(1))?;
                world_frame(plane(2).intersect_axis(&meet)?)
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
                let (p, tangent) = along_edge(&inputs[0], *position)?;
                frame_along(p, &tangent)
            }
            Construction::Offset { distance } => {
                let f = frame(0);
                moved(
                    &f,
                    f.origin().add(&f.w().prod_scalar(S::from_f64(*distance))),
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
                let a: S = radians(*angle);
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
                let (p, tangent) = along_edge(&inputs[0], *position)?;
                frame_along(p, &tangent)
            }
        }
    }
}

/// How far out along its axis an offset point's handle sits, in world units.
const POINT_HANDLE_OUT: f64 = 0.3;

impl<S: Scalar> Operation<S> for AddDatum {
    type Args = AddDatumArgs;

    fn apply(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &AddDatumArgs,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("add_datum({operation_id}, {args:?})");
        let frame = args
            .construction
            .build(&args.inputs(&part).with_context(ctx)?)
            .with_context(ctx)?;
        let datum = Datum {
            kind: args.construction.schema().result,
            frame,
        };
        part.add_datum(datum, operation_id).with_context(ctx)?;
        Ok(part)
    }

    /// An offset plane's distance, as a handle on the plane sliding along
    /// its normal; an offset point's offsets, as a handle on the point per
    /// axis, sliding along it.
    fn handles(&self, before: &Part<S>, args: &AddDatumArgs) -> GeopResult<Vec<Handle>> {
        let inputs = args.inputs(before)?;
        let built = args.construction.build(&inputs)?;
        let at = to_f64(built.origin());
        // Handles of one point sit a little out along their axes, so each
        // can be grabbed.
        let handle = |label: &str, direction: &Vector3<S>, value: f64, out: f64| Handle {
            label: label.into(),
            group: HandleGroup::Feature,
            position: {
                let d = to_f64(direction);
                [0, 1, 2].map(|k| at[k] + d[k] * out)
            },
            motion: HandleMotion::Linear {
                direction: to_f64(direction),
                arg: arg_path(&["construction", label]),
                value,
                scale: 1.0,
            },
        };
        Ok(match &args.construction {
            Construction::Offset { distance } => {
                vec![handle("distance", built.w(), *distance, 0.0)]
            }
            Construction::Point { x, y, z } => vec![
                handle("x", built.u(), *x, POINT_HANDLE_OUT),
                handle("y", built.v(), *y, POINT_HANDLE_OUT),
                handle("z", built.w(), *z, POINT_HANDLE_OUT),
            ],
            _ => Vec::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::scalars::ScalInF64 as S;

    use super::*;
    use crate::{PartOperation, Program, WorldAxis, examples};

    fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    fn face(name: &str) -> EntityRef {
        EntityRef::Face { name: name.into() }
    }
    fn edge(name: &str) -> EntityRef {
        EntityRef::Edge { name: name.into() }
    }
    fn vertex(name: &str) -> EntityRef {
        EntityRef::Vertex { name: name.into() }
    }
    fn axis(axis: WorldAxis) -> EntityRef {
        EntityRef::Axis { axis }
    }
    fn base(normal: WorldAxis) -> EntityRef {
        EntityRef::Plane { normal }
    }

    /// The 2 x 2 x 1 box with a hole of radius 0.4 drilled 0.5 deep into
    /// the middle of its top.
    fn drilled_box() -> Part<S> {
        examples::box_with_drill_hole().apply(Part::new()).unwrap()
    }

    /// The datum built from `selection` by `construction` in `part`.
    fn datum(part: &Part<S>, selection: Vec<EntityRef>, construction: Construction) -> Datum<S> {
        let args = AddDatumArgs {
            selection,
            construction,
        };
        let part = AddDatum.apply(part.clone(), "d", &args).unwrap();
        part.datum(part.datum_id("d").unwrap()).unwrap().clone()
    }

    /// Sketch data is solved, not exact: agreeing to 1e-9 is agreeing.
    fn assert_at(frame: &CoordinateSystem<S>, origin: [f64; 3], w: [f64; 3]) {
        let [x, y, z] = origin;
        let off = frame.origin().sub(&v(x, y, z)).norm().to_f64();
        assert!(off < 1e-9, "origin of {frame} is {off} off {origin:?}");
        let [x, y, z] = w;
        let w = v(x, y, z).normalize().unwrap();
        let off = frame.w().sub(&w).norm().to_f64();
        assert!(off < 1e-9, "w of {frame} is {off} off {w:?}");
    }

    /// Whether `p` lies on the plane (a frame whose `w` is its normal) —
    /// to within what solved sketch data is.
    fn on_plane(frame: &CoordinateSystem<S>, p: [f64; 3]) -> bool {
        let [x, y, z] = p;
        plane_of(frame).signed_distance(&v(x, y, z)).to_f64().abs() < 1e-9
    }

    /// Every construction serializes under its own method, with exactly the
    /// values its schema lists, and back.
    #[test]
    fn constructions_serialize_as_described() {
        for schema in CONSTRUCTIONS {
            let mut json = serde_json::json!({ "method": schema.method });
            for param in schema.params {
                json[param.name] = match param.kind {
                    ArgKind::Number { default, .. } => default.into(),
                    ArgKind::Bool { default } => default.into(),
                    ref other => panic!("{}: unexpected parameter kind {other:?}", schema.method),
                };
            }
            let construction: Construction = serde_json::from_value(json.clone())
                .unwrap_or_else(|e| panic!("{}: {e}", schema.method));
            assert_eq!(construction.schema(), schema);
            assert_eq!(serde_json::to_value(&construction).unwrap(), json);
        }
    }

    /// What a selection fits follows from the shape of what is selected,
    /// not only its kind.
    #[test]
    fn selections_fit_by_shape() {
        let part = drilled_box();
        let fits = |selection: Vec<EntityRef>| inspect_selection(&part, &selection).fits;

        let top = fits(vec![face("extrude(box,end)")]);
        assert!(top.contains(&"offset"), "{top:?}");
        assert!(!top.contains(&"axis_of"), "{top:?}");
        // The hole's wall is round, not flat.
        let wall = fits(vec![face("extrude(hole,hole_sketch,c1)")]);
        assert_eq!(wall, ["axis_of"]);
        // A straight edge is a line and an edge; a circular one has a
        // center and an axis.
        let straight = fits(vec![edge("extrude(box,outline,c4,end)")]);
        for method in ["along_line", "edge_point", "tangent", "normal_to_edge"] {
            assert!(straight.contains(&method), "{method}: {straight:?}");
        }
        assert!(!straight.contains(&"center"), "{straight:?}");
        let rim = fits(vec![edge("extrude(hole,hole_sketch,c1,start)")]);
        for method in ["center", "axis_of", "edge_point"] {
            assert!(rim.contains(&method), "{method}: {rim:?}");
        }
        assert!(!rim.contains(&"along_line"), "{rim:?}");
        // In any order.
        let point_plane = fits(vec![
            base(WorldAxis::Z),
            vertex("extrude(box,outline,p2,end)"),
        ]);
        for method in ["project_on_plane", "perpendicular", "parallel_plane"] {
            assert!(point_plane.contains(&method), "{method}: {point_plane:?}");
        }
        assert!(
            fits(vec![
                EntityRef::Origin,
                EntityRef::Origin,
                EntityRef::Origin
            ])
            .contains(&"three_points")
        );
        assert!(fits(Vec::new()).is_empty());
        // Nothing fits what the part does not have.
        let missing = inspect_selection(&part, &[face("nowhere")]);
        assert!(missing.fits.is_empty());
        assert_eq!(missing.roles, [Vec::<Role>::new()]);
    }

    #[test]
    fn a_selection_that_does_not_fit_says_what_it_needs() {
        let args = AddDatumArgs {
            selection: vec![EntityRef::Origin],
            construction: Construction::Offset { distance: 1.0 },
        };
        let Err(e) = AddDatum.apply(Part::<S>::new(), "d", &args) else {
            panic!("an offset plane from a point");
        };
        assert!(e.to_string().contains("needs a plane selected"), "{e}");
    }

    #[test]
    fn points() {
        let part = drilled_box();
        let corner = vertex("extrude(box,outline,p2,end)");
        let d = datum(
            &part,
            vec![corner.clone()],
            Construction::Point {
                x: 0.5,
                y: 0.0,
                z: 1.0,
            },
        );
        assert_eq!(d.kind, DatumKind::Point);
        assert_at(&d.frame, [2.5, 2.0, 2.0], [0., 0., 1.]);
        // The top's front edge runs from (2, 0, 1) back to (0, 0, 1); a
        // point on it has its z axis along it.
        let on_edge = AddDatumArgs {
            selection: vec![edge("extrude(box,outline,c4,end)")],
            construction: Construction::EdgePoint { position: 0.25 },
        };
        let with_point = AddDatum.apply(part.clone(), "on_edge", &on_edge).unwrap();
        let d = with_point
            .datum(with_point.datum_id("on_edge").unwrap())
            .unwrap();
        assert_at(&d.frame, [1.5, 0.0, 1.0], [-1., 0., 0.]);
        // Offset from a datum point: along its own axes.
        let d = datum(
            &with_point,
            vec![EntityRef::Datum {
                name: "on_edge".into(),
            }],
            Construction::Point {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        );
        assert_at(&d.frame, [1.0, 0.0, 1.0], [-1., 0., 0.]);

        let d = datum(
            &part,
            vec![EntityRef::Origin, corner.clone()],
            Construction::Midpoint {},
        );
        assert_at(&d.frame, [1.0, 1.0, 0.5], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![edge("extrude(hole,hole_sketch,c1,start)")],
            Construction::Center {},
        );
        assert_at(&d.frame, [1.0, 1.0, 1.0], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![corner.clone(), base(WorldAxis::Z)],
            Construction::ProjectOnPlane {},
        );
        assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![axis(WorldAxis::X), corner.clone()],
            Construction::ProjectOnLine {},
        );
        assert_at(&d.frame, [2.0, 0.0, 0.0], [1., 0., 0.]);
        let d = datum(
            &part,
            vec![edge("extrude(box,outline,p2)"), base(WorldAxis::Z)],
            Construction::LinePlane {},
        );
        assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![axis(WorldAxis::Z), edge("extrude(box,outline,c4,start)")],
            Construction::LineLine {},
        );
        assert_at(&d.frame, [0.0, 0.0, 0.0], [0., -1., 0.]);
        let d = datum(
            &part,
            vec![
                face("extrude(box,end)"),
                face("extrude(box,outline,c5)"),
                face("extrude(box,outline,c6)"),
            ],
            Construction::ThreePlanes {},
        );
        assert_at(&d.frame, [2.0, 2.0, 1.0], [0., 0., 1.]);
    }

    #[test]
    fn axes() {
        let part = drilled_box();
        let corner = vertex("extrude(box,outline,p2,end)");
        let d = datum(
            &part,
            vec![EntityRef::Origin, corner.clone()],
            Construction::TwoPoints {},
        );
        assert_eq!(d.kind, DatumKind::Axis);
        assert_at(&d.frame, [0., 0., 0.], [2., 2., 1.]);
        let d = datum(
            &part,
            vec![edge("extrude(box,outline,p1)")],
            Construction::AlongLine {},
        );
        assert_at(&d.frame, [2., 0., 0.], [0., 0., 1.]);
        // The hole's wall turns around the vertical through its center.
        let d = datum(
            &part,
            vec![face("extrude(hole,hole_sketch,c1#2)")],
            Construction::AxisOf {},
        );
        assert!(datum_line(&d).could_contain(&v(1., 1., 7.)));
        let d = datum(
            &part,
            vec![base(WorldAxis::X), base(WorldAxis::Y)],
            Construction::PlanePlane {},
        );
        assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
        // The perpendicular dropped from the corner onto the base plane.
        let d = datum(
            &part,
            vec![corner.clone(), base(WorldAxis::Z)],
            Construction::Perpendicular {},
        );
        assert_at(&d.frame, [2., 2., 1.], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![corner.clone(), axis(WorldAxis::X)],
            Construction::Parallel {},
        );
        assert_at(&d.frame, [2., 2., 1.], [1., 0., 0.]);
        let d = datum(
            &part,
            vec![corner.clone(), axis(WorldAxis::X)],
            Construction::PerpendicularToLine {},
        );
        assert_at(&d.frame, [2., 2., 1.], [0., -2., -1.]);
        let d = datum(
            &part,
            vec![axis(WorldAxis::X), axis(WorldAxis::Y)],
            Construction::Bisector { other: false },
        );
        assert_at(&d.frame, [0., 0., 0.], [1., 1., 0.]);
        let d = datum(
            &part,
            vec![axis(WorldAxis::X), axis(WorldAxis::Y)],
            Construction::Bisector { other: true },
        );
        assert_at(&d.frame, [0., 0., 0.], [1., -1., 0.]);
        // Between two parallel edges: halfway.
        let d = datum(
            &part,
            vec![
                edge("extrude(box,outline,p0)"),
                edge("extrude(box,outline,p2)"),
            ],
            Construction::Bisector { other: false },
        );
        let off = datum_line(&d).project(&v(1., 1., 0.)).sub(&v(1., 1., 0.));
        assert!(off.norm().to_f64() < 1e-9, "{off:?}");
        let d = datum(
            &part,
            vec![edge("extrude(hole,hole_sketch,c1,start)")],
            Construction::Tangent { position: 0.0 },
        );
        assert_at(&d.frame, [1.4, 1.0, 1.0], [0., 1., 0.]);
    }

    fn datum_line(d: &Datum<S>) -> geop_core_geometry::shape::Axis<S> {
        geop_core_geometry::shape::Axis::try_new(*d.frame.origin(), *d.frame.w()).unwrap()
    }

    #[test]
    fn planes() {
        let part = drilled_box();
        let top = face("extrude(box,end)");
        let d = datum(
            &part,
            vec![top.clone()],
            Construction::Offset { distance: 0.5 },
        );
        assert_eq!(d.kind, DatumKind::Plane);
        assert_at(&d.frame, [0., 0., 1.5], [0., 0., 1.]);
        // Between the box's top and bottom, which face apart.
        let d = datum(
            &part,
            vec![top.clone(), face("extrude(box,start)")],
            Construction::Midplane { other: false },
        );
        assert!(on_plane(&d.frame, [1., 1., 0.5]));
        assert_at(&d.frame, [0., 0., 0.5], [0., 0., 1.]);
        // Between two planes facing the same way.
        let lifted = AddDatum
            .apply(
                part.clone(),
                "lifted",
                &AddDatumArgs {
                    selection: vec![top.clone()],
                    construction: Construction::Offset { distance: 1.0 },
                },
            )
            .unwrap();
        let d = datum(
            &lifted,
            vec![
                top.clone(),
                EntityRef::Datum {
                    name: "lifted".into(),
                },
            ],
            Construction::Midplane { other: false },
        );
        assert!(on_plane(&d.frame, [5., 5., 1.5]));
        // Halving the angle between two sides that meet at an edge.
        let d = datum(
            &part,
            vec![
                face("extrude(box,outline,c4)"),
                face("extrude(box,outline,c5)"),
            ],
            Construction::Midplane { other: false },
        );
        assert!(on_plane(&d.frame, [2., 0., 0.]) && on_plane(&d.frame, [1., 1., 0.]));
        let d = datum(
            &part,
            vec![
                EntityRef::Origin,
                vertex("extrude(box,outline,p1,start)"),
                vertex("extrude(box,outline,p2,end)"),
            ],
            Construction::ThreePoints {},
        );
        assert!(
            on_plane(&d.frame, [0., 0., 0.])
                && on_plane(&d.frame, [2., 0., 0.])
                && on_plane(&d.frame, [2., 2., 1.])
        );
        // Hinged on the top's front edge, turned up by 90 degrees: the
        // front face's plane.
        let d = datum(
            &part,
            vec![top.clone(), edge("extrude(box,outline,c4,end)")],
            Construction::Angle { angle: 90.0 },
        );
        assert!(on_plane(&d.frame, [0.5, 0., 0.]) && on_plane(&d.frame, [0.5, 0., 7.]));
        let d = datum(
            &part,
            vec![axis(WorldAxis::Z), vertex("extrude(box,outline,p1,start)")],
            Construction::LinePoint {},
        );
        assert!(on_plane(&d.frame, [5., 0., 3.]));
        let d = datum(
            &part,
            vec![
                edge("extrude(box,outline,p0)"),
                edge("extrude(box,outline,p2)"),
            ],
            Construction::TwoLines {},
        );
        assert!(on_plane(&d.frame, [1., 1., 3.]));
        let d = datum(
            &part,
            vec![top.clone(), EntityRef::Origin],
            Construction::ParallelPlane {},
        );
        assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
        let d = datum(
            &part,
            vec![axis(WorldAxis::Y), vertex("extrude(box,outline,p2,end)")],
            Construction::NormalToLine {},
        );
        assert_at(&d.frame, [2., 2., 1.], [0., 1., 0.]);
        let d = datum(
            &part,
            vec![edge("extrude(hole,hole_sketch,c1,start)")],
            Construction::NormalToEdge { position: 0.0 },
        );
        assert!(on_plane(&d.frame, [1.4, 1., 1.]) && on_plane(&d.frame, [1., 1., 1.]));
    }

    /// Degenerate selections are refused, saying why.
    #[test]
    fn degenerate_selections_fail() {
        let part = Part::<S>::new();
        let origin = EntityRef::Origin;
        let err = |selection: Vec<EntityRef>, construction: Construction| {
            let args = AddDatumArgs {
                selection,
                construction,
            };
            match AddDatum.apply(part.clone(), "d", &args) {
                Ok(_) => panic!("{args:?} built a datum"),
                Err(e) => format!("{e:?}"),
            }
        };
        assert!(
            err(
                vec![origin.clone(), origin.clone()],
                Construction::TwoPoints {}
            )
            .contains("coincide")
        );
        assert!(
            err(
                vec![base(WorldAxis::Z), base(WorldAxis::Z)],
                Construction::PlanePlane {}
            )
            .contains("parallel")
        );
        assert!(
            err(
                vec![origin.clone(), axis(WorldAxis::X)],
                Construction::PerpendicularToLine {}
            )
            .contains("lies on the line")
        );
        assert!(
            err(
                vec![base(WorldAxis::Z), axis(WorldAxis::Z)],
                Construction::Angle { angle: 10.0 }
            )
            .contains("perpendicular")
        );
        assert!(err(vec![edge("nowhere")], Construction::AlongLine {}).contains("nowhere"));
    }

    /// A datum is a named entity, and a sketch can be placed on a datum
    /// plane: its frame, exactly.
    #[test]
    fn sketches_go_on_datum_planes() {
        let program = examples::boss_on_reference_plane();
        let part = program.apply(Part::<S>::new()).unwrap();
        part.check_names().unwrap();
        let placed = part.sketch(part.sketch_id("boss_sketch").unwrap()).unwrap();
        let lifted = part.datum(part.datum_id("lifted").unwrap()).unwrap();
        assert!(placed.plane.origin().could_be_equal(lifted.frame.origin()));
        assert!(placed.plane.u().could_be_equal(lifted.frame.u()));
        assert_at(&placed.plane, [0., 0., 1.5], [0., 0., 1.]);
    }

    /// Offsets are handles: dragging one edits the construction's value.
    #[test]
    fn offsets_have_handles() {
        let mut program = Program::new();
        program.push(
            "p",
            AddDatumArgs {
                selection: vec![EntityRef::Origin],
                construction: Construction::Point {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
            },
        );
        let PartOperation::AddDatum(args) = &program.steps[0].operation else {
            unreachable!()
        };
        let handles = AddDatum.handles(&Part::<S>::new(), args).unwrap();
        let labels: Vec<&str> = handles.iter().map(|h| h.label.as_str()).collect();
        assert_eq!(labels, ["x", "y", "z"]);
        let HandleMotion::Linear { arg, value, .. } = &handles[2].motion else {
            panic!("a linear handle")
        };
        assert_eq!(arg, &arg_path(&["construction", "z"]));
        assert_eq!(*value, 3.0);
    }
}
