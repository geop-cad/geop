//! [`AddDatum`]: reference geometry — a point, an axis, a plane or a
//! coordinate system — built from entities picked in the part, in one of the
//! ways CAD systems commonly offer (see [`Construction`]).
//!
//! Which constructions can be chosen depends on what is selected, and on its
//! shape, not only its kind: a straight edge is a line, a circular one has a
//! center and an axis, a flat face is a plane (see [`Geometry`]).
//! [`inspect_selection`] tells an editor which constructions fit a
//! selection, by exactly the matching a step applies — and makes a step's
//! [`Dialog`].

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use serde::{Deserialize, Serialize};

use geop_ops::{
    Part,
    operation::{
        ArgDialog, ArgKind, ArgSchema, ConstructionSchema, Dialog, EntityRef, Geometry, Handle,
        HandleGroup, HandleMotion, Operation, OperationArgs, Role, arg_path, frame_along, to_f64,
        world_frame,
    },
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
                CoordinateSystem::try_new(a, u, w.prod_cross(&u), w)
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

    /// What the selected entities can be used as, and which constructions
    /// fit them (see [`inspect_selection`]).
    fn dialog(&self, before: &Part<S>, args: &AddDatumArgs) -> Dialog {
        let SelectionFit { roles, fits } = inspect_selection(before, &args.selection);
        Dialog {
            args: [
                ("selection", ArgDialog::Selection { roles }),
                ("construction", ArgDialog::Options { fit: fits }),
            ]
            .into(),
        }
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

    use geop_ops::WorldAxis;

    use super::*;

    fn edge(name: &str) -> EntityRef {
        EntityRef::Edge { name: name.into() }
    }
    fn axis(axis: WorldAxis) -> EntityRef {
        EntityRef::Axis { axis }
    }
    fn base(normal: WorldAxis) -> EntityRef {
        EntityRef::Plane { normal }
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

    /// Offsets are handles: dragging one edits the construction's value.
    #[test]
    fn offsets_have_handles() {
        let args = AddDatumArgs {
            selection: vec![EntityRef::Origin],
            construction: Construction::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        };
        let handles = AddDatum.handles(&Part::<S>::new(), &args).unwrap();
        let labels: Vec<&str> = handles.iter().map(|h| h.label.as_str()).collect();
        assert_eq!(labels, ["x", "y", "z"]);
        let HandleMotion::Linear { arg, value, .. } = &handles[2].motion else {
            panic!("a linear handle")
        };
        assert_eq!(arg, &arg_path(&["construction", "z"]));
        assert_eq!(*value, 3.0);
    }

    /// The dialog says what each selected entity can be used as, and which
    /// constructions fit the selection — for a step that does not build too.
    #[test]
    fn dialogs_say_what_a_selection_fits() {
        let args = AddDatumArgs {
            selection: vec![EntityRef::Origin, edge("nowhere")],
            construction: Construction::Offset { distance: 1.0 },
        };
        let part = Part::<S>::new();
        assert!(AddDatum.apply(part.clone(), "d", &args).is_err());
        let dialog = AddDatum.dialog(&part, &args);
        assert_eq!(
            dialog.args["selection"],
            ArgDialog::Selection {
                roles: vec![vec![Role::Point], vec![]]
            }
        );
        assert_eq!(
            dialog.args["construction"],
            ArgDialog::Options { fit: vec![] }
        );

        let args = AddDatumArgs {
            selection: vec![base(WorldAxis::Z)],
            ..args
        };
        let ArgDialog::Options { fit } = &AddDatum.dialog(&part, &args).args["construction"] else {
            panic!("the constructions are options");
        };
        assert!(fit.contains(&"offset"));
        assert!(!fit.contains(&"midpoint"));
    }
}
