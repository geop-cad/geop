//! [`BoundarySurface`]: a face standing on its own, spanned by edges — the
//! ruled face between two of them, or a closed loop of them filled.
//!
//! **Two edges** that do not close a loop are joined by the ruled surface
//! between them (see [`NurbSurface3D::ruled`]), the second turned, if need
//! be, so that their ends line up, and a straight edge at either end.
//!
//! **A closed loop** — the edges picked in any order, each running either
//! way, end to end — is filled:
//!
//! - **flat**, where every edge lies in one plane: one face on that plane,
//!   trimmed by the loop, exactly as an extrude's cap is. Any number of
//!   edges, a single circle included.
//! - **four edges**: their Coons patch (see [`NurbSurface3D::coons`]), which
//!   runs exactly along every one of them. Along edges picked as `tangent`
//!   it is made tangent to the flat face each of those edges bounds,
//!   continuing it smoothly (see [`NurbSurface3D::tangent_to_planes`]).
//! - **three, or five and more**: a flat inner polygon — each corner of the
//!   loop's halfway to their average, on the plane through it square to
//!   the loop — and a quadrilateral along every edge, between the edge, the
//!   straight spokes from its ends to the inner polygon and the polygon's
//!   side. Each is a Coons patch, so the fill runs exactly along the loop,
//!   every edge whole — the fill knits to the faces around the hole; across
//!   the spokes the patches meet at an angle, not smoothly: an honest
//!   patchwork, not a fair surface.
//!
//! A Coons patch needs a corner wherever two of its sides meet: a loop of
//! four that runs on smoothly through a vertex leaves a patch with no
//! normal there, and is refused, naming the vertex. So are two edges sharing an end — the
//! ruled face would come to a point — and tangency anywhere but along a
//! loop of four, or to a face that is not flat.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D, flatten_point, loop_signed_area},
    nurb_surface::{FlatPatch, NurbSurface3D},
    shape::Plane,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_core_topology::{
    Curve3, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops_model::sketch3d::CurveSource;
use geop_ops_model::{
    BodyNames, Chain, Edge, ModelOperation, NamedModel, Namer, build_model, read_model,
};
use serde::{Deserialize, Serialize};

use crate::{NamedCurve, SheetSpec};

/// Spans a face standing on its own between the curves `edges` — two of
/// them ruled, a closed loop of them filled, see the module docs — for the
/// operation `B`. A curve is an edge, of a face or of a wire — a 3-D
/// sketch's line, arc or spline — or a planar sketch's curve. The curves
/// are copied, the originals left as they are; with `tangent`, the fill
/// continues the flat faces those of its edges bound smoothly.
///
/// The face is named `boundary(B)`. The copy of each curve or end `X`
/// picked is `boundary(B,X)` — for a sketch's curve `X` is `K,c3`, and
/// its ends `K,p1` (see [`NamedCurve`]); a straight edge a ruled face adds between
/// vertices `V` and `W` is `boundary(B,V,W)`. A patch of quadrilaterals
/// names the one along edge `E` `boundary(B,E,patch)`, the inner polygon
/// `boundary(B,inner)`, its side alongside `E` `boundary(B,E,inner)`, its
/// corner off vertex `V` `boundary(B,V,inner)` and the spoke between them
/// `boundary(B,V,spoke)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundarySurface;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundarySurfaceArgs {
    /// The curves to span: two, or a closed loop.
    pub edges: Vec<CurveSource>,
    /// Those of them the fill is to be tangent along, to the flat face each
    /// bounds.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tangent: Vec<ElementReference<NamedModel, Edge>>,
}

part_operation!(BoundarySurface(BoundarySurfaceArgs));

impl PartOperation for BoundarySurface {
    const KIND: &'static str = "boundary_surface";
    const INFO: OperationInfo = OperationInfo::new(
        "Boundary surface",
        "Span a face standing on its own between two edges, or fill a closed loop of edges — optionally tangent to the flat faces along them.",
        OperationGroup::Surface,
        OperationTier::Small,
    )
    .experimental()
    .icon(crate::icons::BOUNDARY_SURFACE);
    type Args = BoundarySurfaceArgs;
    type Session = ();

    /// Nothing picked yet.
    fn new_args(_: &Context) -> BoundarySurfaceArgs {
        BoundarySurfaceArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &BoundarySurfaceArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The edges, picked, and those to be tangent along.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &BoundarySurfaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, BoundarySurfaceArgs> {
        let before = &context.before;
        let mut f = EditForm::<BoundarySurfaceArgs>::default();
        f.pick(
            "edges",
            "edges",
            before,
            &args.edges,
            true,
            |edit, picked| edit.args.edges = picked,
        );
        f.pick(
            "tangent",
            "tangent along",
            before,
            &args.tangent,
            true,
            |edit, picked| edit.args.tangent = picked,
        );
        f.form.dialog.optional("tangent");
        f
    }
}

impl ModelOperation for BoundarySurface {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &BoundarySurfaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("boundary", step)?;
        let edges = (args.edges.iter())
            .map(|e| e.resolve(reader))
            .collect::<GeopResult<Vec<_>>>()?;
        let tangent: Vec<&str> = args.tangent.iter().map(|t| t.name()).collect();
        let (spec, names) = boundary_surface(&*read_model(reader)?, &namer, &edges, &tangent)?;
        model.build_body(spec, names)?;
        Ok(())
    }
}

/// A curve picked, as it runs along the boundary: its curve and its ends'
/// points and names in that direction — `forward` if that is the curve's
/// own.
#[derive(Clone, Debug)]
struct Side {
    /// The name of the curve picked, as `tangent` names it.
    entity: String,
    name: String,
    /// The edge's curve, as the edge runs.
    curve: Curve3,
    forward: bool,
    start: Vector3,
    end: Vector3,
    start_name: String,
    end_name: String,
}

impl Side {
    fn of(chain: &Chain<NurbCurve3D>) -> GeopResult<Self> {
        let curve = NamedCurve::of(chain)?;
        Ok(Self {
            entity: curve.name.clone(),
            name: curve.name,
            curve: curve.curve,
            forward: true,
            start: curve.start.1,
            end: curve.end.1,
            start_name: curve.start.0,
            end_name: curve.end.0,
        })
    }

    fn turned(&self) -> Self {
        Self {
            forward: !self.forward,
            start: self.end,
            end: self.start,
            start_name: self.end_name.clone(),
            end_name: self.start_name.clone(),
            ..self.clone()
        }
    }

    /// The curve, run the way the side runs.
    fn oriented(&self) -> Curve3 {
        self.sense().orient(&self.curve)
    }

    /// How a coedge running the way the side runs uses its edge's copy.
    fn sense(&self) -> Sense {
        Sense::from_forward(self.forward)
    }
}

/// A plain number to compare by: a free choice between candidates.
fn distance(a: &Vector3, b: &Vector3) -> f64 {
    a.sub(b).norm().to_f64()
}

/// The sheet spanned between `edges`, tangent along `tangent` (see the
/// module docs), each a curve picked, the edges of `tangent` by name, in
/// `part`: what to build, named after `namer`, as [`BoundarySurface`]
/// says.
pub fn boundary_surface(
    part: &NamedModel,
    namer: &Namer,
    edges: &[Chain<NurbCurve3D>],
    tangent: &[&str],
) -> GeopResult<SheetSpec> {
    let sides = edges.iter().map(Side::of).collect::<GeopResult<Vec<_>>>()?;
    if let Some(t) = (tangent.iter()).find(|t| !sides.iter().any(|s| s.entity == **t)) {
        return Err(GeopError::new(format!(
            "edge {t} is to be tangent along, but is not one of the curves spanned"
        )));
    }
    let (spec, names) = match sides.as_slice() {
        [] => return Err(GeopError::new("no edges to span a surface between")),
        [a, b] if !closes(a, b) => {
            if !tangent.is_empty() {
                return Err(GeopError::new(
                    "a ruled face between two edges cannot be made tangent: tangency is supported along a loop of four edges",
                ));
            }
            ruled(namer, a, b)?
        }
        _ => {
            let sides = chain(&sides)?;
            let planes = tangent
                .iter()
                .map(|&t| Ok((t.to_string(), tangent_plane(part, t)?)))
                .collect::<GeopResult<Vec<_>>>()?;
            fill(namer, &sides, &planes)?
        }
    };
    Ok((spec, names))
}

/// Whether the two edges close a loop between them: each end of one at an
/// end of the other.
fn closes(a: &Side, b: &Side) -> bool {
    let meet = |p: &Vector3, q: &Vector3| p.could_be_equal(q);
    (meet(&a.end, &b.start) && meet(&b.end, &a.start))
        || (meet(&a.end, &b.end) && meet(&b.start, &a.start))
}

/// The ruled face between `a` and `b`, `b` turned if its ends line up
/// better with `a`'s that way round.
fn ruled(namer: &Namer, a: &Side, b: &Side) -> GeopResult<(BodySpec, BodyNames)> {
    let along = distance(&a.start, &b.start) + distance(&a.end, &b.end);
    let across = distance(&a.start, &b.end) + distance(&a.end, &b.start);
    let b = if across < along {
        b.turned()
    } else {
        b.clone()
    };
    if a.start.could_be_equal(&b.start) || a.end.could_be_equal(&b.end) {
        return Err(GeopError::new(format!(
            "edges {} and {} share an end: the ruled face between them would come to a point there, which is not supported",
            a.name, b.name
        )));
    }
    // Vertices 0 -> 1 along `a`, 2 -> 3 along `b`.
    let vertices = vec![a.start, a.end, b.start, b.end];
    let vertex_names = [&a.start_name, &a.end_name, &b.start_name, &b.end_name]
        .map(|n| namer.name(&[n]))
        .to_vec();
    let copy = |side: &Side, start: usize, end: usize| EdgeSpec {
        curve: side.curve.clone(),
        start: if side.forward { start } else { end },
        end: if side.forward { end } else { start },
    };
    let edges = vec![
        copy(a, 0, 1),
        copy(&b, 2, 3),
        EdgeSpec {
            curve: NurbCurve3D::line(a.end, b.end)?,
            start: 1,
            end: 3,
        },
        EdgeSpec {
            curve: NurbCurve3D::line(b.start, a.start)?,
            start: 2,
            end: 0,
        },
    ];
    let edge_names = vec![
        namer.name(&[&a.name]),
        namer.name(&[&b.name]),
        namer.name(&[&a.end_name, &b.end_name]),
        namer.name(&[&b.start_name, &a.start_name]),
    ];
    let surface = NurbSurface3D::ruled(&a.oriented(), &b.oriented())?;
    let face = FaceSpec::on_unit_square(
        surface,
        [
            CoedgeOn::Edge(0, a.sense()),
            CoedgeOn::Edge(2, Sense::Forward),
            CoedgeOn::Edge(1, b.sense().opposite()),
            CoedgeOn::Edge(3, Sense::Forward),
        ],
    )?;
    Ok((
        BodySpec {
            vertices,
            edges,
            faces: vec![face],
            shells: vec![vec![0]],
            solid: false,
        },
        BodyNames {
            vertices: vertex_names,
            edges: edge_names,
            faces: vec![namer.root()],
            solid: None,
        },
    ))
}

/// `sides` put end to end into a closed loop, starting with the first as
/// it runs: each turned to run on from where the one before ends. An error
/// naming the edge where the loop breaks off or forks.
fn chain(sides: &[Side]) -> GeopResult<Vec<Side>> {
    let mut left: Vec<Side> = sides[1..].to_vec();
    let mut out = vec![sides[0].clone()];
    // Each round takes one side out of `left`, or refuses.
    for _ in 1..sides.len() {
        let end = out.last().expect("one side at least").end;
        let mut next: Vec<(usize, bool)> = Vec::new();
        for (i, s) in left.iter().enumerate() {
            if s.start.could_be_equal(&end) {
                next.push((i, false));
            }
            if s.end.could_be_equal(&end) {
                next.push((i, true));
            }
        }
        let last = &out.last().expect("one side at least").name;
        match next.as_slice() {
            [] => {
                return Err(GeopError::new(format!(
                    "the edges are not a closed loop: none of them goes on from where edge {last} ends"
                )));
            }
            [(i, turn)] => {
                let s = left.remove(*i);
                out.push(if *turn { s.turned() } else { s });
            }
            _ => {
                return Err(GeopError::new(format!(
                    "the edges are not a simple loop: several of them go on from where edge {last} ends"
                )));
            }
        }
    }
    let (first, last) = (&out[0], out.last().expect("one side at least"));
    if !last.end.could_be_equal(&first.start) {
        return Err(GeopError::new(format!(
            "the edges are not a closed loop: edge {} ends where edge {} does not start",
            last.name, first.name
        )));
    }
    Ok(out)
}

/// What a fill is tangent to along an edge: a plane, and the direction in
/// it away from the face it continues.
type Tangency = (Plane, Vector3);

/// The plane of the flat face the edge named `name` bounds, and the direction
/// in it away from that face at the middle of the edge — what a fill
/// tangent along the edge continues.
fn tangent_plane(part: &NamedModel, name: &str) -> GeopResult<Tangency> {
    let model = part.topology();
    let edge = part.edge_id(name)?;
    let coedges = model.coedges_of_edge(edge);
    let [coedge] = coedges.as_slice() else {
        return Err(GeopError::new(format!(
            "edge {name} bounds {} faces: tangency is to the one face an edge bounds, standing on its own",
            coedges.len()
        )));
    };
    let coedge = model.get_coedge(*coedge)?;
    let plane = part.plane_of_face(
        coedge.face,
        &format!("tangency along edge {name} is supported to flat faces"),
    )?;
    // The face lies to the left of its boundary, seen from where its normal
    // points: away from it is the edge's direction, as the face runs it,
    // crossed with the normal.
    let curve = &model.get_edge(edge)?.curve;
    let mut along = curve.tangent(curve.mid_parameter()?)?;
    if coedge.sense == Sense::Reversed {
        along = along.neg();
    }
    let away = along.prod_cross(&plane.normal);
    Ok((plane, away))
}

/// Whether the loop of `sides` has a corner where side `k` meets the next:
/// their tangents there are not parallel.
fn has_corner(a: &Curve3, b: &Curve3) -> GeopResult<bool> {
    let end = a.tangent(a.domain().1)?;
    let start = b.tangent(b.domain().0)?;
    Ok(!end
        .prod_cross(&start)
        .norm_sq()
        .could_be_equal(Scalar::ZERO))
}

/// The closed loop `sides` filled (see the module docs), tangent along the
/// edges of `planes` to their planes.
fn fill(
    namer: &Namer,
    sides: &[Side],
    planes: &[(String, Tangency)],
) -> GeopResult<(BodySpec, BodyNames)> {
    let n = sides.len();
    // The loop's corners, side `k` running from corner `k` to `k + 1`, and
    // each side's copy.
    let mut spec = BodySpec {
        vertices: sides.iter().map(|s| s.start).collect(),
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: false,
    };
    let mut names = BodyNames {
        vertices: sides.iter().map(|s| namer.name(&[&s.start_name])).collect(),
        ..Default::default()
    };

    if let Some(plane) = flat(sides)? {
        for (e, (p, _)) in planes {
            let same = p
                .normal
                .prod_cross(&plane.normal)
                .norm_sq()
                .could_be_equal(Scalar::ZERO)
                && p.signed_distance(&plane.point).could_be_equal(Scalar::ZERO);
            if !same {
                let side = sides.iter().find(|s| s.entity == *e).expect("a side");
                return Err(GeopError::new(format!(
                    "the loop is flat, in a plane other than that of the face along edge {}: the fill cannot be tangent to it",
                    side.name
                )));
            }
        }
        for (k, s) in sides.iter().enumerate() {
            let (a, b) = (k, (k + 1) % n);
            spec.edges.push(EdgeSpec {
                curve: s.curve.clone(),
                start: if s.forward { a } else { b },
                end: if s.forward { b } else { a },
            });
            names.edges.push(namer.name(&[&s.name]));
        }
        spec.faces.push(flat_face(sides, &plane)?);
        spec.shells.push(vec![0]);
        names.faces.push(namer.root());
        return Ok((spec, names));
    }

    let curves: Vec<Curve3> = sides.iter().map(Side::oriented).collect();
    if n == 4 {
        for k in 0..n {
            if !has_corner(&curves[(k + n - 1) % n], &curves[k])? {
                return Err(GeopError::new(format!(
                    "the loop runs on smoothly through vertex {}, between edges {} and {}: a Coons patch filling it needs a corner there",
                    sides[k].start_name,
                    sides[(k + n - 1) % n].name,
                    sides[k].name
                )));
            }
        }
        for (k, s) in sides.iter().enumerate() {
            spec.edges.push(EdgeSpec {
                curve: s.curve.clone(),
                start: if s.forward { k } else { (k + 1) % 4 },
                end: if s.forward { (k + 1) % 4 } else { k },
            });
            names.edges.push(namer.name(&[&s.name]));
        }
        let mut surface = NurbSurface3D::coons([&curves[0], &curves[1], &curves[2], &curves[3]])?;
        if !planes.is_empty() {
            let mut at: [Option<Tangency>; 4] = Default::default();
            for (e, plane) in planes {
                let k = sides.iter().position(|s| s.entity == *e).expect("a side");
                at[k] = Some(plane.clone());
            }
            surface = surface.tangent_to_planes(&at)?;
        }
        spec.faces.push(FaceSpec::on_unit_square(
            surface,
            [0, 1, 2, 3].map(|k| CoedgeOn::Edge(k, sides[k].sense())),
        )?);
        spec.shells.push(vec![0]);
        names.faces.push(namer.root());
        return Ok((spec, names));
    }

    if let Some((e, _)) = planes.first() {
        let side = sides.iter().find(|s| s.entity == *e).expect("a side");
        return Err(GeopError::new(format!(
            "a loop of {n} edges is filled with a patch of quadrilaterals, which cannot be made tangent along edge {}: tangency is supported along a loop of four edges",
            side.name
        )));
    }
    if n < 3 {
        return Err(GeopError::new(format!(
            "a loop of {n} edges that is not flat is not supported: it needs three corners at least"
        )));
    }
    quads(namer, sides, &curves, spec, names)
}

/// The plane `sides` all lie in, if they do: through three of their
/// control points far apart, every other control point on it — so every
/// curve is in it. `None` for a loop that is not flat.
fn flat(sides: &[Side]) -> GeopResult<Option<Plane>> {
    let points: Vec<Vector3> = sides
        .iter()
        .flat_map(|s| &s.curve.control_points)
        .map(Vector4::dehomogenized)
        .collect::<GeopResult<_>>()?;
    // Which three is a free choice: far apart, so the plane through them is
    // well determined.
    let p0 = points[0];
    let far = |key: &dyn Fn(&Vector3) -> f64| {
        *points
            .iter()
            .max_by(|a, b| key(a).total_cmp(&key(b)))
            .expect("points")
    };
    let p1 = far(&|p| distance(p, &p0));
    let p2 = far(&|p| p.sub(&p0).prod_cross(&p1.sub(&p0)).norm().to_f64());
    let normal = p1.sub(&p0).prod_cross(&p2.sub(&p0));
    if normal.norm_sq().could_be_equal(Scalar::ZERO) {
        return Err(GeopError::new(
            "the edges all lie along one line: they enclose nothing to fill",
        ));
    }
    let plane = Plane::try_new(p0, normal)?;
    Ok(points
        .iter()
        .all(|p| plane.signed_distance(p).could_be_equal(Scalar::ZERO))
        .then_some(plane))
}

/// The flat face on `plane` within the loop `sides`: the box around the
/// loop in the plane, as a bilinear patch, the loop's pcurves the curves'
/// control points in its coordinates — an affine map, exact for any curve.
/// Parametrized along the plane's `(x, y)` or `(y, x)`, whichever the loop
/// winds counter-clockwise in.
fn flat_face(sides: &[Side], plane: &Plane) -> GeopResult<FaceSpec> {
    let origin = plane.point;
    let first = &sides[0].curve.control_points;
    let towards = Vector3::from_array([0, 1, 2].map(|k| first[first.len() - 1][k]))
        .prod_scalar(Scalar::ONE.div(first[first.len() - 1][3])?);
    // Any direction in the plane would do for `x`: towards the far end of
    // the first curve, unless that is where it starts.
    let mut e1 = towards.sub(&origin);
    if e1.norm_sq().could_be_equal(Scalar::ZERO) {
        e1 = plane
            .normal
            .orthonormal_complement()?
            .into_iter()
            .next()
            .expect("a complement");
    }
    let e1 = e1
        .sub(&plane.normal.prod_scalar(e1.prod_dot(&plane.normal)))
        .normalize()?;
    let e2 = plane.normal.prod_cross(&e1);
    // The loop in the plane's coordinates: each curve's control points
    // flattened, an affine map.
    let local = sides
        .iter()
        .map(|side| {
            let curve = side.oriented();
            let control_points = curve
                .control_points
                .iter()
                .map(|cp| flatten_point(cp, &origin, &e1, &e2))
                .collect();
            NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())
        })
        .collect::<GeopResult<Vec<NurbCurve2D>>>()?;
    let area = loop_signed_area(&local)?;
    let swapped = if area.definitely_greater(Scalar::ZERO) {
        false
    } else if area.definitely_less(Scalar::ZERO) {
        true
    } else {
        return Err(GeopError::new(
            "cannot tell which way the loop runs round: it encloses no area",
        ));
    };
    let patch = FlatPatch::around(origin, e1, e2, &local, swapped)?;
    let surface = patch.surface()?;
    let pcurves = local
        .iter()
        .map(|curve| patch.pcurve(curve))
        .collect::<GeopResult<Vec<_>>>()?;
    let outer = sides
        .iter()
        .zip(pcurves)
        .enumerate()
        .map(|(k, (s, pcurve))| CoedgeSpec {
            on: CoedgeOn::Edge(k, s.sense()),
            pcurve,
        })
        .collect();
    Ok(FaceSpec {
        surface,
        outer,
        holes: Vec::new(),
    })
}

/// The loop `sides` filled with a quadrilateral along each side around a
/// flat inner polygon (see the module docs), added to `spec`, which has
/// the loop's corners.
fn quads(
    namer: &Namer,
    sides: &[Side],
    curves: &[Curve3],
    mut spec: BodySpec,
    mut names: BodyNames,
) -> GeopResult<(BodySpec, BodyNames)> {
    let n = sides.len();
    // The inner polygon: each corner halfway to the corners' average,
    // moved onto the plane through that average square to the loop's
    // (Newell) normal — all free choices, sharpened, so long as the
    // polygon is flat and inside the loop.
    let center = sides
        .iter()
        .fold(Vector3::zero(), |sum, s| sum.add(&s.start))
        .prod_scalar(Scalar::ONE.div(Scalar::from_i64(n as i64))?)
        .sharpen();
    let mut normal = [0.0f64; 3];
    for k in 0..n {
        let (a, b) = (&sides[k].start, &sides[(k + 1) % n].start);
        let (a, b) = (
            a.to_array().map(|x| x.to_f64()),
            b.to_array().map(|x| x.to_f64()),
        );
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let plane = Plane::try_new(center, Vector3::from_array(normal.map(Scalar::from_f64)))?;
    let inner: Vec<Vector3> = sides
        .iter()
        .map(|s| {
            let halfway = center.add(
                &s.start
                    .sub(&center)
                    .prod_scalar(Scalar::ONE.div(Scalar::TWO)?),
            );
            Ok(plane.project(&halfway).sharpen())
        })
        .collect::<GeopResult<_>>()?;
    // Vertices: the corners `0..n`, the inner corners `n..2n`.
    for (s, p) in sides.iter().zip(&inner) {
        spec.vertices.push(*p);
        names.vertices.push(namer.name(&[&s.start_name, "inner"]));
    }
    let inner_at = |k: usize| n + k % n;
    // Edges: per side its copy, `3k`, the spoke from its start to the
    // inner polygon, `3k + 1`, and the inner polygon's side alongside it,
    // `3k + 2`.
    let mut inner_sides = Vec::with_capacity(n);
    for (k, s) in sides.iter().enumerate() {
        let (a, b) = (k, (k + 1) % n);
        spec.edges.push(EdgeSpec {
            curve: s.curve.clone(),
            start: if s.forward { a } else { b },
            end: if s.forward { b } else { a },
        });
        names.edges.push(namer.name(&[&s.name]));
        spec.edges.push(EdgeSpec {
            curve: NurbCurve3D::line(s.start, inner[k])?,
            start: k,
            end: inner_at(k),
        });
        names.edges.push(namer.name(&[&s.start_name, "spoke"]));
        let curve = NurbCurve3D::line(inner[k], inner[(k + 1) % n])?;
        spec.edges.push(EdgeSpec {
            curve: curve.clone(),
            start: inner_at(k),
            end: inner_at(k + 1),
        });
        names.edges.push(namer.name(&[&s.name, "inner"]));
        inner_sides.push(Side {
            entity: s.entity.clone(),
            name: namer.name(&[&s.name, "inner"]),
            curve,
            forward: true,
            start: inner[k],
            end: inner[(k + 1) % n],
            start_name: String::new(),
            end_name: String::new(),
        });
    }
    let mut faces = Vec::with_capacity(n + 1);
    for k in 0..n {
        let next = (k + 1) % n;
        let spoke_out = spec.edges[3 * next + 1].curve.clone();
        let back = spec.edges[3 * k + 2].curve.reverse();
        let spoke_in = spec.edges[3 * k + 1].curve.reverse();
        for (a, b, what) in [
            (&curves[k], &spoke_out, &sides[next].start_name),
            (&back, &spoke_in, &sides[k].start_name),
        ] {
            if !has_corner(a, b)? {
                return Err(GeopError::new(format!(
                    "the spoke at vertex {what} runs along edge {}: the patch along it would have no corner there",
                    sides[k].name
                )));
            }
        }
        let surface = NurbSurface3D::coons([&curves[k], &spoke_out, &back, &spoke_in])?;
        faces.push(spec.faces.len());
        spec.faces.push(FaceSpec::on_unit_square(
            surface,
            [
                CoedgeOn::Edge(3 * k, sides[k].sense()),
                CoedgeOn::Edge(3 * next + 1, Sense::Forward),
                CoedgeOn::Edge(3 * k + 2, Sense::Reversed),
                CoedgeOn::Edge(3 * k + 1, Sense::Reversed),
            ],
        )?);
        names.faces.push(namer.name(&[&sides[k].name, "patch"]));
    }
    // The inner polygon, flat, its sides run as the loop runs.
    let mut middle = flat_face(&inner_sides, &plane)?;
    for (k, c) in middle.outer.iter_mut().enumerate() {
        c.on = CoedgeOn::Edge(3 * k + 2, Sense::Forward);
    }
    faces.push(spec.faces.len());
    spec.faces.push(middle);
    names.faces.push(namer.name(&["inner"]));
    spec.shells.push(faces);
    Ok((spec, names))
}

#[cfg(test)]
pub(crate) mod tests;
