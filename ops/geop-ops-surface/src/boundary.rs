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

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::NurbSurface3D,
    shape::Plane,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    polygon::polygon_signed_area,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Curve3, Sense,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{
    BodyNames, Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::Form,
};
use geop_ops_extrude_revolve::common::{bilinear, line2, line3};
use serde::{Deserialize, Serialize};

use crate::name_of;

/// Spans a face standing on its own between the curves `edges` — two of
/// them ruled, a closed loop of them filled, see the module docs — for the
/// operation `B`. A curve is an edge, of a face or of a wire — a 3-D
/// sketch's line, arc or spline — or a planar sketch's curve. The curves
/// are copied, the originals left as they are; with `tangent`, the fill
/// continues the flat faces those of its edges bound smoothly.
///
/// The face is named `boundary(B)`. The copy of each curve or end `X`
/// picked is `boundary(B,X)` — for a sketch's curve `X` is `K,c3`, and
/// its ends `K,p1` (see [`EntityRef::resolve_curve`]); a straight edge a ruled face adds between
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
    pub edges: Vec<EntityRef>,
    /// Those of them the fill is to be tangent along, to the flat face each
    /// bounds.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tangent: Vec<EntityRef>,
}

impl Operation for BoundarySurface {
    type Args = BoundarySurfaceArgs;
    type Session = ();

    /// Nothing picked yet.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> BoundarySurfaceArgs {
        BoundarySurfaceArgs::default()
    }

    /// The edges, picked, and those to be tangent along.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &BoundarySurfaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, BoundarySurfaceArgs> {
        let mut f = Form::<S, BoundarySurfaceArgs>::new();
        f.reference(
            "edges",
            "edges",
            args.edges.clone(),
            &[Role::Curve],
            None,
            true,
            |e, picked| e.args.edges = picked,
        );
        f.reference(
            "tangent",
            "tangent along",
            args.tangent.clone(),
            &[Role::Edge],
            None,
            true,
            |e, picked| e.args.tangent = picked,
        );
        f.optional("tangent");
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &BoundarySurfaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("boundary_surface({operation_id}, {args:?})");
        let namer = Namer::new("boundary", operation_id)?;
        boundary_surface(&mut part, &namer, &args.edges, &args.tangent).with_context(ctx)?;
        Ok(part)
    }
}

/// A curve picked, as it runs along the boundary: its curve and its ends'
/// points and names in that direction — `forward` if that is the curve's
/// own.
#[derive(Clone, Debug)]
struct Side<S: Scalar> {
    entity: EntityRef,
    name: String,
    /// The edge's curve, as the edge runs.
    curve: Curve3<S>,
    forward: bool,
    start: Vector3<S>,
    end: Vector3<S>,
    start_name: String,
    end_name: String,
}

impl<S: Scalar> Side<S> {
    fn of(part: &Part<S>, entity: &EntityRef) -> GeopResult<Self> {
        let curve = entity.resolve_curve(part)?;
        Ok(Self {
            entity: entity.clone(),
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
    fn oriented(&self) -> Curve3<S> {
        if self.forward {
            self.curve.clone()
        } else {
            self.curve.reverse()
        }
    }

    /// How a coedge running the way the side runs uses its edge's copy.
    fn sense(&self) -> Sense {
        if self.forward {
            Sense::Forward
        } else {
            Sense::Reversed
        }
    }
}

/// A side of the unit square, as a pcurve, counter-clockwise from `(0, 0)`.
fn square_side<S: Scalar>(k: usize) -> GeopResult<NurbCurve2D<S>> {
    let corner = |k: usize| {
        let (u, v) = [(0, 0), (1, 0), (1, 1), (0, 1)][k % 4];
        Vector2::from_array([S::from_i64(u), S::from_i64(v)])
    };
    line2(corner(k), corner(k + 1))
}

/// A face on a patch over the unit square, bounded by `sides` — an edge
/// and the sense it is run in — counter-clockwise from `(0, 0)`.
pub(crate) fn patch_face<S: Scalar>(
    surface: NurbSurface3D<S>,
    sides: [(usize, Sense); 4],
) -> GeopResult<FaceSpec<S>> {
    Ok(FaceSpec {
        surface,
        outer: sides
            .iter()
            .enumerate()
            .map(|(k, &(e, sense))| {
                Ok(CoedgeSpec {
                    on: CoedgeOn::Edge(e, sense),
                    pcurve: square_side(k)?,
                })
            })
            .collect::<GeopResult<_>>()?,
        holes: Vec::new(),
    })
}

/// A plain number to compare by: a free choice between candidates.
fn distance<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> f64 {
    a.sub(b).norm().to_f64()
}

/// Spans a sheet between `edges`, tangent along `tangent` (see the module
/// docs), and returns it. What it builds is named after `namer`, as
/// [`BoundarySurface`] says.
pub fn boundary_surface<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    edges: &[EntityRef],
    tangent: &[EntityRef],
) -> GeopResult<BuiltBody> {
    if let Some(t) = tangent.iter().find(|t| !edges.contains(t)) {
        return Err(GeopError::new(format!(
            "{t} is to be tangent along, but is not one of the curves spanned"
        )));
    }
    let sides = edges
        .iter()
        .map(|e| Side::of(part, e))
        .collect::<GeopResult<Vec<_>>>()?;
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
                .map(|t| Ok((t.clone(), tangent_plane(part, t)?)))
                .collect::<GeopResult<Vec<_>>>()?;
            fill(namer, &sides, &planes)?
        }
    };
    part.build_body(spec, names)
}

/// Whether the two edges close a loop between them: each end of one at an
/// end of the other.
fn closes<S: Scalar>(a: &Side<S>, b: &Side<S>) -> bool {
    let meet = |p: &Vector3<S>, q: &Vector3<S>| p.could_be_equal(q);
    (meet(&a.end, &b.start) && meet(&b.end, &a.start))
        || (meet(&a.end, &b.end) && meet(&b.start, &a.start))
}

/// The ruled face between `a` and `b`, `b` turned if its ends line up
/// better with `a`'s that way round.
fn ruled<S: Scalar>(
    namer: &Namer,
    a: &Side<S>,
    b: &Side<S>,
) -> GeopResult<(BodySpec<S>, BodyNames)> {
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
    let copy = |side: &Side<S>, start: usize, end: usize| EdgeSpec {
        curve: side.curve.clone(),
        start: if side.forward { start } else { end },
        end: if side.forward { end } else { start },
    };
    let edges = vec![
        copy(a, 0, 1),
        copy(&b, 2, 3),
        EdgeSpec {
            curve: line3(a.end, b.end)?,
            start: 1,
            end: 3,
        },
        EdgeSpec {
            curve: line3(b.start, a.start)?,
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
    let face = patch_face(
        surface,
        [
            (0, a.sense()),
            (2, Sense::Forward),
            (1, b.sense().opposite()),
            (3, Sense::Forward),
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
fn chain<S: Scalar>(sides: &[Side<S>]) -> GeopResult<Vec<Side<S>>> {
    let mut left: Vec<Side<S>> = sides[1..].to_vec();
    let mut out = vec![sides[0].clone()];
    while !left.is_empty() {
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
type Tangency<S> = (Plane<S>, Vector3<S>);

/// The plane of the flat face the edge `entity` bounds, and the direction
/// in it away from that face at the middle of the edge — what a fill
/// tangent along the edge continues.
fn tangent_plane<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> GeopResult<Tangency<S>> {
    let model = part.topology();
    let EntityRef::Edge { name } = entity else {
        return Err(GeopError::new(format!(
            "{entity} bounds no face: tangency is to the one face an edge bounds"
        )));
    };
    let edge = part.edge_id(name)?;
    let coedges = model.coedges_of_edge(edge);
    let [coedge] = coedges.as_slice() else {
        return Err(GeopError::new(format!(
            "edge {name} bounds {} faces: tangency is to the one face an edge bounds, standing on its own",
            coedges.len()
        )));
    };
    let coedge = model.get_coedge(*coedge)?;
    let face = model.get_face(coedge.face)?;
    let Some(plane) = face.surface.as_plane()? else {
        return Err(GeopError::new(format!(
            "face {} along edge {name} is not flat: tangency is supported to flat faces",
            name_of(part, coedge.face)?
        )));
    };
    // The face lies to the left of its boundary, seen from where its normal
    // points: away from it is the edge's direction, as the face runs it,
    // crossed with the normal.
    let curve = &model.get_edge(edge)?.curve;
    let (t0, t1) = curve.domain();
    let mut along = curve.tangent(t0.add(t1).div(S::TWO)?.sharpen())?;
    if coedge.sense == Sense::Reversed {
        along = along.neg();
    }
    let away = along.prod_cross(&plane.normal);
    Ok((plane, away))
}

/// Whether the loop of `sides` has a corner where side `k` meets the next:
/// their tangents there are not parallel.
fn has_corner<S: Scalar>(a: &Curve3<S>, b: &Curve3<S>) -> GeopResult<bool> {
    let end = a.tangent(a.domain().1)?;
    let start = b.tangent(b.domain().0)?;
    Ok(!end.prod_cross(&start).norm_sq().could_be_equal(S::ZERO))
}

/// The closed loop `sides` filled (see the module docs), tangent along the
/// edges of `planes` to their planes.
fn fill<S: Scalar>(
    namer: &Namer,
    sides: &[Side<S>],
    planes: &[(EntityRef, Tangency<S>)],
) -> GeopResult<(BodySpec<S>, BodyNames)> {
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
                .could_be_equal(S::ZERO)
                && p.signed_distance(&plane.point).could_be_equal(S::ZERO);
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

    let curves: Vec<Curve3<S>> = sides.iter().map(Side::oriented).collect();
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
            let mut at: [Option<Tangency<S>>; 4] = Default::default();
            for (e, plane) in planes {
                let k = sides.iter().position(|s| s.entity == *e).expect("a side");
                at[k] = Some(plane.clone());
            }
            surface = surface.tangent_to_planes(&at)?;
        }
        spec.faces.push(patch_face(
            surface,
            [0, 1, 2, 3].map(|k| (k, sides[k].sense())),
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
fn flat<S: Scalar>(sides: &[Side<S>]) -> GeopResult<Option<Plane<S>>> {
    let points: Vec<Vector3<S>> = sides
        .iter()
        .flat_map(|s| &s.curve.control_points)
        .map(|cp| {
            Ok(Vector3::from_array([
                cp[0].div(cp[3])?,
                cp[1].div(cp[3])?,
                cp[2].div(cp[3])?,
            ]))
        })
        .collect::<GeopResult<_>>()?;
    // Which three is a free choice: far apart, so the plane through them is
    // well determined.
    let p0 = points[0];
    let far = |key: &dyn Fn(&Vector3<S>) -> f64| {
        *points
            .iter()
            .max_by(|a, b| key(a).total_cmp(&key(b)))
            .expect("points")
    };
    let p1 = far(&|p| distance(p, &p0));
    let p2 = far(&|p| p.sub(&p0).prod_cross(&p1.sub(&p0)).norm().to_f64());
    let normal = p1.sub(&p0).prod_cross(&p2.sub(&p0));
    if normal.norm_sq().could_be_equal(S::ZERO) {
        return Err(GeopError::new(
            "the edges all lie along one line: they enclose nothing to fill",
        ));
    }
    let plane = Plane::try_new(p0, normal)?;
    Ok(points
        .iter()
        .all(|p| plane.signed_distance(p).could_be_equal(S::ZERO))
        .then_some(plane))
}

/// The flat face on `plane` within the loop `sides`: the box around the
/// loop in the plane, as a bilinear patch, the loop's pcurves the curves'
/// control points in its coordinates — an affine map, exact for any curve.
/// Parametrized along the plane's `(x, y)` or `(y, x)`, whichever the loop
/// winds counter-clockwise in.
fn flat_face<S: Scalar>(sides: &[Side<S>], plane: &Plane<S>) -> GeopResult<FaceSpec<S>> {
    let origin = plane.point;
    let first = &sides[0].curve.control_points;
    let towards = Vector3::from_array([0, 1, 2].map(|k| first[first.len() - 1][k]))
        .prod_scalar(S::ONE.div(first[first.len() - 1][3])?);
    // Any direction in the plane would do for `x`: towards the far end of
    // the first curve, unless that is where it starts.
    let mut e1 = towards.sub(&origin);
    if e1.norm_sq().could_be_equal(S::ZERO) {
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
    // In the plane's coordinates, homogeneous.
    let local = |curve: &Curve3<S>| -> Vec<Vector3<S>> {
        curve
            .control_points
            .iter()
            .map(|cp| {
                let w = cp[3];
                let d = Vector3::from_array([cp[0], cp[1], cp[2]]).sub(&origin.prod_scalar(w));
                Vector3::from_array([d.prod_dot(&e1), d.prod_dot(&e2), w])
            })
            .collect()
    };
    let oriented: Vec<Curve3<S>> = sides.iter().map(Side::oriented).collect();
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for curve in &oriented {
        for cp in local(curve) {
            for k in 0..2 {
                let x = cp[k].div(cp[2])?;
                lo[k] = lo[k].min(x.lower().to_f64());
                hi[k] = hi[k].max(x.upper().to_f64());
            }
        }
    }
    // The patch spans exactly this box: a curve stays within the hull of
    // its control points, and these are the outer bounds of theirs.
    let lo = [S::from_f64(lo[0]), S::from_f64(lo[1])];
    let size = [S::from_f64(hi[0]).sub(lo[0]), S::from_f64(hi[1]).sub(lo[1])];
    let pcurves = oriented
        .iter()
        .map(|curve| {
            let control_points = local(curve)
                .iter()
                .map(|cp| {
                    Ok(Vector3::from_array([
                        cp[0].sub(cp[2].mul(lo[0])).div(size[0])?,
                        cp[1].sub(cp[2].mul(lo[1])).div(size[1])?,
                        cp[2],
                    ]))
                })
                .collect::<GeopResult<Vec<_>>>()?;
            NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())
        })
        .collect::<GeopResult<Vec<NurbCurve2D<S>>>>()?;
    let mut polygon = Vec::new();
    for pcurve in &pcurves {
        let (t0, t1) = pcurve.domain();
        for i in 0..8 {
            let fraction = S::from_ratio(i, 8)?;
            polygon.push(pcurve.evaluate(S::interpolate(t0, t1, fraction))?);
        }
    }
    let area = polygon_signed_area(&polygon);
    let swapped = if area.definitely_greater(S::ZERO) {
        false
    } else if area.definitely_less(S::ZERO) {
        true
    } else {
        return Err(GeopError::new(
            "cannot tell which way the loop runs round: it encloses no area",
        ));
    };
    let corner = |i: usize, j: usize| {
        let x = if i == 1 { lo[0].add(size[0]) } else { lo[0] };
        let y = if j == 1 { lo[1].add(size[1]) } else { lo[1] };
        origin.add(&e1.prod_scalar(x)).add(&e2.prod_scalar(y))
    };
    let surface = if swapped {
        bilinear(corner(0, 0), corner(0, 1), corner(1, 1), corner(1, 0))?
    } else {
        bilinear(corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1))?
    };
    let outer = sides
        .iter()
        .zip(pcurves)
        .enumerate()
        .map(|(k, (s, pcurve))| CoedgeSpec {
            on: CoedgeOn::Edge(k, s.sense()),
            pcurve: if swapped { pcurve.swap_xy() } else { pcurve },
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
fn quads<S: Scalar>(
    namer: &Namer,
    sides: &[Side<S>],
    curves: &[Curve3<S>],
    mut spec: BodySpec<S>,
    mut names: BodyNames,
) -> GeopResult<(BodySpec<S>, BodyNames)> {
    let n = sides.len();
    // The inner polygon: each corner halfway to the corners' average,
    // moved onto the plane through that average square to the loop's
    // (Newell) normal — all free choices, sharpened, so long as the
    // polygon is flat and inside the loop.
    let center = sides
        .iter()
        .fold(Vector3::zero(), |sum, s| sum.add(&s.start))
        .prod_scalar(S::ONE.div(S::from_i64(n as i64))?)
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
    let plane = Plane::try_new(center, Vector3::from_array(normal.map(S::from_f64)))?;
    let inner: Vec<Vector3<S>> = sides
        .iter()
        .map(|s| {
            let halfway = center.add(&s.start.sub(&center).prod_scalar(S::ONE.div(S::TWO)?));
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
            curve: line3(s.start, inner[k])?,
            start: k,
            end: inner_at(k),
        });
        names.edges.push(namer.name(&[&s.start_name, "spoke"]));
        let curve = line3(inner[k], inner[(k + 1) % n])?;
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
        spec.faces.push(patch_face(
            surface,
            [
                (3 * k, sides[k].sense()),
                (3 * next + 1, Sense::Forward),
                (3 * k + 2, Sense::Reversed),
                (3 * k + 1, Sense::Reversed),
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
