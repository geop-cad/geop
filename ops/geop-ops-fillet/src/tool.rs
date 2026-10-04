//! A blend's tool as a solid built whole (see [`build_tools`]): the
//! cross-section — an arc between two contacts, closed off by two lines to
//! an apex — at stations along the blended edges, swept between them by
//! spans, capped where a tool ends on its own, and joined to the other
//! tools at a corner every edge of which is blended (see
//! [`crate::corner`]) by the corner's own faces.

use std::collections::HashMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3, Vector4},
};
use geop_core_topology::{
    Sense, SolidId,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_extrude_revolve::common::{bilinear, embed_point, line2};

/// One span of a tool: its control rows along it — each the four section
/// control points (see [`ToolStation::points`]) — of `degree` on `knots`, and
/// what its faces are called after, if anything.
#[derive(Clone, Debug)]
pub(crate) struct ToolSpan<S: Scalar> {
    pub(crate) degree: usize,
    pub(crate) knots: Vec<S>,
    pub(crate) rows: Vec<[Vector4<S>; 4]>,
    pub(crate) name: Option<String>,
}

/// A flat cap of a tool: the plane it lies in, `(origin, e1, e2)` with
/// `e1 x e2` out of the tool, and the section's control points in it,
/// homogeneous — what both the cap's pcurves and the section's edges in
/// space are made of, so that they agree exactly.
#[derive(Clone, Debug)]
pub(crate) struct Cap<S: Scalar> {
    pub(crate) plane: [Vector3<S>; 3],
    pub(crate) flat: [Vector3<S>; 4],
}

/// Where a tool's section lies at one of its stations: the section's
/// control points, its vertices — the two contacts and the apex — what it
/// is called, and its cap, if it is capped there.
#[derive(Clone, Debug)]
pub(crate) struct ToolStation<S: Scalar> {
    pub(crate) points: [Vector4<S>; 4],
    pub(crate) vertices: [Vector3<S>; 3],
    pub(crate) name: String,
    pub(crate) cap: Option<Cap<S>>,
}

/// A blend's tool, ready to build: its stations and the spans between
/// them, around to the first again if `closed` — else ending at its first
/// and last station, each capped or joined to a [`Corner`].
#[derive(Clone, Debug)]
pub(crate) struct Tool<S: Scalar> {
    pub(crate) stations: Vec<ToolStation<S>>,
    pub(crate) spans: Vec<ToolSpan<S>>,
    pub(crate) closed: bool,
    /// What the walls rising from the first and the second contact are
    /// called: `a` and `b` after the faces on the edge's left and right.
    pub(crate) sides: [&'static str; 2],
}

/// The homogeneous point `h`'s coordinates in the plane `(origin, e1, e2)`,
/// homogeneous: `(w x, w y, w)`.
pub(crate) fn flatten<S: Scalar>(h: &Vector4<S>, [o, e1, e2]: &[Vector3<S>; 3]) -> Vector3<S> {
    let w = h[3];
    let d = Vector3::from_array([h[0], h[1], h[2]]).sub(&o.prod_scalar(w));
    Vector3::from_array([d.prod_dot(e1), d.prod_dot(e2), w])
}

/// The point `v`'s coordinates in the plane `plane`, homogeneous.
pub(crate) fn flatten_point<S: Scalar>(v: &Vector3<S>, plane: &[Vector3<S>; 3]) -> Vector3<S> {
    flatten(&Vector4::from_array([v[0], v[1], v[2], S::ONE]), plane)
}

/// The homogeneous plane coordinates `p` in space, on `plane`.
pub(crate) fn embed<S: Scalar>(p: &Vector3<S>, [o, e1, e2]: &[Vector3<S>; 3]) -> Vector4<S> {
    embed_point(p, o, e1, e2)
}

/// The wall section curve `c` (see [`CURVE_POINTS`]) sweeps through
/// `span`: `u` along the span, `v` along the curve.
pub(crate) fn wall_surface<S: Scalar>(
    span: &ToolSpan<S>,
    c: usize,
) -> GeopResult<NurbSurface3D<S>> {
    let control_points = span
        .rows
        .iter()
        .flat_map(|row| CURVE_POINTS[c].iter().map(move |&k| row[k]))
        .collect();
    let (degree_v, knots_v) = if c == 0 {
        (2, vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE])
    } else {
        (1, vec![S::ZERO, S::ZERO, S::ONE, S::ONE])
    };
    NurbSurface::try_new(
        span.degree,
        degree_v,
        control_points,
        span.knots.clone(),
        knots_v,
    )
}

/// The joints a section curve runs between, by curve: the arc from the
/// first contact to the second, the line on to the apex, the line back.
const CURVE_JOINTS: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
/// The section control points (see [`ToolStation::points`]) each curve is made
/// of.
const CURVE_POINTS: [&[usize]; 3] = [&[0, 1, 2], &[2, 3], &[3, 0]];
/// The section control point each joint is.
const JOINT_POINT: [usize; 3] = [0, 2, 3];

/// The section's curve `c` (see [`CURVE_POINTS`]) of control points
/// `points`: rational quadratic for the arc, lines else.
fn section_curve<S: Scalar, const D: usize>(
    points: &[Vector<S, D>; 4],
    c: usize,
) -> GeopResult<NurbCurve<S, D>> {
    let control_points = CURVE_POINTS[c].iter().map(|&k| points[k]).collect();
    let knots = if c == 0 {
        vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE]
    } else {
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE]
    };
    NurbCurve::try_new(if c == 0 { 2 } else { 1 }, control_points, knots)
}

/// A wall's sides in its own parameters: the curve at the span's first
/// station, run backwards along `u = 0`; the path of its start along
/// `v = 0`; the curve at the last station along `u = 1`; the path of its
/// end, backwards along `v = 1`.
fn wall_pcurves<S: Scalar>() -> GeopResult<[NurbCurve2D<S>; 4]> {
    let p = |u: f64, v: f64| Vector2::from_array([S::from_f64(u), S::from_f64(v)]);
    Ok([
        line2(p(0.0, 1.0), p(0.0, 0.0))?,
        line2(p(0.0, 0.0), p(1.0, 0.0))?,
        line2(p(1.0, 0.0), p(1.0, 1.0))?,
        line2(p(1.0, 1.0), p(0.0, 1.0))?,
    ])
}

/// The flat cap on `cap`'s plane holding its section: the box around the
/// section in the plane, as a bilinear patch facing out of the tool, and
/// the section's curves in its parameters.
fn cap_face<S: Scalar>(cap: &Cap<S>) -> GeopResult<(NurbSurface3D<S>, [NurbCurve2D<S>; 3])> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in &cap.flat {
        for k in 0..2 {
            let x = p[k].div(p[2])?;
            lo[k] = lo[k].min(x.lower().to_f64());
            hi[k] = hi[k].max(x.upper().to_f64());
        }
    }
    // The cap spans exactly this box, which encloses the section: a NURBS
    // curve stays within the convex hull of its control points.
    let lo = [S::from_f64(lo[0]), S::from_f64(lo[1])];
    let size = [
        S::from_f64(hi[0]).sub(lo[0]).upper(),
        S::from_f64(hi[1]).sub(lo[1]).upper(),
    ];
    let [o, e1, e2] = &cap.plane;
    let corner = |i: usize, j: usize| {
        let x = if i == 1 { lo[0].add(size[0]) } else { lo[0] };
        let y = if j == 1 { lo[1].add(size[1]) } else { lo[1] };
        o.add(&e1.prod_scalar(x)).add(&e2.prod_scalar(y))
    };
    let surface = bilinear(corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1))?;
    let mut uv = [Vector3::zero(); 4];
    for (k, p) in cap.flat.iter().enumerate() {
        uv[k] = Vector3::from_array([
            p[0].sub(p[2].mul(lo[0])).div(size[0])?,
            p[1].sub(p[2].mul(lo[1])).div(size[1])?,
            p[2],
        ]);
    }
    Ok((
        surface,
        [
            section_curve(&uv, 0)?,
            section_curve(&uv, 1)?,
            section_curve(&uv, 2)?,
        ],
    ))
}

/// A corner every edge of which is blended, where their tools meet (see
/// [`crate::corner`]): the ball rounding it, centered at `center` with
/// `radius`, touching its faces at `contacts`; the tools ending there; the
/// apex `apex` their apexes are joined to, on the far side of the faces;
/// and a point `inside` the tools there — the solid's own corner.
#[derive(Clone, Debug)]
pub(crate) struct Corner<S: Scalar> {
    /// What the corner's own entities are named by: the vertex's.
    pub(crate) namer: Namer,
    pub(crate) center: Vector3<S>,
    pub(crate) radius: S,
    pub(crate) contacts: Vec<Vector3<S>>,
    pub(crate) apex: Vector3<S>,
    pub(crate) inside: Vector3<S>,
    pub(crate) ends: Vec<CornerEnd>,
}

/// A tool ending at a [`Corner`]: which, at its last station or its first,
/// and which of the corner's contacts its section's first and second
/// contact are.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CornerEnd {
    pub(crate) tool: usize,
    pub(crate) at_end: bool,
    pub(crate) contacts: [usize; 2],
}

impl CornerEnd {
    /// The index of the station of `tool` that ends here.
    fn station<S: Scalar>(&self, tool: &Tool<S>) -> usize {
        if self.at_end {
            tool.stations.len() - 1
        } else {
            0
        }
    }
}

/// The line from `a` to `b`.
fn line3<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> GeopResult<NurbCurve<S, 4>> {
    let h = |p: &Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
    NurbCurve::try_new(1, vec![h(a), h(b)], vec![S::ZERO, S::ZERO, S::ONE, S::ONE])
}

/// The point `(u, v)` of a face's parameters.
fn uv<S: Scalar>(u: f64, v: f64) -> Vector2<S> {
    Vector2::from_array([S::from_f64(u), S::from_f64(v)])
}

/// A face's loop through the vertices `corners`, each side along the edge
/// — or at the vertex — given with its pcurve, the way the loop runs: the
/// coedges, each in the sense its edge runs from one corner to the next.
fn face_loop<S: Scalar>(
    spec: &BodySpec<S>,
    corners: &[usize],
    sides: Vec<(CoedgeOn, NurbCurve2D<S>)>,
) -> GeopResult<Vec<CoedgeSpec<S>>> {
    sides
        .into_iter()
        .enumerate()
        .map(|(i, (on, pcurve))| {
            let on = match on {
                CoedgeOn::Edge(e, _) => {
                    let edge = &spec.edges[e];
                    let (from, to) = (corners[i], corners[(i + 1) % corners.len()]);
                    let sense = if edge.start == from && edge.end == to {
                        Sense::Forward
                    } else if edge.start == to && edge.end == from {
                        Sense::Reversed
                    } else {
                        return Err(GeopError::new(format!(
                            "a corner's face runs from vertex {from} to {to} along an edge between {} and {}",
                            edge.start, edge.end
                        )));
                    };
                    CoedgeOn::Edge(e, sense)
                }
                vertex => vertex,
            };
            Ok(CoedgeSpec { on, pcurve })
        })
        .collect()
}

/// Builds `tools`, each with the namer of the edge it blends (see
/// [`crate::blend::blend`]), and the `corners` joining them into one solid
/// named `solid`.
///
/// Each tool has a vertex at each joint of every station, `N(ta,st0)`,
/// `N(tb,st0)`, `N(q,st0)`, ...; the section's curves at every station,
/// `N(fillet,st0)`, `N(b,st0)`, `N(a,st0)`; each joint's path through every
/// span, `N(ta,s0)`, ...; the walls each curve sweeps through every span,
/// the blend `N(fillet,s0)` — the span left out of a tool of one, and the
/// run-out spans `start` and `end` — and the caps of an open one, `N(start)`
/// and `N(end)`, where it ends on its own.
///
/// Where it ends at a corner, its contacts are the corner's — `N(t0)`,
/// `N(t1)`, ... by the corner's namer — and the corner closes the tools off
/// with its own faces: the ball's piece between the tools' arcs, `N(corner)`
/// (for three tools, a spherical triangle, see
/// [`NurbSurface3D::spherical_triangle`]); and round each contact the
/// bilinear patch from it along both tools' lines to their apexes, and on
/// along the lines `N(q0)`, ... to the corner's apex `N(apex)`: `N(side0)`,
/// ....
pub(crate) fn build_tools<S: Scalar>(
    part: &mut Part<S>,
    tools: &[(&Namer, &Tool<S>)],
    corners: &[Corner<S>],
    solid: String,
) -> GeopResult<SolidId> {
    let mut spec = BodySpec {
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: true,
    };
    let mut names = BodyNames {
        solid: Some(solid),
        ..BodyNames::default()
    };

    // The corners' contacts, and the tools' stations that end on them.
    let mut contact_vertex: Vec<Vec<usize>> = Vec::new();
    let mut glued: HashMap<(usize, usize), [usize; 2]> = HashMap::new();
    for corner in corners {
        let mut ids = Vec::new();
        for (k, p) in corner.contacts.iter().enumerate() {
            spec.vertices.push(*p);
            names.vertices.push(corner.namer.name(&[&format!("t{k}")]));
            ids.push(spec.vertices.len() - 1);
        }
        for end in &corner.ends {
            let station = end.station(tools[end.tool].1);
            glued.insert((end.tool, station), end.contacts.map(|k| ids[k]));
        }
        contact_vertex.push(ids);
    }

    // Per tool, per station: its joints' vertices and its curves' edges.
    let mut vertex: Vec<Vec<[usize; 3]>> = Vec::new();
    let mut station_edge: Vec<Vec<[usize; 3]>> = Vec::new();
    for (t, (namer, tool)) in tools.iter().enumerate() {
        let joint_names = [
            format!("t{}", tool.sides[0]),
            format!("t{}", tool.sides[1]),
            "q".to_string(),
        ];
        let curve_names = ["fillet", tool.sides[1], tool.sides[0]];
        let count = tool.stations.len();
        let next = |j: usize| (j + 1) % count;
        let qualified = |name: &str, span: &Option<String>| match span {
            Some(s) => namer.name(&[name, s]),
            None => namer.name(&[name]),
        };

        let mut tool_vertex = Vec::new();
        for (s, station) in tool.stations.iter().enumerate() {
            let shared = glued.get(&(t, s));
            let mut ids = [0; 3];
            for k in 0..3 {
                if let (Some(shared), true) = (shared, k < 2) {
                    ids[k] = shared[k];
                    continue;
                }
                spec.vertices.push(station.vertices[k]);
                names
                    .vertices
                    .push(namer.name(&[&joint_names[k], &station.name]));
                ids[k] = spec.vertices.len() - 1;
            }
            tool_vertex.push(ids);
        }
        let mut tool_station_edge = Vec::new();
        for (s, station) in tool.stations.iter().enumerate() {
            let mut ids = [0; 3];
            for c in 0..3 {
                let (a, b) = CURVE_JOINTS[c];
                spec.edges.push(EdgeSpec {
                    curve: section_curve(&station.points, c)?,
                    start: tool_vertex[s][a],
                    end: tool_vertex[s][b],
                });
                names
                    .edges
                    .push(namer.name(&[curve_names[c], &station.name]));
                ids[c] = spec.edges.len() - 1;
            }
            tool_station_edge.push(ids);
        }
        let mut lateral = Vec::new();
        for (j, span) in tool.spans.iter().enumerate() {
            let mut ids = [0; 3];
            for k in 0..3 {
                spec.edges.push(EdgeSpec {
                    curve: NurbCurve::try_new(
                        span.degree,
                        span.rows.iter().map(|r| r[JOINT_POINT[k]]).collect(),
                        span.knots.clone(),
                    )?,
                    start: tool_vertex[j][k],
                    end: tool_vertex[next(j)][k],
                });
                names.edges.push(qualified(&joint_names[k], &span.name));
                ids[k] = spec.edges.len() - 1;
            }
            lateral.push(ids);
        }

        let [first_back, start_side, last_forward, end_side] = wall_pcurves::<S>()?;
        for (j, span) in tool.spans.iter().enumerate() {
            for c in 0..3 {
                let (a, b) = CURVE_JOINTS[c];
                let surface = wall_surface(span, c)?;
                let on = |edge: usize, sense: Sense, pcurve: &NurbCurve2D<S>| CoedgeSpec {
                    on: CoedgeOn::Edge(edge, sense),
                    pcurve: pcurve.clone(),
                };
                spec.faces.push(FaceSpec {
                    surface,
                    outer: vec![
                        on(tool_station_edge[j][c], Sense::Reversed, &first_back),
                        on(lateral[j][a], Sense::Forward, &start_side),
                        on(tool_station_edge[next(j)][c], Sense::Forward, &last_forward),
                        on(lateral[j][b], Sense::Reversed, &end_side),
                    ],
                    holes: Vec::new(),
                });
                names.faces.push(qualified(curve_names[c], &span.name));
            }
        }

        if !tool.closed {
            for (s, forward, name) in [(0, true, "start"), (count - 1, false, "end")] {
                let Some(cap) = &tool.stations[s].cap else {
                    if glued.contains_key(&(t, s)) {
                        continue;
                    }
                    return Err(GeopError::new(format!("the tool's {name} has no cap")));
                };
                let (surface, pcurves) = cap_face(cap)?;
                let mut outer: Vec<CoedgeSpec<S>> = (0..3)
                    .map(|c| {
                        if forward {
                            CoedgeSpec {
                                on: CoedgeOn::Edge(tool_station_edge[s][c], Sense::Forward),
                                pcurve: pcurves[c].clone(),
                            }
                        } else {
                            CoedgeSpec {
                                on: CoedgeOn::Edge(tool_station_edge[s][c], Sense::Reversed),
                                pcurve: pcurves[c].reverse(),
                            }
                        }
                    })
                    .collect();
                if !forward {
                    outer.reverse();
                }
                spec.faces.push(FaceSpec {
                    surface,
                    outer,
                    holes: Vec::new(),
                });
                names.faces.push(namer.name(&[name]));
            }
        }
        vertex.push(tool_vertex);
        station_edge.push(tool_station_edge);
    }

    for (corner, contacts) in corners.iter().zip(&contact_vertex) {
        let built = Built {
            tools,
            contacts,
            vertex: &vertex,
            station_edge: &station_edge,
        };
        corner_faces(&mut spec, &mut names, corner, &built)?;
    }
    spec.shells = vec![(0..spec.faces.len()).collect()];
    part.build_body(spec, names)?
        .solid
        .ok_or_else(|| GeopError::new("the blend's tool came out as no solid"))
}

/// What a corner's faces are joined to: the tools, the corner's contacts'
/// vertices, and per tool and station its joints' vertices and its curves'
/// edges.
struct Built<'a, S: Scalar> {
    tools: &'a [(&'a Namer, &'a Tool<S>)],
    contacts: &'a [usize],
    vertex: &'a [Vec<[usize; 3]>],
    station_edge: &'a [Vec<[usize; 3]>],
}

impl<S: Scalar> Built<'_, S> {
    /// The edge of curve `c` of the station `end` ends at.
    fn curve(&self, end: &CornerEnd, c: usize) -> usize {
        self.station_edge[end.tool][end.station(self.tools[end.tool].1)][c]
    }

    /// The vertex of the apex of the station `end` ends at, and its point.
    fn apex(&self, end: &CornerEnd) -> (usize, Vector3<S>) {
        let tool = self.tools[end.tool].1;
        let s = end.station(tool);
        (self.vertex[end.tool][s][2], tool.stations[s].vertices[2])
    }
}

/// Adds `corner`'s own faces (see [`build_tools`]) to `spec`, joined to
/// the tools as `built`.
fn corner_faces<S: Scalar>(
    spec: &mut BodySpec<S>,
    names: &mut BodyNames,
    corner: &Corner<S>,
    built: &Built<S>,
) -> GeopResult<()> {
    if corner.contacts.len() != 3 || corner.ends.len() != 3 {
        return Err(GeopError::new(format!(
            "a corner of {} faces and {} blended edges: only corners of three are rounded",
            corner.contacts.len(),
            corner.ends.len()
        )));
    }
    let contacts = built.contacts;
    spec.vertices.push(corner.apex);
    names.vertices.push(corner.namer.name(&["apex"]));
    let apex = spec.vertices.len() - 1;
    // Each tool's apex to the corner's.
    let mut to_apex = Vec::new();
    for (i, end) in corner.ends.iter().enumerate() {
        let (q, at) = built.apex(end);
        spec.edges.push(EdgeSpec {
            curve: line3(&at, &corner.apex)?,
            start: q,
            end: apex,
        });
        names.edges.push(corner.namer.name(&[&format!("q{i}")]));
        to_apex.push(spec.edges.len() - 1);
    }

    // The ball's piece between the tools' arcs, facing the ball.
    let arc = |from: usize, to: usize| -> GeopResult<usize> {
        corner
            .ends
            .iter()
            .find(|e| e.contacts == [from, to] || e.contacts == [to, from])
            .map(|e| built.curve(e, 0))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "no blended edge of the corner runs between its contacts {from} and {to}"
                ))
            })
    };
    let triangle = |order: [usize; 3]| {
        NurbSurface3D::spherical_triangle(
            &corner.center,
            corner.radius,
            order.map(|k| corner.contacts[k]),
        )
    };
    let mut order = [0, 1, 2];
    let mut surface = triangle(order)?;
    let (u, v) = (S::from_f64(0.4), S::from_f64(0.3));
    let towards = surface
        .normal(u, v)?
        .prod_dot(&corner.center.sub(&surface.evaluate(u, v)?));
    if towards.definitely_less(S::ZERO) {
        order = [1, 0, 2];
        surface = triangle(order)?;
    } else if !towards.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "cannot tell which way the corner's ball faces ({towards:?})"
        )));
    }
    let [p, q, r] = order;
    let outer = face_loop(
        spec,
        &[contacts[p], contacts[q], contacts[r], contacts[r]],
        vec![
            (
                CoedgeOn::Edge(arc(p, q)?, Sense::Forward),
                line2(uv(0.0, 0.0), uv(1.0, 0.0))?,
            ),
            (
                CoedgeOn::Edge(arc(q, r)?, Sense::Forward),
                line2(uv(1.0, 0.0), uv(1.0, 1.0))?,
            ),
            (
                CoedgeOn::Vertex(contacts[r]),
                line2(uv(1.0, 1.0), uv(0.0, 1.0))?,
            ),
            (
                CoedgeOn::Edge(arc(r, p)?, Sense::Forward),
                line2(uv(0.0, 1.0), uv(0.0, 0.0))?,
            ),
        ],
    )?;
    spec.faces.push(FaceSpec {
        surface,
        outer,
        holes: Vec::new(),
    });
    names.faces.push(corner.namer.name(&["corner"]));

    // Round each contact, from it along both tools' lines to the apex,
    // facing away from the corner of the solid.
    for k in 0..corner.contacts.len() {
        let touching: Vec<usize> = (0..corner.ends.len())
            .filter(|&i| corner.ends[i].contacts.contains(&k))
            .collect();
        let [mut i, mut j] = touching[..] else {
            return Err(GeopError::new(format!(
                "{} blended edges of the corner reach its contact {k}, not two",
                touching.len()
            )));
        };
        // The line from the contact to tool `i`'s apex: its curve `b`
        // where the contact is its section's second, `a` where its first.
        let line = |i: usize| {
            let end = &corner.ends[i];
            built.curve(end, if end.contacts[1] == k { 1 } else { 2 })
        };
        let quad = |i: usize, j: usize| {
            bilinear(
                corner.contacts[k],
                built.apex(&corner.ends[i]).1,
                corner.apex,
                built.apex(&corner.ends[j]).1,
            )
        };
        let mut surface = quad(i, j)?;
        let half = S::from_f64(0.5);
        let middle = surface.evaluate(half, half)?;
        let inward = surface
            .normal(half, half)?
            .prod_dot(&corner.inside.sub(&middle));
        if inward.definitely_greater(S::ZERO) {
            std::mem::swap(&mut i, &mut j);
            surface = quad(i, j)?;
        } else if !inward.definitely_less(S::ZERO) {
            return Err(GeopError::new(format!(
                "cannot tell which way the corner's side at contact {k} faces ({inward:?})"
            )));
        }
        let (qi, _) = built.apex(&corner.ends[i]);
        let (qj, _) = built.apex(&corner.ends[j]);
        let outer = face_loop(
            spec,
            &[contacts[k], qi, apex, qj],
            vec![
                (
                    CoedgeOn::Edge(line(i), Sense::Forward),
                    line2(uv(0.0, 0.0), uv(1.0, 0.0))?,
                ),
                (
                    CoedgeOn::Edge(to_apex[i], Sense::Forward),
                    line2(uv(1.0, 0.0), uv(1.0, 1.0))?,
                ),
                (
                    CoedgeOn::Edge(to_apex[j], Sense::Forward),
                    line2(uv(1.0, 1.0), uv(0.0, 1.0))?,
                ),
                (
                    CoedgeOn::Edge(line(j), Sense::Forward),
                    line2(uv(0.0, 1.0), uv(0.0, 0.0))?,
                ),
            ],
        )?;
        spec.faces.push(FaceSpec {
            surface,
            outer,
            holes: Vec::new(),
        });
        names.faces.push(corner.namer.name(&[&format!("side{k}")]));
    }
    Ok(())
}

/// Builds `tool` into a solid named `N(tool)`, `namer` the blended edge's
/// (see [`build_tools`]).
pub(crate) fn build_tool<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    tool: &Tool<S>,
) -> GeopResult<SolidId> {
    build_tools(part, &[(namer, tool)], &[], namer.name(&["tool"]))
}
