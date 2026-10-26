//! A blend's tool as a solid built whole (see [`build_tools`]): the
//! cross-section — an arc between two contacts, closed off by two lines to
//! an apex — at stations along the blended edges, swept between them by
//! spans, capped where a tool ends on its own, and joined to the other
//! tools at a corner every edge of which is blended (see
//! [`crate::corner`]) by the corner's own faces. The tools are strips of a
//! skinned body (see [`geop_lib_sweep::skin`]), each span given by
//! its control rows outright.

use geop_core_geometry::{
    nurb_curve::{self, NurbCurve, NurbCurve2D, NurbCurve3D, bezier_knots, embed_point},
    nurb_surface::{FlatPatch, NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3, Vector4},
};
use geop_core_topology::{
    FaceId, Sense, SolidId,
    build::{BodySpec, CoedgeOn, EdgeSpec},
};
use geop_lib_sweep::{Glue, Skinned, Strip, StripCap, StripGeometry, skin};
use geop_ops_model::{BodyNames, NamedModel, Namer};

/// One span of a tool: its control rows along it — each the four section
/// control points (see [`ToolStation::points`]) — of `degree` on `knots`, and
/// what its faces are called after, if anything.
#[derive(Clone, Debug)]
pub(crate) struct ToolSpan {
    pub(crate) degree: usize,
    pub(crate) knots: Vec<Scalar>,
    pub(crate) rows: Vec<[Vector4; 4]>,
    pub(crate) name: Option<String>,
}

/// A flat cap of a tool: the plane it lies in, `(origin, e1, e2)` with
/// `e1 x e2` out of the tool, and the section's control points in it,
/// homogeneous — what both the cap's pcurves and the section's edges in
/// space are made of, so that they agree exactly.
#[derive(Clone, Debug)]
pub(crate) struct Cap {
    pub(crate) plane: [Vector3; 3],
    pub(crate) flat: [Vector3; 4],
}

/// Where a tool's section lies at one of its stations: the section's
/// control points, its vertices — the two contacts and the apex — what it
/// is called, and its cap, if it is capped there.
#[derive(Clone, Debug)]
pub(crate) struct ToolStation {
    pub(crate) points: [Vector4; 4],
    pub(crate) vertices: [Vector3; 3],
    pub(crate) name: String,
    pub(crate) cap: Option<Cap>,
}

/// A blend's tool, ready to build: its stations and the spans between
/// them, around to the first again if `closed` — else ending at its first
/// and last station, each capped or joined to a [`Corner`].
#[derive(Clone, Debug)]
pub(crate) struct Tool {
    pub(crate) stations: Vec<ToolStation>,
    pub(crate) spans: Vec<ToolSpan>,
    pub(crate) closed: bool,
    /// What the walls rising from the first and the second contact are
    /// called: `a` and `b` after the faces on the edge's left and right.
    pub(crate) sides: [&'static str; 2],
    /// What the blend's own wall, between the contacts, is called:
    /// `fillet` or `chamfer`.
    pub(crate) blend: &'static str,
}

impl Tool {
    /// What its joints are called: its contacts `ta` and `tb`, after their
    /// walls, and its apex `q`.
    fn joint_names(&self) -> [String; 3] {
        [
            format!("t{}", self.sides[0]),
            format!("t{}", self.sides[1]),
            "q".to_string(),
        ]
    }
}

/// The homogeneous point `h`'s coordinates in the plane `(origin, e1, e2)`,
/// homogeneous: `(w x, w y, w)`.
pub(crate) fn flatten(h: &Vector4, [o, e1, e2]: &[Vector3; 3]) -> Vector3 {
    nurb_curve::flatten_point(h, o, e1, e2)
}

/// The point `v`'s coordinates in the plane `plane`, homogeneous.
pub(crate) fn flatten_point(v: &Vector3, plane: &[Vector3; 3]) -> Vector3 {
    flatten(&v.homogeneous(Scalar::ONE), plane)
}

/// The homogeneous plane coordinates `p` in space, on `plane`.
pub(crate) fn embed(p: &Vector3, [o, e1, e2]: &[Vector3; 3]) -> Vector4 {
    embed_point(p, o, e1, e2)
}

/// The wall section curve `c` (see [`CURVE_POINTS`]) sweeps through
/// `span`: `u` along the span, `v` along the curve.
pub(crate) fn wall_surface(span: &ToolSpan, c: usize) -> GeopResult<NurbSurface3D> {
    let control_points = span
        .rows
        .iter()
        .flat_map(|row| CURVE_POINTS[c].iter().map(move |&k| row[k]))
        .collect();
    let degree_v = if c == 0 { 2 } else { 1 };
    NurbSurface::try_new(
        span.degree,
        degree_v,
        control_points,
        span.knots.clone(),
        bezier_knots(degree_v),
    )
}

/// The section control points (see [`ToolStation::points`]) each curve is made
/// of: the arc from the first contact to the second, the line on to the
/// apex, the line back — curve `c` from joint `c` to the next, as a
/// [`Strip`]'s.
const CURVE_POINTS: [&[usize]; 3] = [&[0, 1, 2], &[2, 3], &[3, 0]];
/// The section control point each joint is.
const JOINT_POINT: [usize; 3] = [0, 2, 3];

/// The section's curve `c` (see [`CURVE_POINTS`]) of control points
/// `points`: rational quadratic for the arc, lines else.
fn section_curve<const D: usize>(points: &[Vector<D>; 4], c: usize) -> GeopResult<NurbCurve<D>> {
    let control_points = CURVE_POINTS[c].iter().map(|&k| points[k]).collect();
    let degree = if c == 0 { 2 } else { 1 };
    NurbCurve::try_new(degree, control_points, bezier_knots(degree))
}

/// The flat cap on `cap`'s plane holding its section (see [`FlatPatch`]),
/// facing out of the tool, and the section's curves in its parameters.
fn cap_face(cap: &Cap) -> GeopResult<(NurbSurface3D, [NurbCurve2D; 3])> {
    let curves = [
        section_curve(&cap.flat, 0)?,
        section_curve(&cap.flat, 1)?,
        section_curve(&cap.flat, 2)?,
    ];
    let [o, e1, e2] = cap.plane;
    let patch = FlatPatch::around(o, e1, e2, &curves, false)?;
    Ok((
        patch.surface()?,
        [
            patch.pcurve(&curves[0])?,
            patch.pcurve(&curves[1])?,
            patch.pcurve(&curves[2])?,
        ],
    ))
}

/// A corner every edge of which is blended, where their tools meet (see
/// [`crate::corner`]): the ball rounding it, centered at `center` with
/// `radius`, touching its faces at `contacts`; the tools ending there; the
/// apex `apex` their apexes are joined to, on the far side of the faces;
/// and a point `inside` the tools there — the solid's own corner.
#[derive(Clone, Debug)]
pub(crate) struct Corner {
    /// What the corner's own entities are named by: the vertex's.
    pub(crate) namer: Namer,
    pub(crate) center: Vector3,
    pub(crate) radius: Scalar,
    /// The faces the ball touches, one per contact.
    pub(crate) faces: Vec<FaceId>,
    pub(crate) contacts: Vec<Vector3>,
    pub(crate) apex: Vector3,
    pub(crate) inside: Vector3,
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
    pub(crate) fn station(&self, tool: &Tool) -> usize {
        if self.at_end {
            tool.stations.len() - 1
        } else {
            0
        }
    }
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
pub(crate) fn build_tools(
    model: &mut NamedModel,
    tools: &[(&Namer, &Tool)],
    corners: &[Corner],
    solid: String,
) -> GeopResult<SolidId> {
    let mut spec = BodySpec::new(true);
    let mut names = BodyNames {
        solid: Some(solid),
        ..BodyNames::default()
    };

    // The corners' contacts first, each tool's contacts at a corner joined
    // to them.
    let mut glued: Vec<Vec<Glue>> = vec![Vec::new(); tools.len()];
    let mut contact_vertex = Vec::with_capacity(corners.len());
    for corner in corners {
        let first = spec.vertices.len();
        for (k, p) in corner.contacts.iter().enumerate() {
            spec.vertices.push(*p);
            names.vertices.push(corner.namer.name(&[&format!("t{k}")]));
        }
        for end in &corner.ends {
            let station = end.station(tools[end.tool].1);
            for k in 0..2 {
                glued[end.tool].push(Glue::Vertex {
                    station,
                    joint: k,
                    vertex: first + end.contacts[k],
                });
            }
        }
        contact_vertex.push((first..spec.vertices.len()).collect::<Vec<_>>());
    }
    let joint_names: Vec<[String; 3]> = tools.iter().map(|(_, tool)| tool.joint_names()).collect();
    let curve_names: Vec<[String; 3]> = tools
        .iter()
        .map(|(_, tool)| [tool.blend, tool.sides[1], tool.sides[0]].map(String::from))
        .collect();
    let station_names: Vec<Vec<String>> = tools
        .iter()
        .map(|(_, tool)| tool.stations.iter().map(|s| s.name.clone()).collect())
        .collect();
    let span_names: Vec<Vec<Option<String>>> = tools
        .iter()
        .map(|(_, tool)| tool.spans.iter().map(|s| s.name.clone()).collect())
        .collect();
    let flags = [false; 3];
    let mut strips = Vec::with_capacity(tools.len());
    let mut caps = Vec::new();
    for (t, ((namer, tool), glued)) in tools.iter().zip(glued).enumerate() {
        let ends_glued = |s: usize| {
            corners
                .iter()
                .flat_map(|c| &c.ends)
                .any(|e| e.tool == t && e.station(tool) == s)
        };
        if !tool.closed {
            let last = tool.stations.len() - 1;
            for (s, forward, name) in [(0, true, "start"), (last, false, "end")] {
                let Some(cap) = &tool.stations[s].cap else {
                    if ends_glued(s) {
                        continue;
                    }
                    return Err(GeopError::new(format!("the tool's {name} has no cap")));
                };
                let (surface, pcurves) = cap_face(cap)?;
                caps.push(StripCap {
                    name: namer.name(&[name]),
                    station: s,
                    forward,
                    surface,
                    loops: vec![(t, pcurves.to_vec())],
                });
            }
        }
        strips.push(Strip {
            namer,
            joint_names: &joint_names[t],
            curve_names: &curve_names[t],
            station_names: &station_names[t],
            span_names: &span_names[t],
            closed: tool.closed,
            poles: &flags,
            on_axis: &flags,
            glued,
            geometry: *tool,
        });
    }
    let built = skin(&mut spec, &mut names, &strips, &caps)?;

    for (corner, contacts) in corners.iter().zip(&contact_vertex) {
        let built = Built {
            tools,
            contacts,
            skinned: &built,
        };
        corner_faces(&mut spec, &mut names, corner, &built)?;
    }
    spec.shells = vec![(0..spec.faces.len()).collect()];
    model
        .build_body(spec, names)?
        .solid
        .ok_or_else(|| GeopError::new("the blend's tool came out as no solid"))
}

impl StripGeometry for Tool {
    fn joint(&self, s: usize, k: usize) -> GeopResult<Vector3> {
        Ok(self.stations[s].vertices[k])
    }

    fn curve(&self, s: usize, c: usize) -> GeopResult<NurbCurve3D> {
        section_curve(&self.stations[s].points, c)
    }

    fn lateral(&self, j: usize, k: usize) -> GeopResult<NurbCurve3D> {
        let span = &self.spans[j];
        NurbCurve::try_new(
            span.degree,
            span.rows.iter().map(|r| r[JOINT_POINT[k]]).collect(),
            span.knots.clone(),
        )
    }

    fn wall(&self, j: usize, c: usize) -> GeopResult<NurbSurface3D> {
        wall_surface(&self.spans[j], c)
    }
}

/// What a corner's faces are joined to: the tools, the corner's contacts'
/// vertices, and the tools' vertices and curves' edges (see [`skin`]).
struct Built<'a> {
    tools: &'a [(&'a Namer, &'a Tool)],
    contacts: &'a [usize],
    skinned: &'a Skinned,
}

impl Built<'_> {
    /// The edge of curve `c` of the station `end` ends at.
    fn curve(&self, end: &CornerEnd, c: usize) -> usize {
        let s = end.station(self.tools[end.tool].1);
        self.skinned.edges[end.tool][s][c]
            .expect("a tool's curves have edges")
            .0
    }

    /// The vertex of the apex of the station `end` ends at, and its point.
    fn apex(&self, end: &CornerEnd) -> (usize, Vector3) {
        let tool = self.tools[end.tool].1;
        let s = end.station(tool);
        (
            self.skinned.vertices[end.tool][s][2],
            tool.stations[s].vertices[2],
        )
    }
}

/// Adds `corner`'s own faces (see [`build_tools`]) to `spec`, joined to
/// the tools as `built`.
fn corner_faces(
    spec: &mut BodySpec,
    names: &mut BodyNames,
    corner: &Corner,
    built: &Built,
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
            curve: NurbCurve3D::line(at, corner.apex)?,
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
    let (u, v) = (Scalar::from_f64(0.4), Scalar::from_f64(0.3));
    let towards = surface
        .normal(u, v)?
        .prod_dot(&corner.center.sub(&surface.evaluate(u, v)?));
    if towards.definitely_less(Scalar::ZERO) {
        order = [1, 0, 2];
        surface = triangle(order)?;
    } else if !towards.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "cannot tell which way the corner's ball faces ({towards:?})"
        )));
    }
    let [p, q, r] = order;
    let face = spec.square_face(
        surface,
        [contacts[p], contacts[q], contacts[r], contacts[r]],
        [
            CoedgeOn::Edge(arc(p, q)?, Sense::Forward),
            CoedgeOn::Edge(arc(q, r)?, Sense::Forward),
            CoedgeOn::Vertex(contacts[r]),
            CoedgeOn::Edge(arc(r, p)?, Sense::Forward),
        ],
    )?;
    spec.faces.push(face);
    names.faces.push(corner.namer.name(&["corner"]));

    // Round each contact, from it along both tools' lines to the apex,
    // facing away from the corner of the solid.
    for (k, (&at, &contact)) in corner.contacts.iter().zip(contacts).enumerate() {
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
            NurbSurface3D::bilinear(
                at,
                built.apex(&corner.ends[i]).1,
                corner.apex,
                built.apex(&corner.ends[j]).1,
            )
        };
        let mut surface = quad(i, j)?;
        let half = Scalar::from_f64(0.5);
        let middle = surface.evaluate(half, half)?;
        let inward = surface
            .normal(half, half)?
            .prod_dot(&corner.inside.sub(&middle));
        if inward.definitely_greater(Scalar::ZERO) {
            std::mem::swap(&mut i, &mut j);
            surface = quad(i, j)?;
        } else if !inward.definitely_less(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "cannot tell which way the corner's side at contact {k} faces ({inward:?})"
            )));
        }
        let (qi, _) = built.apex(&corner.ends[i]);
        let (qj, _) = built.apex(&corner.ends[j]);
        let face = spec.square_face(
            surface,
            [contact, qi, apex, qj],
            [
                CoedgeOn::Edge(line(i), Sense::Forward),
                CoedgeOn::Edge(to_apex[i], Sense::Forward),
                CoedgeOn::Edge(to_apex[j], Sense::Forward),
                CoedgeOn::Edge(line(j), Sense::Forward),
            ],
        )?;
        spec.faces.push(face);
        names.faces.push(corner.namer.name(&[&format!("side{k}")]));
    }
    Ok(())
}
