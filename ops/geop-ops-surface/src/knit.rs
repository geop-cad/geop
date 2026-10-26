//! [`Knit`]: sheets whose edges meet, joined into one — and into a solid,
//! once they close up.
//!
//! Joining is done on the sheets' description ([`BodySpec`]), which is then
//! built anew as one body:
//!
//! - **vertices** of different sheets at one point become one;
//! - **edges** then running between the same two vertices become one where
//!   their curves do too — the middle of each on the other. That an edge
//!   joins the same vertices is the topological question; the curves are
//!   asked only to tell apart edges that share both ends but run
//!   differently, as the two halves of a circle do;
//! - **faces** are turned where a shared edge would otherwise be run the
//!   same way by both: a body's faces wind consistently, each edge run one
//!   way by one face and the other way by the other.
//!
//! What is left used by one face only is an open edge. With none, the
//! sheets close up into a solid, its faces turned to point out of it.
//!
//! An edge that runs along two of another sheet — a T-junction, where a
//! vertex of that sheet lies inside the edge — is first split there
//! (`split_at_junctions`), so the edges meet end to end. Otherwise edges
//! that meet only in part are not joined: they are open edges, and a solid
//! asked for is refused, naming them.
//!
//! Sheets made to some accuracy — another system's, a file's — meet only
//! to within it. The user can state it, as a gap: then what is within the
//! gap of each other is joined, and, once the sheets close up, every vertex
//! joined is moved to where the surfaces around it meet and every edge
//! from it to where its two faces do (`geop_lib_shell::rebuilt`). The gap is
//! never guessed, nor grown until things join.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use std::collections::HashMap;

use geop_core_geometry::{
    contains::curve::curve_could_contain,
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::walk::{Take, flood},
    union_find::UnionFind,
    vector::{Vector3, Vector4},
};
use geop_core_topology::{
    Body, Curve2, Curve3, Sense, ShellId,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
    validation::{Outward, ValidationParameters, normal_points_outward},
};
use geop_lib_shell::rebuilt;
use geop_ops_model::{BodyNames, Face, ModelOperation, NamedModel, Namer, build_model};
use serde::{Deserialize, Serialize};

/// Joins the sheets the faces named `faces` stand in into one, along the
/// edges where they meet, for the operation `K` — see the module docs.
/// Closed up, the result is a solid named `knit(K)`; with `solid`, it must
/// close up, and open edges are refused by name. Every face, edge and
/// vertex keeps its name, of two joined into one the first sheet's.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Knit;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KnitArgs {
    /// A face of each sheet to join.
    pub faces: Vec<ElementReference<NamedModel, Face>>,
    /// Whether the result must close up into a solid.
    #[serde(default)]
    pub solid: bool,
    /// How far apart edges and vertices of the sheets may be and still be
    /// joined: the accuracy the sheets were made to. None, to join only
    /// what meets.
    #[serde(default)]
    pub gap: f64,
}

part_operation!(Knit(KnitArgs));

impl PartOperation for Knit {
    const KIND: &'static str = "knit";
    const INFO: OperationInfo = OperationInfo::new(
        "Knit",
        "Join faces standing on their own along the edges where they meet, into a solid once they close up.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::KNIT);
    type Args = KnitArgs;
    type Session = ();

    /// Nothing picked yet; a solid wanted.
    fn new_args(_: &Context) -> KnitArgs {
        KnitArgs {
            faces: Vec::new(),
            solid: true,
            gap: 0.0,
        }
    }

    fn run(ctx: &mut StepContext, args: &KnitArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The sheets, picked by a face of each, and whether they must close.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &KnitArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, KnitArgs> {
        let mut f = EditForm::<KnitArgs>::default();
        f.pick(
            "faces",
            "sheets",
            &context.before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        f.checkbox("solid", "close into a solid", args.solid, |edit, solid| {
            edit.args.solid = solid
        });
        let gap = Number::new("gap", args.gap, Unit::Length).range(0.0, 1.0);
        f.number("gap", gap, |edit, gap| edit.args.gap = gap);
        f
    }
}

impl ModelOperation for Knit {
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &KnitArgs) -> GeopResult<()> {
        let namer = Namer::new("knit", step)?;
        let mut sheets: Vec<ShellId> = Vec::new();
        for face in &args.faces {
            let sheet = model.sheet_of_face(model.face_id(face.name())?, "sheets are knit")?;
            if !sheets.contains(&sheet) {
                sheets.push(sheet);
            }
        }
        if !(args.gap.is_finite() && args.gap >= 0.0) {
            return Err(GeopError::new(format!(
                "a gap is a distance of zero or more, not {}",
                args.gap
            )));
        }
        knit(
            model,
            &namer,
            &sheets,
            args.solid,
            Scalar::from_f64(args.gap),
        )?;
        Ok(())
    }
}

/// Joins `sheets` (see the module docs), what is within `gap` of each
/// other as one, and returns what they became: a solid named `namer`'s
/// root once they close up, a sheet otherwise — an error naming the open
/// edges, if `solid` asks for one.
pub fn knit(
    part: &mut NamedModel,
    namer: &Namer,
    sheets: &[ShellId],
    solid: bool,
    gap: Scalar,
) -> GeopResult<Body> {
    if sheets.is_empty() {
        return Err(GeopError::new(
            "knitting needs faces standing on their own: pick the sheets to join",
        ));
    }
    if sheets.len() < 2 && !solid {
        return Err(GeopError::new("knitting needs two sheets at least"));
    }
    let model = part.topology();
    let mut faces = Vec::new();
    let mut sheet_of_face = Vec::new();
    for (k, &sheet) in sheets.iter().enumerate() {
        for face in model.body_faces(Body::Sheet(sheet))? {
            faces.push(face);
            sheet_of_face.push(k);
        }
    }
    let (mut spec, sources) = model.body_spec(&faces, false)?;
    let BodyNames {
        vertices: mut vertex_names,
        edges: mut edge_names,
        faces: face_names,
        ..
    } = part.source_names(&sources)?;
    let near = Near { gap };
    split_at_junctions(
        &mut spec,
        &sheet_of_face,
        &mut vertex_names,
        &mut edge_names,
        namer,
        near,
    )?;

    let joined = Joined::new(&spec, &sheet_of_face, &vertex_names, &edge_names, near)?;
    let (mut out, uses) = joined.spec(&spec)?;
    let open: Vec<&str> = uses
        .iter()
        .enumerate()
        .filter(|&(_, &n)| n == 1)
        .map(|(e, _)| edge_names[joined.edge_source[e]].as_str())
        .collect();
    if solid && !open.is_empty() {
        return Err(GeopError::new(format!(
            "the sheets do not close up into a solid: edges {open:?} are open — each is used by one face only, with no edge of another sheet running along it from end to end"
        )));
    }
    out.solid = open.is_empty();
    let names = BodyNames {
        vertices: joined
            .vertex_source
            .iter()
            .map(|&v| vertex_names[v].clone())
            .collect(),
        edges: joined
            .edge_source
            .iter()
            .map(|&e| edge_names[e].clone())
            .collect(),
        faces: face_names,
        solid: out.solid.then(|| namer.root()),
    };
    turn_consistently(&mut out, &names)?;
    // Joined across a gap, a vertex is the union of the points joined and
    // an edge the first of the curves: wide, and off the faces by as much
    // as the gap. Closed up, every vertex joined moves to where the
    // surfaces around it meet, every edge from it to where its faces do
    // (see `geop_lib_shell::rebuilt`), and the width comes back down to
    // how well the surfaces agree.
    if gap.definitely_greater(Scalar::ZERO) && out.solid {
        let reseat: Vec<bool> = (0..out.vertices.len())
            .map(|v| (joined.vertex.iter()).filter(|&&w| w == v).count() > 1)
            .collect();
        let kept = vec![false; out.faces.len()];
        out = rebuilt(&out, &names, &kept, &reseat).map_err(|e| {
            e.with_context("moving what was joined across the gap onto where the surfaces meet")
        })?;
    }
    let consumed: Vec<Body> = sheets.iter().map(|&s| Body::Sheet(s)).collect();
    part.delete_bodies(&consumed)?;
    let built = part.build_body(out, names)?;
    let Some(solid_id) = built.solid else {
        return built.body();
    };
    // The faces wind consistently, but which way round is a guess: turned
    // all, if they point into the solid.
    let params = ValidationParameters::default();
    let model = part.topology();
    let mut inward = None;
    for &f in &built.faces {
        if let Some(verdict) = normal_points_outward(&params, model, solid_id, built.shells[0], f)?
        {
            inward = Some(matches!(verdict, Outward::No(_)));
            break;
        }
    }
    match inward {
        Some(true) => {
            for &f in &built.faces {
                part.reverse_face(f)?;
            }
        }
        Some(false) => {}
        None => {
            return Err(GeopError::new(
                "the sheets close up, but no face tells which side of them the solid is on",
            ));
        }
    }
    Ok(Body::Solid(solid_id))
}

/// How the entities of the sheets' description join: each vertex's and
/// edge's representative, and which of them stay.
struct Joined {
    /// Per vertex of the description, the index of the vertex it becomes.
    vertex: Vec<usize>,
    /// Per vertex kept, the vertex of the description it is.
    vertex_source: Vec<usize>,
    /// Per edge of the description, the edge it becomes, and whether it
    /// runs the other way than that one.
    edge: Vec<(usize, bool)>,
    /// Per edge kept, the edge of the description it is.
    edge_source: Vec<usize>,
}

impl Joined {
    fn new(
        spec: &BodySpec,
        sheet_of_face: &[usize],
        vertex_names: &[String],
        edge_names: &[String],
        near: Near,
    ) -> GeopResult<Self> {
        // The sheet each vertex and edge is of.
        let mut vertex_sheet = vec![usize::MAX; spec.vertices.len()];
        let mut edge_sheet = vec![usize::MAX; spec.edges.len()];
        for (f, face) in spec.faces.iter().enumerate() {
            for c in std::iter::once(&face.outer).chain(&face.holes).flatten() {
                match c.on {
                    CoedgeOn::Edge(e, _) => {
                        edge_sheet[e] = sheet_of_face[f];
                        vertex_sheet[spec.edges[e].start] = sheet_of_face[f];
                        vertex_sheet[spec.edges[e].end] = sheet_of_face[f];
                    }
                    CoedgeOn::Vertex(v) => vertex_sheet[v] = sheet_of_face[f],
                }
            }
        }

        // Vertices at one point, of different sheets, joined.
        let n = spec.vertices.len();
        let mut groups = UnionFind::new(n);
        for i in 0..n {
            for j in i + 1..n {
                if vertex_sheet[i] != vertex_sheet[j]
                    && near.points(&spec.vertices[i], &spec.vertices[j])
                {
                    groups.union(i, j);
                }
            }
        }
        let mut vertex = vec![0; n];
        let mut vertex_source = Vec::new();
        for group in groups.groups() {
            let mut sheets: Vec<usize> = group.iter().map(|&v| vertex_sheet[v]).collect();
            sheets.sort();
            if sheets.windows(2).any(|w| w[0] == w[1]) {
                let names: Vec<&str> = group.iter().map(|&v| vertex_names[v].as_str()).collect();
                return Err(GeopError::new(format!(
                    "vertices {names:?} could all be at one point, two of them of the same sheet: which to join is ambiguous"
                )));
            }
            for &v in &group {
                vertex[v] = vertex_source.len();
            }
            vertex_source.push(group[0]);
        }

        // Edges between the same two vertices, of different sheets, whose
        // curves coincide, joined.
        let ends = |e: usize| {
            let (a, b) = (vertex[spec.edges[e].start], vertex[spec.edges[e].end]);
            (a.min(b), a.max(b))
        };
        let mut between: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for e in 0..spec.edges.len() {
            between.entry(ends(e)).or_default().push(e);
        }
        let mut partner: Vec<Option<usize>> = vec![None; spec.edges.len()];
        let mut keys: Vec<_> = between.keys().copied().collect();
        keys.sort();
        for key in keys {
            let candidates = &between[&key];
            for (i, &a) in candidates.iter().enumerate() {
                for &b in &candidates[i + 1..] {
                    if edge_sheet[a] == edge_sheet[b]
                        || !coincide(&spec.edges[a], &spec.edges[b], near)?
                    {
                        continue;
                    }
                    if let Some(other) = partner[a].or(partner[b]) {
                        return Err(GeopError::new(format!(
                            "edges {}, {} and {} could all run along one another: which to join is ambiguous",
                            edge_names[a], edge_names[b], edge_names[other]
                        )));
                    }
                    partner[a] = Some(b);
                    partner[b] = Some(a);
                }
            }
        }
        let mut edge = vec![(0, false); spec.edges.len()];
        let mut edge_source = Vec::new();
        for e in 0..spec.edges.len() {
            match partner[e] {
                Some(first) if first < e => {
                    let (kept, this) = (&spec.edges[first], &spec.edges[e]);
                    let turned = vertex[kept.start] != vertex[this.start]
                        || (kept.start == kept.end) && turned_closed(kept, this)?;
                    edge[e] = (edge[first].0, turned);
                }
                _ => {
                    edge[e] = (edge_source.len(), false);
                    edge_source.push(e);
                }
            }
        }
        Ok(Self {
            vertex,
            vertex_source,
            edge,
            edge_source,
        })
    }

    /// The description of the joined body, every coedge on the vertices and
    /// edges kept, and how many coedges use each edge kept.
    fn spec(&self, spec: &BodySpec) -> GeopResult<(BodySpec, Vec<usize>)> {
        let mut uses = vec![0; self.edge_source.len()];
        let mut faces = Vec::with_capacity(spec.faces.len());
        for face in &spec.faces {
            let mut remap = |lp: &Vec<geop_core_topology::build::CoedgeSpec>| {
                lp.iter()
                    .map(|c| {
                        let mut c = c.clone();
                        c.on = match c.on {
                            CoedgeOn::Edge(e, sense) => {
                                let (kept, turned) = self.edge[e];
                                uses[kept] += 1;
                                CoedgeOn::Edge(kept, if turned { sense.opposite() } else { sense })
                            }
                            CoedgeOn::Vertex(v) => CoedgeOn::Vertex(self.vertex[v]),
                        };
                        c
                    })
                    .collect::<Vec<_>>()
            };
            let outer = remap(&face.outer);
            let holes = face.holes.iter().map(&mut remap).collect();
            faces.push(FaceSpec {
                surface: face.surface.clone(),
                outer,
                holes,
            });
        }
        let out = BodySpec {
            vertices: self
                .vertex_source
                .iter()
                .map(|&v| {
                    // One point, the union of every vertex joined into it.
                    (0..spec.vertices.len())
                        .filter(|&w| self.vertex[w] == self.vertex[v])
                        .map(|w| spec.vertices[w])
                        .reduce(|a, b| a.union(&b))
                        .expect("the vertex itself")
                })
                .collect(),
            edges: self
                .edge_source
                .iter()
                .map(|&e| {
                    let edge = &spec.edges[e];
                    EdgeSpec {
                        curve: edge.curve.clone(),
                        start: self.vertex[edge.start],
                        end: self.vertex[edge.end],
                    }
                })
                .collect(),
            faces,
            shells: vec![(0..spec.faces.len()).collect()],
            solid: false,
        };
        Ok((out, uses))
    }
}

/// Whether two edges between the same vertices are one: the middle of each
/// lies on the other's curve, as far as `near` tells.
fn coincide(a: &EdgeSpec, b: &EdgeSpec, near: Near) -> GeopResult<bool> {
    let middle = |e: &EdgeSpec| e.curve.evaluate(e.curve.mid_parameter()?);
    let on = |e: &EdgeSpec, p| -> GeopResult<bool> { Ok(near.on_curve(&e.curve, &p)?.is_some()) };
    Ok(on(b, middle(a)?)? && on(a, middle(b)?)?)
}

/// How far apart what is joined may be: the gap the sheets were made with,
/// which the user states — nothing is guessed. Without one, only what
/// could be at one point, as the enclosures say, is joined.
#[derive(Clone, Copy, Debug)]
struct Near {
    gap: Scalar,
}

impl Near {
    /// `p` grown by the gap in every direction: every point within it.
    fn grown(&self, p: &Vector3) -> Vector3 {
        Vector3::from_array(std::array::from_fn(|k| {
            p[k].sub(self.gap).union(p[k].add(self.gap))
        }))
    }

    /// Whether `a` and `b` could be within the gap of each other.
    fn points(&self, a: &Vector3, b: &Vector3) -> bool {
        self.grown(a).could_be_equal(b)
    }

    /// Where on `curve` a point within the gap of `p` could be, if any.
    fn on_curve(&self, curve: &Curve3, p: &Vector3) -> GeopResult<Option<Scalar>> {
        curve_could_contain(curve, &self.grown(p), MAX_NODES, min_subdivision_size())
    }
}

/// Splits every edge of one sheet at each vertex of another that lies on
/// it, inside it — a T-junction: one sheet's edge running along the edges
/// of two of another — so the edges meet end to end and join. The split
/// point is the edge's own, refined to its point nearest the vertex: the
/// edge's two pieces run on exactly as it did, their shared end a vertex
/// of its own sheet, which joins the other's vertex as any two do. The
/// new vertex and the piece from it on are named `K(E,V)` for the knit
/// `K`, the edge `E` and the vertex `V`; the edge keeps its name for the
/// piece up to it.
fn split_at_junctions(
    spec: &mut BodySpec,
    sheet_of_face: &[usize],
    vertex_names: &mut Vec<String>,
    edge_names: &mut Vec<String>,
    namer: &Namer,
    near: Near,
) -> GeopResult<()> {
    let mut edge_sheet = vec![usize::MAX; spec.edges.len()];
    let mut vertex_sheet = vec![usize::MAX; spec.vertices.len()];
    for (f, face) in spec.faces.iter().enumerate() {
        for c in face.loops().into_iter().flatten() {
            let (a, b) = spec.ends(c.on)?;
            vertex_sheet[a] = sheet_of_face[f];
            vertex_sheet[b] = sheet_of_face[f];
            if let CoedgeOn::Edge(e, _) = c.on {
                edge_sheet[e] = sheet_of_face[f];
            }
        }
    }
    let mut e = 0;
    while e < spec.edges.len() {
        let mut split = None;
        for (v, &point) in spec.vertices.iter().enumerate() {
            let edge = &spec.edges[e];
            if vertex_sheet[v] == edge_sheet[e]
                || near.points(&point, &spec.vertices[edge.start])
                || near.points(&point, &spec.vertices[edge.end])
            {
                continue;
            }
            let Some(t) = near.on_curve(&edge.curve, &point)? else {
                continue;
            };
            let t = edge.curve.closest_parameter_from(&point, t.sharpen())?;
            let (t0, t1) = edge.curve.domain();
            if t.definitely_greater(t0)
                && t.definitely_less(t1)
                && near.points(&edge.curve.evaluate(t)?, &point)
            {
                split = Some((v, t));
                break;
            }
        }
        match split {
            Some((v, t)) => {
                let name = namer.piece(&edge_names[e], &vertex_names[v]);
                split_edge(spec, e, t)?;
                vertex_sheet.push(edge_sheet[e]);
                vertex_names.push(name.clone());
                edge_sheet.push(edge_sheet[e]);
                edge_names.push(name);
                // The piece up to the vertex may meet another.
            }
            None => e += 1,
        }
    }
    Ok(())
}

/// Splits the edge `e` of `spec` at its parameter `t`, sharp: a new vertex
/// there, `e` running up to it and a new edge on from it, and every coedge
/// along `e` two, its pcurve split where it passes the vertex's foot point
/// on its face.
fn split_edge(spec: &mut BodySpec, e: usize, t: Scalar) -> GeopResult<()> {
    let edge = spec.edges[e].clone();
    let (t0, t1) = edge.curve.domain();
    let (first, second) = (edge.curve.sub_curve(t0, t)?, edge.curve.sub_curve(t, t1)?);
    let point = edge.curve.evaluate(t)?;
    let v = spec.vertices.len();
    spec.vertices.push(point);
    spec.edges[e] = EdgeSpec {
        curve: first,
        start: edge.start,
        end: v,
    };
    let next = spec.edges.len();
    spec.edges.push(EdgeSpec {
        curve: second,
        start: v,
        end: edge.end,
    });
    for face in &mut spec.faces {
        let surface = face.surface.clone();
        for lp in std::iter::once(&mut face.outer).chain(face.holes.iter_mut()) {
            let Some(k) = lp
                .iter()
                .position(|c| matches!(c.on, CoedgeOn::Edge(x, _) if x == e))
            else {
                continue;
            };
            let coedge = lp[k].clone();
            let CoedgeOn::Edge(_, sense) = coedge.on else {
                unreachable!("found on an edge")
            };
            let (before, after) = split_pcurve(&surface, &coedge.pcurve, &point)?;
            let pieces = match sense {
                Sense::Forward => [(e, before), (next, after)],
                Sense::Reversed => [(next, before), (e, after)],
            };
            lp.splice(
                k..=k,
                pieces.map(|(edge, pcurve)| CoedgeSpec {
                    on: CoedgeOn::Edge(edge, sense),
                    pcurve,
                }),
            );
        }
    }
    Ok(())
}

/// `pcurve`, on `surface`, split where it passes `point`'s foot point: at
/// its parameter nearest that `(u, v)`, found as a curve in space lying in
/// the plane `z = 0`.
fn split_pcurve(
    surface: &NurbSurface3D,
    pcurve: &Curve2,
    point: &Vector3,
) -> GeopResult<(Curve2, Curve2)> {
    let (s0, s1) = pcurve.domain();
    let middle = pcurve.evaluate(pcurve.mid_parameter()?)?;
    let (u, v) = surface.project(
        *point,
        middle[0].sharpen(),
        middle[1].sharpen(),
        PROJECT_ITERATIONS,
    )?;
    let lifted = Curve3::try_new(
        pcurve.degree,
        (pcurve.control_points.iter())
            .map(|cp| Vector4::from_array([cp[0], cp[1], Scalar::ZERO, cp[2]]))
            .collect(),
        pcurve.knot_vector.clone(),
    )?;
    let target = Vector3::from_array([u, v, Scalar::ZERO]);
    let s = lifted.closest_parameter_from(&target, pcurve.mid_parameter()?.sharpen())?;
    if !(s.definitely_greater(s0) && s.definitely_less(s1)) {
        return Err(GeopError::new(format!(
            "the pcurve does not pass the split point's foot point {:?} inside it",
            (u, v)
        )));
    }
    Ok((pcurve.sub_curve(s0, s)?, pcurve.sub_curve(s, s1)?))
}

/// Whether the closed edge `b` runs round the other way than `a`, which it
/// coincides with: their directions at their common start compared.
fn turned_closed(a: &EdgeSpec, b: &EdgeSpec) -> GeopResult<bool> {
    let ta = a.curve.tangent(a.curve.domain().0)?;
    let tb = b.curve.tangent(b.curve.domain().0)?;
    let along = ta.prod_dot(&tb);
    if along.definitely_less(Scalar::ZERO) {
        Ok(true)
    } else if along.definitely_greater(Scalar::ZERO) {
        Ok(false)
    } else {
        Err(GeopError::new(
            "cannot tell which way round two closed edges run along one another",
        ))
    }
}

/// Turns faces of `spec` so that every edge two faces share is run one way
/// by one and the other way by the other — keeping the first face of each
/// connected part as it is. An error if the faces cannot be turned so (a
/// one-sided surface), or do not all hang together.
fn turn_consistently(spec: &mut BodySpec, names: &BodyNames) -> GeopResult<()> {
    // Per edge, the faces using it and the sense they use it in.
    let mut users: Vec<Vec<(usize, geop_core_topology::Sense)>> =
        vec![Vec::new(); spec.edges.len()];
    for (f, face) in spec.faces.iter().enumerate() {
        for c in std::iter::once(&face.outer).chain(&face.holes).flatten() {
            if let CoedgeOn::Edge(e, sense) = c.on {
                users[e].push((f, sense));
            }
        }
    }
    let n = spec.faces.len();
    let mut turned: Vec<Option<bool>> = vec![None; n];
    turned[0] = Some(false);
    flood([0], Take::Newest, |&f| {
        let flip = turned[f].expect("visited");
        let mut found = Vec::new();
        for (e, at) in users.iter().enumerate() {
            let [(a, sa), (b, sb)] = at.as_slice() else {
                continue;
            };
            let (g, same) = if *a == f {
                (*b, sa == sb)
            } else if *b == f {
                (*a, sa == sb)
            } else {
                continue;
            };
            // Run the same way by both, one of them has to turn.
            let want = flip ^ same;
            match turned[g] {
                None => {
                    turned[g] = Some(want);
                    found.push(g);
                }
                Some(have) if have != want => {
                    return Err(GeopError::new(format!(
                        "faces {} and {} cannot be turned to wind the same way along their shared edge {}: the sheets make a one-sided surface",
                        names.faces[f], names.faces[g], names.edges[e]
                    )));
                }
                Some(_) => {}
            }
        }
        Ok(found)
    })?;
    if let Some(f) = turned.iter().position(Option::is_none) {
        return Err(GeopError::new(format!(
            "face {} meets none of the others along an edge: the sheets do not hang together",
            names.faces[f]
        )));
    }
    for (f, t) in turned.into_iter().enumerate() {
        if t == Some(true) {
            spec.faces[f] = spec.faces[f].reversed();
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
