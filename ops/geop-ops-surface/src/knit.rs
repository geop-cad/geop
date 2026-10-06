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
//! Edges that meet only in part — one running along part of another — are
//! not joined: they are open edges, and a solid asked for is refused,
//! naming them.

use std::collections::HashMap;

use geop_core_geometry::contains::curve::curve_could_contain;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
    with_context,
};
use geop_core_topology::{
    Body, ShellId,
    build::{BodySpec, CoedgeOn, EdgeSpec, FaceSpec},
    validation::{Outward, ValidationParameters, normal_points_outward},
};
use geop_ops::{
    BodyNames, Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::Form,
};
use serde::{Deserialize, Serialize};

use crate::{MAX_NODES, min_subdivision_size, name_of, sheet_of};

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
    pub faces: Vec<String>,
    /// Whether the result must close up into a solid.
    #[serde(default)]
    pub solid: bool,
}

impl Operation for Knit {
    type Args = KnitArgs;
    type Session = ();

    /// Nothing picked yet; a solid wanted.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> KnitArgs {
        KnitArgs {
            faces: Vec::new(),
            solid: true,
        }
    }

    /// The sheets, picked by a face of each, and whether they must close.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &KnitArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, KnitArgs> {
        let mut f = Form::<S, KnitArgs>::new();
        f.reference(
            "faces",
            "sheets",
            EntityRef::of_names(EntityRef::face, &args.faces),
            &[Role::Sheet],
            None,
            true,
            |e, picked| e.args.faces = EntityRef::names_of(EntityRef::face, &picked),
        );
        f.checkbox("solid", "close into a solid", args.solid, |args, solid| {
            args.solid = solid
        });
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &KnitArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("knit({operation_id}, {args:?})");
        let namer = Namer::new("knit", operation_id)?;
        let mut sheets: Vec<ShellId> = Vec::new();
        for name in &args.faces {
            let (_, sheet) = sheet_of(&part, name).with_context(ctx)?;
            if !sheets.contains(&sheet) {
                sheets.push(sheet);
            }
        }
        knit(&mut part, &namer, &sheets, args.solid).with_context(ctx)?;
        Ok(part)
    }
}

/// Joins `sheets` (see the module docs) and returns what they became: a
/// solid named `namer`'s root once they close up, a sheet otherwise — an
/// error naming the open edges, if `solid` asks for one.
pub fn knit<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    sheets: &[ShellId],
    solid: bool,
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
    let (spec, sources) = model.body_spec(&faces, false)?;
    let names = |ids: Vec<geop_ops::RefId>| -> GeopResult<Vec<String>> {
        ids.into_iter().map(|id| name_of(part, id)).collect()
    };
    let vertex_names = names(sources.vertices.iter().map(|&v| v.into()).collect())?;
    let edge_names = names(sources.edges.iter().map(|&e| e.into()).collect())?;
    let face_names = names(sources.faces.iter().map(|&f| f.into()).collect())?;

    let joined = Joined::new(&spec, &sheet_of_face, &vertex_names, &edge_names)?;
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
    let consumed: Vec<Body> = sheets.iter().map(|&s| Body::Sheet(s)).collect();
    part.assemble_sheet(&consumed, &[])?;
    let built = part.build_body(out, names)?;
    let Some(solid_id) = built.solid else {
        return Ok(Body::Sheet(built.shells[0]));
    };
    // The faces wind consistently, but which way round is a guess: turned
    // all, if they point into the solid.
    let params = ValidationParameters::default();
    let model = part.topology();
    let inward = built
        .faces
        .iter()
        .find_map(|&f| normal_points_outward(&params, model, solid_id, built.shells[0], f))
        .map(|verdict| matches!(verdict, Outward::No { .. }));
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
    fn new<S: Scalar>(
        spec: &BodySpec<S>,
        sheet_of_face: &[usize],
        vertex_names: &[String],
        edge_names: &[String],
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
                    && spec.vertices[i].could_be_equal(&spec.vertices[j])
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
                    if edge_sheet[a] == edge_sheet[b] || !coincide(&spec.edges[a], &spec.edges[b])?
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
    fn spec<S: Scalar>(&self, spec: &BodySpec<S>) -> GeopResult<(BodySpec<S>, Vec<usize>)> {
        let mut uses = vec![0; self.edge_source.len()];
        let mut faces = Vec::with_capacity(spec.faces.len());
        for face in &spec.faces {
            let mut remap = |lp: &Vec<geop_core_topology::build::CoedgeSpec<S>>| {
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
/// lies on the other's curve.
fn coincide<S: Scalar>(a: &EdgeSpec<S>, b: &EdgeSpec<S>) -> GeopResult<bool> {
    let middle = |e: &EdgeSpec<S>| -> GeopResult<_> {
        let (t0, t1) = e.curve.domain();
        e.curve.evaluate(t0.add(t1).div(S::TWO)?)
    };
    let on = |e: &EdgeSpec<S>, p| -> GeopResult<bool> {
        Ok(curve_could_contain(&e.curve, &p, MAX_NODES, min_subdivision_size())?.is_some())
    };
    Ok(on(b, middle(a)?)? && on(a, middle(b)?)?)
}

/// Whether the closed edge `b` runs round the other way than `a`, which it
/// coincides with: their directions at their common start compared.
fn turned_closed<S: Scalar>(a: &EdgeSpec<S>, b: &EdgeSpec<S>) -> GeopResult<bool> {
    let ta = a.curve.tangent(a.curve.domain().0)?;
    let tb = b.curve.tangent(b.curve.domain().0)?;
    let along = ta.prod_dot(&tb);
    if along.definitely_less(S::ZERO) {
        Ok(true)
    } else if along.definitely_greater(S::ZERO) {
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
fn turn_consistently<S: Scalar>(spec: &mut BodySpec<S>, names: &BodyNames) -> GeopResult<()> {
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
    let mut queue = vec![0];
    while let Some(f) = queue.pop() {
        let flip = turned[f].expect("visited");
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
                    queue.push(g);
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
    }
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
