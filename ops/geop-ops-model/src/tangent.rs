//! [`TangentChain`]: a tangent chain of edges — edges running on into one another
//! between faces tangent across every vertex of it, as one smooth edge —
//! found from any edge of it ([`tangent_chain`]), and the [`Link`]s it is
//! made of: each edge the way the chain runs along it, with the faces
//! either side.

use geop_core_geometry::search::{MAX_NODES, min_subdivision_size};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::walk::{Walked, walk_by},
    vector::Vector3,
};
use geop_core_topology::{EdgeId, FaceId, Model, VertexId};

/// The faces either side of `edge`: an error for one that does not join
/// two.
fn faces(model: &Model, edge: EdgeId) -> GeopResult<[FaceId; 2]> {
    model.faces_of_edge(edge).ok_or_else(|| {
        GeopError::new(format!(
            "edge {edge} does not join two faces, so it is in no chain"
        ))
    })
}

/// One edge of a chain, as the chain runs along it: along the edge's own
/// direction, `forward`, or against it; with the faces on its left and
/// right as it runs (seen from outside the solid).
#[derive(Clone, Debug)]
pub struct Link {
    pub edge: EdgeId,
    pub forward: bool,
    pub faces: [FaceId; 2],
}

impl Link {
    /// The unit tangent of the edge at `p`, where its faces' normals there
    /// are `normals`: square to both, turned to run `along` the link. An
    /// error where the faces could meet tangentially.
    pub fn tangent(
        &self,
        normals: [Vector3; 2],
        along: &Vector3,
        p: &Vector3,
    ) -> GeopResult<Vector3> {
        let across = normals[0].prod_cross(&normals[1]);
        if across.prod_dot(along).definitely_less(Scalar::ZERO) {
            across.neg()
        } else {
            across
        }
        .normalize()
        .map_err(|_| {
            GeopError::new(format!(
                "faces {} and {} could meet tangentially at {p:?} on edge {}: there is no corner to blend there (normals {:?} and {:?})",
                self.faces[0], self.faces[1], self.edge, normals[0], normals[1]
            ))
        })
    }

    pub fn new(model: &Model, edge: EdgeId, forward: bool) -> GeopResult<Self> {
        let [l, r] = faces(model, edge)?;
        Ok(Link {
            edge,
            forward,
            faces: if forward { [l, r] } else { [r, l] },
        })
    }

    pub fn reversed(&self) -> Self {
        Link {
            edge: self.edge,
            forward: !self.forward,
            faces: [self.faces[1], self.faces[0]],
        }
    }

    /// The vertex the chain enters the edge at, and the one it leaves at.
    pub fn vertices(&self, model: &Model) -> GeopResult<[VertexId; 2]> {
        let e = model.get_edge(self.edge)?;
        Ok(if self.forward {
            [e.start_vertex, e.end_vertex]
        } else {
            [e.end_vertex, e.start_vertex]
        })
    }

    /// The edge's point `fraction` of the way along it as the chain runs —
    /// of its parameter — and the unit tangent there, the way it runs.
    pub fn at(&self, model: &Model, fraction: Scalar) -> GeopResult<(Vector3, Vector3)> {
        let curve = &model.get_edge(self.edge)?.curve;
        let (t0, t1) = curve.domain();
        let f = if self.forward {
            fraction
        } else {
            Scalar::ONE.sub(fraction)
        };
        let t = Scalar::interpolate(t0, t1, f);
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Link::at(edge={}, fraction={fraction:?}, t={t:?}): control points {:?}",
                self.edge, curve.control_points
            ))
        };
        let tangent = curve.tangent(t).map_err(ctx)?.normalize().map_err(ctx)?;
        Ok((
            curve.evaluate(t)?,
            if self.forward { tangent } else { tangent.neg() },
        ))
    }
}

/// The edges blended as one: a tangent chain (see the module docs), in the
/// order it runs, around to the first again if `closed`.
#[derive(Clone, Debug)]
pub struct TangentChain {
    pub links: Vec<Link>,
    pub closed: bool,
}

impl TangentChain {
    /// The chain's vertices in order: where each link starts, and — for an
    /// open chain — where the last one ends.
    pub fn vertices(&self, model: &Model) -> GeopResult<Vec<VertexId>> {
        let mut vertices = Vec::new();
        for link in &self.links {
            vertices.push(link.vertices(model)?[0]);
        }
        if !self.closed {
            let last = self.last()?;
            vertices.push(last.vertices(model)?[1]);
        }
        Ok(vertices)
    }

    /// Its first link.
    pub fn first(&self) -> GeopResult<&Link> {
        (self.links.first()).ok_or_else(|| GeopError::new("a chain has no links"))
    }

    /// Its last link.
    pub fn last(&self) -> GeopResult<&Link> {
        (self.links.last()).ok_or_else(|| GeopError::new("a chain has no links"))
    }

    pub fn edges(&self) -> Vec<EdgeId> {
        self.links.iter().map(|l| l.edge).collect()
    }

    /// An open chain's start and end: at each its vertex, the faces either
    /// side of the link there, and the unit direction out of the chain.
    #[allow(clippy::type_complexity)]
    pub fn ends(&self, model: &Model) -> GeopResult<[(VertexId, [FaceId; 2], Vector3); 2]> {
        let vertices = self.vertices(model)?;
        let first = self.first()?;
        let last = self.last()?;
        let (_, t0) = first.at(model, Scalar::ZERO)?;
        let (_, t1) = last.at(model, Scalar::ONE)?;
        Ok([
            (vertices[0], first.faces, t0.neg()),
            (
                *vertices
                    .last()
                    .ok_or_else(|| GeopError::new("a chain has no vertices"))?,
                last.faces,
                t1,
            ),
        ])
    }
}

/// Whether `faces` could meet tangentially at `vertex`, their normals
/// there parallel: where the edge between them vanishes, the corner
/// between the faces opening out flat — two equal pipes joined, where
/// they touch. Its chain ends there, and a blend of it narrows to nothing.
pub fn vanishes_at(model: &Model, vertex: VertexId, faces: [FaceId; 2]) -> GeopResult<bool> {
    let at = model.get_vertex(vertex)?.point;
    let normal = |face| {
        let surface = &model.get_face(face)?.surface;
        surface.normal_at(&at, MAX_NODES, min_subdivision_size())
    };
    let (a, b) = (normal(faces[0])?, normal(faces[1])?);
    Ok(a.could_be_parallel(&b) && !a.prod_dot(&b).definitely_less(Scalar::ZERO))
}

/// Whether the unit vectors `a` and `b` could point the same way.
fn same_direction(a: &Vector3, b: &Vector3) -> bool {
    a.could_be_parallel(b) && a.prod_dot(b).definitely_greater(Scalar::ZERO)
}

/// The link the chain goes on along from `link`, past the vertex it leaves
/// at: the one other edge there it runs on into tangentially, between faces
/// tangent to its own — or none. An error if more than one could be.
///
/// Tangency is asked of the faces, whose normals at the vertex the
/// surfaces state exactly: where both faces either side go on tangentially,
/// so does the edge between them, which runs along both tangent planes.
/// The edges' own tangents only tell which way the next one runs on — an
/// edge traced as an intersection only encloses its points, not its
/// direction.
///
/// That holds only where the faces cross. Where the link's own faces could
/// be tangent to each other at the vertex — two equal pipes joined, where
/// they touch — their normals state no direction at all, every edge there
/// runs along both tangent planes, and the chain ends: an edge between
/// faces turning tangent vanishes there (see [`vanishes_at`]), and one
/// between faces folding back onto each other is no chain's to go on.
fn next_link(model: &Model, link: &Link) -> GeopResult<Option<Link>> {
    let vertex = link.vertices(model)?[1];
    let at = model.get_vertex(vertex)?.point;
    let (_, arriving) = link.at(model, Scalar::ONE)?;
    let normals_at = |faces: [FaceId; 2]| -> GeopResult<[Vector3; 2]> {
        let normal = |face| {
            let surface = &model.get_face(face)?.surface;
            surface.normal_at(&at, MAX_NODES, min_subdivision_size())
        };
        Ok([normal(faces[0])?, normal(faces[1])?])
    };
    let normals = normals_at(link.faces)?;
    if normals[0].could_be_parallel(&normals[1]) {
        return Ok(None);
    }
    let mut found = Vec::new();
    for (&id, e) in &model.edges {
        if id == link.edge {
            continue;
        }
        for forward in [true, false] {
            let leaves = if forward {
                e.start_vertex
            } else {
                e.end_vertex
            };
            if leaves != vertex {
                continue;
            }
            let candidate = Link::new(model, id, forward)?;
            let (_, leaving) = candidate.at(model, Scalar::ZERO)?;
            if !arriving.prod_dot(&leaving).definitely_greater(Scalar::ZERO) {
                continue;
            }
            let theirs = normals_at(candidate.faces)?;
            if (0..2).all(|k| same_direction(&normals[k], &theirs[k])) {
                found.push(candidate);
            }
        }
    }
    match found.len() {
        0 => Ok(None),
        1 => Ok(found.pop()),
        n => Err(GeopError::new(format!(
            "at vertex {vertex}, the edge runs on tangentially into {n} edges: no one chain to blend"
        ))),
    }
}

/// The tangent chain of `edge` (see the module docs), running the way the
/// edge does.
pub fn tangent_chain(model: &Model, edge: EdgeId) -> GeopResult<TangentChain> {
    let walk = |first: Link| -> GeopResult<(Vec<Link>, bool)> {
        let chain = walk_by(first, |l| l.edge, |last| next_link(model, last))?;
        match chain.end {
            Walked::Closed => Ok((chain.nodes, true)),
            Walked::Open => Ok((chain.nodes, false)),
            Walked::Revisited(next) => Err(GeopError::new(format!(
                "the tangent chain of the edge runs round a loop not through it, at edge {}",
                next.edge
            ))),
        }
    };
    let (ahead, closed) = walk(Link::new(model, edge, true)?)?;
    if closed {
        return Ok(TangentChain {
            links: ahead,
            closed,
        });
    }
    let (behind, _) = walk(Link::new(model, edge, false)?)?;
    let mut links: Vec<Link> = behind[1..].iter().rev().map(Link::reversed).collect();
    links.extend(ahead);
    Ok(TangentChain {
        links,
        closed: false,
    })
}
