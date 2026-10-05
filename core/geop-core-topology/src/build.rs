//! Building a whole body at once from a description of it ([`BodySpec`]),
//! and describing part of a model the same way ([`Model::body_spec`]).
//!
//! The euler operators grow a model one entity at a time, each step keeping
//! the model a valid solid. That suits constructions with one fixed shape —
//! but a sweep comes in many shapes (one or several spans, closed around an
//! axis or not, with poles, holes, caps or none), each of which would need
//! its own sequence of euler steps. A sweep is far simpler said whole: these
//! vertices, these edges between them, these faces, each bounded by these
//! coedges. [`Model::build_body`] takes exactly that, checks it is
//! topologically consistent, and only then inserts it. A description
//! without faces is a [`Wire`]: edges and vertices standing on their own.

use std::collections::HashMap;

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Curve2, Curve3, Edge, EdgeId, Face, FaceId, Model, Sense,
    Shell, ShellId, Solid, SolidId, Vertex, VertexId, Wire, WireId, boundary::BoundaryType,
};

/// A body to build, its entities referring to each other by their index in
/// the lists here: a solid, a sheet, or — with no faces — a wire.
#[derive(Clone, Debug)]
pub struct BodySpec<S: Scalar> {
    pub vertices: Vec<Vector3<S>>,
    pub edges: Vec<EdgeSpec<S>>,
    pub faces: Vec<FaceSpec<S>>,
    /// Each shell's faces. Every face is in exactly one.
    pub shells: Vec<Vec<usize>>,
    /// Whether the shells bound a solid — then they must be closed, every
    /// edge used by exactly two faces — or are each a sheet of their own.
    pub solid: bool,
}

/// An edge: its curve, running from vertex `start` to vertex `end`.
#[derive(Clone, Debug)]
pub struct EdgeSpec<S: Scalar> {
    pub curve: Curve3<S>,
    pub start: usize,
    pub end: usize,
}

/// A face: its surface, bounded by its `outer` loop and with its `holes`.
#[derive(Clone, Debug)]
pub struct FaceSpec<S: Scalar> {
    pub surface: NurbSurface3D<S>,
    pub outer: Vec<CoedgeSpec<S>>,
    pub holes: Vec<Vec<CoedgeSpec<S>>>,
}

/// One coedge of a loop, in the order the loop runs, with its pcurve on the
/// loop's face.
#[derive(Clone, Debug)]
pub struct CoedgeSpec<S: Scalar> {
    pub on: CoedgeOn,
    pub pcurve: Curve2<S>,
}

/// What a [`CoedgeSpec`] runs along: an edge, in a sense, or — a degenerate
/// coedge, see [`CoedgeGeometry::Vertex`] — a single vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoedgeOn {
    Edge(usize, Sense),
    Vertex(usize),
}

/// The ids [`Model::build_body`] gave what a [`BodySpec`] describes, index
/// for index.
#[derive(Clone, Debug)]
pub struct BuiltBody {
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
    pub faces: Vec<FaceId>,
    pub shells: Vec<ShellId>,
    pub solid: Option<SolidId>,
    /// The wire, for a description without faces.
    pub wire: Option<WireId>,
}

/// Where each entity of a [`BodySpec`] made by [`Model::body_spec`] came
/// from, index for index.
#[derive(Clone, Debug)]
pub struct SpecSources {
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
    pub faces: Vec<FaceId>,
}

impl<S: Scalar> FaceSpec<S> {
    /// The same face with its material side turned around, exactly as
    /// [`Model::reverse_face`] turns a face of a model around: the surface
    /// mirrored in `u`, each pcurve mirrored with it, and every loop run the
    /// other way.
    pub fn reversed(&self) -> Self {
        let (u_lo, u_hi) = self.surface.domain_u();
        let span = u_lo.add(u_hi);
        let reverse = |lp: &Vec<CoedgeSpec<S>>| {
            lp.iter()
                .rev()
                .map(|c| {
                    let mut pcurve = c.pcurve.clone();
                    crate::edit::reverse_face::mirror_u(&mut pcurve, span);
                    CoedgeSpec {
                        on: match c.on {
                            CoedgeOn::Edge(e, sense) => CoedgeOn::Edge(e, sense.opposite()),
                            on @ CoedgeOn::Vertex(_) => on,
                        },
                        pcurve: pcurve.reverse(),
                    }
                })
                .collect()
        };
        Self {
            surface: self.surface.reverse_u(),
            outer: reverse(&self.outer),
            holes: self.holes.iter().map(reverse).collect(),
        }
    }
}

impl<S: Scalar> BodySpec<S> {
    /// The same body moved by the rigid motion `pose`: its vertices, edge
    /// curves and surfaces moved, its pcurves — on surfaces moved with the
    /// same parametrization — as they are.
    pub fn placed(&self, pose: &Pose<S>) -> Self {
        Self {
            vertices: self.vertices.iter().map(|p| pose.apply(p)).collect(),
            edges: self
                .edges
                .iter()
                .map(|e| EdgeSpec {
                    curve: e.curve.transform(&pose.motion()),
                    ..e.clone()
                })
                .collect(),
            faces: self
                .faces
                .iter()
                .map(|f| FaceSpec {
                    surface: f.surface.transform(&pose.motion()),
                    ..f.clone()
                })
                .collect(),
            shells: self.shells.clone(),
            solid: self.solid,
        }
    }

    /// The vertex a coedge starts at, and the one it ends at.
    fn ends(&self, coedge: &CoedgeSpec<S>) -> GeopResult<(usize, usize)> {
        match coedge.on {
            CoedgeOn::Edge(e, sense) => {
                let edge = self
                    .edges
                    .get(e)
                    .ok_or_else(|| GeopError::new(format!("BodySpec: no edge {e}")))?;
                Ok(match sense {
                    Sense::Forward => (edge.start, edge.end),
                    Sense::Reversed => (edge.end, edge.start),
                })
            }
            CoedgeOn::Vertex(v) => Ok((v, v)),
        }
    }

    /// Whether it describes a wire: vertices, but no faces.
    pub fn is_wire(&self) -> bool {
        self.faces.is_empty() && !self.vertices.is_empty()
    }

    /// Checks that the description is a consistent body: every index
    /// names something, every loop closes up — each coedge ends where the
    /// next one starts — every face is in exactly one shell, every edge and
    /// vertex is used, and an edge is used at most twice, then in opposite
    /// senses — and, for a solid, exactly twice. A wire's edges are used by
    /// no face, and its vertices need not be used at all: a point on its
    /// own is a vertex of the wire.
    pub fn check(&self) -> GeopResult<()> {
        let fail = |what: String| Err(GeopError::new(format!("BodySpec: {what}")));
        for (e, edge) in self.edges.iter().enumerate() {
            if edge.start >= self.vertices.len() || edge.end >= self.vertices.len() {
                return fail(format!("edge {e} runs between vertices that do not exist"));
            }
        }
        if self.is_wire() {
            if self.solid || !self.shells.is_empty() {
                return fail("a solid or a shell without faces".to_string());
            }
            return Ok(());
        }
        let mut edge_uses: Vec<Vec<Sense>> = vec![Vec::new(); self.edges.len()];
        let mut vertex_used = vec![false; self.vertices.len()];
        for edge in &self.edges {
            vertex_used[edge.start] = true;
            vertex_used[edge.end] = true;
        }
        for (f, face) in self.faces.iter().enumerate() {
            for (l, lp) in std::iter::once(&face.outer).chain(&face.holes).enumerate() {
                if lp.is_empty() {
                    return fail(format!("loop {l} of face {f} is empty"));
                }
                for (k, coedge) in lp.iter().enumerate() {
                    let (_, end) = self.ends(coedge)?;
                    let (start, _) = self.ends(&lp[(k + 1) % lp.len()])?;
                    if end != start {
                        return fail(format!(
                            "loop {l} of face {f}: coedge {k} ends at vertex {end}, but the next one starts at vertex {start}"
                        ));
                    }
                    match coedge.on {
                        CoedgeOn::Edge(e, sense) => edge_uses[e].push(sense),
                        CoedgeOn::Vertex(v) => match vertex_used.get_mut(v) {
                            Some(used) => *used = true,
                            None => {
                                return fail(format!(
                                    "face {f} sits at vertex {v}, which does not exist"
                                ));
                            }
                        },
                    }
                }
            }
        }
        for (e, uses) in edge_uses.iter().enumerate() {
            match uses[..] {
                [] => return fail(format!("edge {e} is used by no face")),
                [_] if self.solid => {
                    return fail(format!(
                        "edge {e} is used by one face only, so the solid is open"
                    ));
                }
                [_] => {}
                [a, b] if a != b => {}
                [_, _] => return fail(format!("edge {e} is used twice in the same sense")),
                _ => return fail(format!("edge {e} is used {} times", uses.len())),
            }
        }
        if let Some(v) = vertex_used.iter().position(|used| !used) {
            return fail(format!("vertex {v} is used by nothing"));
        }
        let mut shell_of = vec![None; self.faces.len()];
        for (s, faces) in self.shells.iter().enumerate() {
            for &f in faces {
                match shell_of.get_mut(f) {
                    None => return fail(format!("shell {s} lists face {f}, which does not exist")),
                    Some(Some(other)) => {
                        return fail(format!("face {f} is in both shell {other} and shell {s}"));
                    }
                    Some(slot) => *slot = Some(s),
                }
            }
        }
        if let Some(f) = shell_of.iter().position(Option::is_none) {
            return fail(format!("face {f} is in no shell"));
        }
        Ok(())
    }
}

impl<S: Scalar> Model<S> {
    /// Adds the body `spec` describes, once [`BodySpec::check`] has found
    /// it consistent — nothing is added otherwise. One without faces is
    /// added as a [`Wire`].
    pub fn build_body(&mut self, spec: BodySpec<S>) -> GeopResult<BuiltBody> {
        spec.check()?;
        let BodySpec {
            vertices,
            edges,
            faces,
            shells,
            solid,
        } = spec;
        let vertices: Vec<VertexId> = vertices
            .into_iter()
            .map(|point| self.insert_vertex(Vertex { point }))
            .collect();
        let edges: Vec<EdgeId> = edges
            .into_iter()
            .map(|e| {
                self.insert_edge(Edge {
                    curve: e.curve,
                    start_vertex: vertices[e.start],
                    end_vertex: vertices[e.end],
                })
            })
            .collect();
        let shell_ids: Vec<ShellId> = shells
            .iter()
            .map(|faces| {
                self.insert_shell(Shell {
                    faces: Vec::with_capacity(faces.len()),
                    solid: None,
                })
            })
            .collect();
        let mut shell_of = vec![ShellId(0); faces.len()];
        for (s, members) in shells.iter().enumerate() {
            for &f in members {
                shell_of[f] = shell_ids[s];
            }
        }

        let mut face_ids = Vec::with_capacity(faces.len());
        for (f, face) in faces.into_iter().enumerate() {
            let face_id = self.insert_face(Face {
                surface: face.surface,
                // Replaced by the real outer loop just below.
                outer: BoundaryType::Vertex(vertices[0]),
                holes: Vec::new(),
                shell: shell_of[f],
            });
            let insert_loop = |model: &mut Self, lp: Vec<CoedgeSpec<S>>| {
                let ids: Vec<CoedgeId> = lp
                    .into_iter()
                    .map(|c| {
                        let (geometry, sense) = match c.on {
                            CoedgeOn::Edge(e, sense) => (CoedgeGeometry::Edge(edges[e]), sense),
                            CoedgeOn::Vertex(v) => {
                                (CoedgeGeometry::Vertex(vertices[v]), Sense::Forward)
                            }
                        };
                        model.insert_coedge(Coedge {
                            geometry,
                            sense,
                            pcurve: c.pcurve,
                            // Linked just below, once every id exists.
                            next: CoedgeId(0),
                            prev: CoedgeId(0),
                            face: face_id,
                        })
                    })
                    .collect();
                let n = ids.len();
                for (k, &id) in ids.iter().enumerate() {
                    let coedge = model.coedges.get_mut(&id).expect("just inserted");
                    coedge.next = ids[(k + 1) % n];
                    coedge.prev = ids[(k + n - 1) % n];
                }
                BoundaryType::Loop(ids[0])
            };
            let outer = insert_loop(self, face.outer);
            let holes = face
                .holes
                .into_iter()
                .map(|h| insert_loop(self, h))
                .collect();
            let built = self.faces.get_mut(&face_id).expect("just inserted");
            built.outer = outer;
            built.holes = holes;
            self.get_shell_mut(shell_of[f])?.faces.push(face_id);
            face_ids.push(face_id);
        }

        let solid = solid.then(|| {
            self.insert_solid(Solid {
                shells: shell_ids.clone(),
            })
        });
        for &shell in &shell_ids {
            self.get_shell_mut(shell)?.solid = solid;
        }
        let wire = (face_ids.is_empty() && !vertices.is_empty()).then(|| {
            self.insert_wire(Wire {
                vertices: vertices.clone(),
                edges: edges.clone(),
            })
        });
        Ok(BuiltBody {
            vertices,
            edges,
            faces: face_ids,
            shells: shell_ids,
            solid,
            wire,
        })
    }

    /// `faces` described as a body of one shell — a solid's if `solid` —
    /// with every vertex and edge they use, copied: building it makes a
    /// copy of them, sharing nothing with the originals. Also says where in
    /// the model each entity of the description came from.
    pub fn body_spec(
        &self,
        faces: &[FaceId],
        solid: bool,
    ) -> GeopResult<(BodySpec<S>, SpecSources)> {
        let mut vertex_index: HashMap<VertexId, usize> = HashMap::new();
        let mut edge_index: HashMap<EdgeId, usize> = HashMap::new();
        let mut sources = SpecSources {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: faces.to_vec(),
        };
        let mut spec = BodySpec {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::with_capacity(faces.len()),
            shells: vec![(0..faces.len()).collect()],
            solid,
        };
        let mut vertex = |spec: &mut BodySpec<S>, sources: &mut SpecSources, id: VertexId| {
            *vertex_index.entry(id).or_insert_with(|| {
                spec.vertices.push(self.vertices[&id].point);
                sources.vertices.push(id);
                spec.vertices.len() - 1
            })
        };
        for &face_id in faces {
            let face = self.get_face(face_id)?;
            let mut loops = Vec::new();
            for boundary in face.boundaries() {
                let BoundaryType::Loop(anchor) = boundary else {
                    return Err(GeopError::new(format!(
                        "Model::body_spec: face {face_id} is bounded by a bare vertex"
                    )));
                };
                let mut lp = Vec::new();
                for coedge_id in self.iterate_loop_coedges(anchor) {
                    let coedge = self.get_coedge(coedge_id)?;
                    let on = match coedge.geometry {
                        CoedgeGeometry::Edge(e) => {
                            let index = match edge_index.get(&e) {
                                Some(&index) => index,
                                None => {
                                    let edge = self.get_edge(e)?;
                                    let start = vertex(&mut spec, &mut sources, edge.start_vertex);
                                    let end = vertex(&mut spec, &mut sources, edge.end_vertex);
                                    spec.edges.push(EdgeSpec {
                                        curve: edge.curve.clone(),
                                        start,
                                        end,
                                    });
                                    sources.edges.push(e);
                                    edge_index.insert(e, spec.edges.len() - 1);
                                    spec.edges.len() - 1
                                }
                            };
                            CoedgeOn::Edge(index, coedge.sense)
                        }
                        CoedgeGeometry::Vertex(v) => {
                            CoedgeOn::Vertex(vertex(&mut spec, &mut sources, v))
                        }
                    };
                    lp.push(CoedgeSpec {
                        on,
                        pcurve: coedge.pcurve.clone(),
                    });
                }
                loops.push(lp);
            }
            let outer = loops.remove(0);
            spec.faces.push(FaceSpec {
                surface: face.surface.clone(),
                outer,
                holes: loops,
            });
        }
        Ok((spec, sources))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_geometry::nurb_curve::NurbCurve;
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use super::{BodySpec, EdgeSpec};
    use crate::{
        Body, CoedgeGeometry, Model,
        test_fixtures::test_cube_solid,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    fn point<S: Scalar>(p: [f64; 3]) -> Vector3<S> {
        Vector3::from_array(p.map(S::from_f64))
    }

    fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> crate::Curve3<S> {
        let h = |p: [f64; 3]| Vector4::from_array([p[0], p[1], p[2], 1.0].map(S::from_f64));
        NurbCurve::try_new(
            1,
            vec![h(a), h(b)],
            [0.0, 0.0, 1.0, 1.0].map(S::from_f64).to_vec(),
        )
        .unwrap()
    }

    /// Two lines from one corner, and a point on its own, beside the unit
    /// cube.
    fn wire_spec<S: Scalar>() -> BodySpec<S> {
        let corners = [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0]];
        BodySpec {
            vertices: corners
                .iter()
                .map(|&p| point(p))
                .chain([point([2.0, 0.0, 1.0])])
                .collect(),
            edges: vec![
                EdgeSpec {
                    curve: line(corners[0], corners[1]),
                    start: 0,
                    end: 1,
                },
                EdgeSpec {
                    curve: line(corners[0], corners[2]),
                    start: 0,
                    end: 2,
                },
            ],
            faces: Vec::new(),
            shells: Vec::new(),
            solid: false,
        }
    }

    /// A description without faces builds a wire: its edges used by no
    /// face, a point on its own among its vertices — valid beside a solid,
    /// whose manifold check it leaves alone.
    fn check_a_spec_without_faces_builds_a_wire<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let built = model.build_body(wire_spec()).unwrap();
        let wire = built.wire.expect("a wire");
        assert_eq!(built.solid, None);
        assert_eq!(model.get_wire(wire).unwrap().vertices.len(), 4);
        assert_eq!(model.get_wire(wire).unwrap().edges.len(), 2);
        assert_eq!(model.wire_of_edge(built.edges[1]), Some(wire));
        assert_eq!(model.wire_of_vertex(built.vertices[3]), Some(wire));
        let params = ValidationParameters::default();
        validate(&params, &model).unwrap();
        validate_manifold(&params, &model).unwrap();
    }
    #[test]
    fn a_spec_without_faces_builds_a_wire() {
        for_all_scalars!(check_a_spec_without_faces_builds_a_wire);
    }

    /// Deleting a solid deletes what only its faces reached, and leaves a
    /// wire as it is.
    fn check_a_wire_outlives_a_deleted_solid<S: Scalar>() {
        let mut model = Model::<S>::new();
        let solid = test_cube_solid(&mut model);
        model.build_body(wire_spec()).unwrap();
        model.assemble_solid(&[Body::Solid(solid)], &[]).unwrap();
        assert_eq!(model.vertices.len(), 4);
        assert_eq!(model.edges.len(), 2);
        validate(&ValidationParameters::default(), &model).unwrap();
    }
    #[test]
    fn a_wire_outlives_a_deleted_solid() {
        for_all_scalars!(check_a_wire_outlives_a_deleted_solid);
    }

    /// A face using an edge of a wire makes it no wire's: invalid.
    fn check_a_face_on_a_wire_edge_is_invalid<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let built = model.build_body(wire_spec()).unwrap();
        let coedge = model.coedges.values_mut().next().unwrap();
        coedge.geometry = CoedgeGeometry::Edge(built.edges[0]);
        let errors = validate(&ValidationParameters::default(), &model).unwrap_err();
        assert!(
            errors
                .iter()
                .any(|e| format!("{e:?}").contains("which bounds no face")),
            "{errors:?}"
        );
    }
    #[test]
    fn a_face_on_a_wire_edge_is_invalid() {
        for_all_scalars!(check_a_face_on_a_wire_edge_is_invalid);
    }

    /// Two edges of a wire that cross without a vertex there are invalid,
    /// as two edges of a solid would be.
    fn check_crossing_wire_edges_are_invalid<S: Scalar>() {
        let mut model = Model::<S>::new();
        let mut spec = wire_spec::<S>();
        spec.vertices.push(point([2.5, -1.0, 0.0]));
        spec.vertices.push(point([2.5, 1.0, 0.0]));
        spec.edges.push(EdgeSpec {
            curve: line([2.5, -1.0, 0.0], [2.5, 1.0, 0.0]),
            start: 4,
            end: 5,
        });
        model.build_body(spec).unwrap();
        assert!(validate(&ValidationParameters::default(), &model).is_err());
    }
    #[test]
    fn crossing_wire_edges_are_invalid() {
        for_all_scalars!(check_crossing_wire_edges_are_invalid);
    }
}
