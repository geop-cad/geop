//! Forwarding wrappers for every `Model` operation that creates or deletes
//! entities: each registers what it creates under the caller's name and
//! forgets what it deletes.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{
    Body, Edge, EdgeId, FaceId, ShellId, SolidId, Vertex, VertexId,
    build::{BodySpec, BuiltBody, SpecSources},
};

use super::NamedModel;
use crate::ids::RefId;

impl NamedModel {
    /// Forwards to [`geop_core_topology::Model::insert_vertex`]: a vertex on
    /// nothing yet, about to be split into an edge (see
    /// [`NamedModel::split_edge_at_vertex`]).
    pub fn insert_vertex(
        &mut self,
        point: Vector3,
        name: impl Into<String>,
    ) -> GeopResult<VertexId> {
        let vertex = self.topology.insert_vertex(Vertex { point });
        self.names.insert(vertex, name)?;
        Ok(vertex)
    }

    /// Forwards to [`geop_core_topology::Model::insert_edge`]: an edge on no
    /// face yet, about to be spliced into its faces (see
    /// [`NamedModel::splice_edge_into_face`]).
    pub fn insert_edge(&mut self, edge: Edge, name: impl Into<String>) -> GeopResult<EdgeId> {
        let edge = self.topology.insert_edge(edge);
        self.names.insert(edge, name)?;
        Ok(edge)
    }

    /// Forwards to [`geop_core_topology::Model::merge_vertex`], forgetting
    /// the name of the vertex it deletes. The survivor keeps its own name.
    pub fn merge_vertex(
        &mut self,
        vertex_into_id: VertexId,
        vertex_deleted_id: VertexId,
    ) -> GeopResult<()> {
        self.topology
            .merge_vertex(vertex_into_id, vertex_deleted_id)?;
        self.names.remove(vertex_deleted_id);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::merge_edge`], forgetting the
    /// name of the edge it deletes. The survivor keeps its own name.
    pub fn merge_edge(
        &mut self,
        edge_into_id: EdgeId,
        edge_deleted_id: EdgeId,
        reversed: bool,
    ) -> GeopResult<()> {
        self.topology
            .merge_edge(edge_into_id, edge_deleted_id, reversed)?;
        self.names.remove(edge_deleted_id);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::reverse_face`]. Creates and
    /// deletes nothing.
    pub fn reverse_face(&mut self, face_id: FaceId) -> GeopResult<()> {
        self.topology.reverse_face(face_id)
    }

    /// Forwards to [`geop_core_topology::Model::splice_edge_into_face`].
    ///
    /// Whether that creates a face depends on how the edge's ends sit on the
    /// face's boundary, which a caller imprinting a traced curve cannot know
    /// beforehand — so `new_face_name` is the name for the face *if* one is
    /// created, and unused otherwise.
    pub fn splice_edge_into_face(
        &mut self,
        edge_id: EdgeId,
        face_id: FaceId,
        max_nodes: usize,
        min_subdivision_size: Scalar,
        new_face_name: impl Into<String>,
    ) -> GeopResult<Option<FaceId>> {
        let new_face = self.topology.splice_edge_into_face(
            edge_id,
            face_id,
            max_nodes,
            min_subdivision_size,
        )?;
        if let Some(face) = new_face {
            self.names.insert(face, new_face_name)?;
        }
        Ok(new_face)
    }

    /// Forwards to [`geop_core_topology::Model::split_edge_at_vertex`],
    /// naming the new edge it creates: the (vertex -> end) segment, while
    /// `edge_id` keeps the (start -> vertex) one and its name.
    pub fn split_edge_at_vertex(
        &mut self,
        edge_id: EdgeId,
        edge_t: Scalar,
        vertex_id: VertexId,
        max_nodes: usize,
        min_subdivision_size: Scalar,
        new_edge_name: impl Into<String>,
    ) -> GeopResult<EdgeId> {
        let new_edge = self.topology.split_edge_at_vertex(
            edge_id,
            edge_t,
            vertex_id,
            max_nodes,
            min_subdivision_size,
        )?;
        self.names.insert(new_edge, new_edge_name)?;
        Ok(new_edge)
    }

    /// Forwards to [`geop_core_topology::Model::build_body`], naming
    /// everything it builds after `names`, index for index. Fails, leaving
    /// the part unchanged, if a name is missing, repeated or taken.
    pub fn build_body(&mut self, spec: BodySpec, names: BodyNames) -> GeopResult<BuiltBody> {
        let counts = [
            (names.vertices.len(), spec.vertices.len(), "vertices"),
            (names.edges.len(), spec.edges.len(), "edges"),
            (names.faces.len(), spec.faces.len(), "faces"),
        ];
        if let Some((n, m, what)) = counts.iter().find(|(n, m, _)| n != m) {
            return Err(GeopError::new(format!(
                "NamedModel::build_body: {n} names for {m} {what}"
            )));
        }
        if names.solid.is_some() != spec.solid {
            return Err(GeopError::new(format!(
                "NamedModel::build_body: a solid name {:?} for a body that {} a solid",
                names.solid,
                if spec.solid { "is" } else { "is not" }
            )));
        }
        let all = names
            .vertices
            .iter()
            .chain(&names.edges)
            .chain(&names.faces)
            .chain(&names.solid);
        let mut seen = HashSet::new();
        for name in all {
            if !seen.insert(name) || self.names.is_taken(name) {
                return Err(GeopError::new(format!(
                    "NamedModel::build_body: the name {name:?} is taken"
                )));
            }
        }
        let built = self.topology.build_body(spec)?;
        for (&id, name) in built.vertices.iter().zip(names.vertices) {
            self.names.insert(id, name)?;
        }
        for (&id, name) in built.edges.iter().zip(names.edges) {
            self.names.insert(id, name)?;
        }
        for (&id, name) in built.faces.iter().zip(names.faces) {
            self.names.insert(id, name)?;
        }
        if let (Some(id), Some(name)) = (built.solid, names.solid) {
            self.names.insert(id, name)?;
        }
        Ok(built)
    }

    /// Copies `faces`, with every edge and vertex they use, into a body of
    /// their own that shares nothing with the originals (see
    /// [`geop_core_topology::Model::body_spec`]): a solid named `solid`, if
    /// given, or else a sheet. The copy of each entity named `X` is named
    /// `rename(X)`.
    pub fn copy_faces(
        &mut self,
        faces: &[FaceId],
        solid: Option<String>,
        rename: impl Fn(&str) -> String,
    ) -> GeopResult<BuiltBody> {
        let (spec, sources) = self.topology().body_spec(faces, solid.is_some())?;
        let names = BodyNames {
            solid,
            ..self.source_names(&sources)?.renamed(rename)
        };
        self.build_body(spec, names)
    }

    /// Copies the whole of `body` — a solid, its shells kept apart, or a
    /// sheet — into a body of its own that shares nothing with it. The copy
    /// of each entity named `X`, the solid included, is named `rename(X)`.
    pub fn copy_body(
        &mut self,
        body: Body,
        rename: impl Fn(&str) -> String,
    ) -> GeopResult<BuiltBody> {
        let (spec, names) = self.body_record(body)?;
        self.build_body(spec, names.renamed(rename))
    }

    /// The whole of `body` — a solid, its shells kept apart, or a sheet —
    /// as a body to build again (see [`NamedModel::build_body`]), with the names
    /// of its entities: what [`NamedModel::copy_body`] builds, kept to build
    /// later.
    pub fn body_record(&self, body: Body) -> GeopResult<(BodySpec, BodyNames)> {
        // In the order they were made: ids are given in creation order.
        let mut faces = self.topology().body_faces(body)?;
        faces.sort();
        let solid = match body {
            Body::Solid(solid) => Some(self.name(solid)?.to_string()),
            Body::Sheet(_) => None,
        };
        let (mut spec, sources) = self.topology().body_spec(&faces, solid.is_some())?;
        // `body_spec` puts every face into one shell; a solid with a void
        // has more, which the record keeps.
        let mut shells: Vec<(ShellId, Vec<usize>)> = Vec::new();
        for (index, &face) in sources.faces.iter().enumerate() {
            let shell = self.topology().get_face(face)?.shell;
            match shells.iter_mut().find(|(s, _)| *s == shell) {
                Some((_, members)) => members.push(index),
                None => shells.push((shell, vec![index])),
            }
        }
        spec.shells = shells.into_iter().map(|(_, members)| members).collect();
        let names = BodyNames {
            solid,
            ..self.source_names(&sources)?
        };
        Ok((spec, names))
    }

    /// The names of the entities `sources` says a body to build was made
    /// from (see [`geop_core_topology::Model::body_spec`]), index for index,
    /// with no solid.
    pub fn source_names(&self, sources: &SpecSources) -> GeopResult<BodyNames> {
        fn names<Id: Into<super::RefId> + Copy>(
            model: &NamedModel,
            ids: &[Id],
        ) -> GeopResult<Vec<String>> {
            (ids.iter())
                .map(|&id| Ok(model.name(id)?.to_string()))
                .collect()
        }
        Ok(BodyNames {
            vertices: names(self, &sources.vertices)?,
            edges: names(self, &sources.edges)?,
            faces: names(self, &sources.faces)?,
            solid: None,
        })
    }

    /// Takes the topology `body` does not need out of it (see
    /// [`BodySpec::merged`]): faces that lie on one surface made one, and
    /// vertices between two edges of the same two faces dropped, the edges
    /// made one. Returns the body as it is now, rebuilt if anything was
    /// taken out, the very same otherwise.
    ///
    /// What is made one keeps the name of the oldest of what it is made
    /// of; the names of the others stand for it from then on (see
    /// [`crate::names::NameRegistry::alias`]), so a step that refers to any
    /// of them still
    /// finds it. An edge between two faces made one is gone, and so is its
    /// name; so is a vertex dropped.
    pub fn merge_faces_on_one_surface(&mut self, body: Body) -> GeopResult<Body> {
        let (spec, names) = self.body_record(body)?;
        let (merged, map) = spec.merged()?;
        if merged.faces.len() == spec.faces.len() && merged.edges.len() == spec.edges.len() {
            return Ok(body);
        }
        // The very entities `spec` was made from, index for index.
        let mut faces = self.topology().body_faces(body)?;
        faces.sort();
        let (_, sources) = self.topology().body_spec(&faces, false)?;
        let face_into: Vec<Option<usize>> = map.faces.iter().map(|&f| Some(f)).collect();
        let faces = self.merged_names(&sources.faces, &face_into, merged.faces.len())?;
        let edges = self.merged_names(&sources.edges, &map.edges, merged.edges.len())?;
        let vertices =
            self.merged_names(&sources.vertices, &map.vertices, merged.vertices.len())?;
        let new_names = BodyNames {
            vertices: vertices.names,
            edges: edges.names.clone(),
            faces: faces.names.clone(),
            solid: names.solid.clone(),
        };
        self.delete_bodies(&[body])?;
        let built = self.build_body(merged, new_names)?;
        for merged in [faces, edges] {
            for (alias, k) in &merged.aliases {
                self.names.alias(alias.clone(), merged.names[*k].clone())?;
            }
        }
        built.body()
    }

    /// The names of what [`NamedModel::merge_faces_on_one_surface`] builds
    /// of the entities `ids`, each made part of the entity `into` says of
    /// `count`: each named after the oldest — the least id — of what it is
    /// made of, the names of the others its aliases.
    fn merged_names<Id: Into<RefId> + Copy + Ord>(
        &self,
        ids: &[Id],
        into: &[Option<usize>],
        count: usize,
    ) -> GeopResult<MergedNames> {
        let mut by: Vec<Vec<Id>> = vec![Vec::new(); count];
        for (&id, &into) in ids.iter().zip(into) {
            if let Some(into) = into {
                by[into].push(id);
            }
        }
        let mut out = MergedNames::default();
        for (k, mut ids) in by.into_iter().enumerate() {
            ids.sort();
            let Some((&oldest, rest)) = ids.split_first() else {
                return Err(GeopError::new(
                    "NamedModel::merge_faces_on_one_surface: an entity made of nothing",
                ));
            };
            out.names.push(self.name(oldest)?.to_string());
            for &id in rest {
                out.aliases.push((self.name(id)?.to_string(), k));
            }
        }
        Ok(out)
    }

    /// Forwards to [`geop_core_topology::Model::transform_body`]. Creates
    /// and deletes nothing, so every entity keeps its name.
    pub fn transform_body(&mut self, body: Body, motion: &Motion) -> GeopResult<()> {
        self.topology.transform_body(body, motion)
    }

    /// Forwards to [`geop_core_topology::Model::assemble_solid`], naming the
    /// solid it creates (if any) and forgetting the names of everything it
    /// deletes.
    pub fn assemble_solid(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
        solid_name: impl Into<String>,
    ) -> GeopResult<Option<SolidId>> {
        let solid = self.topology.assemble_solid(consumed, keep)?;
        self.forget_dead_names();
        if let Some(solid) = solid {
            self.names.insert(solid, solid_name)?;
        }
        Ok(solid)
    }

    /// Forwards to [`geop_core_topology::Model::assemble_sheet`], forgetting
    /// the names of everything it deletes.
    pub fn assemble_sheet(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<ShellId>> {
        let sheet = self.topology.assemble_sheet(consumed, keep)?;
        self.forget_dead_names();
        Ok(sheet)
    }

    /// Deletes `bodies` whole, with every entity in them, forgetting their
    /// names.
    pub fn delete_bodies(&mut self, bodies: &[Body]) -> GeopResult<()> {
        self.assemble_sheet(bodies, &[])?;
        Ok(())
    }

    /// Deletes `bodies` and builds the solid `spec` in their place, named
    /// after `names` (see [`NamedModel::build_body`]), which may take up
    /// the names of what it replaces. Fails if `spec` builds no solid.
    pub fn replace_with_solid(
        &mut self,
        bodies: &[Body],
        spec: BodySpec,
        names: BodyNames,
    ) -> GeopResult<SolidId> {
        self.delete_bodies(bodies)?;
        self.build_body(spec, names)?
            .solid
            .ok_or_else(|| GeopError::new("building the result made no solid"))
    }

    /// Forwards to [`geop_core_topology::Model::merge_solids`], forgetting
    /// the name of the solid it deletes.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        self.topology.merge_solids(into, from)?;
        self.names.remove(from);
        Ok(())
    }
}

/// The names of entities made of others (see
/// [`NamedModel::merge_faces_on_one_surface`]): each one's own, and the
/// names of the others that stand for it, with its index.
#[derive(Default)]
struct MergedNames {
    names: Vec<String>,
    aliases: Vec<(String, usize)>,
}

/// The names [`NamedModel::build_body`] gives what it builds: one per vertex, edge
/// and face of the [`BodySpec`], index for index, and the solid's, if it
/// builds one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyNames {
    pub vertices: Vec<String>,
    pub edges: Vec<String>,
    pub faces: Vec<String>,
    pub solid: Option<String>,
}

impl BodyNames {
    /// The same names, each `X` as `rename(X)`.
    pub fn renamed(&self, rename: impl Fn(&str) -> String) -> BodyNames {
        let all = |names: &[String]| names.iter().map(|n| rename(n)).collect();
        BodyNames {
            vertices: all(&self.vertices),
            edges: all(&self.edges),
            faces: all(&self.faces),
            solid: self.solid.as_deref().map(&rename),
        }
    }
}
