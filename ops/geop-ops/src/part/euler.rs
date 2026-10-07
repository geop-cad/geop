//! Forwarding wrappers for every `Model` euler operator: each method here
//! calls the identically-named `geop_core_topology::Model` method, then
//! registers whichever new vertex/edge/face/solid it created under the name
//! the caller supplied for it (see the crate docs for how names are chosen), and forgets the name of whichever
//! one it deleted.

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{CoedgeId, EdgeId, FaceId, SolidId, VertexId, boundary::BoundaryType};

use super::Part;

impl<S: Scalar> Part<S> {
    /// Forwards to [`geop_core_topology::Model::mvfs`], naming the new
    /// vertex, face and solid it creates.
    pub fn mvfs(
        &mut self,
        point: Vector3<S>,
        vertex_name: impl Into<String>,
        face_name: impl Into<String>,
        solid_name: impl Into<String>,
    ) -> GeopResult<(VertexId, FaceId, SolidId)> {
        let (vertex, face, solid) = self.store.topology_mut().mvfs(point);
        self.store.insert_name(vertex, vertex_name)?;
        self.store.insert_name(face, face_name)?;
        self.store.insert_name(solid, solid_name)?;
        Ok((vertex, face, solid))
    }

    /// Forwards to [`geop_core_topology::Model::mve`], naming the new vertex
    /// and edge it creates.
    pub fn mve(
        &mut self,
        coedge: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        p: Vector3<S>,
        vertex_name: impl Into<String>,
        edge_name: impl Into<String>,
    ) -> GeopResult<(VertexId, CoedgeId, CoedgeId, EdgeId)> {
        let (vertex, c_out, c_in, edge) =
            self.store
                .topology_mut()
                .mve(coedge, curve, pcurve, pcurve_reversed, p)?;
        self.store.insert_name(vertex, vertex_name)?;
        self.store.insert_name(edge, edge_name)?;
        Ok((vertex, c_out, c_in, edge))
    }

    /// Forwards to [`geop_core_topology::Model::mve_from_vertex`], naming the
    /// new vertex and edge it creates.
    pub fn mve_from_vertex(
        &mut self,
        face_id: FaceId,
        vertex: VertexId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        p: Vector3<S>,
        vertex_name: impl Into<String>,
        edge_name: impl Into<String>,
    ) -> GeopResult<(VertexId, CoedgeId, CoedgeId, EdgeId)> {
        let (new_vertex, c_out, c_in, edge) = self.store.topology_mut().mve_from_vertex(
            face_id,
            vertex,
            curve,
            pcurve,
            pcurve_reversed,
            p,
        )?;
        self.store.insert_name(new_vertex, vertex_name)?;
        self.store.insert_name(edge, edge_name)?;
        Ok((new_vertex, c_out, c_in, edge))
    }

    /// Forwards to [`geop_core_topology::Model::mef`], naming the new edge
    /// and face it creates.
    pub fn mef(
        &mut self,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        new_surface: NurbSurface3D<S>,
        edge_name: impl Into<String>,
        face_name: impl Into<String>,
    ) -> GeopResult<(EdgeId, FaceId, CoedgeId, CoedgeId)> {
        let (edge, face, c_forward, c_backward) = self.store.topology_mut().mef(
            coedge1,
            coedge2,
            curve,
            pcurve,
            pcurve_reversed,
            new_surface,
        )?;
        self.store.insert_name(edge, edge_name)?;
        self.store.insert_name(face, face_name)?;
        Ok((edge, face, c_forward, c_backward))
    }

    /// Forwards to [`geop_core_topology::Model::mer`], naming the new edge it
    /// creates (`existing_face_id` already has a name of its own).
    pub fn mer(
        &mut self,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        existing_face_id: FaceId,
        edge_name: impl Into<String>,
    ) -> GeopResult<(EdgeId, CoedgeId, CoedgeId)> {
        let (edge, c_backward, c_forward) = self.store.topology_mut().mer(
            coedge1,
            coedge2,
            curve,
            pcurve,
            pcurve_reversed,
            existing_face_id,
        )?;
        self.store.insert_name(edge, edge_name)?;
        Ok((edge, c_backward, c_forward))
    }

    /// Forwards to [`geop_core_topology::Model::mekr`], naming the new edge
    /// it creates.
    pub fn mekr(
        &mut self,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        edge_name: impl Into<String>,
    ) -> GeopResult<(EdgeId, CoedgeId, CoedgeId)> {
        let (edge, c_a, c_b) = self
            .store
            .topology_mut()
            .mekr(coedge1, coedge2, curve, pcurve)?;
        self.store.insert_name(edge, edge_name)?;
        Ok((edge, c_a, c_b))
    }

    /// Forwards to [`geop_core_topology::Model::mvr`], naming the new vertex
    /// it creates.
    pub fn mvr(
        &mut self,
        face_id: FaceId,
        point: Vector3<S>,
        vertex_name: impl Into<String>,
    ) -> GeopResult<VertexId> {
        let vertex = self.store.topology_mut().mvr(face_id, point)?;
        self.store.insert_name(vertex, vertex_name)?;
        Ok(vertex)
    }

    /// Forwards to [`geop_core_topology::Model::add_vertex_coedge`]. Creates
    /// no vertex/edge/face/solid of its own (only a coedge, which `Part`
    /// never names), so it takes no name argument.
    pub fn add_vertex_coedge(
        &mut self,
        after: CoedgeId,
        vertex: VertexId,
        pcurve: NurbCurve2D<S>,
    ) -> GeopResult<CoedgeId> {
        self.store
            .topology_mut()
            .add_vertex_coedge(after, vertex, pcurve)
    }

    /// Forwards to [`geop_core_topology::Model::kill_vertex_coedge`]. Deletes
    /// no named entity, so it takes no name argument.
    pub fn kill_vertex_coedge(&mut self, coedge: CoedgeId) -> GeopResult<()> {
        self.store.topology_mut().kill_vertex_coedge(coedge)
    }

    /// Forwards to [`geop_core_topology::Model::replace_face`]. Creates and
    /// deletes nothing: the face keeps its name with its new surface.
    pub fn replace_face(&mut self, face_id: FaceId, surface: NurbSurface3D<S>) -> GeopResult<()> {
        self.store.topology_mut().replace_face(face_id, surface)
    }

    /// Forwards to [`geop_core_topology::Model::replace_pcurve`]. Coedges are
    /// never named, so this takes no name.
    pub fn replace_pcurve(
        &mut self,
        coedge_id: CoedgeId,
        pcurve: NurbCurve2D<S>,
    ) -> GeopResult<()> {
        self.store.topology_mut().replace_pcurve(coedge_id, pcurve)
    }

    /// Forwards to [`geop_core_topology::Model::kef`], forgetting the names
    /// of the edge and face it deletes (both already given as arguments).
    pub fn kef(&mut self, edge: EdgeId, killed_face: FaceId) -> GeopResult<()> {
        self.store.topology_mut().kef(edge, killed_face)?;
        self.store.remove_name(edge);
        self.store.remove_name(killed_face);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::kemr`], forgetting the name
    /// of the edge it deletes (`ca_id`/`cb_id`'s shared edge).
    pub fn kemr(&mut self, ca_id: CoedgeId, cb_id: CoedgeId) -> GeopResult<()> {
        let edge = self.topology().get_coedge(ca_id)?.edge()?;
        self.store.topology_mut().kemr(ca_id, cb_id)?;
        self.store.remove_name(edge);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::ker`], forgetting the name of
    /// the edge it deletes (`coedge_backward`/`coedge_forward`'s shared edge).
    pub fn ker(&mut self, coedge_backward: CoedgeId, coedge_forward: CoedgeId) -> GeopResult<()> {
        let edge = self.topology().get_coedge(coedge_forward)?.edge()?;
        self.store
            .topology_mut()
            .ker(coedge_backward, coedge_forward)?;
        self.store.remove_name(edge);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::kve`], forgetting the names
    /// of the edge (`c_out`/`c_in`'s shared edge) and vertex it deletes.
    pub fn kve(&mut self, c_out: CoedgeId, c_in: CoedgeId, vertex: VertexId) -> GeopResult<()> {
        let edge = self.topology().get_coedge(c_out)?.edge()?;
        self.store.topology_mut().kve(c_out, c_in, vertex)?;
        self.store.remove_name(edge);
        self.store.remove_name(vertex);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::kvfs`], forgetting the names
    /// of the vertex, face and solid it deletes. `Model::kvfs` doesn't hand
    /// those ids back (it just takes `solid`), so they're read off the
    /// still-intact topology first, the same way `Model::kvfs` itself finds
    /// them; if that shape isn't there, `Model::kvfs` below rejects it for
    /// the same reason, so nothing is left half-forgotten.
    pub fn kvfs(&mut self, solid: SolidId) -> GeopResult<()> {
        let shell_id = *self
            .topology()
            .get_solid(solid)?
            .shells
            .first()
            .ok_or_else(|| GeopError::new(format!("solid {solid} must have exactly one shell")))?;
        let face_id = *self
            .topology()
            .get_shell(shell_id)?
            .faces
            .first()
            .ok_or_else(|| {
                GeopError::new(format!("shell {shell_id} must have exactly one face"))
            })?;
        let vertex_id = match self.topology().get_face(face_id)?.outer {
            BoundaryType::Vertex(v) => v,
            BoundaryType::Loop(_) => {
                return Err(GeopError::new(format!(
                    "face {face_id}'s boundary must be a bare vertex"
                )));
            }
        };
        self.store.topology_mut().kvfs(solid)?;
        self.store.remove_name(vertex_id);
        self.store.remove_name(face_id);
        self.store.remove_name(solid);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::kvr`], forgetting the name of
    /// the vertex it deletes (already given as an argument).
    pub fn kvr(&mut self, face_id: FaceId, vertex: VertexId) -> GeopResult<()> {
        self.store.topology_mut().kvr(face_id, vertex)?;
        self.store.remove_name(vertex);
        Ok(())
    }
}
