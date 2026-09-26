use crate::{CoedgeGeometry, Model, VertexId, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Merges `vertex_deleted_id` into `vertex_into_id`: unions their points
    /// into `vertex_into_id`, repoints every edge/coedge/boundary reference
    /// from `vertex_deleted_id` to `vertex_into_id`, then drops the now
    ///-unreferenced vertex.
    pub fn merge_vertex(
        &mut self,
        vertex_into_id: VertexId,
        vertex_deleted_id: VertexId,
    ) -> GeopResult<()> {
        // A self-merge would delete a vertex still referenced everywhere it
        // stood in for `vertex_into_id` a moment ago — always a caller bug
        // (typically: merging against a stale vertex snapshot taken before
        // an earlier merge already welded the two together), so this is
        // rejected rather than silently absorbed.
        if vertex_into_id == vertex_deleted_id {
            return Err(GeopError::new(format!(
                "Model::merge_vertex: vertex_into_id and vertex_deleted_id are both {vertex_into_id} — refusing to merge a vertex into itself"
            )));
        }

        // union the deleted vertex into point
        let deleted_point = self.get_vertex(vertex_deleted_id)?.point;
        let vertex_into = self.get_vertex_mut(vertex_into_id)?;
        vertex_into.point = vertex_into.point.union(&deleted_point);

        // Find all occurences of vertex_deleted_id and replace with vertex_into_id
        for edge in self.edges.values_mut() {
            if edge.start_vertex == vertex_deleted_id {
                edge.start_vertex = vertex_into_id;
            }
            if edge.end_vertex == vertex_deleted_id {
                edge.end_vertex = vertex_into_id;
            }
        }

        for coedge in self.coedges.values_mut() {
            if coedge.geometry == CoedgeGeometry::Vertex(vertex_deleted_id) {
                coedge.geometry = CoedgeGeometry::Vertex(vertex_into_id);
            }
        }

        for face in self.faces.values_mut() {
            for boundary in face.boundaries_mut() {
                if *boundary == BoundaryType::Vertex(vertex_deleted_id) {
                    *boundary = BoundaryType::Vertex(vertex_into_id);
                }
            }
        }

        self.vertices.remove(&vertex_deleted_id);

        Ok(())
    }
}
