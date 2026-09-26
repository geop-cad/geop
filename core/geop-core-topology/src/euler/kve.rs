use crate::{CoedgeId, Model, VertexId, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Kill the vertex and edge spliced in by mve: c_out and c_in are mve's own two
    // coedges (its returned coedge_forward and coedge_reversed) and must be adjacent
    // to each other (c_out immediately followed by c_in), forming a dead-end spur off
    // the rest of the loop. vertex must be one of their shared edge's two endpoints;
    // it is removed along with the edge.
    pub fn kve(
        self: &mut Model<S>,
        c_out: CoedgeId,
        c_in: CoedgeId,
        vertex: VertexId,
    ) -> GeopResult<()> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::kve(c_out={c_out}, c_in={c_in}, vertex={vertex})"
            ))
        };

        let ce_out = self.get_coedge(c_out)?.clone();
        let ce_in = self.get_coedge(c_in)?.clone();
        if ce_out.edge().with_context(&ctx)? != ce_in.edge().with_context(&ctx)? {
            return Err(ctx(GeopError::new(
                "c_out and c_in must belong to the same edge",
            )));
        }
        if ce_out.next != c_in || ce_in.prev != c_out {
            return Err(ctx(GeopError::new(
                "c_out and c_in must be adjacent to each other",
            )));
        }

        let p = ce_out.prev;
        let n = ce_in.next;
        if self.get_coedge(p)?.next != c_out || self.get_coedge(n)?.prev != c_in {
            return Err(ctx(GeopError::new(
                "coedges surrounding the edge are not consistently linked",
            )));
        }

        let edge = ce_out.edge().with_context(&ctx)?;
        let edge_data = self.get_edge(edge)?.clone();
        if edge_data.start_vertex != vertex && edge_data.end_vertex != vertex {
            return Err(ctx(GeopError::new(
                "vertex must be one of edge's two endpoints",
            )));
        }

        self.coedges.get_mut(&p).unwrap().next = n;
        self.coedges.get_mut(&n).unwrap().prev = p;
        self.coedges.remove(&c_in);
        self.coedges.remove(&c_out);
        self.edges.remove(&edge);
        self.vertices.remove(&vertex);

        let face = self.faces.get_mut(&ce_in.face).unwrap();
        for b in face.boundaries_mut() {
            if *b == BoundaryType::Loop(c_in) || *b == BoundaryType::Loop(c_out) {
                *b = BoundaryType::Loop(n);
            }
        }

        Ok(())
    }
}
