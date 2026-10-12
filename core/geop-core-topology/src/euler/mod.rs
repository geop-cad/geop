mod add_vertex_coedge;
mod kef;
mod kemr;
mod ker;
mod kill_vertex_coedge;
mod kve;
mod kvfs;
mod kvr;
mod mef;
mod mekr;
mod mer;
mod mve;
mod mvfs;
mod mvr;

mod replace_face;
mod replace_pcurve;

#[cfg(test)]
mod round_trip_test;

use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, FaceId, Model, Sense, Vertex, VertexId,
    argument_validation::{
        validate_curve_start_and_end, validate_pcurve_start_and_end, validate_same_loop,
    },
    boundary::BoundaryIndex,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::walk::{Walked, walk},
    vector::Vector3,
    with_context,
};

/// The steps the Euler operators share.
impl<S: Scalar> Model<S> {
    /// A new edge `p`-wards from vertex `from` on `face_id`: a new vertex at
    /// `p`, the edge, its forward coedge (`pcurve`) and its reversed one
    /// (`pcurve_reversed`), the forward one followed by the reversed one.
    /// What comes before the forward one and after the reversed one is the
    /// caller's to link. Returns `(vertex, forward, reversed, edge)`.
    fn insert_spur(
        &mut self,
        face_id: FaceId,
        from: VertexId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        p: Vector3<S>,
    ) -> GeopResult<(VertexId, CoedgeId, CoedgeId, EdgeId)> {
        let surface = &self.get_face(face_id)?.surface;
        let start = self.get_vertex(from)?.point;
        validate_pcurve_start_and_end(surface, &pcurve, &start, &p)?;
        validate_pcurve_start_and_end(surface, &pcurve_reversed, &p, &start)?;
        validate_curve_start_and_end(&curve, &start, &p)?;

        let v = self.insert_vertex(Vertex { point: p });
        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: from,
            end_vertex: v,
        });
        let coedge = |sense, pcurve| Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense,
            pcurve,
            next: CoedgeId(0), // set below, or by the caller
            prev: CoedgeId(0),
            face: face_id,
        };
        let forward = self.insert_coedge(coedge(Sense::Forward, pcurve));
        let reversed = self.insert_coedge(coedge(Sense::Reversed, pcurve_reversed));
        self.coedges.get_mut(&forward).unwrap().next = reversed;
        self.coedges.get_mut(&reversed).unwrap().prev = forward;
        Ok((v, forward, reversed, edge_id))
    }

    /// Check the arguments of an edge from `coedge1`'s end to `coedge2`'s
    /// start that cuts their loop in two, the ring from `coedge2` round to
    /// `coedge1` moving onto a face of `ring_surface`: the curves end where
    /// they must, the two coedges share a loop, and the ring's pcurves lie
    /// on `ring_surface`. Returns which boundary of the face is cut, found
    /// now — before any pointer is rewired, the original ring is intact
    /// and reaches `coedge1` wherever its anchor sits — and the ring's
    /// coedges.
    fn check_ring_split(
        &self,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: &NurbCurve3D<S>,
        pcurve: &NurbCurve2D<S>,
        pcurve_reversed: &NurbCurve2D<S>,
        ring_surface: &NurbSurface3D<S>,
    ) -> GeopResult<(BoundaryIndex, Vec<CoedgeId>)> {
        validate_same_loop(self, coedge1, coedge2)?;
        let face_id = self.get_coedge(coedge1)?.face;
        let start = self.coedge_end_vertex(coedge1)?.point;
        let end = self.coedge_start_vertex(coedge2)?.point;
        let surface = &self.get_face(face_id)?.surface;
        validate_pcurve_start_and_end(ring_surface, pcurve, &start, &end)
            .with_context("initial arg validation")?;
        validate_pcurve_start_and_end(surface, pcurve_reversed, &end, &start)
            .with_context("initial arg validation")?;
        validate_curve_start_and_end(curve, &start, &end).with_context("initial arg validation")?;

        let boundary = self.find_boundary_containing(face_id, coedge1)?;

        // From `coedge2` round to `coedge1`, which `validate_same_loop`
        // found on its loop.
        let ring = walk(coedge2, |&c| {
            Ok::<_, GeopError>(if c == coedge1 {
                None
            } else {
                Some(self.get_coedge(c)?.next)
            })
        })?;
        if ring.end != Walked::Open {
            return Err(GeopError::new(format!(
                "the loop of coedge {coedge2} does not reach coedge {coedge1}"
            )));
        }
        let ring = ring.nodes;
        for &member in &ring {
            let c = self.get_coedge(member)?;
            let sp = self.coedge_start_vertex(member)?.point;
            let ep = self.coedge_end_vertex(member)?.point;
            validate_pcurve_start_and_end(ring_surface, &c.pcurve, &sp, &ep).with_context(
                with_context!("reassigning coedge {member} to the ring's face"),
            )?;
        }
        Ok((boundary, ring))
    }

    /// A new edge from `coedge1`'s end vertex to `coedge2`'s start vertex,
    /// on `coedge1`'s face: its forward coedge (`pcurve`) spliced in
    /// between `coedge1` and `coedge2`, its reversed one (`pcurve_reversed`)
    /// between `coedge2`'s predecessor and `coedge1`'s successor. On one
    /// loop that cuts it in two, across two it joins them. Returns
    /// `(edge, forward, reversed)`.
    fn insert_edge_between(
        &mut self,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
    ) -> GeopResult<(EdgeId, CoedgeId, CoedgeId)> {
        let ce1 = self.get_coedge(coedge1)?;
        let (face, next1) = (ce1.face, ce1.next);
        let prev2 = self.get_coedge(coedge2)?.prev;
        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: self.coedge_end_vertex_id(coedge1)?,
            end_vertex: self.coedge_start_vertex_id(coedge2)?,
        });
        let forward = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve,
            next: coedge2,
            prev: coedge1,
            face,
        });
        let reversed = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: pcurve_reversed,
            next: next1,
            prev: prev2,
            face,
        });
        self.coedges.get_mut(&coedge1).unwrap().next = forward;
        self.coedges.get_mut(&coedge2).unwrap().prev = forward;
        self.coedges.get_mut(&prev2).unwrap().next = reversed;
        self.coedges.get_mut(&next1).unwrap().prev = reversed;
        Ok((edge_id, forward, reversed))
    }

    /// Remove the edge whose two coedges are `a` and `b`, the inverse of
    /// [`Self::insert_edge_between`]: each sits between one ring's root and
    /// the other ring's predecessor, so removing them reconnects across —
    /// `a`'s predecessor to `b`'s successor and `b`'s predecessor to `a`'s
    /// successor — joining two loops into one, or cutting one in two.
    fn remove_edge_between(&mut self, a: CoedgeId, b: CoedgeId) -> GeopResult<()> {
        let ca = self.get_coedge(a)?.clone();
        let cb = self.get_coedge(b)?.clone();
        let edge = ca.edge()?;
        if cb.edge()? != edge {
            return Err(GeopError::new(format!(
                "coedges {a} and {b} must belong to the same edge"
            )));
        }
        self.coedges.get_mut(&ca.prev).unwrap().next = cb.next;
        self.coedges.get_mut(&cb.next).unwrap().prev = ca.prev;
        self.coedges.get_mut(&cb.prev).unwrap().next = ca.next;
        self.coedges.get_mut(&ca.next).unwrap().prev = cb.prev;
        self.coedges.remove(&a);
        self.coedges.remove(&b);
        self.edges.remove(&edge);
        Ok(())
    }
}
