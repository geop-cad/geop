//! Topology that is no longer needed, taken out of a body described as a
//! [`BodySpec`] ([`BodySpec::merged`]): faces that lie on one surface made
//! one, and vertices between two edges of the same two faces dropped, the
//! edges made one.
//!
//! A union leaves both solids' faces where they lay: two blocks standing
//! flush leave two faces in one plane, a seam edge between them that is no
//! edge of the shape. A face that lies on another's surface is decided on
//! the representation, not by recognising a plane or a cylinder: the
//! other's patch carried on reaches it, and the two are one polynomial
//! there
//! ([`geop_core_geometry::nurb_surface::NurbSurface3D::carrying`]). Two faces of one surface parted
//! because their patches cannot be one — a cylinder's quarters, which
//! would wrap round — stay apart: their loops would not close in one
//! patch's `(u, v)`.

use geop_core_geometry::nurb_curve::loop_signed_area;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use crate::{
    Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};

/// Where the entities of a [`BodySpec`] went in [`BodySpec::merged`]'s
/// result, index for index.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Merged {
    /// Per face: the face it is part of now.
    pub faces: Vec<usize>,
    /// Per edge: the edge it is part of now; none for an edge between two
    /// faces made one.
    pub edges: Vec<Option<usize>>,
    /// Per vertex: the vertex it is now; none for one dropped.
    pub vertices: Vec<Option<usize>>,
}

impl BodySpec {
    /// The same body without topology it does not need (see the module
    /// docs): every two faces across an edge that lie on one surface made
    /// one, on the surface of the one first in order — the older, for a
    /// body described in the order its faces were made — and then every
    /// vertex that only two edges of the same two faces meet at dropped,
    /// the two edges made one, the first in order. Also says where each
    /// entity went.
    pub fn merged(&self) -> GeopResult<(BodySpec, Merged)> {
        let mut work = Work::new(self);
        while work.merge_one_pair()? {}
        while work.join_one_pair()? {}
        work.compacted()
    }
}

/// A body being simplified: the original's entities, by their index there,
/// each gone or changed.
struct Work<'a> {
    spec: &'a BodySpec,
    /// Per face: what it is now, none if made part of another.
    faces: Vec<Option<FaceSpec>>,
    /// Per face: the face it is part of.
    face_into: Vec<usize>,
    /// Per edge: what it is now, none if gone.
    edges: Vec<Option<EdgeSpec>>,
    /// Per edge: the edge it is part of, none if it went between two faces
    /// made one.
    edge_into: Vec<Option<usize>>,
}

impl<'a> Work<'a> {
    fn new(spec: &'a BodySpec) -> Self {
        Self {
            spec,
            faces: spec.faces.iter().cloned().map(Some).collect(),
            face_into: (0..spec.faces.len()).collect(),
            edges: spec.edges.iter().cloned().map(Some).collect(),
            edge_into: (0..spec.edges.len()).map(Some).collect(),
        }
    }

    /// Per edge, the faces using it, once per use.
    fn uses(&self) -> Vec<Vec<usize>> {
        let mut uses = vec![Vec::new(); self.edges.len()];
        for (f, face) in self.faces.iter().enumerate() {
            let Some(face) = face else { continue };
            for c in face.loops().into_iter().flatten() {
                if let CoedgeOn::Edge(e, _) = c.on {
                    uses[e].push(f);
                }
            }
        }
        uses
    }

    /// The vertex a coedge running along `on` starts at, and the one it
    /// ends at, in the body as it is now.
    fn ends(&self, on: CoedgeOn) -> (usize, usize) {
        match on {
            CoedgeOn::Edge(e, sense) => {
                let edge = self.edges[e]
                    .as_ref()
                    .expect("a coedge runs along a live edge");
                match sense {
                    Sense::Forward => (edge.start, edge.end),
                    Sense::Reversed => (edge.end, edge.start),
                }
            }
            CoedgeOn::Vertex(v) => (v, v),
        }
    }

    /// Makes one pair of faces across an edge, on one surface, one face.
    /// Whether it found one.
    fn merge_one_pair(&mut self) -> GeopResult<bool> {
        let uses = self.uses();
        for faces in &uses {
            let [f, g] = faces.as_slice() else { continue };
            if f == g {
                continue;
            }
            let (keep, gone) = ((*f).min(*g), (*f).max(*g));
            if let Some(face) = self.merged_face(keep, gone, &uses)? {
                self.faces[keep] = Some(face);
                self.faces[gone] = None;
                for into in self.face_into.iter_mut() {
                    if *into == gone {
                        *into = keep;
                    }
                }
                for (e, faces) in uses.iter().enumerate() {
                    if faces.len() == 2 && faces.iter().all(|&h| h == keep || h == gone) {
                        self.edges[e] = None;
                        self.edge_into[e] = None;
                    }
                }
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// The faces `keep` and `gone` as one, if they lie on one surface and
    /// their loops close up as the loops of one face on it.
    fn merged_face(
        &self,
        keep: usize,
        gone: usize,
        uses: &[Vec<usize>],
    ) -> GeopResult<Option<FaceSpec>> {
        let (a, b) = (
            self.faces[keep].as_ref().expect("live"),
            self.faces[gone].as_ref().expect("live"),
        );
        // On the surface of the one that carries the other, its pcurves
        // as they are and the other's mapped onto it.
        let (surface, coedges) = if let Some((surface, map)) = a.surface.carrying(&b.surface)? {
            (surface, self.coedges(a, b, |c| map.apply_to(c))?)
        } else if let Some((surface, map)) = b.surface.carrying(&a.surface)? {
            (surface, self.coedges(b, a, |c| map.apply_to(c))?)
        } else {
            return Ok(None);
        };
        // Coedges along the edges between the two go.
        let inside = |c: &CoedgeSpec| match c.on {
            CoedgeOn::Edge(e, _) => {
                uses[e].len() == 2 && uses[e].iter().all(|&h| h == keep || h == gone)
            }
            CoedgeOn::Vertex(_) => false,
        };
        let coedges: Vec<CoedgeSpec> = coedges.into_iter().filter(|c| !inside(c)).collect();
        let Some(loops) = self.loops(coedges)? else {
            return Ok(None);
        };
        let mut outer = None;
        let mut holes = Vec::new();
        for lp in loops {
            let area = loop_signed_area(lp.iter().map(|c| &c.pcurve))?;
            if area.definitely_greater(Scalar::ZERO) && outer.is_none() {
                outer = Some(lp);
            } else if area.definitely_less(Scalar::ZERO) {
                holes.push(lp);
            } else {
                return Ok(None);
            }
        }
        Ok(outer.map(|outer| FaceSpec {
            surface,
            outer,
            holes,
        }))
    }

    /// Every coedge of `first` as it is, and of `second` with its pcurve
    /// mapped by `map`.
    fn coedges(
        &self,
        first: &FaceSpec,
        second: &FaceSpec,
        map: impl Fn(&crate::Curve2) -> GeopResult<crate::Curve2>,
    ) -> GeopResult<Vec<CoedgeSpec>> {
        let mut out: Vec<CoedgeSpec> = first.loops().into_iter().flatten().cloned().collect();
        for c in second.loops().into_iter().flatten() {
            out.push(CoedgeSpec {
                on: c.on,
                pcurve: map(&c.pcurve)?,
            });
        }
        Ok(out)
    }

    /// `coedges` followed end to end into loops, if they close up as one
    /// face's: one coedge leaving each vertex, each meeting the next where
    /// it starts in `(u, v)` too.
    fn loops(&self, coedges: Vec<CoedgeSpec>) -> GeopResult<Option<Vec<Vec<CoedgeSpec>>>> {
        let mut leaving: std::collections::HashMap<usize, usize> = Default::default();
        for (i, c) in coedges.iter().enumerate() {
            if leaving.insert(self.ends(c.on).0, i).is_some() {
                return Ok(None);
            }
        }
        let mut used = vec![false; coedges.len()];
        let mut loops = Vec::new();
        for first in 0..coedges.len() {
            if used[first] {
                continue;
            }
            let mut lp = Vec::new();
            let mut at = first;
            loop {
                if used[at] {
                    if at != first {
                        return Ok(None);
                    }
                    break;
                }
                used[at] = true;
                lp.push(at);
                let Some(&next) = leaving.get(&self.ends(coedges[at].on).1) else {
                    return Ok(None);
                };
                let (end, start) = (coedges[at].pcurve.ends()?.1, coedges[next].pcurve.ends()?.0);
                if !end.could_be_equal(&start) {
                    return Ok(None);
                }
                at = next;
            }
            loops.push(lp.into_iter().map(|i| coedges[i].clone()).collect());
        }
        Ok(Some(loops))
    }

    /// Makes one pair of edges that only meet each other at a vertex, and
    /// run between the same two faces, one edge. Whether it found one.
    fn join_one_pair(&mut self) -> GeopResult<bool> {
        let uses = self.uses();
        let n = self.spec.vertices.len();
        let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (e, edge) in self.edges.iter().enumerate() {
            let Some(edge) = edge else { continue };
            at_vertex[edge.start].push(e);
            at_vertex[edge.end].push(e);
        }
        let mut pole = vec![false; n];
        for face in self.faces.iter().flatten() {
            for c in face.loops().into_iter().flatten() {
                if let CoedgeOn::Vertex(v) = c.on {
                    pole[v] = true;
                }
            }
        }
        for v in 0..n {
            let [e1, e2] = at_vertex[v].as_slice() else {
                continue;
            };
            let (e1, e2) = ((*e1).min(*e2), (*e1).max(*e2));
            if e1 == e2 || pole[v] {
                continue;
            }
            let (one, two) = (
                self.edges[e1].as_ref().unwrap(),
                self.edges[e2].as_ref().unwrap(),
            );
            let far = |edge: &EdgeSpec| {
                if edge.start == v {
                    edge.end
                } else {
                    edge.start
                }
            };
            if far(one) == far(two) || one.start == one.end || two.start == two.end {
                continue;
            }
            let mut faces = (uses[e1].clone(), uses[e2].clone());
            faces.0.sort();
            faces.1.sort();
            if faces.0 != faces.1 || faces.0.len() != 2 || faces.0[0] == faces.0[1] {
                continue;
            }
            if self.join(v, e1, e2)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// The edges `e1` and `e2`, meeting at `v` only, made one, `e1`; every
    /// loop's two coedges there one, their pcurves joined. Whether the
    /// curves could be joined.
    fn join(&mut self, v: usize, e1: usize, e2: usize) -> GeopResult<bool> {
        let (one, two) = (
            self.edges[e1].clone().unwrap(),
            self.edges[e2].clone().unwrap(),
        );
        // Along `e1`, through `v`, on along `e2`, or the other way round.
        let toward_v = |edge: &EdgeSpec| {
            if edge.end == v {
                edge.curve.clone()
            } else {
                edge.curve.reverse()
            }
        };
        let (curve, start, end) = if one.end == v {
            let from_v = toward_v(&two).reverse();
            (
                one.curve.joined(&from_v),
                one.start,
                if two.start == v { two.end } else { two.start },
            )
        } else {
            let into_v = toward_v(&two);
            (
                into_v.joined(&one.curve),
                if two.end == v { two.start } else { two.end },
                one.end,
            )
        };
        let Ok(curve) = curve else {
            return Ok(false);
        };
        // Each loop through `v`: its two coedges there, in order, as one.
        let mut faces = self.faces.clone();
        for face in faces.iter_mut().flatten() {
            let mut loops: Vec<Vec<CoedgeSpec>> = face.loops().into_iter().cloned().collect();
            for lp in &mut loops {
                let n = lp.len();
                let on = |c: &CoedgeSpec, e: usize| matches!(c.on, CoedgeOn::Edge(x, _) if x == e);
                let Some(k) = (0..n).find(|&k| {
                    let next = &lp[(k + 1) % n];
                    (on(&lp[k], e1) && on(next, e2)) || (on(&lp[k], e2) && on(next, e1))
                }) else {
                    continue;
                };
                let (first, second) = (lp[k].clone(), lp[(k + 1) % n].clone());
                let Ok(pcurve) = first.pcurve.joined(&second.pcurve) else {
                    return Ok(false);
                };
                // Running from the vertex `first` starts at to the one
                // `second` ends at: the joined edge forward if that is its
                // start.
                let from = self.ends(first.on).0;
                let sense = if from == start {
                    Sense::Forward
                } else if from == end {
                    Sense::Reversed
                } else {
                    return Err(GeopError::new(format!(
                        "BodySpec::merged: a loop runs edges {e1} and {e2} from vertex {from}, which neither ends"
                    )));
                };
                let joined = CoedgeSpec {
                    on: CoedgeOn::Edge(e1, sense),
                    pcurve,
                };
                if k + 1 < n {
                    lp.splice(k..k + 2, [joined]);
                } else {
                    // The pair wraps round the loop's start.
                    lp.remove(k);
                    lp[0] = joined;
                }
            }
            let outer = loops.remove(0);
            *face = FaceSpec {
                surface: face.surface.clone(),
                outer,
                holes: loops,
            };
        }
        self.faces = faces;
        self.edges[e1] = Some(EdgeSpec { curve, start, end });
        self.edges[e2] = None;
        for into in self.edge_into.iter_mut() {
            if *into == Some(e2) {
                *into = Some(e1);
            }
        }
        Ok(true)
    }

    /// The body as it is now, its entities numbered anew, and where each of
    /// the original's went.
    fn compacted(self) -> GeopResult<(BodySpec, Merged)> {
        let spec = self.spec;
        let mut face_index = vec![None; spec.faces.len()];
        let mut faces = Vec::new();
        for (f, face) in self.faces.iter().enumerate() {
            if let Some(face) = face {
                face_index[f] = Some(faces.len());
                faces.push(face.clone());
            }
        }
        let mut vertex_index = vec![None; spec.vertices.len()];
        let mut vertices = Vec::new();
        let mut vertex = |v: usize, vertices: &mut Vec<_>| {
            *vertex_index[v].get_or_insert_with(|| {
                vertices.push(spec.vertices[v]);
                vertices.len() - 1
            })
        };
        let mut edge_index = vec![None; spec.edges.len()];
        let mut edges = Vec::new();
        for (e, edge) in self.edges.iter().enumerate() {
            if let Some(edge) = edge {
                edge_index[e] = Some(edges.len());
                edges.push(EdgeSpec {
                    curve: edge.curve.clone(),
                    start: vertex(edge.start, &mut vertices),
                    end: vertex(edge.end, &mut vertices),
                });
            }
        }
        for face in &mut faces {
            let mut renumber = |lp: &mut Vec<CoedgeSpec>, vertices: &mut Vec<_>| {
                for c in lp {
                    c.on = match c.on {
                        CoedgeOn::Edge(e, sense) => {
                            CoedgeOn::Edge(edge_index[e].expect("a live edge"), sense)
                        }
                        CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertex(v, vertices)),
                    };
                }
            };
            renumber(&mut face.outer, &mut vertices);
            for hole in &mut face.holes {
                renumber(hole, &mut vertices);
            }
        }
        let shells = (spec.shells.iter())
            .map(|members| {
                let mut kept: Vec<usize> = members.iter().filter_map(|&f| face_index[f]).collect();
                kept.dedup();
                kept
            })
            .collect();
        let out = BodySpec {
            vertices,
            edges,
            faces,
            shells,
            solid: spec.solid,
        };
        out.check()?;
        let merged = Merged {
            faces: (self.face_into.iter())
                .map(|&f| face_index[f].expect("a face is part of a live one"))
                .collect(),
            edges: (self.edge_into.iter())
                .map(|e| e.and_then(|e| edge_index[e]))
                .collect(),
            vertices: vertex_index,
        };
        Ok((out, merged))
    }
}

#[cfg(test)]
mod tests {

    use crate::{
        Model,
        test_fixtures::test_cube_solid,
        validation::{ValidationParameters, validate},
    };

    /// A cube is as simple as it gets: merged, it is the same.
    #[test]
    fn a_cube_stays_a_cube() {
        let mut model = Model::new();
        let solid = test_cube_solid(&mut model);
        let faces = model.solid_faces(solid).unwrap();
        let (spec, _) = model.body_spec(&faces, true).unwrap();
        let (merged, map) = spec.merged().unwrap();
        assert_eq!(merged.faces.len(), 6);
        assert_eq!(merged.edges.len(), 12);
        assert_eq!(merged.vertices.len(), 8);
        assert_eq!(map.faces, (0..6).collect::<Vec<_>>());
        let mut model = Model::new();
        model.build_body(merged).unwrap();
        validate(&ValidationParameters::default(), &model).unwrap();
    }
}
