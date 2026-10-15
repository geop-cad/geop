//! The model as the viewport draws it: its vertices, edges and faces, each
//! by name, drawn into a [`PartView`] from its rasterization.

use std::collections::HashMap;

use geop_core_editor::view::{PartView, RESOLUTION, ViewEdge, ViewFace, ViewVertex};
use geop_core_math::geop_error::GeopResult;
use geop_core_rasterize::rasterize;
use geop_core_topology::{EdgeId, Model, SolidId};

use crate::{NamedModel, RefId};
use geop_core_program::values::Design;

/// The solid `edge` bounds: that of a face it runs along.
fn solid_of_edge(model: &Model<Design>, edge: EdgeId) -> Option<SolidId> {
    let coedge = *model.coedges_of_edge(edge).first()?;
    model
        .solid_of_face(model.get_coedge(coedge).ok()?.face)
        .ok()?
}

impl NamedModel<Design> {
    /// Draws its vertices, edges and faces, and lists its solids, into
    /// `view`, as elements of the target `target`.
    pub fn draw(&self, target: &str, view: &mut PartView) -> GeopResult<()> {
        let model = self.topology();
        let raster = rasterize(model, RESOLUTION)?;
        let name = |id: RefId| self.name_of(id).unwrap_or_default().to_string();

        // A vertex is a corner of the solid an edge at it bounds, and of the
        // faces those edges bound.
        let mut corner_of = HashMap::new();
        let mut faces_of_edge = HashMap::new();
        let mut faces_at_corner: HashMap<_, Vec<String>> = HashMap::new();
        for (&id, edge) in &model.edges {
            if let Some(solid) = solid_of_edge(model, id) {
                corner_of.insert(edge.start_vertex, solid);
                corner_of.insert(edge.end_vertex, solid);
            }
            let mut faces: Vec<String> = (model.coedges_of_edge(id).into_iter())
                .filter_map(|c| model.get_coedge(c).ok())
                .map(|c| name(c.face.into()))
                .collect();
            faces.sort();
            faces.dedup();
            for v in [edge.start_vertex, edge.end_vertex] {
                let at = faces_at_corner.entry(v).or_default();
                at.extend(faces.iter().cloned());
                at.sort();
                at.dedup();
            }
            faces_of_edge.insert(id, faces);
        }
        let features = self.feature_faces();
        view.model = target.to_string();
        // Every map here is ordered by id, so these come in id order.
        view.vertices = (raster.vertices.iter())
            .map(|(&id, p)| ViewVertex {
                name: name(id.into()),
                solid: corner_of.get(&id).map(|&s| name(s.into())),
                faces: faces_at_corner.get(&id).cloned().unwrap_or_default(),
                at: *p,
            })
            .collect();
        view.edges = (raster.edges.iter())
            .map(|(&id, polyline)| ViewEdge {
                ends: (model.get_edge(id))
                    .map(|e| [e.start_vertex, e.end_vertex].map(|v| name(v.into())))
                    .unwrap_or_default(),
                solid: solid_of_edge(model, id).map(|s| name(s.into())),
                faces: faces_of_edge.get(&id).cloned().unwrap_or_default(),
                name: name(id.into()),
                polyline: polyline.clone(),
            })
            .collect();
        view.faces = (raster.faces.iter())
            .map(|(&id, tris)| {
                let face = name(id.into());
                ViewFace {
                    feature: features.of(&face).map(str::to_string),
                    name: face,
                    solid: (model.solid_of_face(id).ok().flatten()).map(|s| name(s.into())),
                    triangles: tris.iter().map(|t| [t.a, t.b, t.c]).collect(),
                    normals: (tris.iter())
                        .map(|t| t.vertex_normals.unwrap_or([t.normal; 3]))
                        .collect(),
                }
            })
            .collect();
        view.solids = model.solids.keys().map(|&s| name(s.into())).collect();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use geop_core_editor::view::drawn;
    use geop_core_editor::{
        editing::{Pointer, Reach},
        selectable::selected,
    };
    use geop_core_math::{primitives::Ray, scalars::Scalar, vector::Vector3};
    use geop_core_program::{element::ElementReference, part::Part};

    use crate::{
        Body, Line, NamedModel, model,
        testing::{given_part, unit_cube},
    };

    use super::*;

    /// Looking down at `(x, y)` from above the unit cube.
    fn from_above(x: f64, y: f64) -> Pointer<Design> {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(Design::from_f64));
        Pointer {
            ray: Ray::try_new(v([x, y, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
            reach: Reach::Tube {
                radius: Design::from_f64(0.01),
            },
        }
    }

    fn accepts(kinds: &[&str]) -> Vec<String> {
        kinds.iter().map(|k| k.to_string()).collect()
    }

    /// Looking down at the cube, a corner is a vertex, a side's middle an
    /// edge and the middle of the top a face — of the target it was drawn
    /// as. What a field does not accept is passed over for what it does.
    #[test]
    fn a_pick_hits_the_smallest_element_under_the_pointer() {
        let mut view = PartView::new();
        unit_cube().draw("model", &mut view).unwrap();
        let view = view.finish();
        assert_eq!(
            (view.vertices.len(), view.edges.len(), view.faces.len()),
            (8, 12, 6)
        );
        let all = accepts(&[drawn::VERTEX, drawn::EDGE, drawn::FACE]);
        let hit = |x, y, accepts: &[String]| view.pick(&from_above(x, y), accepts).unwrap().element;
        assert_eq!(hit(1.0, 1.0, &all).kind, drawn::VERTEX);
        assert_eq!(hit(0.5, 1.0, &all).kind, drawn::EDGE);
        let top = hit(0.5, 0.5, &all);
        assert_eq!(
            (top.kind.as_str(), top.target.as_str()),
            (drawn::FACE, "model")
        );
        assert_eq!(hit(1.0, 1.0, &accepts(&[drawn::FACE])).kind, drawn::FACE);
        assert!(view.pick(&from_above(5.0, 5.0), &all).is_none());
    }

    /// What a field asks for is selected from the pick: a line from an
    /// edge, the solid as a body from its face.
    #[test]
    fn a_pick_selects_what_a_field_asks_for() {
        let mut view = PartView::new();
        unit_cube().draw("model", &mut view).unwrap();
        let view = view.finish();
        let part: Part = given_part(&unit_cube()).unwrap();
        let reader = part.current();
        let edge = view
            .pick(&from_above(0.5, 1.0), &accepts(&[drawn::EDGE]))
            .unwrap();
        let lines: Vec<ElementReference<NamedModel<Design>, Line>> =
            selected(&reader, &[edge.element.clone()]);
        assert_eq!(lines[0].name(), edge.element.name);
        let face = view
            .pick(&from_above(0.5, 0.5), &accepts(&[drawn::FACE]))
            .unwrap();
        let bodies: Vec<ElementReference<NamedModel<Design>, Body>> =
            selected(&reader, &[face.element]);
        assert_eq!(bodies, [ElementReference::new(model(), "s0")]);
    }
}
