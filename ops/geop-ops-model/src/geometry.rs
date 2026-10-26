//! Geometric kinds of element: what an entity can be used as — a
//! [`Point`], a [`Line`], a [`Plane`], something [`Round`], a [`Circle`],
//! a [`Curve`] — whatever target it is in. The model has them where its
//! entities have that shape: a straight edge is a line, a flat face a
//! plane, a circular edge a circle and something round. Other targets — the
//! datums, a sketch, a placed part — have them too, each in its own crate,
//! and an operation that takes several of them holds one of them
//! ([`geop_core_editor::one_of!`]).
//!
//! Each is picked from what an editor draws of the model, if that has the
//! shape: a line by a straight edge, a plane by a flat face.
//!
//! The model also has [`Body`]: a solid by its name, or a sheet — faces
//! standing on their own, which have no name as a whole — by one of its
//! faces.

use geop_core_editor::selectable::SelectableElement;
use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    vector::Vector3,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement},
    reader::Reader,
};
use geop_core_topology::{Body as TopologyBody, EdgeId, FaceId, Sense};

use crate::{
    NamedModel, RefId,
    curves::Chain,
    elements::{Edge, Face, Vertex, drawn_entity},
    model, read_model,
};

/// The name the part's model is a target by.
const MODEL: &str = "model";

/// A point: a vertex, a datum point, a sketch's point.
pub struct Point;

/// A line: a straight edge, a datum axis, a sketch's line.
pub struct Line;

/// A plane, as a frame with `w` its normal and `u`/`v` a sketch's `x`/`y`
/// on it: a flat face, a datum plane, a sketch's plane.
pub struct Plane;

/// The axis something turns around: a circular edge, a face of
/// revolution — a cylinder, a cone — a sketch's circle or arc.
pub struct Round;

/// A circle or an arc of one: a circular edge, a sketch's circle or arc.
pub struct Circle;

/// A curve, whatever its shape, as a chain of one curve named after it and
/// its ends: an edge — of a face or of a wire — a sketch's curve.
pub struct Curve;

/// Curves to run along, in order, as a chain named after what they are: an
/// edge of a face, a sketch's one chain or loop, a 3-D sketch's.
pub struct Path;

/// A body of the model: a solid, or a sheet of faces standing on their
/// own.
pub struct Body;

impl Kind for Point {
    const NAME: &'static str = "point";
}

impl Kind for Line {
    const NAME: &'static str = "line";
}

impl Kind for Plane {
    const NAME: &'static str = "plane";
}

impl Kind for Round {
    const NAME: &'static str = "round";
}

impl Kind for Circle {
    const NAME: &'static str = "circle";
}

impl Kind for Curve {
    const NAME: &'static str = "curve";
}

impl Kind for Path {
    const NAME: &'static str = "path";
}

impl Kind for Body {
    const NAME: &'static str = "body";
}

/// A right-handed orthonormal frame at `origin` with `w` along `normal`,
/// and `u` the world axis most parallel to the plane normal to it,
/// projected into that plane (see [`Vector3::across`]) — so frames with
/// parallel normals line up, and a frame put on an axis has `u` where a
/// joint measures its turns from: two bodies mated on the same axis of each
/// are at a turn of zero.
pub fn frame_along(origin: Vector3, normal: &Vector3) -> GeopResult<CoordinateSystem> {
    let n = normal.normalize()?;
    let [u, v] = n.across()?;
    CoordinateSystem::try_new(origin, u, v, n)
}

impl NamedModel {
    /// The edge `name`'s curve.
    fn edge_curve(&self, name: &str) -> GeopResult<(EdgeId, &NurbCurve3D)> {
        let id = self.edge_id(name)?;
        Ok((id, &self.topology().get_edge(id)?.curve))
    }

    /// The plane of the flat face `face`, as a frame: normal pointing out of
    /// its solid, origin the world origin's projection onto it and `u` the
    /// world axis most parallel to it, so sketches on parallel faces line
    /// up.
    pub fn frame_of_face(&self, face: FaceId) -> GeopResult<CoordinateSystem> {
        let plane = self.plane_of_face(face, "only a flat face is a plane")?;
        frame_along(plane.project(&Vector3::zero()), &plane.normal)
    }

    /// The one curve `edge` is, as a chain named after it and its
    /// vertices.
    pub fn edge_chain(&self, edge: EdgeId) -> GeopResult<Chain<NurbCurve3D>> {
        let name = self.name(edge)?.to_string();
        let edge = self.topology().get_edge(edge)?;
        Ok(Chain {
            curves: vec![edge.curve.clone()],
            curve_names: vec![name.clone()],
            joint_names: vec![
                self.name(edge.start_vertex)?.to_string(),
                self.name(edge.end_vertex)?.to_string(),
            ],
            name,
        })
    }

    /// The body `name` is: a solid's, or the one a face is part of.
    pub fn body(&self, name: &str) -> GeopResult<TopologyBody> {
        match self.id_of(name) {
            Some(RefId::Solid(solid)) => Ok(TopologyBody::Solid(solid)),
            Some(RefId::Face(face)) => self.topology().body_of_face(face),
            _ => Err(GeopError::new(format!(
                "{name:?} is no body: pick a solid or a face standing on its own"
            ))),
        }
    }
}

impl HasElements<Point> for NamedModel {
    type Element<'a> = Vector3;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Vector3> {
        Ok(self.topology().get_vertex(self.vertex_id(name)?)?.point)
    }
}

impl HasElements<Line> for NamedModel {
    type Element<'a> = Axis;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Axis> {
        let (_, curve) = self.edge_curve(name)?;
        (curve.as_line()?).ok_or_else(|| GeopError::new(format!("edge {name:?} is not straight")))
    }
}

impl HasElements<Plane> for NamedModel {
    type Element<'a> = CoordinateSystem;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<CoordinateSystem> {
        self.frame_of_face(self.face_id(name)?)
    }
}

impl HasElements<Round> for NamedModel {
    type Element<'a> = Axis;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Axis> {
        let axis = match self.id_of(name) {
            Some(RefId::Edge(_)) => {
                let arc = HasElements::<Circle>::element(self, name)?;
                Some(arc.circle.axis())
            }
            Some(RefId::Face(face)) => {
                (self.topology().get_face(face)?.surface).axis_of_revolution()?
            }
            _ => None,
        };
        axis.ok_or_else(|| {
            GeopError::new(format!(
                "{name:?} turns around no axis: pick a circular edge, or a cylindrical or conical face"
            ))
        })
    }
}

impl HasElements<Circle> for NamedModel {
    type Element<'a> = Arc;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Arc> {
        let (_, curve) = self.edge_curve(name)?;
        (curve.as_arc()?).ok_or_else(|| GeopError::new(format!("edge {name:?} is not circular")))
    }
}

impl HasElements<Curve> for NamedModel {
    type Element<'a> = Chain<NurbCurve3D>;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Chain<NurbCurve3D>> {
        self.edge_chain(self.edge_id(name)?)
    }
}

/// An edge of a face — of a solid or of a sheet — as the one curve it is;
/// an edge of a wire as the run of the wire's edges it is in, from end to
/// end or round (see [`geop_core_topology::Model::wire_chains`]), named
/// after its edges and the vertices between them. (A 3-D sketch's wire is
/// picked as the sketch, whose path is named after its elements.)
impl HasElements<Path> for NamedModel {
    type Element<'a> = Chain<NurbCurve3D>;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<Chain<NurbCurve3D>> {
        let edge = self.edge_id(name)?;
        let topology = self.topology();
        let Some(wire) = topology.wire_of_edge(edge) else {
            return self.edge_chain(edge);
        };
        let chains = topology.wire_chains(wire)?;
        let run = (chains.into_iter())
            .find(|chain| chain.iter().any(|&(e, _)| e == edge))
            .ok_or_else(|| GeopError::new(format!("edge {name:?} is in no run of its wire")))?;
        let mut chain = Chain {
            name: name.to_string(),
            curves: Vec::new(),
            curve_names: Vec::new(),
            joint_names: Vec::new(),
        };
        let mut end = None;
        for &(e, sense) in &run {
            let e_name = self.name(e)?.to_string();
            let e = topology.get_edge(e)?;
            let (start, stop, curve) = match sense {
                Sense::Forward => (e.start_vertex, e.end_vertex, e.curve.clone()),
                Sense::Reversed => (e.end_vertex, e.start_vertex, e.curve.reverse()),
            };
            chain.curves.push(curve);
            chain.curve_names.push(e_name);
            chain.joint_names.push(self.name(start)?.to_string());
            end = Some(stop);
        }
        let first = topology.get_edge(run[0].0)?;
        let start = match run[0].1 {
            Sense::Forward => first.start_vertex,
            Sense::Reversed => first.end_vertex,
        };
        // An open run has a joint at its end too; a closed one ends where
        // it starts.
        match end {
            Some(end) if end != start => chain.joint_names.push(self.name(end)?.to_string()),
            _ => {}
        }
        Ok(chain)
    }
}

impl HasElements<Body> for NamedModel {
    type Element<'a> = TopologyBody;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<TopologyBody> {
        self.body(name)
    }
}

/// Geometric kinds picked from what the editor draws of the model where
/// that has their shape: the hit, if it resolves as one.
macro_rules! picked_where_it_has_the_shape {
    ($($kind:ty => [$($from:ty),+];)*) => {$(
        impl SelectableElement<NamedModel> for $kind {
            const PICKED_FROM: &'static [&'static str] = &[$(<$from as Kind>::NAME),+];

            fn select(
                reader: &Reader,
                hit: &PickedElement,
            ) -> Option<ElementReference<NamedModel, Self>> {
                (hit.target == *model().location()
                    && <Self as SelectableElement<NamedModel>>::PICKED_FROM
                        .contains(&hit.kind.as_str()))
                    .then_some(())?;
                let reference = ElementReference::new(model(), &hit.name);
                reader.retrieve_element(&reference).ok()?;
                Some(reference)
            }

            fn drawn(
                reader: &Reader,
                element: &ElementReference<NamedModel, Self>,
            ) -> Vec<PickedElement> {
                (read_model(reader).ok())
                    .map(|model| drawn_entity(&model, element.name()))
                    .unwrap_or_default()
            }
        }
    )*};
}

picked_where_it_has_the_shape! {
    Point => [Vertex];
    Line => [Edge];
    Plane => [Face];
    Round => [Edge, Face];
    Circle => [Edge];
    Curve => [Edge];
    Path => [Edge];
}

/// A body is picked by a face of it: a solid's face selects the solid, a
/// sheet's face itself; it is lit as the solid's faces, or the face.
impl SelectableElement<NamedModel> for Body {
    const PICKED_FROM: &'static [&'static str] = &[Face::NAME];

    fn select(reader: &Reader, hit: &PickedElement) -> Option<ElementReference<NamedModel, Self>> {
        let face = ElementReference::<NamedModel, Face>::from_picked(hit)?;
        let face = reader.retrieve_element(&face).ok()?;
        let name = match face.model.topology().body_of_face(face.id).ok()? {
            TopologyBody::Solid(solid) => face.model.name_of(solid)?,
            TopologyBody::Sheet(_) => &hit.name,
        };
        Some(ElementReference::new(model(), name))
    }

    fn drawn(reader: &Reader, body: &ElementReference<NamedModel, Self>) -> Vec<PickedElement> {
        (read_model(reader).ok())
            .map(|model| drawn_entity(&model, body.name()))
            .unwrap_or_default()
    }
}

/// The bodies `bodies` refer to, each once, in the order picked. None
/// picked is refused.
pub fn resolve_bodies(
    reader: &Reader,
    bodies: &[ElementReference<NamedModel, Body>],
) -> GeopResult<Vec<TopologyBody>> {
    if bodies.is_empty() {
        return Err(GeopError::new("pick the bodies first"));
    }
    let mut resolved = Vec::new();
    for body in bodies {
        let body = reader.retrieve_element(body)?;
        if !resolved.contains(&body) {
            resolved.push(body);
        }
    }
    Ok(resolved)
}

#[cfg(test)]
mod tests {
    use geop_core_editor::selectable::{Selectable, selected};
    use geop_core_math::scalars::Scalar;
    use geop_core_program::part::Part;

    use super::*;
    use crate::testing::{given_part, unit_cube};

    fn hit(kind: &str, name: &str) -> PickedElement {
        PickedElement {
            target: model().location().clone(),
            kind: kind.to_string(),
            name: name.to_string(),
        }
    }

    /// What a cube's entities are used as: its vertices points, its edges
    /// lines and curves, its faces planes; nothing of it is round.
    #[test]
    fn a_cube_has_points_lines_and_planes() {
        let part: Part = given_part(&unit_cube()).unwrap();
        let reader = part.current();
        let point = ElementReference::<NamedModel, Point>::new(model(), "v0");
        let at = reader.retrieve_element(&point).unwrap();
        assert!(
            at.to_array()
                .iter()
                .all(|c| c.to_f64() == 0.0 || c.to_f64() == 1.0)
        );

        let line = ElementReference::<NamedModel, Line>::new(model(), "e0");
        let axis = reader.retrieve_element(&line).unwrap();
        assert!(axis.direction.norm().definitely_greater(Scalar::ZERO));
        let plane = ElementReference::<NamedModel, Plane>::new(model(), "f0");
        let frame = reader.retrieve_element(&plane).unwrap();
        assert!(frame.w().norm().could_be_equal(Scalar::ONE));
        let curve = ElementReference::<NamedModel, Curve>::new(model(), "e0");
        let chain = reader.retrieve_element(&curve).unwrap();
        assert_eq!(chain.curve_names, ["e0"]);
        assert_eq!(chain.joint_names.len(), 2);

        let round = ElementReference::<NamedModel, Round>::new(model(), "f0");
        assert!(reader.retrieve_element(&round).is_err());
        let wrong = ElementReference::<NamedModel, Line>::new(model(), "f0");
        assert!(reader.retrieve_element(&wrong).is_err());
    }

    /// A line is picked by a straight edge, held by the edge's name and lit
    /// as the edge; a click on what has not the shape selects nothing.
    #[test]
    fn a_geometric_kind_is_picked_where_the_model_has_its_shape() {
        let part: Part = given_part(&unit_cube()).unwrap();
        let reader = part.current();
        let lines: Vec<ElementReference<NamedModel, Line>> =
            selected(&reader, &[hit("edge", "e3"), hit("face", "f0")]);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].name(), "e3");
        assert_eq!(lines[0].drawn(&reader), [hit("edge", "e3")]);
        let rounds: Vec<ElementReference<NamedModel, Round>> =
            selected(&reader, &[hit("edge", "e3"), hit("face", "f0")]);
        assert!(rounds.is_empty());
        assert_eq!(
            <ElementReference<NamedModel, Round>>::picked_from(),
            ["edge", "face"]
        );
    }

    /// A face of a solid picks the solid as a body, lit as its faces; the
    /// bodies picked resolve each once.
    #[test]
    fn a_body_is_picked_by_a_face_of_it() {
        let part: Part = given_part(&unit_cube()).unwrap();
        let reader = part.current();
        let bodies: Vec<ElementReference<NamedModel, Body>> =
            selected(&reader, &[hit("face", "f2")]);
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies[0].name(), "s0");
        assert_eq!(bodies[0].drawn(&reader).len(), 6);
        let twice = [bodies[0].clone(), ElementReference::new(model(), "f1")];
        assert_eq!(resolve_bodies(&reader, &twice).unwrap().len(), 1);
        assert!(resolve_bodies(&reader, &[]).is_err());
    }
}
