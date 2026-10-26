//! Section curves: where two bodies' boundaries cross, or a body's crosses
//! a plane, as a wire of edges standing on their own.
//!
//! Nothing new finds them. A copy of each body is imprinted onto the other
//! exactly as a boolean imprints two solids ([`remesh`]), after which every
//! curve where their boundaries cross is an edge both copies share. Those
//! edges, with the vertices they run between, become the wire; the copies
//! go, and the bodies are left as they are.

use std::collections::HashMap;

use geop_core_geometry::{aabb::union_aabb, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_topology::{
    Body, VertexId, WireId,
    build::{BodySpec, EdgeSpec},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

use crate::{
    imprint::shared_edges,
    remesh::{RemeshParams, remesh},
};

/// The curves where the boundaries of the bodies `a` and `b` cross, as a
/// wire, both bodies left as they are. Where a face of one lies on a face
/// of the other, the outline of what they share is among the curves.
///
/// Each body is imprinted as a copy, the copy of the entity `X` named
/// `N(X)`; a curve, and a vertex it ends at, is then named as a boolean of
/// the copies names it (see `naming`), and the wire's edges and vertices
/// keep those names.
///
/// Fails, leaving the model as it was, if the bodies' boundaries do not
/// meet: there is nothing to make a wire of.
pub fn section(
    model: &mut NamedModel,
    namer: &Namer,
    a: Body,
    b: Body,
    params: RemeshParams,
) -> GeopResult<WireId> {
    let before = model.clone();
    section_of(model, namer, a, b, params).inspect_err(|_| *model = before)
}

/// [`section`], leaving what it got to in the model where it fails.
fn section_of(
    model: &mut NamedModel,
    namer: &Namer,
    a: Body,
    b: Body,
    params: RemeshParams,
) -> GeopResult<WireId> {
    let ctx = with_context!("section(name={}, a={a}, b={b})", namer.root());
    let copies = [a, b]
        .map(|body| {
            model
                .copy_body(body, |name| namer.name(&[name]))
                .and_then(|built| built.body())
        })
        .into_iter()
        .collect::<GeopResult<Vec<Body>>>()
        .with_context(&ctx)?;
    let [copy_a, copy_b] = copies[..] else {
        unreachable!("two bodies copied")
    };
    remesh(model, namer, copy_a, copy_b, params).with_context(&ctx)?;
    let curves = shared_edges(model, copy_a, copy_b).with_context(&ctx)?;
    if curves.is_empty() {
        return Err(ctx(GeopError::new(
            "section: the bodies' boundaries do not meet",
        )));
    }

    // The wire: every curve, between the vertices it ends at.
    let topology = model.topology();
    let mut spec = BodySpec::new(false);
    let mut names = BodyNames::default();
    let mut index: HashMap<VertexId, usize> = HashMap::new();
    let mut vertex = |spec: &mut BodySpec, names: &mut BodyNames, v: VertexId| {
        Ok::<_, GeopError>(match index.get(&v) {
            Some(&i) => i,
            None => {
                spec.vertices.push(topology.get_vertex(v)?.point);
                names.vertices.push(model.name(v)?.to_string());
                index.insert(v, spec.vertices.len() - 1);
                spec.vertices.len() - 1
            }
        })
    };
    for &edge in &curves {
        let e = topology.get_edge(edge)?;
        let start = vertex(&mut spec, &mut names, e.start_vertex)?;
        let end = vertex(&mut spec, &mut names, e.end_vertex)?;
        spec.edges.push(EdgeSpec {
            curve: e.curve.clone(),
            start,
            end,
        });
        names.edges.push(model.name(edge)?.to_string());
    }
    model.delete_bodies(&[copy_a, copy_b]).with_context(&ctx)?;
    let built = model.build_body(spec, names).with_context(&ctx)?;
    built
        .wire
        .ok_or_else(|| ctx(GeopError::new("section: the curves built no wire")))
}

/// The curves where the boundary of the body `body` crosses the plane
/// `plane` (through its origin, along its `u` and `v`), as a wire, the body
/// left as it is: [`section`] of the body and a square of the plane reaching
/// past the whole body. The square's corners and sides are named
/// `N(plane,v0)` ... and `N(plane,e0)` ..., its face `N(plane)`. Fails, as
/// [`section`] does, leaving the model as it was.
pub fn section_by_plane(
    model: &mut NamedModel,
    namer: &Namer,
    body: Body,
    plane: &CoordinateSystem,
    params: RemeshParams,
) -> GeopResult<WireId> {
    let ctx = with_context!("section_by_plane(name={}, body={body})", namer.root());
    let topology = model.topology();
    let faces = topology.body_faces(body).with_context(&ctx)?;
    let bounds = union_aabb(faces.iter().map(|&f| topology.faces[&f].surface.aabb()))
        .ok_or_else(|| ctx(GeopError::new("section: the body has no face")))?;
    // The square, centred where the body's box's centre falls on the plane,
    // reaching past the box's diagonal each way: a free choice, as long as
    // the square's own sides are clear of the body.
    let center = geop_core_math::vector::Vector3::from_array(bounds.map(|x| x.midpoint()));
    let diagonal = bounds
        .iter()
        .map(|x| x.width().mul(x.width()))
        .fold(Scalar::ZERO, |sum, w| sum.add(w))
        .sqrt()?;
    let reach = Scalar::from_f64((diagonal.upper().to_f64() + 1.0).ceil());
    let middle = plane.to_uv(&center);
    let (ru, rv) = (
        reach.div(plane.u().norm())?.sharpen(),
        reach.div(plane.v().norm())?.sharpen(),
    );
    let corner = |su: Scalar, sv: Scalar| {
        plane.uv_to_xyz(&Vector2::from_array([
            middle[0].sharpen().add(su.mul(ru)),
            middle[1].sharpen().add(sv.mul(rv)),
        ]))
    };
    let (minus, plus) = (Scalar::ONE.neg(), Scalar::ONE);
    let square = NurbSurface3D::bilinear(
        corner(minus, minus),
        corner(plus, minus),
        corner(plus, plus),
        corner(minus, plus),
    )
    .with_context(&ctx)?;
    let spec = BodySpec::sheet_on(square).with_context(&ctx)?;
    let names = BodyNames {
        vertices: (0..4)
            .map(|k| namer.name(&["plane", &format!("v{k}")]))
            .collect(),
        edges: (0..4)
            .map(|k| namer.name(&["plane", &format!("e{k}")]))
            .collect(),
        faces: vec![namer.name(&["plane"])],
        solid: None,
    };
    let sheet = model
        .build_body(spec, names)
        .and_then(|built| built.body())
        .with_context(&ctx)?;
    let wire = section(model, namer, body, sheet, params).with_context(&ctx);
    model.delete_bodies(&[sheet]).with_context(&ctx)?;
    wire
}

#[cfg(test)]
mod tests {
    use super::{section, section_by_plane};
    use crate::remesh::RemeshParams;
    use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar};
    use geop_core_topology::{Body, Sense};
    use geop_lib_sweep::shapes::{Axis, block, revolved_cylinder_along_axis, v3};
    use geop_ops_model::{NamedModel, Namer, testing::assert_valid};

    /// Two unit cubes, the second moved by half along each axis, cross in
    /// one loop: six edges, three across each cube's faces, joined at the
    /// six points where an edge of one pierces a face of the other. Both
    /// cubes stay as they were.
    #[test]
    fn two_overlapping_cubes_cross_in_one_loop() {
        let mut model = NamedModel::new();
        let a = block(&mut model, "a", [0.0; 3], [1.0; 3]);
        let b = block(&mut model, "b", [0.5; 3], [1.5; 3]);
        let (faces, edges) = (model.topology().faces.len(), model.topology().edges.len());
        let namer = Namer::new("section", "s").unwrap();
        let wire = section(
            &mut model,
            &namer,
            a.into(),
            b.into(),
            RemeshParams::default(),
        )
        .unwrap();
        assert_valid(&model);
        assert_eq!(
            model.topology().faces.len(),
            faces,
            "the cubes as they were"
        );
        assert_eq!(model.topology().edges.len(), edges + 6);
        let chains = model.topology().wire_chains(wire).unwrap();
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].len(), 6);
        let topology = model.topology();
        let end = |&(e, sense): &(_, Sense)| {
            let e = topology.get_edge(e).unwrap();
            match sense {
                Sense::Forward => (e.start_vertex, e.end_vertex),
                Sense::Reversed => (e.end_vertex, e.start_vertex),
            }
        };
        let ends: Vec<_> = chains[0].iter().map(end).collect();
        for k in 0..6 {
            assert_eq!(ends[k].1, ends[(k + 1) % 6].0, "a closed chain");
        }
    }

    /// A plane across a cylinder's axis crosses its side in a circle, one
    /// edge per quarter of the side.
    #[test]
    fn a_plane_across_a_cylinder_cuts_a_circle() {
        let mut model = NamedModel::new();
        let cylinder = revolved_cylinder_along_axis(
            &mut model,
            "c",
            v3(0.0, 0.0, 0.0),
            Scalar::from_f64(0.5),
            Scalar::from_f64(2.0),
            Axis::Z,
        )
        .unwrap();
        let namer = Namer::new("section", "s").unwrap();
        let plane = CoordinateSystem::world_at(v3(0.0, 0.0, 0.7));
        let wire = section_by_plane(
            &mut model,
            &namer,
            Body::Solid(cylinder),
            &plane,
            RemeshParams::default(),
        )
        .unwrap();
        assert_valid(&model);
        assert!(model.sheet_face_names().is_empty(), "the plane is gone");
        let chains = model.topology().wire_chains(wire).unwrap();
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].len(), 4);
        for &(edge, _) in &chains[0] {
            let curve = &model.topology().get_edge(edge).unwrap().curve;
            let mid = curve.evaluate(curve.mid_parameter().unwrap()).unwrap();
            assert!(mid[2].could_be_equal(Scalar::from_f64(0.7)), "{mid:?}");
            let r = mid[0].mul(mid[0]).add(mid[1].mul(mid[1])).sqrt().unwrap();
            assert!((r.to_f64() - 0.5).abs() < 1e-9, "{r:?}");
        }
    }

    /// A plane beside the body crosses nothing, and is refused.
    #[test]
    fn a_plane_beside_the_body_is_refused() {
        let mut model = NamedModel::new();
        let cube = block(&mut model, "a", [0.0; 3], [1.0; 3]);
        let namer = Namer::new("section", "s").unwrap();
        let plane = CoordinateSystem::world_at(v3(0.0, 0.0, 3.0));
        let error = section_by_plane(
            &mut model,
            &namer,
            Body::Solid(cube),
            &plane,
            RemeshParams::default(),
        )
        .unwrap_err();
        assert!(error.root_message().contains("do not meet"), "{error:?}");
        assert!(model.sheet_face_names().is_empty(), "the plane is gone");
    }
}
