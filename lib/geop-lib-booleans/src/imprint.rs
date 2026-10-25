//! Imprinting sheets onto one face: dividing the face along the curves where
//! the sheets cross it, and nowhere else.
//!
//! [`remesh`] imprints whole bodies onto each other, so a sheet remeshed
//! with the face's solid would be imprinted on every face of the solid it
//! crosses. The crossings with the one face are therefore found on a copy of
//! that face standing on its own: remeshed with the copy, each sheet carries
//! an edge along every curve where it crosses the face, shared with the
//! copy. The copy then goes, and those edges are imprinted onto the face
//! itself the way a remesh imprints an edge lying within a face: the face's
//! boundary edges are split where the curves end on them, and each curve is
//! spliced into the piece of the face it lies in.

use std::collections::{HashMap, HashSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    vector::Vector2,
    with_context,
};
use geop_core_topology::{
    Body, EdgeId, FaceId, Model, ShellId, VertexId,
    contains::face::{PointClassification, face_contains},
};
use geop_ops_model::{NamedModel, Namer};

use crate::{
    naming::BooleanNaming,
    remesh::{
        RemeshParams, remesh, remesh_vertices::remesh_vertices,
        remesh_vertices_x_edges::remesh_vertices_x_edges,
    },
};

/// Fixed PRNG seed for `face_contains`' ray casting, whose answer does not
/// depend on it: a constant keeps the imprint reproducible.
const FACE_CONTAINS_SEED: u64 = 0x5DEE_CE66_D1CE_4E5B;

/// Every edge the faces of `body` run along.
fn body_edges(model: &Model, body: Body) -> GeopResult<HashSet<EdgeId>> {
    let mut edges = HashSet::new();
    for face in model.body_faces(body)? {
        edges.extend(model.face_edges(face));
    }
    Ok(edges)
}

/// Divides `face` along the curves where the sheets `sheets` cross it, and
/// returns the edges imprinted there. The sheets are consumed; nothing else
/// of the face's body changes but the boundary edges of the face being
/// split where a curve ends on them.
///
/// Named by `namer` as a boolean names what it creates (see
/// `naming`): the copy of the face's entity `X` is `N(copy,X)`, a
/// curve is named after the copy's face and the sheet's face it is traced
/// along, and the face's pieces and its boundary edges' pieces after the
/// face, the edge and the curves.
///
/// Fails, leaving the part with the face's copy and the sheets in it, if no
/// sheet crosses the face, or a curve ends inside it: a curve has to run
/// from boundary to boundary of the face, or close on itself, to divide it.
pub fn imprint_on_face(
    model: &mut NamedModel,
    namer: &Namer,
    face: FaceId,
    sheets: &[ShellId],
    params: RemeshParams,
) -> GeopResult<Vec<EdgeId>> {
    let ctx = with_context!(
        "imprint_on_face(name={}, face={face}, sheets={sheets:?})",
        namer.root()
    );
    let body = model.topology().body_of_face(face).with_context(&ctx)?;
    let sheets: Vec<Body> = sheets.iter().map(|&s| Body::Sheet(s)).collect();

    // The face on its own, crossed by every sheet.
    let copy = model
        .copy_faces(&[face], None, |name| namer.name(&["copy", name]))
        .and_then(|built| built.body())
        .with_context(&ctx)?;
    for &sheet in &sheets {
        remesh(model, namer, copy, sheet, params).with_context(&ctx)?;
    }

    // The curves: the edges the copy now shares with a sheet, each with
    // where on the face it runs — its pcurve's middle, on the copy, whose
    // surface is the face's. In the order of their names, which does not
    // depend on how they were found.
    let topology = model.topology();
    let on_copy = body_edges(topology, copy).with_context(&ctx)?;
    let mut on_sheets = HashSet::new();
    for &sheet in &sheets {
        on_sheets.extend(body_edges(topology, sheet).with_context(&ctx)?);
    }
    let mut curves: Vec<(String, EdgeId, Vector2)> = Vec::new();
    for &edge in on_copy.intersection(&on_sheets) {
        let coedge = topology
            .coedges_of_edge(edge)
            .into_iter()
            .find(|&c| {
                topology
                    .get_coedge(c)
                    .is_ok_and(|c| topology.body_of_face(c.face).is_ok_and(|b| b == copy))
            })
            .ok_or_else(|| {
                ctx(GeopError::new(format!(
                    "imprint: edge {edge} is on no face of the copy"
                )))
            })?;
        let pcurve = &topology.get_coedge(coedge)?.pcurve;
        let middle = pcurve.evaluate(pcurve.mid_parameter()?)?;
        curves.push((model.name(edge).map(str::to_string)?, edge, middle));
    }
    curves.sort_by(|a, b| a.0.cmp(&b.0));
    if curves.is_empty() {
        return Err(ctx(GeopError::new(format!(
            "imprint: nothing crosses the face {:?}",
            model.name(face).map(str::to_string)?
        ))));
    }

    // A curve ending inside the face divides nothing: each of its ends has
    // to be on the face's boundary — an edge of the copy no sheet shares —
    // or where another curve goes on.
    let boundary: HashSet<VertexId> = on_copy
        .difference(&on_sheets)
        .map(|&e| topology.get_edge(e).map(|e| [e.start_vertex, e.end_vertex]))
        .collect::<GeopResult<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect();
    let mut ends: HashMap<VertexId, usize> = HashMap::new();
    for (_, edge, _) in &curves {
        let edge = topology.get_edge(*edge)?;
        *ends.entry(edge.start_vertex).or_default() += 1;
        *ends.entry(edge.end_vertex).or_default() += 1;
    }
    let mut loose: Vec<String> = ends
        .iter()
        .filter(|&(v, &count)| count == 1 && !boundary.contains(v))
        .map(|(&v, _)| model.name(v).map(str::to_string))
        .collect::<GeopResult<_>>()?;
    if !loose.is_empty() {
        loose.sort();
        return Err(ctx(GeopError::new(format!(
            "imprint: the curves end inside the face {:?}, at {loose:?}; to divide it, they have to cross it or close on themselves",
            model.name(face).map(str::to_string)?
        ))));
    }

    model.delete_bodies(&[copy]).with_context(&ctx)?;

    // The curves onto the face itself: its boundary split where they end on
    // it, each spliced into the piece it lies in.
    let mut bodies = vec![body];
    bodies.extend(&sheets);
    let mut naming = BooleanNaming::new(model, namer, &bodies).with_context(&ctx)?;
    for &sheet in &sheets {
        remesh_vertices(model, body, sheet).with_context(&ctx)?;
        remesh_vertices_x_edges(model, &mut naming, body, sheet, &params).with_context(&ctx)?;
    }
    let mut pieces = vec![face];
    for (curve_name, edge, middle) in &curves {
        let mut target = None;
        for &piece in &pieces {
            let class = face_contains(
                model.topology(),
                piece,
                middle[0],
                middle[1],
                params.max_nodes,
                params.min_subdivision_size,
                FACE_CONTAINS_SEED,
            )
            .with_context(&ctx)?;
            if class == PointClassification::Inside {
                target = Some(piece);
                break;
            }
        }
        let target = target.ok_or_else(|| {
            ctx(GeopError::new(format!(
                "imprint: the curve {curve_name:?}, at {middle:?} on the face, lies inside none of its pieces"
            )))
        })?;
        let split = model
            .splice_edge_into_face(
                *edge,
                target,
                params.max_nodes,
                params.min_subdivision_size,
                naming.provisional(),
            )
            .with_context(&ctx)?;
        naming.face_split(target, *edge, split)?;
        pieces.extend(split);
    }
    naming.finish(model).with_context(&ctx)?;
    model.delete_bodies(&sheets).with_context(&ctx)?;
    Ok(curves.into_iter().map(|(_, edge, _)| edge).collect())
}

#[cfg(test)]
mod tests {
    use super::imprint_on_face;
    use crate::remesh::RemeshParams;
    use geop_core_math::{
        geop_error::GeopResult, primitives::CoordinateSystem, scalars::Scalar, vector::Vector2,
    };
    use geop_core_topology::{EdgeId, SolidId};
    use geop_lib_sweep::{
        SweepLoop, extrude, polygon, polyline,
        shapes::{Axis, block, revolved_cylinder_along_axis, v3},
    };
    use geop_ops_model::Profile;
    use geop_ops_model::{NamedModel, Namer, testing::assert_valid};

    fn v2s(points: &[[f64; 2]]) -> Vec<Vector2> {
        points
            .iter()
            .map(|p| Vector2::from_array([Scalar::from_f64(p[0]), Scalar::from_f64(p[1])]))
            .collect()
    }

    /// Imprints onto the face named `target` of the solid `build` makes
    /// the profile `profile`, drawn in the plane `z = 2` and swept down to
    /// `z = -3`, through the whole solid.
    fn imprint(
        build: impl Fn(&mut NamedModel) -> SolidId,
        target: &str,
        profile: Profile,
    ) -> GeopResult<(NamedModel, Vec<EdgeId>)> {
        let mut model = NamedModel::new();
        build(&mut model);
        let target = model.face_id(target)?;
        let plane = CoordinateSystem::world_at(v3(0.0, 0.0, 2.0));
        let namer = Namer::new("project", "p")?;
        let swept = extrude(
            &mut model,
            &namer,
            None,
            &plane,
            Scalar::from_f64(-5.0),
            Scalar::from_f64(1.0),
            &[SweepLoop::plain(profile)],
        )?;
        let edges = imprint_on_face(
            &mut model,
            &namer,
            target,
            &swept.shells,
            RemeshParams::default(),
        )?;
        Ok((model, edges))
    }

    fn cube(model: &mut NamedModel) -> SolidId {
        block(model, "c", [0.0; 3], [1.0; 3])
    }

    /// The unit cube's top face, at `z = 1`.
    const TOP: &str = "cube(c,start)";

    fn chain(points: &[[f64; 2]]) -> Profile {
        Profile::open(polyline(&v2s(points)).unwrap())
    }

    /// A line across the top face of a cube divides that face in two, and
    /// only that face: the bottom face, which the sweep crosses too, stays
    /// whole.
    #[test]
    fn a_line_across_a_face_divides_it_in_two() {
        let (model, edges) = imprint(cube, TOP, chain(&[[0.3, -1.0], [0.3, 2.0]])).unwrap();
        assert_valid(&model);
        assert_eq!(edges.len(), 1);
        assert_eq!(model.topology().solids.len(), 1);
        assert!(model.sheet_face_names().is_empty(), "the sweep is gone");
        // Six faces and one more; twelve edges, two of them split, and the
        // one imprinted.
        assert_eq!(model.topology().faces.len(), 7);
        assert_eq!(model.topology().edges.len(), 15);
        assert_eq!(model.topology().vertices.len(), 10);
    }

    /// A bent chain whose corner lies on the face divides it along both
    /// of its legs.
    #[test]
    fn a_bent_chain_divides_a_face_along_both_legs() {
        let (model, edges) =
            imprint(cube, TOP, chain(&[[0.3, -1.0], [0.5, 0.5], [2.0, 0.6]])).unwrap();
        assert_valid(&model);
        assert_eq!(edges.len(), 2);
        assert_eq!(model.topology().faces.len(), 7);
    }

    /// A square inside the face closes on itself: it bounds a new face, a
    /// hole in what is left of the old one.
    #[test]
    fn a_closed_loop_inside_a_face_cuts_out_a_face() {
        let square = polygon(&v2s(&[[0.2, 0.2], [0.6, 0.2], [0.6, 0.6], [0.2, 0.6]])).unwrap();
        let (model, edges) = imprint(cube, TOP, Profile::closed(square)).unwrap();
        assert_valid(&model);
        assert_eq!(edges.len(), 4);
        assert_eq!(model.topology().faces.len(), 7);
        let top = model
            .topology()
            .get_face(model.face_id(TOP).unwrap())
            .unwrap();
        assert_eq!(top.holes.len(), 1);
    }

    /// A line drawn slanting across a cylinder lying along `x`, seen from
    /// above, projects onto the quarter of its side facing up and towards
    /// `+y` as an arc of an ellipse, from where that quarter meets the next
    /// one to the cylinder's end: it divides that quarter in two, splitting
    /// the edge between the quarters where it ends on it.
    #[test]
    fn a_slanted_line_divides_a_round_face_along_an_ellipse() {
        let cylinder = |model: &mut NamedModel| {
            revolved_cylinder_along_axis(
                model,
                "y",
                v3(-0.5, 0.0, 0.0),
                Scalar::from_f64(0.5),
                Scalar::ONE,
                Axis::X,
            )
            .unwrap()
        };
        let (model, edges) = imprint(
            cylinder,
            "cylinder(y,c1,q0)",
            chain(&[[-1.0, -0.3], [1.0, 0.3]]),
        )
        .unwrap();
        assert_valid(&model);
        assert_eq!(edges.len(), 1);
        // Four quarters of the side and four wedges in each cap, and the
        // piece split off.
        assert_eq!(model.topology().faces.len(), 13);
    }

    /// A line ending inside the face divides nothing, and is refused.
    #[test]
    fn a_line_ending_inside_the_face_is_refused() {
        let error = imprint(cube, TOP, chain(&[[0.3, -1.0], [0.3, 0.5]])).expect_err("refused");
        assert!(error.root_message().contains("end inside"), "{error:?}");
    }

    /// A line beside the face misses it, and is refused.
    #[test]
    fn a_line_beside_the_face_is_refused() {
        let error = imprint(cube, TOP, chain(&[[3.0, -1.0], [3.0, 2.0]])).expect_err("refused");
        assert!(
            error.root_message().contains("nothing crosses"),
            "{error:?}"
        );
    }
}
