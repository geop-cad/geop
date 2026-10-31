//! Picking with a [`PartView`] drawn by hand: what a pointer is over, as
//! drawn. How a crate draws its targets into a view is tested where it
//! draws them.

use std::rc::Rc;

use geop_core_math::{
    primitives::{CoordinateSystem, DatumKind, Pose, Quaternion, Ray},
    scalars::Scalar,
    vector::Vector3,
};

use super::*;
use crate::{
    editing::{Pointer, Reach},
    testing::pointer,
};

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_array([x, y, z].map(Scalar::from_f64))
}

/// A square of `n` by `n` cells in the plane `z = 0`, from the origin, two
/// triangles a cell: one face of `2 n²` triangles.
fn grid(n: usize) -> PartView {
    let mut triangles = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (i as f64, j as f64);
            let [a, b, c, d] = [
                v3(x, y, 0.0),
                v3(x + 1.0, y, 0.0),
                v3(x + 1.0, y + 1.0, 0.0),
                v3(x, y + 1.0, 0.0),
            ];
            triangles.extend([[a, b, c], [a, c, d]]);
        }
    }
    let normals = vec![[v3(0.0, 0.0, 1.0); 3]; triangles.len()];
    let mut view = PartView::new();
    view.model = "model".into();
    view.faces.push(ViewFace {
        name: "grid".into(),
        solid: None,
        feature: None,
        mesh: FaceMesh::new(triangles, normals),
    });
    view.finish()
}

/// A hover tests the triangles of the boxes its ray passes through, not
/// every triangle of the parts under it: a face of 20 000 triangles, and a
/// thousand parts placed from it, are a few dozen triangles to a hover —
/// and the face it hits is the one testing every triangle hits.
#[test]
fn a_hover_tests_the_triangles_near_its_ray() {
    let component = Rc::new(grid(100));
    let tested = |view: &PartView, at: Pointer| {
        let before = triangles_tested();
        let hit = view.pick(&at, &[drawn::FACE.to_string()], &[]);
        (hit, triangles_tested() - before)
    };
    let (hit, alone) = tested(&component, down(37.3, 52.6));
    let hit = hit.expect("the grid is under the pointer");
    assert_eq!(hit.element.name, "grid");
    assert_encloses_z(hit.point, 0.0);
    assert!(alone <= 16, "{alone} triangles tested");
    let (missed, outside) = tested(&component, down(-5.0, -5.0));
    assert!(missed.is_none());
    assert_eq!(outside, 0);

    let mut placed = PartView::new();
    for k in 0..1000 {
        let at = v3((k % 40) as f64 * 120.0, (k / 40) as f64 * 120.0, 0.0);
        placed.instances.push(ViewInstance {
            name: format!("tile{k}"),
            component: "grid".into(),
            frame: CoordinateSystem::world_at(at),
            pose: Pose::identity().with_position(at),
            view: component.clone(),
            state: None,
            fixed: true,
            color: None,
        });
    }
    let placed = placed.finish();
    let (hit, many) = tested(&placed, down(37.3 + 5.0 * 120.0, 52.6 + 3.0 * 120.0));
    assert_eq!(
        hit.expect("a tile is under the pointer")
            .element
            .target
            .path(),
        "tile125"
    );
    assert!(many <= 16, "{many} triangles tested");
}

/// Asserts that `p` could lie in the plane `z = z`.
fn assert_encloses_z(p: Vector3, z: f64) {
    assert!(p[2].could_be_equal(Scalar::from_f64(z)), "{p:?}");
}

fn down(x: f64, y: f64) -> Pointer {
    pointer([x, y, 5.0], [0.0, 0.0, -1.0])
}

/// The name of the unit cube's corner `c`, its coordinates 0 or 1.
fn corner(c: [usize; 3]) -> String {
    format!("v{}{}{}", c[0], c[1], c[2])
}

/// The name of the unit cube's face where coordinate `axis` is `side`.
fn face(axis: usize, side: usize) -> String {
    format!("{}{side}", ["x", "y", "z"][axis])
}

fn point(c: [usize; 3]) -> Vector3 {
    v3(c[0] as f64, c[1] as f64, c[2] as f64)
}

/// The unit cube drawn as the target `model`: its eight corners, twelve
/// edges and six faces, each named by where it is, every vertex and edge
/// naming the faces it bounds — as a model is drawn.
fn cube() -> PartView {
    let mut view = PartView::new();
    view.model = "model".into();
    let corners: Vec<[usize; 3]> = (0..8).map(|i| [i & 1, i >> 1 & 1, i >> 2 & 1]).collect();
    for &c in &corners {
        view.vertices.push(ViewVertex {
            name: corner(c),
            solid: Some("cube".into()),
            faces: (0..3).map(|k| face(k, c[k])).collect(),
            at: point(c),
        });
    }
    for &a in &corners {
        for k in (0..3).filter(|&k| a[k] == 0) {
            let mut b = a;
            b[k] = 1;
            view.edges.push(ViewEdge {
                name: format!("{}-{}", corner(a), corner(b)),
                solid: Some("cube".into()),
                faces: (0..3).filter(|&j| j != k).map(|j| face(j, a[j])).collect(),
                polyline: vec![point(a), point(b)],
                ends: [corner(a), corner(b)],
            });
        }
    }
    for k in 0..3 {
        for side in 0..2 {
            let (u, w) = ((k + 1) % 3, (k + 2) % 3);
            let at = |i: usize, j: usize| {
                let mut c = [side; 3];
                (c[u], c[w]) = (i, j);
                point(c)
            };
            let [a, b, c, d] = [at(0, 0), at(1, 0), at(1, 1), at(0, 1)];
            let mut normal = [0.0; 3];
            normal[k] = if side == 0 { -1.0 } else { 1.0 };
            let normal = v3(normal[0], normal[1], normal[2]);
            view.faces.push(ViewFace {
                name: face(k, side),
                solid: Some("cube".into()),
                feature: None,
                mesh: FaceMesh::new(vec![[a, b, c], [a, c, d]], vec![[normal; 3]; 2]),
            });
        }
    }
    view.solids = vec!["cube".into()];
    view.finish()
}

/// What `pointer` picks of the kinds `accepts`.
fn picked(view: &PartView, pointer: &Pointer, accepts: &[&str]) -> Option<PickedElement> {
    let accepts: Vec<String> = accepts.iter().map(|k| k.to_string()).collect();
    view.pick(pointer, &accepts, &[]).map(|h| h.element)
}

/// The kind of what `pointer` picks of the kinds `accepts`.
fn kind(view: &PartView, pointer: &Pointer, accepts: &[&str]) -> Option<String> {
    picked(view, pointer, accepts).map(|p| p.kind)
}

/// A cube's vertices, edges and faces.
const ANY: &[&str] = &[drawn::VERTEX, drawn::EDGE, drawn::FACE];

/// The smallest entity near the pointer wins — a corner, else an edge,
/// else the face — but never one hidden behind the face in front.
#[test]
fn the_smallest_visible_entity_wins() {
    let view = cube();
    let kind = |pointer: Pointer| kind(&view, &pointer, ANY).unwrap();
    assert_eq!(kind(down(1.005, 1.005)), drawn::VERTEX);
    assert_eq!(kind(down(0.5, 0.995)), drawn::EDGE);
    assert_eq!(kind(down(0.5, 0.5)), drawn::FACE);
    // From the side, through the front face: the back face's edges lie
    // right behind the pointer, and are not picked.
    assert_eq!(
        kind(pointer([0.5, -5.0, 0.995], [0.0, 1.0, 0.0])),
        drawn::EDGE
    );
    assert_eq!(
        kind(pointer([0.5, -5.0, 0.5], [0.0, 1.0, 0.0])),
        drawn::FACE
    );
}

/// Everything along the pointer's ray lies under it: the one a click
/// picks first, then the rest front to back, those hidden behind the
/// first face included — and a pointer stepped behind picks each in turn,
/// wrapping round.
#[test]
fn what_lies_under_the_pointer_is_stepped_through_front_to_back() {
    let view = cube();
    let names = |pointer: Pointer, accepts: &[&str]| -> Vec<String> {
        let accepts: Vec<String> = accepts.iter().map(|k| k.to_string()).collect();
        (view.under(&pointer, &accepts, &[]).into_iter())
            .map(|hit| hit.element.name)
            .collect()
    };
    let (top, bottom) = (face(2, 1), face(2, 0));
    assert_eq!(
        names(down(0.5, 0.5), &[drawn::FACE]),
        [top.clone(), bottom.clone()]
    );
    let face_at = |cycle: usize| picked(&view, &down(0.5, 0.5).cycled(cycle), &[drawn::FACE]);
    assert_eq!(face_at(1).map(|f| f.name), Some(bottom.clone()));
    assert_eq!(face_at(2).map(|f| f.name), Some(top.clone()));
    // Near the top's back edge: the edge, the top, and below them the
    // bottom's edge and the bottom, hidden — as far along the ray as each
    // other.
    let near_edge = names(down(0.5, 0.995), ANY);
    assert_eq!(near_edge[..2], ["v011-v111".to_string(), top]);
    let mut behind = near_edge[2..].to_vec();
    behind.sort();
    assert_eq!(behind, ["v010-v110".to_string(), bottom], "{near_edge:?}");
}

/// Datums are hit as drawn — a plane as a square around the drawing's
/// center — and only for the kinds they can be.
#[test]
fn datums() {
    let mut view = cube();
    view.datums.push(ViewDatum {
        name: "above".into(),
        kind: DatumKind::Plane,
        frame: CoordinateSystem::world_at(v3(0.0, 0.0, 3.0)),
        components: Vec::new(),
    });
    let datum = Some("above".to_string());
    let name =
        |pointer: Pointer, accepts: &[&str]| picked(&view, &pointer, accepts).map(|p| p.name);
    // Beside the cube, but within the square drawn around its center: √3
    // across, the cube's diagonal.
    assert_eq!(name(down(1.2, 0.5), &[drawn::DATUM]), datum);
    assert_eq!(name(down(1.2, 0.5), &[drawn::VERTEX]), None);
    // In front of the cube's top, the plane is nearer.
    assert_eq!(name(down(0.5, 0.5), &[drawn::DATUM, drawn::FACE]), datum);
    assert_eq!(name(down(50.0, 0.5), &[drawn::DATUM]), None);
}

/// Seen at a glancing angle, an edge of the face the ray hits is picked
/// wherever the pointer is within reach of it: aimed at the cube's top 0.1
/// short of its far edge, from nearly level with the top, the ray passes
/// a thousandth from the edge, within its reach of about 0.005 — but meets
/// it 0.1 further along than the top, twenty reaches. The face does not
/// hide its own edge. (The near top edge it passes 0.008 above, out of
/// reach.)
#[test]
fn edge_of_the_face_hit_at_a_glancing_angle() {
    let view = cube();
    let (origin, aim) = ([0.5, -10.0, 1.1], [0.5, 0.9, 1.0]);
    let glancing = Pointer {
        cycle: 0,
        ray: Ray::try_new(
            v3(origin[0], origin[1], origin[2]),
            v3(aim[0] - origin[0], aim[1] - origin[1], aim[2] - origin[2]),
        )
        .unwrap(),
        reach: Reach::Cone {
            slope: Scalar::from_f64(0.0005),
        },
    };
    let edge = picked(&view, &glancing, &[drawn::EDGE]).expect("no edge picked");
    let edge = view.edges.iter().find(|e| e.name == edge.name).unwrap();
    for p in &edge.polyline {
        assert!(
            p[1].could_be_equal(Scalar::ONE) && p[2].could_be_equal(Scalar::ONE),
            "{} is not the far top edge",
            edge.name
        );
    }
}

/// One view placed three times, as instances of one component: each is
/// picked where it is, named behind its instance's name, and one in front
/// hides what is behind it — a corner of another included.
#[test]
fn a_view_placed_three_times_is_picked_where_each_is() {
    let component = Rc::new(cube());
    let place = |name: &str, at: Vector3| ViewInstance {
        name: name.into(),
        component: "cube".into(),
        frame: CoordinateSystem::world_at(at),
        pose: Pose::new(at, Quaternion::identity()).unwrap(),
        view: component.clone(),
        state: None,
        fixed: true,
        color: None,
    };
    let mut view = PartView::new();
    view.instances = vec![
        place("a", v3(0.0, 0.0, 0.0)),
        place("b", v3(3.0, 0.0, 0.0)),
        place("c", v3(0.0, 0.0, 2.0)),
    ];
    let view = view.finish();
    // Framed by the boxes of the cubes where they are: from the origin to
    // (4, 1, 3), not by cubes as wide as their diagonals.
    let size = view.extent.size.to_f64();
    assert!((size - 26f64.sqrt()).abs() < 1e-9, "{size}");
    let hit = |x: f64, y: f64, accepts: &[&str]| picked(&view, &down(x, y), accepts).unwrap();
    let top_of_b = hit(3.5, 0.5, &[drawn::FACE]);
    assert_eq!(top_of_b.target, Located::in_instance("b", "model"));
    assert_eq!(top_of_b.name, face(2, 1));
    // `c` stands above `a`: its top is hit, and its corner, not `a`'s.
    assert_eq!(
        hit(0.5, 0.5, &[drawn::FACE]).target,
        Located::in_instance("c", "model")
    );
    let corner_hit = hit(1.005, 1.005, ANY);
    assert_eq!(
        (
            corner_hit.target.to_string().as_str(),
            corner_hit.name.as_str()
        ),
        ("/c:model", "v111")
    );
    assert_eq!(picked(&view, &down(2.0, 0.5), ANY), None);
}

/// A mesh is sent as a viewer draws it: each corner once, in base64 — as
/// `web/test/kept.test.ts` reads it.
#[test]
fn a_mesh_is_sent_as_its_corners_and_their_indices() {
    let (a, b, c) = (v3(0.0, 0.0, 1.0), v3(1.0, 0.0, 1.0), v3(0.0, 1.0, 1.0));
    let mesh = FaceMesh::new(vec![[a, b, c]], vec![[v3(0.0, 0.0, 1.0); 3]]);
    let sent = serde_json::to_value(&*mesh).unwrap();
    assert_eq!(
        sent,
        serde_json::json!({
            "positions": "AAAAAAAAAAAAAIA/AACAPwAAAAAAAIA/AAAAAAAAgD8AAIA/",
            "normals": "AAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/AAAAAAAAAAAAAIA/",
            "indices": "AAAAAAEAAAACAAAA",
        })
    );
}
