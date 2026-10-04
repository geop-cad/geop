//! Blending the edges of basic solids.

use geop_core_math::{
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::blend::{BlendShape, blend};

fn v<T: Scalar>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::from_array([T::from_f64(x), T::from_f64(y), T::from_f64(z)])
}

/// The unit cube `[0, 1]^3`, named `cube(b)`.
fn unit_cube() -> Part<S> {
    let mut part = Part::new();
    cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    part
}

/// A cylinder of radius 0.5 and height 1 standing on the origin, named
/// `cylinder(c)`.
fn cylinder() -> Part<S> {
    let mut part = Part::new();
    revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.0), S::from_f64(0.5), S::ONE).unwrap();
    part
}

/// Blends `edges` of `part` into `shape` as the step `F`, and checks the
/// result is a valid solid named `fillet(F)`.
fn blended(mut part: Part<S>, edges: &[&str], shape: BlendShape) -> Part<S> {
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    let namer = Namer::new("fillet", "F").unwrap();
    blend(&mut part, &namer, &edges, &shape).unwrap();
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
    assert_eq!(part.solid_names(), vec!["fillet(F)".to_string()]);
    part
}

/// Checks `part` has a face named `name`, saying which it has if not.
fn assert_has_face(part: &Part<S>, name: &str) {
    let faces: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .collect();
    assert!(
        part.face_id(name).is_ok(),
        "no face {name:?}, only {faces:?}"
    );
}

/// Whether `part` has a vertex that could be at `p`.
fn has_vertex(part: &Part<S>, p: [f64; 3]) -> bool {
    let p = v::<S>(p[0], p[1], p[2]);
    part.topology()
        .vertices
        .values()
        .any(|vertex| vertex.point.could_be_equal(&p))
}

/// One vertical edge of the cube rounded: one round face more, touching
/// the two sides `0.2` from the edge, which is gone.
#[test]
fn cube_fillet_one_edge() {
    let part = blended(
        unit_cube(),
        &["cube(b,p0)"],
        BlendShape::round(0.2),
    );
    assert_eq!(part.topology().faces.len(), 7);
    assert_has_face(&part, "fillet(F,cube(b,p0),fillet)");
    for p in [
        [0.2, 0.0, 0.0],
        [0.0, 0.2, 0.0],
        [0.2, 0.0, 1.0],
        [0.0, 0.2, 1.0],
    ] {
        assert!(has_vertex(&part, p), "no vertex at {p:?}");
    }
    assert!(!has_vertex(&part, [0.0, 0.0, 0.0]));
}

/// One edge of the cube's top, along `x` at `y = 0`, bevelled alike and
/// with two distances: the first into the face on the edge's left — the
/// top, for this edge — and the second into the side.
#[test]
fn cube_chamfer_one_edge() {
    for [top, side] in [[0.2, 0.2], [0.1, 0.3]] {
        let part = blended(
            unit_cube(),
            &["cube(b,c0,start)"],
            BlendShape::Chamfer {
                distances: [top, side],
            },
        );
        assert_eq!(part.topology().faces.len(), 7);
        assert_has_face(&part, "fillet(F,cube(b,c0,start),chamfer)");
        for x in [0.0, 1.0] {
            for p in [[x, top, 1.0], [x, 0.0, 1.0 - side]] {
                assert!(has_vertex(&part, p), "no vertex at {p:?}");
            }
        }
    }
}

/// Two parallel edges of the cube, rounded one after the other.
#[test]
fn cube_fillet_two_parallel_edges() {
    let part = blended(
        unit_cube(),
        &["cube(b,p0)", "cube(b,p2)"],
        BlendShape::round(0.3),
    );
    assert_eq!(part.topology().faces.len(), 8);
}

/// The cylinder's top rim rounded: picking one quarter arc blends the whole
/// circle, and picking two of them does too, once.
#[test]
fn cylinder_rim_fillet() {
    for edges in [
        vec!["cylinder(c,p1,q0)"],
        vec!["cylinder(c,p1,q0)", "cylinder(c,p1,q2)"],
    ] {
        let part = blended(cylinder(), &edges, BlendShape::round(0.1));
        assert_has_face(&part, "fillet(F,cylinder(c,p1,q0),fillet,q0)");
        let on_circle = |r: f64, z: f64| {
            part.topology().vertices.values().any(|vertex| {
                let p = vertex.point;
                p[2].could_be_equal(S::from_f64(z))
                    && p[0]
                        .mul(p[0])
                        .add(p[1].mul(p[1]))
                        .could_be_equal(S::from_f64(r * r))
            })
        };
        assert!(on_circle(0.4, 1.0) && on_circle(0.5, 0.9));
        assert!(!on_circle(0.5, 1.0));
    }
}

/// Both rims of the cylinder rounded, one after the other.
#[test]
fn cylinder_both_rims_fillet() {
    blended(
        cylinder(),
        &["cylinder(c,p1,q0)", "cylinder(c,p2,q0)"],
        BlendShape::round(0.2),
    );
}

/// The four edges around the cube's top rounded: each pair meets at a
/// corner, where the second blend crosses the first.
#[test]
fn cube_fillet_top_edges() {
    blended(
        unit_cube(),
        &[
            "cube(b,c0,start)",
            "cube(b,c1,start)",
            "cube(b,c2,start)",
            "cube(b,c3,start)",
        ],
        BlendShape::round(0.2),
    );
}

/// Three edges meeting at one corner of the cube bevelled.
#[test]
fn cube_chamfer_corner_edges() {
    blended(
        unit_cube(),
        &["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"],
        BlendShape::Chamfer {
            distances: [0.2, 0.2],
        },
    );
}

/// The cylinder's bottom rim bevelled.
#[test]
fn cylinder_rim_chamfer() {
    blended(
        cylinder(),
        &["cylinder(c,p2,q1)"],
        BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
}

/// Why blending `edges` of `part` into `shape` is refused, which it has to
/// be.
fn refusal(mut part: Part<S>, edges: &[&str], shape: BlendShape) -> String {
    let namer = Namer::new("fillet", "F").unwrap();
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    match blend(&mut part, &namer, &edges, &shape) {
        Ok(()) => panic!("blending {edges:?} into {shape:?} is not refused"),
        Err(e) => format!("{e:?}"),
    }
}

/// What is refused: a radius that is not positive, no edges, an edge that
/// is not there, and blends too large for the faces they run into — past
/// their boundary, or ending exactly on the cube's opposite edge.
#[test]
fn refuses_what_it_cannot_blend() {
    let round = |radius| BlendShape::round(radius);
    let bevel = |d| BlendShape::Chamfer { distances: [d, d] };
    refusal(unit_cube(), &["cube(b,p0)"], round(0.0));
    refusal(unit_cube(), &[], round(0.1));
    refusal(unit_cube(), &["nothing"], round(0.1));
    let past = refusal(unit_cube(), &["cube(b,p0)"], round(1.5));
    assert!(past.contains("too large"), "{past}");
    let on_edge = refusal(unit_cube(), &["cube(b,p0)"], bevel(1.0));
    assert!(on_edge.contains("exactly on another edge"), "{on_edge}");
    let wide = refusal(cylinder(), &["cylinder(c,p1,q0)"], round(0.6));
    assert!(wide.contains("too large"), "{wide}");
}
