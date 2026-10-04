//! Mass properties, measurements and interference of basic solids, against
//! their closed forms — each within the bounds returned.

use std::f64::consts::PI;

use geop_core_math::{
    primitives::Pose,
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{EntityRef, Part, parameters::Material, parameters::Parameters};
use geop_ops_extrude_revolve::shapes::{
    cube::cube_solid, cylinder::revolved_cylinder, sphere::sphere_solid,
};

use crate::{
    Bounded, interference::Contact, interference_report, mass::MassSummary, mass_report, measure,
};

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// A part of steel, 7850 kg/m³.
fn steel() -> Part<S> {
    Part::new().with_parameters(Parameters {
        material: Some(Material {
            name: "Steel".into(),
            density: 7850.0,
        }),
        ..Parameters::default()
    })
}

/// `bounded` contains `exact`, and is no wider than `relative` of it.
#[track_caller]
fn assert_near(what: &str, bounded: Bounded, exact: f64, relative: f64) {
    assert!(
        bounded.contains(exact),
        "{what}: {bounded:?} does not contain {exact}"
    );
    assert!(
        bounded.error <= relative * exact.abs().max(1.0),
        "{what}: {bounded:?} is wider than {relative} of {exact}"
    );
}

/// The one solid's summary of a report.
fn only(part: &Part<S>) -> MassSummary {
    let report = mass_report(part).unwrap();
    assert_eq!(report.bodies.len(), 1, "{report:?}");
    let body = &report.bodies[0];
    assert!(body.error.is_none(), "{body:?}");
    body.properties.clone().unwrap()
}

/// The inertia tensor about the centre, against a diagonal one.
#[track_caller]
fn assert_diagonal(summary: &MassSummary, diagonal: [f64; 3], relative: f64) {
    let scale = diagonal.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    for a in 0..3 {
        for b in 0..3 {
            let want = if a == b { diagonal[a] } else { 0.0 };
            let got = summary.inertia[a][b];
            assert!(got.contains(want), "I[{a}][{b}] = {got:?}, want {want}");
            assert!(
                got.error <= relative * scale,
                "I[{a}][{b}] = {got:?} is too wide"
            );
        }
    }
}

/// A 2 x 4 x 1 box of steel: volume, area, mass, centre and inertia.
#[test]
fn box_mass_properties() {
    let mut part = steel();
    cube_solid(&mut part, "b", v(1.0, 2.0, 3.0), v(3.0, 6.0, 4.0)).unwrap();
    let summary = only(&part);
    assert!(summary.converged);
    let (a, b, c) = (2.0, 4.0, 1.0);
    let volume = a * b * c;
    let mass = volume * 7850.0e-9;
    assert_near("volume", summary.volume, volume, 1e-9);
    assert_near("area", summary.area, 2.0 * (a * b + b * c + c * a), 1e-9);
    assert_near("mass", summary.mass, mass, 1e-9);
    for (k, want) in [2.0, 4.0, 3.5].into_iter().enumerate() {
        assert_near("centre", summary.center[k], want, 1e-9);
    }
    let i = |p: f64, q: f64| mass * (p * p + q * q) / 12.0;
    assert_diagonal(&summary, [i(b, c), i(a, c), i(a, b)], 1e-9);
    let mut principal = [i(b, c), i(a, c), i(a, b)];
    principal.sort_by(f64::total_cmp);
    for (got, want) in summary.principal_moments.iter().zip(principal) {
        assert_near("principal moment", *got, want, 1e-6);
    }
}

/// A cylinder of radius 1.5 and height 4: rational surfaces, integrated
/// within bounds of the closed forms.
#[test]
fn cylinder_mass_properties() {
    let mut part = steel();
    let (r, h) = (1.5, 4.0);
    revolved_cylinder(
        &mut part,
        "c",
        v(1.0, -1.0, 2.0),
        S::from_f64(r),
        S::from_f64(h),
    )
    .unwrap();
    let summary = only(&part);
    let volume = PI * r * r * h;
    let mass = volume * 7850.0e-9;
    assert_near("volume", summary.volume, volume, 1e-8);
    assert_near("area", summary.area, 2.0 * PI * r * (r + h), 1e-8);
    for (k, want) in [1.0, -1.0, 4.0].into_iter().enumerate() {
        assert_near("centre", summary.center[k], want, 1e-8);
    }
    let across = mass * (3.0 * r * r + h * h) / 12.0;
    assert_diagonal(&summary, [across, across, mass * r * r / 2.0], 1e-7);
}

/// A sphere of radius 2: poles where its parametrization collapses.
#[test]
fn sphere_mass_properties() {
    let mut part = steel();
    let r = 2.0;
    sphere_solid(&mut part, "s", v(0.5, 0.0, -1.0), S::from_f64(r)).unwrap();
    let summary = only(&part);
    let volume = 4.0 / 3.0 * PI * r * r * r;
    let mass = volume * 7850.0e-9;
    assert_near("volume", summary.volume, volume, 1e-8);
    assert_near("area", summary.area, 4.0 * PI * r * r, 1e-8);
    for (k, want) in [0.5, 0.0, -1.0].into_iter().enumerate() {
        assert_near("centre", summary.center[k], want, 1e-8);
    }
    let i = 0.4 * mass * r * r;
    assert_diagonal(&summary, [i, i, i], 1e-7);
}

/// Two boxes of one part: the report lists both, the total adds their
/// masses and moves their inertia to the common centre.
#[test]
fn two_boxes_combine() {
    let mut part = steel();
    cube_solid(&mut part, "a", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    cube_solid(&mut part, "b", v(3.0, 0.0, 0.0), v(4.0, 1.0, 1.0)).unwrap();
    let report = mass_report(&part).unwrap();
    assert_eq!(report.bodies.len(), 2);
    let total = report.total.unwrap();
    let m = 7850.0e-9;
    assert_near("mass", total.mass, 2.0 * m, 1e-9);
    assert_near("centre", total.center[0], 2.0, 1e-9);
    // Each cube: m/6 about its own centre, plus m * 1.5² off the common
    // one, about y and z.
    let own = m / 6.0;
    let off = m * 1.5 * 1.5;
    assert_diagonal(
        &total,
        [2.0 * own, 2.0 * (own + off), 2.0 * (own + off)],
        1e-9,
    );
}

/// A vertex above a cylinder's side is as far from it as from its axis,
/// less the radius; the witness on the face is where the radius through
/// the vertex meets it.
#[test]
fn distance_from_a_vertex_to_a_cylinder() {
    let mut part = Part::<S>::new();
    revolved_cylinder(
        &mut part,
        "c",
        v(0.0, 0.0, 0.0),
        S::from_f64(1.0),
        S::from_f64(2.0),
    )
    .unwrap();
    cube_solid(&mut part, "b", v(2.0, 0.5, 0.5), v(3.0, 1.5, 1.5)).unwrap();
    let vertex = EntityRef::Vertex {
        name: "cube(b,p0,end)".into(),
    };
    // The quarter of the side from +x to +y.
    let side = EntityRef::Face {
        name: "cylinder(c,c1,q0)".into(),
    };
    let measured = measure(&part, &[vertex, side]);
    assert!(measured.error.is_none(), "{measured:?}");
    let distance = &measured
        .values
        .iter()
        .find(|m| m.label == "Distance")
        .unwrap()
        .value;
    // The corner (2, 0.5, 0.5): sqrt(4.25) from the axis.
    assert_near("distance", *distance, 4.25f64.sqrt() - 1.0, 1e-9);
    let [_, on_face] = measured.witness.unwrap();
    let r = on_face[0].mul(on_face[0]).add(on_face[1].mul(on_face[1]));
    assert!(r.could_be_equal(S::ONE), "{on_face:?}");
}

/// One entity alone: a circular edge's length and radius, a cylinder's
/// area and radius, a planar face's plane — of quarters, as a full turn is
/// built.
#[test]
fn single_entities() {
    let mut part = Part::<S>::new();
    revolved_cylinder(
        &mut part,
        "c",
        v(0.0, 0.0, 0.0),
        S::from_f64(1.5),
        S::from_f64(2.0),
    )
    .unwrap();
    let value = |entity: EntityRef, label: &str| {
        let measured = measure(&part, &[entity]);
        assert!(measured.error.is_none(), "{measured:?}");
        measured
            .values
            .iter()
            .find(|m| m.label == label)
            .unwrap_or_else(|| panic!("no {label} in {measured:?}"))
            .value
    };
    let rim = || EntityRef::Edge {
        name: "cylinder(c,p1,q0)".into(),
    };
    assert_near("rim", value(rim(), "Length"), 3.0 * PI / 4.0, 1e-9);
    assert_near("rim radius", value(rim(), "Radius"), 1.5, 1e-9);
    let side = || EntityRef::Face {
        name: "cylinder(c,c1,q1)".into(),
    };
    assert_near(
        "side",
        value(side(), "Area"),
        2.0 * PI * 1.5 * 2.0 / 4.0,
        1e-8,
    );
    assert_near("side radius", value(side(), "Radius"), 1.5, 1e-9);
    let top = EntityRef::Face {
        name: "cylinder(c,c0,q2)".into(),
    };
    assert!(measure(&part, &[top]).plane.is_some());
}

/// Two boxes sharing a corner region overlap by its volume; two sharing a
/// face only touch; two apart are not listed.
#[test]
fn overlapping_and_touching_boxes() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "a", v(0.0, 0.0, 0.0), v(2.0, 2.0, 2.0)).unwrap();
    cube_solid(&mut part, "b", v(1.0, 1.0, 1.0), v(3.0, 3.5, 3.0)).unwrap();
    cube_solid(&mut part, "c", v(-1.0, 0.0, 0.0), v(0.0, 1.0, 1.0)).unwrap();
    cube_solid(&mut part, "d", v(10.0, 0.0, 0.0), v(11.0, 1.0, 1.0)).unwrap();
    let report = interference_report(&part).unwrap();
    assert!(report.unchecked.is_empty(), "{report:?}");
    assert_eq!(report.solids, 4);
    assert_eq!(report.found.len(), 2, "{report:?}");
    let overlap = &report.found[0];
    assert_eq!(
        (overlap.a.as_str(), overlap.b.as_str()),
        ("cube(a)", "cube(b)")
    );
    assert_eq!(overlap.contact, Contact::Overlap);
    assert_near("overlap", overlap.volume.unwrap(), 1.0, 1e-9);
    let touch = &report.found[1];
    assert_eq!((touch.a.as_str(), touch.b.as_str()), ("cube(a)", "cube(c)"));
    assert_eq!(touch.contact, Contact::Touch);
}

/// Mass properties move with a pose: the centre with it, the inertia
/// turned.
#[test]
fn mass_properties_are_placed() {
    let mut part = Part::<S>::new();
    let solid = cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(2.0, 1.0, 1.0)).unwrap();
    let mass = part.topology().mass_properties(solid, S::ONE).unwrap();
    // A quarter turn about z, then up by 5.
    let pose = Pose::from_euler(v(0.0, 0.0, 5.0), [0.0, 0.0, 90.0].map(S::from_f64)).unwrap();
    let placed = MassSummary::of(&mass.placed(&pose).unwrap()).unwrap();
    for (k, want) in [-0.5, 1.0, 5.5].into_iter().enumerate() {
        assert_near("centre", placed.center[k], want, 1e-9);
    }
    let i = |p: f64, q: f64| 2.0 * (p * p + q * q) / 12.0;
    // The long side now runs along y.
    assert_diagonal(&placed, [i(2.0, 1.0), i(1.0, 1.0), i(1.0, 2.0)], 1e-9);
}

/// Two long bars overlapping along most of their length, on the scale of
/// millimetres a real part has: their intersection curves run 40 long, and
/// are traced all the way.
#[test]
#[ignore = "known boolean defect: with a 200-step trace budget the first 40-long trace stopped at x=29.9; \
with a larger budget (now 1000) it completes, but a later trace (trace_one_side v=VertexId(109), \
face_a=FaceId(32), face_b=FaceId(99)) widens step by step to a point 1.1 wide in x near (48.5, 10, 5) and \
never reaches its vertex — interval width compounding along the march, not the budget"]
fn long_bars_overlap() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "a", v(0.0, 0.0, 0.0), v(50.0, 10.0, 10.0)).unwrap();
    cube_solid(&mut part, "b", v(10.0, 5.0, 5.0), v(60.0, 15.0, 15.0)).unwrap();
    let report = interference_report(&part).unwrap();
    assert!(report.unchecked.is_empty(), "{report:?}");
    assert_eq!(report.found.len(), 1, "{report:?}");
    assert_near(
        "overlap",
        report.found[0].volume.unwrap(),
        40.0 * 5.0 * 5.0,
        1e-9,
    );
}
