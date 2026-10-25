//! Mass properties, measurements and interference of basic solids, against
//! their closed forms — each within the bounds returned.

use geop_core_program::{
    element::PickedElement,
    operation::Operation,
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    reader::Reader,
    runner::Runner,
    values::{Settings, StateValue, pose_state},
};
use std::f64::consts::PI;

use geop_core_editor::output::Bounded;
use geop_core_editor::selectable::Selectable;
use geop_core_editor::view::drawn;
use geop_core_math::{geop_error::GeopResult, primitives::Pose, scalars::Scalar, vector::Vector3};
use geop_lib_sweep::shapes::{cube_solid, revolved_cylinder, sphere_solid, v3};
use geop_ops_model::parameters::{Material, PartProperties, PartPropertiesArgs};
use geop_ops_model::selection::Selected;
use geop_ops_model::{NamedModel, model};

use crate::{
    interference::Contact, interference_report, mass::MassSummary, mass_report, measure,
    measure::PICKS,
};

/// Steel, 7850 kg/m³.
fn steel() -> Option<Material> {
    Some(Material {
        name: "Steel".into(),
        density: 7850.0,
    })
}

/// `value`, as the generator of a target that is given it.
fn given<T: Clone>(_: &Reader, value: &T) -> GeopResult<T> {
    Ok(value.clone())
}

/// The part a program of one step building `model` builds, its material
/// `material` — and, after it, what `then` adds.
fn built_with(
    given_model: &NamedModel,
    material: Option<Material>,
    then: impl FnOnce(&Runner, &mut Part),
) -> Part {
    let properties = PartPropertiesArgs {
        material,
        ..PartPropertiesArgs::default()
    };
    let runner = Runner::new(OperationRegistry::default());
    let mut part = Part::default();
    (runner.run_step(&mut part, "properties", |ctx| {
        PartProperties::run(ctx, &properties)
    }))
    .unwrap();
    (runner.run_step(&mut part, "model", |ctx| {
        ctx.produce(&model(), given_model, given)
    }))
    .unwrap();
    then(&runner, &mut part);
    part
}

/// The part of `model`, made of `material`.
fn built(model: &NamedModel, material: Option<Material>) -> Part {
    built_with(model, material, |_, _| {})
}

/// A pick of the element of the model named `name`, of what is drawn as
/// `kind`.
fn pick(kind: &str, name: &str) -> PickedElement {
    PickedElement {
        target: model().location().clone(),
        kind: kind.to_string(),
        name: name.to_string(),
    }
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

/// The one solid's summary of a report, of `model` made of steel.
fn only(model: &NamedModel) -> MassSummary {
    let part = built(model, steel());
    let report = mass_report(&part.current()).unwrap();
    assert_eq!(report.bodies.len(), 1, "{report:?}");
    let body = &report.bodies[0];
    assert!(body.error.is_none(), "{body:?}");
    body.properties.clone().unwrap()
}

/// The inertia tensor about the centre, against a diagonal one.
#[track_caller]
#[allow(clippy::needless_range_loop)]
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
    let mut part = NamedModel::new();
    cube_solid(&mut part, "b", v3(1.0, 2.0, 3.0), v3(3.0, 6.0, 4.0)).unwrap();
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
    let mut part = NamedModel::new();
    let (r, h) = (1.5, 4.0);
    revolved_cylinder(
        &mut part,
        "c",
        v3(1.0, -1.0, 2.0),
        Scalar::from_f64(r),
        Scalar::from_f64(h),
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
    let mut part = NamedModel::new();
    let r = 2.0;
    sphere_solid(&mut part, "s", v3(0.5, 0.0, -1.0), Scalar::from_f64(r)).unwrap();
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
    let mut part = NamedModel::new();
    cube_solid(&mut part, "a", v3(0.0, 0.0, 0.0), v3(1.0, 1.0, 1.0)).unwrap();
    cube_solid(&mut part, "b", v3(3.0, 0.0, 0.0), v3(4.0, 1.0, 1.0)).unwrap();
    let part = built(&part, steel());
    let report = mass_report(&part.current()).unwrap();
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
    let mut part = NamedModel::new();
    revolved_cylinder(
        &mut part,
        "c",
        v3(0.0, 0.0, 0.0),
        Scalar::from_f64(1.0),
        Scalar::from_f64(2.0),
    )
    .unwrap();
    cube_solid(&mut part, "b", v3(2.0, 0.5, 0.5), v3(3.0, 1.5, 1.5)).unwrap();
    let vertex = pick(drawn::VERTEX, "cube(b,p0,end)");
    // The quarter of the side from +x to +y.
    let side = pick(drawn::FACE, "cylinder(c,c1,q0)");
    let part = built(&part, None);
    let measured = measure(&part.current(), &[vertex, side]);
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
    assert!(r.could_be_equal(Scalar::ONE), "{on_face:?}");
}

/// One entity alone: a circular edge's length and radius, a cylinder's
/// area and radius, a planar face's plane — of quarters, as a full turn is
/// built.
#[test]
fn single_entities() {
    let mut part = NamedModel::new();
    revolved_cylinder(
        &mut part,
        "c",
        v3(0.0, 0.0, 0.0),
        Scalar::from_f64(1.5),
        Scalar::from_f64(2.0),
    )
    .unwrap();
    let part = built(&part, None);
    let value = |entity: PickedElement, label: &str| {
        let measured = measure(&part.current(), &[entity]);
        assert!(measured.error.is_none(), "{measured:?}");
        measured
            .values
            .iter()
            .find(|m| m.label == label)
            .unwrap_or_else(|| panic!("no {label} in {measured:?}"))
            .value
    };
    let rim = || pick(drawn::EDGE, "cylinder(c,p1,q0)");
    assert_near("rim", value(rim(), "Length"), 3.0 * PI / 4.0, 1e-9);
    assert_near("rim radius", value(rim(), "Radius"), 1.5, 1e-9);
    let side = || pick(drawn::FACE, "cylinder(c,c1,q1)");
    assert_near(
        "side",
        value(side(), "Area"),
        2.0 * PI * 1.5 * 2.0 / 4.0,
        1e-8,
    );
    assert_near("side radius", value(side(), "Radius"), 1.5, 1e-9);
    let top = pick(drawn::FACE, "cylinder(c,c0,q2)");
    assert!(measure(&part.current(), &[top]).plane.is_some());
}

/// Two boxes sharing a corner region overlap by its volume; two sharing a
/// face only touch; two apart are not listed.
#[test]
fn overlapping_and_touching_boxes() {
    let mut part = NamedModel::new();
    cube_solid(&mut part, "a", v3(0.0, 0.0, 0.0), v3(2.0, 2.0, 2.0)).unwrap();
    cube_solid(&mut part, "b", v3(1.0, 1.0, 1.0), v3(3.0, 3.5, 3.0)).unwrap();
    cube_solid(&mut part, "c", v3(-1.0, 0.0, 0.0), v3(0.0, 1.0, 1.0)).unwrap();
    cube_solid(&mut part, "d", v3(10.0, 0.0, 0.0), v3(11.0, 1.0, 1.0)).unwrap();
    let report = interference_report(&built(&part, None).current()).unwrap();
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
    let mut part = NamedModel::new();
    let solid = cube_solid(&mut part, "b", v3(0.0, 0.0, 0.0), v3(2.0, 1.0, 1.0)).unwrap();
    let mass = part.topology().mass_properties(solid).unwrap();
    // A quarter turn about z, then up by 5.
    let pose = Pose::from_euler(v3(0.0, 0.0, 5.0), [0.0, 0.0, 90.0].map(Scalar::from_f64)).unwrap();
    let area = part.topology().solid_area(solid).unwrap();
    let placed = MassSummary::of(&mass.placed(&pose).unwrap(), area).unwrap();
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
///
/// With the trace's stride fixed at 0.1 these took 400 steps each, and the
/// overlap's volume, integrated along the cubic fitted through the marched
/// points, came out wider than this test allows (1.08e-6 of 1000). The
/// stride now scales with the faces.
#[test]
fn long_bars_overlap() {
    let mut part = NamedModel::new();
    cube_solid(&mut part, "a", v3(0.0, 0.0, 0.0), v3(50.0, 10.0, 10.0)).unwrap();
    cube_solid(&mut part, "b", v3(10.0, 5.0, 5.0), v3(60.0, 15.0, 15.0)).unwrap();
    let report = interference_report(&built(&part, None).current()).unwrap();
    assert!(report.unchecked.is_empty(), "{report:?}");
    assert_eq!(report.found.len(), 1, "{report:?}");
    assert_near(
        "overlap",
        report.found[0].volume.unwrap(),
        40.0 * 5.0 * 5.0,
        1e-9,
    );
}

/// A sketch's curves measure as edges do: a planar sketch's half circle its
/// length and radius, a 3-D sketch's line its length — and the distance
/// between the two, which lie a unit apart.
#[test]
fn sketch_curves_are_measured() {
    use geop_core_math::primitives::CoordinateSystem;
    use geop_core_sketch::space::Sketch3d;
    use geop_ops_model::sketch::{PlacedSketch, sketch};
    let d = Scalar::from_f64;
    let mut arc = geop_ops_model::sketch::Sketch::new();
    let (a, b) = (
        arc.add_point(d(1.0), d(0.0)),
        arc.add_point(d(-1.0), d(0.0)),
    );
    let half = arc.add_arc(a, b, d(PI));
    let arc = PlacedSketch {
        name: "arc".into(),
        plane: CoordinateSystem::world_at(Vector3::zero()),
        sketch: arc,
    };
    let mut line = Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(d));
    let (a, b) = (
        line.add_point(at([-1.0, 2.0, 0.0])),
        line.add_point(at([2.0, 2.0, 0.0])),
    );
    line.add_line(a, b);
    let part = built_with(&NamedModel::new(), None, |runner, part| {
        (runner.run_step(part, "arc", |ctx| ctx.produce(&sketch("arc"), &arc, given))).unwrap();
        let args = geop_ops_sketch3d::AddSketch3dArgs {
            sketch: line,
            references: Vec::new(),
        };
        (runner.run_step(part, "route", |ctx| {
            geop_ops_sketch3d::AddSketch3d::run(ctx, &args)
        }))
        .unwrap();
    });

    let curve = PickedElement {
        target: sketch("arc").location().clone(),
        kind: drawn::SKETCH_CURVE.to_string(),
        name: half.to_string(),
    };
    let edge = pick(drawn::EDGE, "sketch3d(route,c2)");
    let value = |entities: &[PickedElement], label: &str| {
        let measured = measure(&part.current(), entities);
        assert!(measured.error.is_none(), "{measured:?}");
        measured
            .values
            .iter()
            .find(|m| m.label == label)
            .unwrap_or_else(|| panic!("no {label} in {measured:?}"))
            .value
    };
    assert_near(
        "half circle",
        value(std::slice::from_ref(&curve), "Length"),
        PI,
        1e-9,
    );
    assert_near(
        "its radius",
        value(std::slice::from_ref(&curve), "Radius"),
        1.0,
        1e-9,
    );
    assert_near(
        "line",
        value(std::slice::from_ref(&edge), "Length"),
        3.0,
        1e-9,
    );
    assert_near("between", value(&[curve, edge], "Distance"), 1.0, 1e-6);
}

/// What the measure tool picks is what a measured element is picked from.
#[test]
fn measure_picks_what_it_measures() {
    let mut accepts = PICKS.to_vec();
    let mut picked_from = Selected::picked_from();
    accepts.sort();
    picked_from.sort();
    picked_from.dedup();
    assert_eq!(accepts, picked_from);
}

/// An element of a part placed is measured where the part is placed: a
/// corner at the origin of a unit cube placed 10 along `x` is at 10.
#[test]
fn placed_parts_are_measured_where_they_are() {
    use geop_ops_assembly::testing::{Shelf, given_model, place, runner};
    let mut cube = NamedModel::new();
    cube_solid(&mut cube, "c", v3(0.0, 0.0, 0.0), v3(1.0, 1.0, 1.0)).unwrap();
    let mut placed = Program::new();
    placed.steps.push(given_model("model", cube, None));
    let at = Pose::identity().with_position(v3(10.0, 0.0, 0.0));
    let mut root = Program::new();
    root.steps.push(place("p", "cube.geop", &Settings::new()));
    root.state.insert(pose_state("p"), StateValue::Pose(at));
    let shelf = Shelf::default()
        .with("cube.geop", placed)
        .with("top.geop", root.clone());
    let runner = runner(shelf, "top.geop");
    let mut part = Part::default();
    (root.run(&runner, &mut part)).unwrap();

    let corner = |name: &str| PickedElement {
        target: model().location().behind("p"),
        ..pick(drawn::VERTEX, name)
    };
    let measured = measure(&part.current(), &[corner("cube(c,p0,end)")]);
    assert!(measured.error.is_none(), "{measured:?}");
    let x = measured.values.iter().find(|m| m.label == "X").unwrap();
    assert!(x.value.contains(10.0), "{measured:?}");
    let report = mass_report(&part.current()).unwrap();
    let body = &report.bodies[0];
    assert!(
        body.drawn
            .iter()
            .all(|face| face.target.to_string() == "/p:model")
    );
    assert_eq!(body.drawn.len(), 6, "{body:?}");
}
