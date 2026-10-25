//! 3-D sketches built on a part: what they are given of it follows it, and
//! their arguments read back as they were written.

use geop_core_math::{primitives::FrameAxis, scalars::Scalar, vector::Vector3};
use geop_core_program::{element::ElementReference, operation::Operation, part::Part};
use geop_core_sketch::space::{Constraint3d, CurveKind3d};
use geop_lib_sweep::shapes::block_part;
use geop_ops_model::datums::{ORIGIN, datums, origin_axis};
use geop_ops_model::{model, testing::vertex_at};

use crate::{AddSketch3d, AddSketch3dArgs, Reference3d};
use geop_ops_model::sketch3d::{
    CurveSource, LineSource, PlacedSketch3d, PointSource, Sketch3d, sketch3d,
};

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

fn close(a: &Vector3, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// A point fixed at a vertex of the part follows it when the part changes,
/// and a line along an axis of the origin turns onto it.
#[test]
fn references_follow_the_part() {
    let mut sketch = Sketch3d::new();
    let p = sketch.add_fixed_point(v([1.0, 1.0, 1.0]));
    let q = sketch.add_point(v([1.2, 1.1, 3.0]));
    let line = sketch.add_line(p, q);
    let along = sketch.constrain(Constraint3d::ParallelTo {
        line,
        direction: v([0.0, 1.0, 0.0]),
    });
    let corner = vertex_at(&block_part("b", [0.0; 3], [1.0; 3]), [1.0, 1.0, 1.0]);
    let args = AddSketch3dArgs {
        sketch,
        references: vec![
            Reference3d::Point {
                point: p,
                from: PointSource::Vertex(ElementReference::new(model(), corner)),
            },
            Reference3d::Direction {
                constraint: along,
                from: LineSource::Datum(origin_axis(FrameAxis::Z)),
            },
        ],
    };
    let at = |size: f64| {
        let block = block_part("b", [0.0; 3], [size; 3]);
        let mut part = Part::default();
        let runner = geop_core_program::runner::Runner::new(Default::default());
        (runner.run_step(&mut part, "block", |ctx| {
            ctx.produce(&model(), &block, |_, block| Ok(block.clone()))
        }))
        .unwrap();
        (runner.run_step(&mut part, "route", |ctx| {
            <AddSketch3d as Operation>::run(ctx, &args)
        }))
        .unwrap();
        let reader = part.current();
        let whole = ElementReference::<PlacedSketch3d, PlacedSketch3d>::whole(sketch3d("route"));
        let s = &reader.retrieve_element(&whole).unwrap().sketch;
        (s.points[&p].at, s.points[&q].at)
    };
    let (a, b) = at(1.0);
    assert!(close(&a, [1.0, 1.0, 1.0]));
    // Not along the axis as drawn: solved onto it.
    assert!(
        (b[0].to_f64() - 1.0).abs() < 1e-9 && (b[1].to_f64() - 1.0).abs() < 1e-9,
        "{b:?}"
    );
    let (a, b) = at(2.0);
    assert!(close(&a, [2.0, 2.0, 2.0]), "{a:?}");
    assert!(
        (b[0].to_f64() - 2.0).abs() < 1e-9 && (b[1].to_f64() - 2.0).abs() < 1e-9,
        "{b:?}"
    );
}

/// A sketch's arguments — curves of every kind, a fixed point given by a
/// datum, a point on an edge, a direction given by an axis — read back
/// from their JSON as they were written.
#[test]
fn arguments_round_trip_through_json() {
    let mut sketch = Sketch3d::new();
    let p = sketch.add_fixed_point(v([0.0, 0.0, 0.0]));
    let q = sketch.add_point(v([1.0, 0.0, 0.0]));
    let r = sketch.add_point(v([2.0, 1.0, 0.0]));
    let line = sketch.add_line(p, q);
    sketch.add_arc(p, q, r);
    sketch.add_spline(vec![p, q, r]);
    let edge = sketch.add_curve(CurveKind3d::Reference);
    sketch.constrain(Constraint3d::OnCurve {
        point: r,
        curve: edge,
    });
    let along = sketch.constrain(Constraint3d::ParallelTo {
        line,
        direction: v([1.0, 0.0, 0.0]),
    });
    let args = AddSketch3dArgs {
        sketch,
        references: vec![
            Reference3d::Point {
                point: p,
                from: PointSource::Datum(ElementReference::new(datums(), ORIGIN)),
            },
            Reference3d::Curve {
                curve: edge,
                from: CurveSource::Edge(ElementReference::new(model(), "e")),
            },
            Reference3d::Direction {
                constraint: along,
                from: LineSource::Datum(origin_axis(FrameAxis::X)),
            },
        ],
    };
    let json = serde_json::to_value(&args).unwrap();
    let back: AddSketch3dArgs =
        serde_json::from_value(json.clone()).unwrap_or_else(|e| panic!("{e}: {json}"));
    assert_eq!(back, args);
}
