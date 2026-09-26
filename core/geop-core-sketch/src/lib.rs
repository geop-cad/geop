//! 2-D constraint sketches: points, lines, arcs, circles and splines, the
//! typical CAD constraints between them, a BFGS solver, and conversion of the
//! solved sketch into closed profiles for extrude and revolve.
//!
//! - [`sketch`]: the entities and constraints (plain `f64` design data).
//! - [`solve`]: [`Sketch::solve`] / [`Sketch::solve_with_drag`].
//! - [`profile`]: [`Sketch::regions`] and [`ProfileLoop::to_nurbs`].

pub mod bfgs;
pub mod dual;
pub mod geometry;
pub mod profile;
pub mod sketch;
pub mod solve;

pub use profile::{ProfileEdge, ProfileJoint, ProfileLoop, ProfilePiece, Region};
pub use sketch::{
    Constraint, ConstraintId, Curve, CurveId, CurveKind, Point, PointId, Positions, Sketch,
};
pub use solve::SolveReport;

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{
        for_all_scalars,
        scalars::{Scalar, scal_in_f64::ScalInF64},
        vector::Vector2,
    };
    use std::f64::consts::{FRAC_PI_2, PI};

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-7
    }

    fn xy(s: &Sketch, p: PointId) -> [f64; 2] {
        s.points[&p].xy()
    }

    /// A sloppily drawn quadrilateral becomes an exact, fully constrained
    /// 2 x 1 rectangle anchored at the origin.
    #[test]
    fn rectangle_is_solved_and_fully_constrained() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(0.1, -0.1),
            s.add_point(2.2, 0.2),
            s.add_point(1.9, 1.3),
            s.add_point(-0.2, 0.8),
        ];
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: 0.0,
            y: 0.0,
        });
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Vertical { line: l[3] });
        s.constrain(Constraint::Length {
            curve: l[0],
            value: 2.0,
        });
        s.constrain(Constraint::Distance {
            a: p[1],
            b: p[2],
            value: 1.0,
        });

        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        assert_eq!(report.dof, 0, "{report:?}");
        assert!(report.free_points.values().all(|f| !f), "{report:?}");
        let expect = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]];
        for (pi, e) in p.iter().zip(expect) {
            let q = xy(&s, *pi);
            assert!(close(q[0], e[0]) && close(q[1], e[1]), "{q:?} vs {e:?}");
        }
    }

    /// Without the anchor and dimensions the rectangle keeps its width,
    /// height and position free: 4 degrees of freedom, every point free.
    #[test]
    fn unanchored_rectangle_has_two_dof() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(0.0, 0.0),
            s.add_point(2.0, 0.1),
            s.add_point(2.0, 1.0),
            s.add_point(0.0, 1.0),
        ];
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Vertical { line: l[3] });
        let report = s.solve().unwrap();
        assert!(report.converged);
        assert_eq!(report.dof, 4, "width, height and translation");
        assert!(report.free_points.values().all(|f| *f));
    }

    /// Coincident points become one: the constraint needs no residual and the
    /// points end up exactly equal.
    #[test]
    fn coincident_points_merge() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let c = s.add_point(1.1, 0.05);
        let d = s.add_point(1.5, 1.0);
        s.add_line(a, b);
        s.add_line(c, d);
        s.constrain(Constraint::Coincident { a: b, b: c });
        let report = s.solve().unwrap();
        assert!(report.converged);
        assert_eq!(xy(&s, b), xy(&s, c));
        assert_eq!(report.dof, 6);
    }

    /// A line tangent to an arc at their shared endpoint, with the arc's
    /// radius fixed: the classic slot end.
    #[test]
    fn line_arc_tangent_at_shared_endpoint() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(2.0, 0.0);
        let c = s.add_point(2.0, 1.0);
        let line = s.add_line(a, b);
        let arc = s.add_arc(b, c, 1.5);
        s.constrain(Constraint::Fix {
            point: a,
            x: 0.0,
            y: 0.0,
        });
        s.constrain(Constraint::Horizontal { line });
        s.constrain(Constraint::Length {
            curve: line,
            value: 2.0,
        });
        s.constrain(Constraint::Tangent { a: line, b: arc });
        s.constrain(Constraint::Radius {
            curve: arc,
            value: 0.5,
        });
        s.constrain(Constraint::Fix {
            point: c,
            x: 2.0,
            y: 1.0,
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let CurveKind::Arc { sweep, .. } = s.curves[&arc].kind else {
            unreachable!()
        };
        // A half circle of radius 0.5 turning left from (2, 0) to (2, 1).
        assert!(close(sweep, PI), "sweep {sweep}");
        assert_eq!(report.dof, 0, "{report:?}");
    }

    /// A circle tangent to two perpendicular lines, with its center then
    /// pinned by the tangencies and the radius.
    #[test]
    fn circle_tangent_to_lines() {
        let mut s = Sketch::new();
        let o = s.add_point(0.0, 0.0);
        let x = s.add_point(3.0, 0.0);
        let y = s.add_point(0.0, 3.0);
        let lx = s.add_line(o, x);
        let ly = s.add_line(o, y);
        let c = s.add_point(0.8, 1.3);
        let circle = s.add_circle(c, 0.7);
        for (p, xy) in [(o, [0.0, 0.0]), (x, [3.0, 0.0]), (y, [0.0, 3.0])] {
            s.constrain(Constraint::Fix {
                point: p,
                x: xy[0],
                y: xy[1],
            });
        }
        s.constrain(Constraint::Tangent { a: lx, b: circle });
        s.constrain(Constraint::Tangent { a: circle, b: ly });
        s.constrain(Constraint::Radius {
            curve: circle,
            value: 1.0,
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let q = xy(&s, c);
        assert!(close(q[0], 1.0) && close(q[1], 1.0), "{q:?}");
    }

    /// Contradicting constraints are reported, not hidden.
    #[test]
    fn conflicting_constraints_do_not_converge() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let l = s.add_line(a, b);
        s.constrain(Constraint::Length {
            curve: l,
            value: 1.0,
        });
        s.constrain(Constraint::Distance { a, b, value: 2.0 });
        let report = s.solve().unwrap();
        assert!(!report.converged);
        assert!(!report.failed_constraints.is_empty());
    }

    /// Dragging a free point moves it to the cursor; dragging a fixed one
    /// leaves it where the constraints say.
    #[test]
    fn drag_follows_cursor_only_where_free() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let l = s.add_line(a, b);
        s.constrain(Constraint::Fix {
            point: a,
            x: 0.0,
            y: 0.0,
        });
        s.constrain(Constraint::Length {
            curve: l,
            value: 1.0,
        });
        let report = s.solve_with_drag(&[(b, [0.0, 3.0])]).unwrap();
        assert!(report.converged, "{report:?}");
        let q = xy(&s, b);
        assert!(close(q[0], 0.0) && close(q[1], 1.0), "{q:?}");

        let report = s.solve_with_drag(&[(a, [5.0, 5.0])]).unwrap();
        assert!(report.converged);
        let q = xy(&s, a);
        assert!(close(q[0], 0.0) && close(q[1], 0.0), "{q:?}");
    }

    /// Symmetric, midpoint, perpendicular, equal, and angle constraints
    /// together: an isosceles triangle with a 60° apex, i.e. equilateral.
    #[test]
    fn equilateral_triangle_from_symmetry_and_angle() {
        let mut s = Sketch::new();
        let a = s.add_point(-1.0, 0.1);
        let b = s.add_point(1.2, -0.1);
        let c = s.add_point(0.1, 1.5);
        let base = s.add_line(a, b);
        let left = s.add_line(c, a);
        let right = s.add_line(c, b);
        let m = s.add_point(0.0, 0.0);
        let axis_top = s.add_point(0.0, 2.0);
        let axis = s.add_line(m, axis_top);
        s.set_construction(axis, true);
        s.constrain(Constraint::Fix {
            point: m,
            x: 0.0,
            y: 0.0,
        });
        s.constrain(Constraint::Vertical { line: axis });
        s.constrain(Constraint::Midpoint {
            point: m,
            curve: base,
        });
        s.constrain(Constraint::Symmetric { a, b, line: axis });
        s.constrain(Constraint::PointOnCurve {
            point: c,
            curve: axis,
        });
        s.constrain(Constraint::Equal { a: left, b: right });
        s.constrain(Constraint::Angle {
            a: left,
            b: right,
            value: PI / 3.0,
        });
        s.constrain(Constraint::Length {
            curve: base,
            value: 2.0,
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let q = xy(&s, c);
        assert!(close(q[0], 0.0) && close(q[1], 3f64.sqrt()), "{q:?}");
    }

    /// A slot: two lines joined by two half circles, around a circular hole.
    /// One region, with the circle as its hole, and NURBS loops that close
    /// up exactly.
    fn check_slot_with_hole_regions<S: Scalar>() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(0.0, 0.0),
            s.add_point(2.0, 0.0),
            s.add_point(2.0, 1.0),
            s.add_point(0.0, 1.0),
        ];
        s.add_line(p[0], p[1]);
        s.add_arc_with_sweep(p[1], p[2], PI);
        s.add_line(p[2], p[3]);
        s.add_arc_with_sweep(p[3], p[0], PI);
        let c = s.add_point(1.0, 0.5);
        s.add_circle(c, 0.25);
        // A dangling helper line is ignored.
        let q = s.add_point(-1.0, -1.0);
        s.add_line(p[0], q);

        let regions = s.regions().unwrap();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].outer.edges.len(), 4);
        assert_eq!(regions[0].holes.len(), 1);

        let positions = s.positions();
        for (lp, count) in [(&regions[0].outer, 6), (&regions[0].holes[0], 4)] {
            let pieces = lp.to_nurbs::<S>(&s, &positions).unwrap();
            let curves: Vec<_> = pieces.iter().map(|p| &p.curve).collect();
            assert_eq!(curves.len(), count);
            // The joints chain up exactly like the curves do.
            for (i, piece) in pieces.iter().enumerate() {
                assert_eq!(piece.end, pieces[(i + 1) % pieces.len()].start);
            }
            for (i, c) in curves.iter().enumerate() {
                let (t0, t1) = c.domain();
                assert!(t0.could_be_equal(S::ZERO) && t1.could_be_equal(S::ONE));
                let next = curves[(i + 1) % curves.len()];
                let end = c.evaluate(S::ONE).unwrap();
                let start = next.evaluate(S::ZERO).unwrap();
                assert!(end.could_be_equal(&start), "{end:?} vs {start:?}");
            }
        }
        // The half circle on the right passes through (2.5, 0.5).
        let outer = regions[0].outer.to_nurbs::<S>(&s, &positions).unwrap();
        let far = Vector2::from_array([S::from_f64(2.5), S::from_f64(0.5)]);
        assert!(
            outer
                .iter()
                .any(|p| p.curve.evaluate(S::ONE).unwrap().could_be_equal(&far)),
            "no quarter piece ends at the right apex"
        );
    }
    #[test]
    fn slot_with_hole_regions() {
        for_all_scalars!(check_slot_with_hole_regions);
    }

    /// Nested loops alternate between outer boundaries and holes, and outer
    /// loops come out counter-clockwise even when drawn clockwise.
    #[test]
    fn nested_squares_alternate() {
        let mut s = Sketch::new();
        for (size, clockwise) in [(3.0, true), (2.0, false), (1.0, true)] {
            let h = size / 2.0;
            let mut corners = [[-h, -h], [h, -h], [h, h], [-h, h]];
            if clockwise {
                corners.reverse();
            }
            let p: Vec<PointId> = corners.iter().map(|c| s.add_point(c[0], c[1])).collect();
            for i in 0..4 {
                s.add_line(p[i], p[(i + 1) % 4]);
            }
        }
        let regions = s.regions().unwrap();
        assert_eq!(regions.len(), 2);
        let with_hole = regions.iter().filter(|r| r.holes.len() == 1).count();
        assert_eq!(with_hole, 1);
        // The outermost loop was drawn clockwise and is flipped.
        let outermost = regions.iter().find(|r| r.holes.len() == 1).unwrap();
        assert!(outermost.outer.edges.iter().all(|e| e.reversed));
    }

    /// Two circles a hair apart: the hole is still nested, because nesting
    /// is decided on the curves themselves. The gap here (5e-5 of the
    /// radius) is smaller than the sagitta of any polyline anyone would
    /// draw these circles with — sampling them and testing the polygons
    /// could not tell this apart from the two touching or crossing.
    #[test]
    fn nesting_resolves_a_gap_finer_than_any_sampling() {
        let mut s = Sketch::new();
        let outer = s.add_point(0.0, 0.0);
        s.add_circle(outer, 1.0);
        let inner = s.add_point(0.0, 0.0);
        s.add_circle(inner, 1.0 - 5e-5);

        let regions = s.regions().unwrap();
        assert_eq!(regions.len(), 1, "one region: the ring between them");
        assert_eq!(regions[0].holes.len(), 1);
        // The outer loop runs counter-clockwise and the hole the other way,
        // whatever order they were drawn in.
        assert!(!regions[0].outer.edges[0].reversed);
        assert!(regions[0].holes[0].edges[0].reversed);
    }

    /// Region finding runs on every solve while a point is being dragged,
    /// so it has to stay quick on a sketch of real size: two 20-sided loops
    /// here, one inside the other.
    #[test]
    fn regions_of_a_large_sketch_are_found_quickly() {
        let mut s = Sketch::new();
        for radius in [3.0, 1.0] {
            let p: Vec<PointId> = (0..20)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / 20.0;
                    s.add_point(radius * a.cos(), radius * a.sin())
                })
                .collect();
            for i in 0..20 {
                s.add_line(p[i], p[(i + 1) % 20]);
            }
        }
        let start = std::time::Instant::now();
        let regions = s.regions().unwrap();
        let elapsed = start.elapsed();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 1);
        assert_eq!(regions[0].outer.edges.len(), 20);
        assert!(
            elapsed < std::time::Duration::from_millis(500),
            "finding the regions of 40 curves took {elapsed:?}"
        );
    }

    #[test]
    fn branching_profile_is_rejected() {
        let mut s = Sketch::new();
        let p: Vec<PointId> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
            .iter()
            .map(|c| s.add_point(c[0], c[1]))
            .collect();
        for i in 0..4 {
            s.add_line(p[i], p[(i + 1) % 4]);
        }
        s.add_line(p[0], p[2]);
        assert!(s.regions().is_err());
    }

    /// An arc drawn with a curvature is the minor arc with that curvature.
    #[test]
    fn arc_from_curvature() {
        let mut s = Sketch::new();
        let a = s.add_point(1.0, 0.0);
        let b = s.add_point(0.0, 1.0);
        let arc = s.add_arc(a, b, 1.0);
        let CurveKind::Arc { sweep, .. } = s.curves[&arc].kind else {
            unreachable!()
        };
        assert!(close(sweep, FRAC_PI_2));
    }

    /// A spline and an arc joined tangentially into a closed loop.
    #[test]
    fn spline_tangent_to_arc() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.5);
        let c = s.add_point(2.0, -0.3);
        let d = s.add_point(3.0, 0.0);
        let spline = s.add_spline(vec![a, b, c, d]);
        let arc = s.add_arc_with_sweep(d, a, 2.5);
        s.constrain(Constraint::Tangent { a: spline, b: arc });
        s.constrain(Constraint::Fix {
            point: a,
            x: 0.0,
            y: 0.0,
        });
        s.constrain(Constraint::Fix {
            point: d,
            x: 3.0,
            y: 0.0,
        });
        s.constrain(Constraint::Fix {
            point: b,
            x: 1.0,
            y: 0.5,
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let regions = s.regions().unwrap();
        assert_eq!(regions.len(), 1);
    }

    /// Every NURBS piece remembers the sketch curve it came from and the
    /// joints it runs between, in the curve's own direction even when the
    /// loop runs against it: a circle's quarters are `c#0..c#3` between its
    /// split points `c@0..c@3`, and a line keeps its end points.
    #[test]
    fn profile_pieces_name_their_sketch_origin() {
        let mut s = Sketch::new();
        let c = s.add_point(0.0, 0.0);
        let circle = s.add_circle(c, 1.0);
        let p: Vec<PointId> = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]]
            .iter()
            .map(|c| s.add_point(c[0], c[1]))
            .collect();
        let lines: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();

        let regions = s.regions().unwrap();
        let positions = s.positions();
        let outer = regions[0]
            .outer
            .to_nurbs::<ScalInF64>(&s, &positions)
            .unwrap();
        let names: Vec<String> = outer.iter().map(|p| p.name()).collect();
        assert_eq!(names.len(), 4);
        for l in &lines {
            assert!(names.contains(&format!("{l}")), "{names:?}");
        }
        assert!(
            outer
                .iter()
                .all(|piece| matches!(piece.start, ProfileJoint::Point(_)))
        );

        // The hole runs clockwise, against the circle's own direction.
        let hole = regions[0].holes[0]
            .to_nurbs::<ScalInF64>(&s, &positions)
            .unwrap();
        let joints: Vec<String> = hole.iter().map(|p| p.start.to_string()).collect();
        assert_eq!(
            joints,
            [
                format!("{circle}@0"),
                format!("{circle}@3"),
                format!("{circle}@2"),
                format!("{circle}@1")
            ]
        );
        let names: Vec<String> = hole.iter().map(|p| p.name()).collect();
        assert_eq!(
            names,
            [
                format!("{circle}#3"),
                format!("{circle}#2"),
                format!("{circle}#1"),
                format!("{circle}")
            ]
        );
    }

    /// A sketch serializes with its entities keyed by id, and reads back
    /// unchanged — ids included, so references into it survive the trip.
    #[test]
    fn sketch_json_round_trip_keeps_ids() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let l = s.add_line(a, b);
        s.constrain(Constraint::Horizontal { line: l });
        let json = serde_json::to_string(&s).unwrap();
        assert!(json.contains(&format!("\"{}\":", l.0)), "{json}");
        let back: Sketch = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
        back.validate().unwrap();
    }
}
