//! 2-D constraint sketches: points, lines, arcs, circles and splines, the
//! typical CAD constraints between them, a solver, and conversion of the
//! solved sketch into closed profiles for extrude and revolve.
//!
//! - [`sketch`]: the entities and constraints: design data, in any scalar.
//! - [`solve`]: [`Sketch::solve`] / [`Sketch::solve_with_drag`], and
//!   [`Sketch::enclose`]: the solution as the kernel builds on it.
//! - [`profile`]: [`Sketch::regions`] and [`ProfileLoop::to_nurbs`].

pub mod geometry;
pub mod profile;
pub mod sketch;
pub mod solve;

pub use profile::{ProfileEdge, ProfileJoint, ProfileLoop, ProfilePiece, Region};
pub use sketch::{
    Constraint, ConstraintId, Curve, CurveId, CurveKind, Enclosure, Point, PointId, Positions,
    Sketch, SplineShape,
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

    type T = ScalInF64;

    fn n(x: f64) -> T {
        T::from_f64(x)
    }

    fn v2(p: [f64; 2]) -> Vector2<T> {
        Vector2::from_array(p.map(T::from_f64))
    }

    fn xy(s: &Sketch<T>, p: PointId) -> [f64; 2] {
        let q = s.points[&p].xy();
        [q[0].to_f64(), q[1].to_f64()]
    }

    /// A sloppily drawn quadrilateral becomes an exact, fully constrained
    /// 2 x 1 rectangle anchored at the origin.
    #[test]
    fn rectangle_is_solved_and_fully_constrained() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(n(0.1), n(-0.1)),
            s.add_point(n(2.2), n(0.2)),
            s.add_point(n(1.9), n(1.3)),
            s.add_point(n(-0.2), n(0.8)),
        ];
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Vertical { line: l[3] });
        s.constrain(Constraint::Length {
            curve: l[0],
            value: n(2.0),
        });
        s.constrain(Constraint::Distance {
            a: p[1],
            b: p[2],
            value: n(1.0),
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
            s.add_point(n(0.0), n(0.0)),
            s.add_point(n(2.0), n(0.1)),
            s.add_point(n(2.0), n(1.0)),
            s.add_point(n(0.0), n(1.0)),
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
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.0));
        let c = s.add_point(n(1.1), n(0.05));
        let d = s.add_point(n(1.5), n(1.0));
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
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(2.0), n(0.0));
        let c = s.add_point(n(2.0), n(1.0));
        let line = s.add_line(a, b);
        let arc = s.add_arc(b, c, n(2.0 * 0.75f64.asin()));
        s.constrain(Constraint::Fix {
            point: a,
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Horizontal { line });
        s.constrain(Constraint::Length {
            curve: line,
            value: n(2.0),
        });
        s.constrain(Constraint::Tangent { a: line, b: arc });
        s.constrain(Constraint::Radius {
            curve: arc,
            value: n(0.5),
        });
        s.constrain(Constraint::Fix {
            point: c,
            x: n(2.0),
            y: n(1.0),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let CurveKind::Arc { sweep, .. } = s.curves[&arc].kind else {
            unreachable!()
        };
        // A half circle of radius 0.5 turning left from (2, 0) to (2, 1).
        assert!(close(sweep.to_f64(), PI), "sweep {sweep:?}");
        assert_eq!(report.dof, 0, "{report:?}");
    }

    /// A circle tangent to two perpendicular lines, with its center then
    /// pinned by the tangencies and the radius.
    #[test]
    fn circle_tangent_to_lines() {
        let mut s = Sketch::new();
        let o = s.add_point(n(0.0), n(0.0));
        let x = s.add_point(n(3.0), n(0.0));
        let y = s.add_point(n(0.0), n(3.0));
        let lx = s.add_line(o, x);
        let ly = s.add_line(o, y);
        let c = s.add_point(n(0.8), n(1.3));
        let circle = s.add_circle(c, n(0.7));
        for (p, xy) in [(o, [0.0, 0.0]), (x, [3.0, 0.0]), (y, [0.0, 3.0])] {
            s.constrain(Constraint::Fix {
                point: p,
                x: n(xy[0]),
                y: n(xy[1]),
            });
        }
        s.constrain(Constraint::Tangent { a: lx, b: circle });
        s.constrain(Constraint::Tangent { a: circle, b: ly });
        s.constrain(Constraint::Radius {
            curve: circle,
            value: n(1.0),
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
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.0));
        let l = s.add_line(a, b);
        s.constrain(Constraint::Length {
            curve: l,
            value: n(1.0),
        });
        s.constrain(Constraint::Distance {
            a,
            b,
            value: n(2.0),
        });
        let report = s.solve().unwrap();
        assert!(!report.converged);
        assert!(!report.failed_constraints.is_empty());
    }

    /// Dragging a free point moves it to the cursor; dragging a fixed one
    /// leaves it where the constraints say.
    #[test]
    fn drag_follows_cursor_only_where_free() {
        let mut s = Sketch::new();
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.0));
        let l = s.add_line(a, b);
        s.constrain(Constraint::Fix {
            point: a,
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Length {
            curve: l,
            value: n(1.0),
        });
        let report = s.solve_with_drag(&[(b, v2([0.0, 3.0]))]).unwrap();
        assert!(report.converged, "{report:?}");
        let q = xy(&s, b);
        assert!(close(q[0], 0.0) && close(q[1], 1.0), "{q:?}: {report:?}");

        let report = s.solve_with_drag(&[(a, v2([5.0, 5.0]))]).unwrap();
        assert!(report.converged);
        let q = xy(&s, a);
        assert!(close(q[0], 0.0) && close(q[1], 0.0), "{q:?}");
    }

    /// Symmetric, midpoint, perpendicular, equal, and angle constraints
    /// together: an isosceles triangle with a 60° apex, i.e. equilateral.
    #[test]
    fn equilateral_triangle_from_symmetry_and_angle() {
        let mut s = Sketch::new();
        let a = s.add_point(n(-1.0), n(0.1));
        let b = s.add_point(n(1.2), n(-0.1));
        let c = s.add_point(n(0.1), n(1.5));
        let base = s.add_line(a, b);
        let left = s.add_line(c, a);
        let right = s.add_line(c, b);
        let m = s.add_point(n(0.0), n(0.0));
        let axis_top = s.add_point(n(0.0), n(2.0));
        let axis = s.add_line(m, axis_top);
        s.set_construction(axis, true);
        s.constrain(Constraint::Fix {
            point: m,
            x: n(0.0),
            y: n(0.0),
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
            value: n(PI / 3.0),
        });
        s.constrain(Constraint::Length {
            curve: base,
            value: n(2.0),
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
            s.add_point(n(0.0), n(0.0)),
            s.add_point(n(2.0), n(0.0)),
            s.add_point(n(2.0), n(1.0)),
            s.add_point(n(0.0), n(1.0)),
        ];
        s.add_line(p[0], p[1]);
        s.add_arc(p[1], p[2], n(PI));
        s.add_line(p[2], p[3]);
        s.add_arc(p[3], p[0], n(PI));
        let c = s.add_point(n(1.0), n(0.5));
        s.add_circle(c, n(0.25));
        // A dangling helper line is ignored.
        let q = s.add_point(n(-1.0), n(-1.0));
        s.add_line(p[0], q);

        let regions = s.regions().unwrap();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].outer.edges.len(), 4);
        assert_eq!(regions[0].holes.len(), 1);

        let geometry = s.enclose::<S>().unwrap();
        for (lp, count) in [(&regions[0].outer, 6), (&regions[0].holes[0], 4)] {
            let pieces = lp.to_nurbs::<T, S>(&s, &geometry).unwrap();
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
        let outer = regions[0].outer.to_nurbs::<T, S>(&s, &geometry).unwrap();
        let far = Vector2::from_array([S::from_f64(2.5), S::from_f64(0.5)]);
        assert!(
            outer
                .iter()
                .any(|p| p.curve.evaluate(S::ONE).unwrap().could_be_equal(&far)),
            "no quarter piece ends at the right apex: {:?}",
            outer
                .iter()
                .map(|p| p.curve.evaluate(S::ONE).unwrap())
                .collect::<Vec<_>>()
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
            let p: Vec<PointId> = corners
                .iter()
                .map(|c| s.add_point(n(c[0]), n(c[1])))
                .collect();
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
        let outer = s.add_point(n(0.0), n(0.0));
        s.add_circle(outer, n(1.0));
        let inner = s.add_point(n(0.0), n(0.0));
        s.add_circle(inner, n(1.0 - 5e-5));

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
                    s.add_point(n(radius * a.cos()), n(radius * a.sin()))
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
            .map(|c| s.add_point(n(c[0]), n(c[1])))
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
        let a = s.add_point(n(1.0), n(0.0));
        let b = s.add_point(n(0.0), n(1.0));
        let arc = s.add_arc(a, b, n(FRAC_PI_2));
        let CurveKind::Arc { sweep, .. } = s.curves[&arc].kind else {
            unreachable!()
        };
        assert!(close(sweep.to_f64(), FRAC_PI_2));
    }

    /// A spline and an arc joined tangentially into a closed loop.
    #[test]
    fn spline_tangent_to_arc() {
        let mut s = Sketch::new();
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.5));
        let c = s.add_point(n(2.0), n(-0.3));
        let d = s.add_point(n(3.0), n(0.0));
        let spline = s.add_spline(vec![a, b, c, d]);
        let arc = s.add_arc(d, a, n(2.5));
        s.constrain(Constraint::Tangent { a: spline, b: arc });
        s.constrain(Constraint::Fix {
            point: a,
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Fix {
            point: d,
            x: n(3.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Fix {
            point: b,
            x: n(1.0),
            y: n(0.5),
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
        let c = s.add_point(n(0.0), n(0.0));
        let circle = s.add_circle(c, n(1.0));
        let p: Vec<PointId> = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]]
            .iter()
            .map(|c| s.add_point(n(c[0]), n(c[1])))
            .collect();
        let lines: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();

        let regions = s.regions().unwrap();
        let geometry = s.enclose::<ScalInF64>().unwrap();
        let outer = regions[0]
            .outer
            .to_nurbs::<T, ScalInF64>(&s, &geometry)
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
            .to_nurbs::<T, ScalInF64>(&s, &geometry)
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
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.0));
        let l = s.add_line(a, b);
        s.constrain(Constraint::Horizontal { line: l });
        let json = serde_json::to_string(&s).unwrap();
        assert!(json.contains(&format!("\"{}\":", l.0)), "{json}");
        let back: Sketch<T> = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
        back.validate().unwrap();
    }

    /// Removing a curve takes its constraints with it, and the points only
    /// it used; a lone point drawn on purpose, and every id left, stay.
    #[test]
    fn remove_takes_what_depends_on_it() {
        let mut s = Sketch::new();
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.0));
        let c = s.add_point(n(1.0), n(1.0));
        let lone = s.add_point(n(5.0), n(5.0));
        let ab = s.add_line(a, b);
        let bc = s.add_line(b, c);
        let horizontal = s.constrain(Constraint::Horizontal { line: ab });
        let vertical = s.constrain(Constraint::Vertical { line: bc });
        let fix = s.constrain(Constraint::Fix {
            point: a,
            x: n(0.0),
            y: n(0.0),
        });
        s.remove(&[], &[ab], &[]);
        assert!(!s.curves.contains_key(&ab) && s.curves.contains_key(&bc));
        assert!(!s.constraints.contains_key(&horizontal));
        assert!(s.constraints.contains_key(&vertical) && s.constraints.contains_key(&fix));
        // `a` is still fixed, so still used; `b` is `bc`'s.
        assert!(s.points.contains_key(&a) && s.points.contains_key(&b));

        s.remove(&[], &[], &[fix]);
        assert!(!s.points.contains_key(&a), "a was only the fix's");
        s.remove(&[c], &[], &[]);
        assert!(s.curves.is_empty() && s.constraints.is_empty());
        assert_eq!(s.points.keys().copied().collect::<Vec<_>>(), [lone]);
        s.validate().unwrap();
    }

    /// A solved sketch encloses the exact solution of its constraints: a
    /// rectangle whose sides the solver only made nearly horizontal and
    /// vertical is built from boxes around its exact corners — narrow, but
    /// each holding the corner the constraints mean.
    #[test]
    fn solutions_are_enclosed() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(n(0.1), n(-0.1)),
            s.add_point(n(2.2), n(0.2)),
            s.add_point(n(1.9), n(1.3)),
            s.add_point(n(-0.2), n(0.8)),
        ];
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Vertical { line: l[3] });
        s.constrain(Constraint::Length {
            curve: l[0],
            value: n(2.0),
        });
        s.constrain(Constraint::Distance {
            a: p[1],
            b: p[2],
            value: n(1.0),
        });
        assert!(s.solve().unwrap().converged);
        let enclosed = s.enclose::<ScalInF64>().unwrap();
        let exact = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]];
        for (pi, e) in p.iter().zip(exact) {
            let q = enclosed.points[pi];
            for k in 0..2 {
                assert!(
                    q[k].could_be_equal(ScalInF64::from_f64(e[k])),
                    "{q:?} vs {e:?}"
                );
                assert!(q[k].width().to_f64() < 1e-12, "{q:?}");
            }
        }
    }

    /// What the constraints leave free is the designer's choice, kept
    /// exactly as drawn; only what they determine is enclosed.
    #[test]
    fn free_choices_stay_sharp() {
        let mut s = Sketch::new();
        let a = s.add_point(n(0.3), n(0.7));
        let b = s.add_point(n(1.9), n(0.75));
        let line = s.add_line(a, b);
        s.constrain(Constraint::Horizontal { line });
        assert!(s.solve().unwrap().converged);
        let enclosed = s.enclose::<ScalInF64>().unwrap();
        // Three of the four coordinates are free; one is set by the others.
        let sharp = [enclosed.points[&a], enclosed.points[&b]]
            .iter()
            .flat_map(|q| [q[0], q[1]])
            .filter(|v| v.is_sharp())
            .count();
        assert!(sharp >= 3, "{enclosed:?}");
        assert!(enclosed.points[&a][1].could_be_equal(enclosed.points[&b][1]));
        // Unmet constraints leave the sketch as drawn.
        s.constrain(Constraint::Vertical { line });
        s.constrain(Constraint::Fix {
            point: a,
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Fix {
            point: b,
            x: n(1.0),
            y: n(0.0),
        });
        let report = s.solve().unwrap();
        assert!(!report.converged);
        assert_eq!(s.enclose::<ScalInF64>().unwrap(), Enclosure::as_drawn(&s));
    }

    /// A fixed point is given, not solved for: a line hanging off it with
    /// a length swings its free end, the fixed one stays, and a point made
    /// coincident with it lands on it — whatever order the ids are in.
    #[test]
    fn fixed_points_stay_where_given() {
        let mut s = Sketch::new();
        let free = s.add_point(n(0.2), n(0.1));
        let end = s.add_point(n(2.0), n(0.3));
        let anchor = s.add_fixed_point(n(1.0), n(1.0));
        s.add_line(anchor, end);
        s.constrain(Constraint::Coincident { a: free, b: anchor });
        let line = s.add_line(free, end);
        s.constrain(Constraint::Length {
            curve: line,
            value: n(1.0),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        assert_eq!(xy(&s, anchor), [1.0, 1.0]);
        assert_eq!(xy(&s, free), [1.0, 1.0]);
        let e = xy(&s, end);
        assert!(close((e[0] - 1.0).hypot(e[1] - 1.0), 1.0), "{e:?}");
        // Only the free end's angle is left.
        assert_eq!(report.dof, 1, "{report:?}");
        assert!(!report.free_points[&anchor] && report.free_points[&end]);
        let enclosed = s.enclose::<ScalInF64>().unwrap();
        assert!(enclosed.points[&anchor][0].is_sharp());
    }

    /// Two fixed points made coincident either are where the other is, or
    /// the coincidence fails — neither is moved to make it hold.
    #[test]
    fn coincident_fixed_points_are_checked_not_merged() {
        let mut s = Sketch::new();
        let a = s.add_fixed_point(n(0.0), n(0.0));
        let b = s.add_fixed_point(n(1.0), n(0.0));
        let k = s.constrain(Constraint::Coincident { a, b });
        let report = s.solve().unwrap();
        assert_eq!(report.failed_constraints, vec![k]);
        assert_eq!(xy(&s, b), [1.0, 0.0]);
    }

    /// A fixed curve keeps its own radius: a circle fixed at radius 2
    /// cannot be given another.
    #[test]
    fn fixed_curves_keep_their_parameter() {
        let mut s = Sketch::new();
        let c = s.add_fixed_point(n(0.0), n(0.0));
        let circle = s.add_circle(c, n(2.0));
        s.curves.get_mut(&circle).unwrap().fixed = true;
        s.validate().unwrap();
        let k = s.constrain(Constraint::Diameter {
            curve: circle,
            value: n(3.0),
        });
        let report = s.solve().unwrap();
        assert_eq!(report.failed_constraints, vec![k]);
        let CurveKind::Circle { radius, .. } = s.curves[&circle].kind else {
            unreachable!()
        };
        assert_eq!(radius.to_f64(), 2.0);
        // A fixed curve on a free point is no fixed curve.
        let free = s.add_point(n(5.0), n(0.0));
        let other = s.add_circle(free, n(1.0));
        s.curves.get_mut(&other).unwrap().fixed = true;
        assert!(s.validate().is_err());
    }

    /// A diameter sizes a circle and an arc alike.
    #[test]
    fn diameter_sizes_circles_and_arcs() {
        let mut s = Sketch::new();
        let c = s.add_point(n(0.0), n(0.0));
        let circle = s.add_circle(c, n(0.7));
        s.constrain(Constraint::Diameter {
            curve: circle,
            value: n(3.0),
        });
        let a = s.add_point(n(5.0), n(0.0));
        let b = s.add_point(n(7.0), n(0.0));
        let arc = s.add_arc(a, b, n(1.0));
        s.constrain(Constraint::Fix {
            point: a,
            x: n(5.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Fix {
            point: b,
            x: n(7.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Diameter {
            curve: arc,
            value: n(4.0),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let CurveKind::Circle { radius, .. } = s.curves[&circle].kind else {
            unreachable!()
        };
        assert!(close(radius.to_f64(), 1.5));
        let CurveKind::Arc { sweep, .. } = s.curves[&arc].kind else {
            unreachable!()
        };
        // A chord of 2 on a circle of diameter 4 subtends 60°.
        assert!(close(sweep.to_f64(), PI / 3.0), "{sweep:?}");
    }

    /// A spline with a shape of its own is that NURBS: a rational quadratic
    /// with the middle weight `cos 45°` is a quarter circle.
    #[test]
    fn rational_splines_are_exact() {
        let mut s = Sketch::new();
        let p = [
            s.add_point(n(1.0), n(0.0)),
            s.add_point(n(1.0), n(1.0)),
            s.add_point(n(0.0), n(1.0)),
        ];
        let spline = s.add_curve(CurveKind::Spline {
            control_points: p.to_vec(),
            shape: Some(SplineShape {
                degree: 2,
                knots: [0.0, 0.0, 0.0, 2.0, 2.0, 2.0].map(n).to_vec(),
                weights: [1.0, 0.5f64.sqrt(), 1.0].map(n).to_vec(),
            }),
        });
        s.validate().unwrap();
        for q in profile::curve_polyline(&s, spline).unwrap() {
            let r = q[0].to_f64().hypot(q[1].to_f64());
            assert!(close(r, 1.0), "{q:?}");
        }
    }

    /// A point made an arc's center stays at the center as the arc's ends
    /// are moved: the center of a center-point arc.
    #[test]
    fn center_follows_the_arc() {
        let mut s = Sketch::new();
        let c = s.add_point(n(0.3), n(0.2));
        let a = s.add_point(n(1.0), n(0.0));
        let b = s.add_point(n(0.0), n(1.0));
        let arc = s.add_arc(a, b, n(FRAC_PI_2));
        s.constrain(Constraint::Center {
            point: c,
            curve: arc,
        });
        for (p, at) in [(a, [2.0, 0.0]), (b, [0.0, 2.0])] {
            s.constrain(Constraint::Fix {
                point: p,
                x: n(at[0]),
                y: n(at[1]),
            });
        }
        s.constrain(Constraint::Radius {
            curve: arc,
            value: n(2.0),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let q = xy(&s, c);
        assert!(close(q[0], 0.0) && close(q[1], 0.0), "{q:?}");
    }
}
