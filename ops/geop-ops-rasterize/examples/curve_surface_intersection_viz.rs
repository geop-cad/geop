use geop_core_geometry::{
    intersection::curve_surface_intersect, nurb_curve::NurbCurve, nurb_surface::NurbSurface,
};
use geop_core_math::{
    scalars::{ScalInF64, Scalar},
    vector::Vector4,
};
use geop_ops_rasterize::debug::{Color10, PrimitiveScene};

fn f(v: f64) -> ScalInF64 {
    ScalInF64::from_f64(v)
}

/// Homogeneous control point with unit weight.
fn pt(x: f64, y: f64, z: f64) -> Vector4<ScalInF64> {
    Vector4::from_array([f(x), f(y), f(z), ScalInF64::ONE])
}

/// Homogeneous control point with weight `w`, given its Cartesian position
/// `(x, y, z)` (used for rational curves/surfaces such as circles/spheres).
fn pt_w(x: f64, y: f64, z: f64, w: f64) -> Vector4<ScalInF64> {
    Vector4::from_array([f(x * w), f(y * w), f(z * w), f(w)])
}

const SOLUTION_COLORS: [Color10; 8] = [
    Color10::Red,
    Color10::Orange,
    Color10::Purple,
    Color10::Brown,
    Color10::Pink,
    Color10::Gray,
    Color10::Olive,
    Color10::Cyan,
];

/// Run `curve_surface_intersect`, print the results, and render a scene
/// with the surface, the curve, and a marker point at each solution to
/// `out_path`.
fn run_case(
    label: &str,
    curve: &NurbCurve<ScalInF64, 4>,
    surface: &NurbSurface<ScalInF64, 4>,
    max_solutions: usize,
    epsilon: ScalInF64,
    out_path: &str,
) {
    let result = curve_surface_intersect(curve, surface, max_solutions, 2000, epsilon)
        .expect("intersection");
    let coincident = result.is_coincident();
    let solutions = result.into_vec();

    println!("\n== {label} ==");
    println!(
        "Found {} solution(s){}:",
        solutions.len(),
        if coincident { " (coincident)" } else { "" }
    );
    for (i, &(t, uv)) in solutions.iter().enumerate() {
        println!(
            "  [{i}] curve t = {:.6}, surface (u,v) = ({:.6}, {:.6})",
            t.to_f64(),
            uv[0].to_f64(),
            uv[1].to_f64()
        );
    }

    let mut scene = PrimitiveScene::<ScalInF64>::new();

    let (u_min, u_max) = surface.domain_u();
    let (v_min, v_max) = surface.domain_v();
    scene
        .add_surface_wireframe(surface, Color10::Green, u_min, u_max, v_min, v_max, 12)
        .expect("render surface");

    let (t_min, t_max) = curve.domain();
    scene
        .add_curve(curve, t_min, t_max, Color10::Blue, 32)
        .expect("render curve");

    for (i, &(t, _)) in solutions.iter().enumerate() {
        let color = SOLUTION_COLORS[i % SOLUTION_COLORS.len()];
        let hit = curve.evaluate(t).expect("evaluate solution point");
        scene.add_point(hit, color);
    }

    scene.save_to_file(out_path).expect("save scene");
    println!("Wrote {out_path}");
}

fn main() {
    // ── Case 1: axis-aligned coplanar curve ────────────────────────────────
    // A flat surface patch in the xy-plane (z = 0), spanning x,y ∈ [0,2], and
    // a straight line lying *in* the surface's plane, parallel to the x axis.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 2., 0.),
                pt(2., 0., 0.),
                pt(2., 2., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("flat surface");

        let curve = NurbCurve::try_new(
            1,
            vec![pt(0.1, 1.0, 0.), pt(1.9, 1.0, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("coplanar curve");

        run_case(
            "axis-aligned coplanar curve",
            &curve,
            &surface,
            8,
            f(0.02),
            "curve_surface_intersection_coplanar.html",
        );
    }

    // ── Case 2: angled coplanar curve ──────────────────────────────────────
    // Same flat surface as above, but the curve crosses it diagonally instead
    // of being axis-aligned.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 2., 0.),
                pt(2., 0., 0.),
                pt(2., 2., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("flat surface");

        let curve = NurbCurve::try_new(
            1,
            vec![pt(0.2, 0.3, 0.), pt(1.7, 1.6, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("angled coplanar curve");

        run_case(
            "angled coplanar curve",
            &curve,
            &surface,
            8,
            f(0.02),
            "curve_surface_intersection_angled.html",
        );
    }

    // ── Case 3: coincident sphere octant and circle patch ──────────────────
    // A degree-(2,2) rational NURBS patch representing one octant of the unit
    // sphere (x,y,z >= 0), built by revolving a quarter-circle meridian
    // (in the xz-plane) by a quarter turn around the z axis. Its v=0 edge is
    // exactly the quarter-circle from (1,0,0) to (0,1,0) in the xy-plane,
    // which is also used as the intersecting curve -- a true coincident
    // overlap on curved geometry.
    {
        let w = 1.0 / 2.0_f64.sqrt();

        let surface = NurbSurface::try_new(
            2,
            2,
            vec![
                // u = 0 (azimuth 0deg)
                pt_w(1., 0., 0., 1.),
                pt_w(1., 0., 1., w),
                pt_w(0., 0., 1., 1.),
                // u = 1 (azimuth 45deg)
                pt_w(1., 1., 0., w),
                pt_w(1., 1., 1., 0.5),
                pt_w(0., 0., 1., w),
                // u = 2 (azimuth 90deg)
                pt_w(0., 1., 0., 1.),
                pt_w(0., 1., 1., w),
                pt_w(0., 0., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .expect("sphere octant patch");

        let curve = NurbCurve::try_new(
            2,
            vec![
                pt_w(1., 0., 0., 1.),
                pt_w(1., 1., 0., w),
                pt_w(0., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .expect("equator quarter-circle");

        run_case(
            "coincident sphere octant and circle patch",
            &curve,
            &surface,
            8,
            f(0.02),
            "curve_surface_intersection_sphere_circle.html",
        );
    }

    // ── Case 4: bent surface, bent curve -- distinct crossing points ───────
    // A non-planar bilinear "saddle" patch (corner heights 0,1,1,0) and a
    // quadratic curve running along its ridge line (z = 0.5) that dips below
    // and rises above the ridge, crossing the surface at two separate points.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 2., 1.),
                pt(2., 0., 1.),
                pt(2., 2., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("bent surface");

        let curve = NurbCurve::try_new(
            2,
            vec![pt(0.2, 1.0, -1.0), pt(1.0, 1.0, 3.0), pt(1.8, 1.0, -1.0)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .expect("bent curve");

        run_case(
            "bent surface, bent curve -- distinct crossings",
            &curve,
            &surface,
            4,
            f(0.02),
            "curve_surface_intersection_bent.html",
        );
    }

    // ── Case 5: partial overlap ─────────────────────────────────────────────
    // A flat unit surface patch (x,y ∈ [0,1], z = 0) and a coplanar line
    // spanning x ∈ [-0.5, 0.5] at y=0.5 -- only the x ∈ [0, 0.5] half of the
    // line lies over the surface.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 1., 0.),
                pt(1., 0., 0.),
                pt(1., 1., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("flat unit surface");

        let curve = NurbCurve::try_new(
            1,
            vec![pt(-0.5, 0.5, 0.), pt(0.5, 0.5, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("partial-overlap line");

        run_case(
            "partial overlap",
            &curve,
            &surface,
            8,
            f(0.02),
            "curve_surface_intersection_partial_overlap.html",
        );
    }

    // ── Case 6: curve larger than the surface ───────────────────────────────
    // Same flat unit surface, but the line spans x ∈ [-1, 2] at y=0.5 --
    // extending well beyond the surface on both sides. Only the middle third
    // (x ∈ [0,1]) overlaps.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 1., 0.),
                pt(1., 0., 0.),
                pt(1., 1., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("flat unit surface");

        let curve = NurbCurve::try_new(
            1,
            vec![pt(-1.0, 0.5, 0.), pt(2.0, 0.5, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("oversized line");

        run_case(
            "curve larger than surface",
            &curve,
            &surface,
            5,
            f(0.02),
            "curve_surface_intersection_oversized.html",
        );
    }

    // ── Case 7: curve the same size as the surface ──────────────────────────
    // Same flat unit surface, with the line spanning exactly x ∈ [0,1] at
    // y=0.5 -- matching the surface's extent.
    {
        let surface = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0.),
                pt(0., 1., 0.),
                pt(1., 0., 0.),
                pt(1., 1., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("flat unit surface");

        let curve = NurbCurve::try_new(
            1,
            vec![pt(0., 0.5, 0.), pt(1., 0.5, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("full-width line");

        run_case(
            "curve same size as surface",
            &curve,
            &surface,
            8,
            f(0.02),
            "curve_surface_intersection_same_size.html",
        );
    }
}
