use geop_core_geometry::{intersection::curve_curve_intersect, nurb_curve::NurbCurve};
use geop_core_math::{
    primitives::{Color10, PrimitiveScene},
    scalars::{ScalInF64, Scalar},
    vector::Vector4,
};

fn f(v: f64) -> ScalInF64 {
    ScalInF64::from_f64(v)
}

/// Homogeneous control point with unit weight.
fn pt(x: f64, y: f64, z: f64) -> Vector4<ScalInF64> {
    Vector4::from_array([f(x), f(y), f(z), ScalInF64::ONE])
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

/// Run `curve_curve_intersect`, print the results, and render a scene with
/// both curves (plus a marker point at each solution) to `out_path`.
fn run_case(
    label: &str,
    curve_a: &NurbCurve<ScalInF64, 4>,
    curve_b: &NurbCurve<ScalInF64, 4>,
    max_solutions: usize,
    epsilon: ScalInF64,
    out_path: &str,
) {
    let result = curve_curve_intersect(curve_a, curve_b, max_solutions, 2000, epsilon)
        .expect("intersection");
    let coincident = result.is_coincident();
    let solutions = result.into_vec();

    println!("\n== {label} ==");
    println!(
        "Found {} solution(s){}:",
        solutions.len(),
        if coincident { " (coincident)" } else { "" }
    );
    for (i, &(t_a, t_b)) in solutions.iter().enumerate() {
        println!(
            "  [{i}] a t = {:.6}, b t = {:.6}",
            t_a.to_f64(),
            t_b.to_f64()
        );
    }

    let mut scene = PrimitiveScene::<ScalInF64>::new();

    let (a_min, a_max) = curve_a.domain();
    scene
        .add_curve(curve_a, a_min, a_max, Color10::Blue, 32)
        .expect("render curve a");

    let (b_min, b_max) = curve_b.domain();
    scene
        .add_curve(curve_b, b_min, b_max, Color10::Green, 32)
        .expect("render curve b");

    for (i, &(t_a, _)) in solutions.iter().enumerate() {
        let color = SOLUTION_COLORS[i % SOLUTION_COLORS.len()];
        let hit = curve_a.evaluate(t_a).expect("evaluate solution point");
        scene.add_point(hit, color);
    }

    scene.save_to_file(out_path).expect("save scene");
    println!("Wrote {out_path}");
}

fn main() {
    // ── Case 1: single crossing ─────────────────────────────────────────────
    // A horizontal line and a vertical line crossing it once at (0.5, 0.3, 0).
    {
        let curve_a = NurbCurve::try_new(
            1,
            vec![pt(0., 0.3, 0.), pt(1., 0.3, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("horizontal line");

        let curve_b = NurbCurve::try_new(
            1,
            vec![pt(0.5, -1., 0.), pt(0.5, 1., 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("vertical line");

        run_case(
            "single crossing",
            &curve_a,
            &curve_b,
            5,
            f(0.02),
            "curve_curve_intersection_single.html",
        );
    }

    // ── Case 2: two distinct crossings ──────────────────────────────────────
    // A horizontal line and a quadratic curve that dips below it and back,
    // crossing it at two separate points.
    {
        let curve_a = NurbCurve::try_new(
            1,
            vec![pt(0., 0.3, 0.), pt(1., 0.3, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("horizontal line");

        let curve_b = NurbCurve::try_new(
            2,
            vec![pt(0.1, 1.0, 0.), pt(0.5, -2.0, 0.), pt(0.9, 1.0, 0.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .expect("double-dip curve");

        run_case(
            "two distinct crossings",
            &curve_a,
            &curve_b,
            4,
            f(0.02),
            "curve_curve_intersection_two_crossings.html",
        );
    }

    // ── Case 3: coincident overlap ───────────────────────────────────────────
    // Two collinear segments along y=0.3, z=0, overlapping in x ∈ [0.5, 1].
    {
        let curve_a = NurbCurve::try_new(
            1,
            vec![pt(0., 0.3, 0.), pt(1., 0.3, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("line a");

        let curve_b = NurbCurve::try_new(
            1,
            vec![pt(0.5, 0.3, 0.), pt(1.5, 0.3, 0.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .expect("line b");

        run_case(
            "coincident overlap",
            &curve_a,
            &curve_b,
            8,
            f(0.02),
            "curve_curve_intersection_coincident.html",
        );
    }
}
