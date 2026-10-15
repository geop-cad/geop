//! Reusable named test scenes for the booleans test suite: pairs of solids,
//! each built in their own fresh [`NamedModel`] purely from
//! `geop_ops_extrude_revolve::shapes`.
//!
//! Two families: a systematic offset *grid* (every relative position of two
//! identical solids, on axis-aligned steps) and a set of hand-picked
//! box/figure8-vs-cylinder arrangements (holes drilled through, blind
//! holes, nested/disjoint, corner overlaps, coincident faces, etc.).

use geop_core_math::{
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_topology::{SolidId, validation::ValidationParameters};
use geop_ops_extrude_revolve::shapes::{
    block, cylinder, figure8_profile::figure8_profile, sphere::sphere_solid,
};
use geop_ops_model::{NamedModel, Namer};
use rayon::prelude::*;

use crate::remesh::remesh::{RemeshParams, remesh};

/// A named pair of solids, built together in one [`NamedModel`] as the
/// operations `a` and `b`, for a boolean sweep to combine.
pub struct TestScene<S: Scalar> {
    pub name: String,
    pub model: NamedModel<S>,
    pub solid_a: SolidId,
    pub solid_b: SolidId,
}

/// The scene `name` of [`all_scenes`].
pub fn scene(name: &str) -> TestScene<ScalInF64> {
    all_scenes()
        .into_iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| panic!("scene {name} must exist"))
}

/// The scene `name`, remeshed with the default [`RemeshParams`].
pub fn remeshed(name: &str) -> TestScene<ScalInF64> {
    let mut scene = scene(name);
    let namer = Namer::new("boolean", "ab").unwrap();
    remesh(
        &mut scene.model,
        &namer,
        scene.solid_a,
        scene.solid_b,
        RemeshParams::default(),
    )
    .unwrap_or_else(|e| panic!("{name}: remesh failed: {e}"));
    scene
}

/// Validation as tolerant as the default [`RemeshParams`]' pairwise
/// searches.
pub fn validation<S: Scalar>() -> ValidationParameters<S> {
    let params = RemeshParams::<S>::default();
    ValidationParameters {
        max_nodes: params.max_nodes,
        min_subdivision_size: params.curve_curve_min_subdivision_size,
        ..ValidationParameters::default()
    }
}

/// Turn an offset value into a filesystem-safe label, e.g. `-0.5` -> `n0p5`.
fn offset_label(v: f64) -> String {
    format!("{v:.2}").replace('-', "n").replace('.', "p")
}

/// A unit cube (side length 1) centered at `(cx, cy, cz)`.
pub fn unit_cube_at<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    cx: f64,
    cy: f64,
    cz: f64,
) -> SolidId {
    let (min, max) = ([cx, cy, cz].map(|c| c - 0.5), [cx, cy, cz].map(|c| c + 0.5));
    block(model, name, min, max)
}

// ─────────────────────────────── grid tests ────────────────────────────────

/// Two unit cubes: one fixed at the origin, the other swept across every
/// combination of `[-1, -0.5, 0, 0.5, 1]` along each axis (125 scenes) —
/// covering every axis-aligned relative position from fully disjoint through
/// every partial-overlap and face/edge/corner-coincident configuration.
pub fn box_grid_scenes<S: Scalar>() -> Vec<TestScene<S>> {
    let offsets = [-1.0_f64, -0.5, 0.0, 0.5, 1.0];
    let mut scenes = Vec::new();
    for &ox in &offsets {
        for &oy in &offsets {
            for &oz in &offsets {
                let mut model = NamedModel::<S>::new();
                let solid_a = unit_cube_at(&mut model, "a", 0.0, 0.0, 0.0);
                let solid_b = unit_cube_at(&mut model, "b", ox, oy, oz);
                let name = format!(
                    "box_grid_{}_{}_{}",
                    offset_label(ox),
                    offset_label(oy),
                    offset_label(oz)
                );
                scenes.push(TestScene {
                    name,
                    model,
                    solid_a,
                    solid_b,
                });
            }
        }
    }
    scenes
}

/// Two unit-radius spheres: one fixed at the origin, the other swept across
/// every combination of `[-0.5, 0, 0.5]` along each axis, skipping the fully
/// coincident `(0, 0, 0)` case (26 scenes) — that configuration is a
/// degenerate coplanar (every point of one sphere's surface touches the
/// other) rather than a transversal intersection, to be handled separately.
pub fn sphere_grid_scenes<S: Scalar>() -> Vec<TestScene<S>> {
    let offsets = [-0.5_f64, 0.0, 0.5];
    let mut scenes = Vec::new();
    for &ox in &offsets {
        for &oy in &offsets {
            for &oz in &offsets {
                if ox == 0.0 && oy == 0.0 && oz == 0.0 {
                    continue;
                }
                let mut model = NamedModel::<S>::new();
                let solid_a = sphere_solid(&mut model, "a", Vector3::zero(), S::ONE).unwrap();
                let center_b =
                    Vector3::from_array([S::from_f64(ox), S::from_f64(oy), S::from_f64(oz)]);
                let solid_b = sphere_solid(&mut model, "b", center_b, S::ONE).unwrap();
                let name = format!(
                    "sphere_grid_{}_{}_{}",
                    offset_label(ox),
                    offset_label(oy),
                    offset_label(oz)
                );
                scenes.push(TestScene {
                    name,
                    model,
                    solid_a,
                    solid_b,
                });
            }
        }
    }
    scenes
}

// ────────────────────────── box/figure8 vs cylinder ────────────────────────

/// One hand-picked `(name, cx, cy, cz, radius, height)` cylinder arrangement.
struct CylinderCase {
    name: &'static str,
    cx: f64,
    cy: f64,
    cz: f64,
    radius: f64,
    height: f64,
}

/// 12 arrangements against a unit box centered at the origin
/// (`[-0.5, 0.5]^3`): holes drilled through, blind holes, fully
/// nested/disjoint, axial/radial partial overlaps, corner/quarter overlaps,
/// coincident-face imprints, etc. — see the old `cylinder_test`
/// (pre-API-change) for the geometric reasoning behind each one.
const BOX_CYLINDER_CASES: &[CylinderCase] = &[
    CylinderCase {
        name: "drilled_hole_through",
        cx: 0.0,
        cy: 0.0,
        cz: -1.0,
        radius: 0.2,
        height: 2.0,
    },
    CylinderCase {
        name: "blind_hole",
        cx: 0.0,
        cy: 0.0,
        cz: -1.0,
        radius: 0.2,
        height: 1.2,
    },
    CylinderCase {
        name: "fully_enclosed",
        cx: 0.0,
        cy: 0.0,
        cz: -0.2,
        radius: 0.1,
        height: 0.4,
    },
    CylinderCase {
        name: "fully_outside",
        cx: 2.0,
        cy: 2.0,
        cz: -0.5,
        radius: 0.2,
        height: 1.0,
    },
    CylinderCase {
        name: "axial_half_out_top",
        cx: 0.0,
        cy: 0.0,
        cz: 0.0,
        radius: 0.2,
        height: 1.0,
    },
    CylinderCase {
        name: "radial_half_out",
        cx: 0.5,
        cy: 0.0,
        cz: -0.2,
        radius: 0.3,
        height: 0.4,
    },
    CylinderCase {
        name: "engulfing_disk",
        cx: 0.0,
        cy: 0.0,
        cz: -0.1,
        radius: 0.9,
        height: 0.2,
    },
    CylinderCase {
        name: "corner_quarter_overlap",
        cx: 0.5,
        cy: 0.5,
        cz: -0.2,
        radius: 0.3,
        height: 0.4,
    },
    CylinderCase {
        name: "sliver_overlap",
        cx: 1.0,
        cy: 0.0,
        cz: -0.2,
        radius: 0.51,
        height: 0.4,
    },
    CylinderCase {
        name: "off_axis_hole",
        cx: 0.3,
        cy: 0.3,
        cz: -1.0,
        radius: 0.15,
        height: 2.0,
    },
    CylinderCase {
        name: "axial_and_radial_partial",
        cx: 0.0,
        cy: 0.0,
        cz: 0.3,
        radius: 0.6,
        height: 0.4,
    },
    CylinderCase {
        name: "cap_coincident_with_faces",
        cx: 0.0,
        cy: 0.0,
        cz: -0.5,
        radius: 0.2,
        height: 1.0,
    },
];

/// 12 arrangements against `figure8_profile` — outer footprint `x, y in [0,
/// 1]`, two square holes centered at `(0.1875, 0.5)` and `(0.8125, 0.5)`,
/// each spanning `y in [1/3, 2/3]`, a narrow neck at `x in [0.375, 0.625]`
/// joining the two lobes, spanning `z in [-1, 0]`. `cz` here is every case's
/// old (pre-API-change) value shifted by `-1`, since `figure8_profile` used
/// to be built at `z in [0, 1]` and is now fixed at `z in [-1, 0]`.
const FIGURE8_CYLINDER_CASES: &[CylinderCase] = &[
    CylinderCase {
        name: "through_left_hole_clear",
        cx: 0.1875,
        cy: 0.5,
        cz: -1.5,
        radius: 0.05,
        height: 2.0,
    },
    CylinderCase {
        name: "through_left_hole_oversized",
        cx: 0.1875,
        cy: 0.5,
        cz: -1.5,
        radius: 0.1,
        height: 2.0,
    },
    CylinderCase {
        name: "through_right_hole_clear",
        cx: 0.8125,
        cy: 0.5,
        cz: -1.5,
        radius: 0.05,
        height: 2.0,
    },
    CylinderCase {
        name: "through_solid_new_hole",
        cx: 0.1875,
        cy: 0.15,
        cz: -1.5,
        radius: 0.08,
        height: 2.0,
    },
    CylinderCase {
        name: "through_neck",
        cx: 0.5,
        cy: 0.5,
        cz: -1.5,
        radius: 0.1,
        height: 2.0,
    },
    CylinderCase {
        name: "through_neck_oversized",
        cx: 0.5,
        cy: 0.5,
        cz: -1.5,
        radius: 0.2,
        height: 2.0,
    },
    CylinderCase {
        name: "straddle_neck_lobe_boundary",
        cx: 0.4,
        cy: 0.5,
        cz: -1.5,
        radius: 0.1,
        height: 2.0,
    },
    CylinderCase {
        name: "fully_outside_footprint",
        cx: 2.0,
        cy: 2.0,
        cz: -1.5,
        radius: 0.1,
        height: 1.0,
    },
    CylinderCase {
        name: "engulfing_thin_slice",
        cx: 0.5,
        cy: 0.5,
        cz: -0.7,
        radius: 1.5,
        height: 0.1,
    },
    CylinderCase {
        name: "cap_coincident_with_faces",
        cx: 0.1875,
        cy: 0.15,
        cz: -1.0,
        radius: 0.1,
        height: 1.0,
    },
    CylinderCase {
        name: "quarter_overlap_hole_corner",
        cx: 0.125,
        cy: 1.0 / 3.0,
        cz: -1.5,
        radius: 0.08,
        height: 2.0,
    },
    CylinderCase {
        name: "engulfing_both_lobes",
        cx: 0.5,
        cy: 0.5,
        cz: -1.5,
        radius: 0.7,
        height: 2.0,
    },
];

/// The `BOX_CYLINDER_CASES` arrangements, each as its own `TestScene`
/// (`solid_a` the box, `solid_b` the cylinder), named `box_cylinder_<case>`.
pub fn box_cylinder_scenes<S: Scalar>() -> Vec<TestScene<S>> {
    BOX_CYLINDER_CASES
        .iter()
        .map(|c| {
            let mut model = NamedModel::<S>::new();
            let solid_a = unit_cube_at(&mut model, "a", 0.0, 0.0, 0.0);
            let solid_b = cylinder(&mut model, "b", [c.cx, c.cy, c.cz], c.radius, c.height);
            TestScene {
                name: format!("box_cylinder_{}", c.name),
                model,
                solid_a,
                solid_b,
            }
        })
        .collect()
}

/// The `FIGURE8_CYLINDER_CASES` arrangements, each as its own `TestScene`
/// (`solid_a` the figure-8 extrusion, `solid_b` the cylinder), named
/// `figure8_cylinder_<case>`.
fn figure8_cylinder_scenes<S: Scalar>() -> Vec<TestScene<S>> {
    FIGURE8_CYLINDER_CASES
        .iter()
        .map(|c| {
            let mut model = NamedModel::<S>::new();
            let solid_a = figure8_profile(&mut model, "a").unwrap();
            let solid_b = cylinder(&mut model, "b", [c.cx, c.cy, c.cz], c.radius, c.height);
            TestScene {
                name: format!("figure8_cylinder_{}", c.name),
                model,
                solid_a,
                solid_b,
            }
        })
        .collect()
}

/// Every scene in the suite: both grid sweeps, then both hand-picked
/// cylinder arrangement sets.
pub fn all_scenes<S: Scalar>() -> Vec<TestScene<S>> {
    let mut scenes = box_grid_scenes::<S>();
    scenes.extend(sphere_grid_scenes::<S>());
    scenes.extend(box_cylinder_scenes::<S>());
    scenes.extend(figure8_cylinder_scenes::<S>());
    scenes
}

/// Run `f` over every scene, spread across threads, and return the results in
/// scene order.
///
/// Each scene owns its own `NamedModel` and shares nothing with the others, so the
/// sweeps over them are embarrassingly parallel — and slow enough
/// (a full remesh or boolean per scene, 175 of them) that running them serially
/// dominates the test suite's wall time. `rayon`'s global pool (fixed at
/// `available_parallelism` threads, lazily started once per process) does the
/// spreading, one job per scene — never more than one thread working on any
/// single scene.
///
/// The pool is process-wide and shared by every caller, in this crate or
/// otherwise, that uses rayon — critically including the several render
/// tests that each call this function and that `cargo test` runs
/// concurrently by default. Spawning a fresh `std::thread::scope` of
/// `available_parallelism` threads per call (the previous approach) let
/// those tests oversubscribe the machine several times over, which was
/// enough contention to make an unrelated timing-sensitive test fail
/// spuriously. Going through the shared pool instead means concurrent
/// callers queue for the same fixed set of workers rather than each getting
/// their own, so total parallelism across the whole test binary stays
/// bounded at the core count.
///
/// `par_iter`'s `collect` preserves the source order regardless of which
/// worker finished a given scene first, so a caller's output does not depend
/// on scheduling.
pub fn map_scenes_parallel<S, T, F>(f: F) -> Vec<T>
where
    S: Scalar,
    T: Send,
    F: Fn(TestScene<S>) -> T + Sync + Send,
{
    all_scenes::<S>().into_par_iter().map(f).collect()
}

#[cfg(test)]
mod topology_render_test {
    use super::*;
    use geop_core_math::scalars::ScalInF64;
    use geop_core_rasterize::{debug::Color10, rasterize};

    /// Render every scene's raw topology (no boolean/remesh operation at
    /// all — just the two solids as built) to `outputs/topology_tests/`,
    /// `solid_a`'s faces in blue and `solid_b`'s in purple: every scene
    /// builds and rasterizes cleanly, before any boolean touches it.
    ///
    /// `ScalInF64` only, not `for_all_scalars!`: this sweeps 175 scenes (125
    /// box-grid + 26 sphere-grid + 24 hand-picked), and interval-arithmetic
    /// scalars pay a real per-operation cost that isn't worth it for a pure
    /// rendering smoke test — see `remesh_test`'s own doc comment for the
    /// same tradeoff.
    #[test]
    fn render_all_scenes_topology() {
        let dir = "outputs/topology_tests";
        std::fs::create_dir_all(dir).unwrap();

        for scene in all_scenes::<ScalInF64>() {
            let model = scene.model.topology();
            let faces_a = model.solid_faces(scene.solid_a).unwrap();
            let scene_render = rasterize(model, 12).unwrap().scene(|face_id| {
                if faces_a.contains(&face_id) {
                    Color10::Blue
                } else {
                    Color10::Purple
                }
            });
            scene_render
                .save_to_file(&format!("{dir}/{}.html", scene.name))
                .unwrap();
        }
    }
}
