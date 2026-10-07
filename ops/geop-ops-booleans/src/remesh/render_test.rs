//! Visual check for `remesh`: run it over every `scenes::all_scenes` scene
//! and render the resulting topology into `outputs/topology_remesh/` —
//! a purely visual sanity check, not a correctness assertion, mirroring
//! `render_test`'s own tradeoffs.
//!
//! `remesh` merges/splits on vertices and edges that already coincide, and
//! (via `remesh_edges_x_faces`) traces and imprints any face x face
//! intersection curves, so for most of these scenes it's a no-op; the
//! scenes with touching corners, edges, or intersecting faces (e.g. the box
//! grid's coincident-face offsets) are the ones where it actually does
//! something. Either way, running it over the whole scene set is a broad
//! smoke test that it never panics or corrupts the model on arbitrary
//! topology. A scene that errors is logged to stderr and skipped rather
//! than failing the test, same as `render_test`.

#[cfg(test)]
mod tests {
    use crate::{
        remesh::remesh::{RemeshParams, remesh},
        scenes::map_scenes_parallel,
    };
    use geop_core_math::scalars::ScalInF64;
    use geop_core_topology::validation::{ValidationParameters, validate, validate_fast};
    use geop_ops::Namer;
    use geop_ops_rasterize::debug::rasterize_topology;

    /// Runs `remesh` over every scene and renders the result. `full_validate`
    /// additionally runs the expensive pairwise-search `validate` (not just
    /// the cheap structural `validate_fast`) on every scene that passes
    /// `validate_fast` — off by default since it's what makes
    /// `render_all_scenes_remesh_full_validate` slow; see its own doc
    /// comment for why it's worth having as a separate, on-demand pass.
    fn run_all_scenes(full_validate: bool) {
        let dir = "outputs/topology_remesh";
        std::fs::create_dir_all(dir).unwrap();

        let params = RemeshParams::<ScalInF64>::default();
        // `validate_fast` now runs `check_edges_disjoint`, the same kind of
        // pairwise `curve_curve_intersect` search `remesh_edges_x_edges`
        // itself uses — so it needs at least as much budget/tolerance as
        // `remesh` was given above, or it can spuriously fail to reach a
        // conclusion `remesh` already reached with more room to work with.
        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };

        // One independent remesh per scene, so they run in parallel; see
        // `scenes::map_scenes_parallel`. Each thread writes its own
        // `<scene>.html`, so the file writes do not collide either.
        let outcomes = map_scenes_parallel::<ScalInF64, _, _>(|mut scene| {
            let mut failure = None;
            let mut invalid = None;
            let mut full_invalid = None;

            let namer = Namer::new("boolean", "ab").unwrap();
            if let Err(e) = remesh(
                &mut scene.part,
                &namer,
                scene.solid_a,
                scene.solid_b,
                params,
            ) {
                failure = Some(format!("{e}"));
            } else if let Err(e) = scene.part.check_names() {
                failure = Some(format!("{e}"));
            }
            let model = scene.part.topology();

            // Structural well-formedness (pointers, two-way references,
            // vertex/curve/surface consistency, pcurve loop continuity) is
            // cheap enough to check after every scene, unlike `validate`'s
            // pairwise numerical-intersection searches — catches a `remesh`
            // that "succeeds" but leaves a structurally broken model behind
            // (dangling ids, a loop that no longer closes, a vertex that no
            // longer matches its edges' curves), which the `Ok(())` return
            // above wouldn't reveal on its own.
            if let Err(errors) = validate_fast(&validation_params, model) {
                invalid = Some(errors);
            } else if full_validate {
                // `validate_fast` is purely structural — it can't catch a
                // model that's structurally fine but geometrically wrong
                // (e.g. two edges that now numerically cross where they
                // shouldn't). Only worth the expensive pairwise searches
                // here since `validate_fast` already passed.
                if let Err(errors) = validate(&validation_params, model) {
                    full_invalid = Some(errors);
                }
            }

            // Even a failed remesh may have left the model partway mutated
            // (e.g. one edge split before a later step errored) — still try
            // to render whatever state it's in, but don't let a corrupted
            // model take the whole test down. `rasterize_topology` (rather
            // than the plain face-colored render) labels every vertex,
            // edge, and coedge with its id, so a failure's `VertexId`/
            // `CoedgeId` can be located directly in the output.
            let render_error = match rasterize_topology(model, 12) {
                Ok(render) => render
                    .save_to_file(&format!("{dir}/{}.html", scene.name))
                    .err()
                    .map(|e| format!("{e}")),
                Err(e) => Some(format!("{e}")),
            };

            (scene.name, failure, invalid, full_invalid, render_error)
        });

        let scene_count = outcomes.len();
        let mut error_count = 0usize;
        let mut invalid_count = 0usize;
        // Reported after the parallel pass, in scene order, so the log reads
        // the same however the work was scheduled.
        for (name, failure, invalid, full_invalid, render_error) in outcomes {
            if let Some(e) = failure {
                error_count += 1;
                eprintln!("remesh failed for scene {name:?}: {e}");
            }
            if let Some(errors) = invalid {
                invalid_count += 1;
                eprintln!(
                    "validate_fast failed for scene {name:?} after remesh ({} error(s)):",
                    errors.len()
                );
                for e in &errors {
                    eprintln!("  {e}");
                }
            }
            if let Some(errors) = full_invalid {
                eprintln!(
                    "validate (full) failed for scene {name:?} after remesh, despite validate_fast passing ({} error(s)):",
                    errors.len()
                );
                for e in &errors {
                    eprintln!("  {e}");
                }
            }
            if let Some(e) = render_error {
                eprintln!("rasterize failed for scene {name:?} after remesh: {e}");
            }
        }

        eprintln!("render_all_scenes_remesh: {error_count}/{scene_count} scenes failed to remesh");
        eprintln!(
            "render_all_scenes_remesh: {invalid_count}/{scene_count} scenes failed validate_fast"
        );
    }

    #[test]
    #[ignore = "slow: 6 s — run with `cargo test -- --ignored`"]
    fn render_all_scenes_remesh() {
        run_all_scenes(false);
    }

    /// Same sweep as `render_all_scenes_remesh`, but also runs the full,
    /// expensive `validate` (pairwise disjointness/face-face searches) on
    /// every scene `validate_fast` already passed — `validate_fast` alone
    /// can't tell a structurally-sound model from one that's geometrically
    /// wrong (e.g. two edges left numerically crossing because a genuine
    /// crossing point got lost — see `remesh_edges_x_edges`'s own doc
    /// comment on `curve_curve_intersect`'s known fragility for a crossing
    /// located exactly at a shared curve-domain boundary). Takes several
    /// minutes over all 175 scenes, hence `#[ignore]`.
    #[test]
    #[ignore = "slow: several minutes, full geometric validate() pairwise search on top of remesh — run explicitly with `cargo test -- --ignored`"]
    fn render_all_scenes_remesh_full_validate() {
        run_all_scenes(true);
    }
}
