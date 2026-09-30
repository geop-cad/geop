//! Visual check for the boolean operators: run union, intersection and
//! difference over every `scenes::all_scenes` scene and render each result
//! into its own `outputs/booleans_<op>/` folder.
//!
//! Like `remesh::render_test`, this is a breadth-first smoke test rather than
//! a correctness assertion — the assertions about *which* faces a boolean
//! keeps live in `boolean`'s own tests, where a handful of probe
//! points can be checked against a known-answer scene. What this adds is
//! coverage: every scene, every operator, so a configuration that makes an
//! operator panic, error, or produce a structurally broken solid shows up
//! somewhere. Counts are reported to stderr and a failing scene is rendered
//! anyway (in whatever state it reached) rather than taking the test down,
//! so one bad scene never hides the other 174.

#[cfg(test)]
mod tests {
    use crate::{
        boolean::{BooleanOp, boolean},
        remesh::remesh::RemeshParams,
        scenes::map_scenes_parallel,
    };
    use geop_core_math::scalars::ScalInF64;
    use geop_core_topology::validation::{ValidationParameters, validate_fast};
    use geop_ops::Namer;
    use geop_ops_rasterize::debug::rasterize_topology;

    fn run_op(op: BooleanOp, dir: &str) {
        std::fs::create_dir_all(dir).unwrap();

        let params = RemeshParams::<ScalInF64>::default();
        // Matched to what `boolean` itself was given, for the same reason
        // `remesh::render_test` matches them: a validation search with less
        // budget than the operation it is checking can fail to reach a
        // conclusion the operation already reached.
        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };

        // One independent remesh + boolean per scene, so they run in
        // parallel; see `scenes::map_scenes_parallel`. Each thread writes its
        // own `<scene>.html`, so the file writes do not collide either.
        let outcomes = map_scenes_parallel::<ScalInF64, _, _>(|mut scene| {
            let mut failure = None;
            let mut invalid = None;
            let mut empty = false;

            let namer = Namer::new("boolean", "ab").unwrap();
            match boolean(
                &mut scene.part,
                &namer,
                scene.solid_a,
                scene.solid_b,
                op,
                params,
            ) {
                // An empty result is an answer, not a failure — most of these
                // scenes place the two solids apart, so their intersection is
                // legitimately nothing.
                Ok(None) => empty = true,
                Ok(Some(_)) => {
                    if let Err(errors) = validate_fast(&validation_params, scene.part.topology()) {
                        invalid = Some(errors);
                    }
                }
                Err(e) => failure = Some(e),
            }
            // Whatever happened, no entity may be left without a name.
            if failure.is_none()
                && let Err(e) = scene.part.check_names()
            {
                failure = Some(e);
            }

            // Even a failed boolean may have left the model partway mutated,
            // so render whatever state it reached — that picture is the whole
            // point of this test for exactly those scenes. A model too broken
            // to rasterize is skipped rather than taking the run down.
            let render_error = match rasterize_topology(scene.part.topology(), 12) {
                Ok(render) => render
                    .save_to_file(&format!("{dir}/{}.html", scene.name))
                    .err()
                    .map(|e| format!("{e}")),
                Err(e) => Some(format!("{e}")),
            };

            (scene.name, failure, invalid, empty, render_error)
        });

        let scene_count = outcomes.len();
        let mut error_count = 0usize;
        let mut invalid_count = 0usize;
        let mut empty_count = 0usize;
        // Reported after the parallel pass, in scene order, so the log reads
        // the same however the work was scheduled.
        for (name, failure, invalid, empty, render_error) in outcomes {
            if let Some(e) = failure {
                error_count += 1;
                eprintln!("{op:?} failed for scene {name:?}: {e}");
            }
            if let Some(errors) = invalid {
                invalid_count += 1;
                eprintln!(
                    "validate_fast failed for scene {name:?} after {op:?} ({} error(s)):",
                    errors.len()
                );
                for e in &errors {
                    eprintln!("  {e}");
                }
            }
            if empty {
                empty_count += 1;
            }
            if let Some(e) = render_error {
                eprintln!("rasterize failed for scene {name:?} after {op:?}: {e}");
            }
        }

        eprintln!("{op:?}: {error_count}/{scene_count} scenes failed to compute");
        eprintln!("{op:?}: {empty_count}/{scene_count} scenes produced an empty result");
        eprintln!("{op:?}: {invalid_count}/{scene_count} scenes failed validate_fast");
    }

    #[test]
    fn render_all_scenes_union() {
        run_op(BooleanOp::Union, "outputs/booleans_union");
    }

    #[test]
    fn render_all_scenes_intersection() {
        run_op(BooleanOp::Intersection, "outputs/booleans_intersection");
    }

    #[test]
    fn render_all_scenes_difference() {
        run_op(BooleanOp::Difference, "outputs/booleans_difference");
    }
}
