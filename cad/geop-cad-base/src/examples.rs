//! Example programs, written in Rust against the operation types.
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use geop_core_math::{primitives::Pose, scalars::Scalar, vector::Vector3};
use geop_core_program::{program::Program, values::Design};

use geop_core_editor::registry::{Example, ExampleSet};

/// A number of an example, as design data.
pub(crate) fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// A pose of an example: at `position`, turned by `degrees` (see
/// [`Pose::from_euler`]).
#[allow(
    dead_code,
    reason = "pruned with the other shared helpers once every example has moved"
)]
pub(crate) fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n))
        .expect("an example's rotation is one")
}

/// An example drawn in the editor, kept as the program it saved (in
/// `src/drawn/`): its sketches as they were drawn, which no code here says
/// better. Its tests build it like every other example, so a change to the
/// format that breaks it is found there: save it again from the editor.
fn drawn(text: &str) -> Program {
    Program::from_json(text).expect("a drawn example is a program")
}

/// Every example, as the editor offers them (see [`all`]).
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: all(),
        workspaces: Vec::new(),
    }
}

/// Every example, by name, each with what makes its program: listing them
/// builds nothing — an editor lists them when it starts, and making one
/// may solve sketches, or build a part to name its faces.
pub fn all() -> Vec<Example<Program>> {
    vec![
        ("turbine_blade", || {
            drawn(include_str!("drawn/turbine_blade.geop"))
        }),
        ("cool_flange", || {
            drawn(include_str!("drawn/cool_flange.geop"))
        }),
        ("edge_part", || drawn(include_str!("drawn/edge_part.geop"))),
        ("conebox", || drawn(include_str!("drawn/conebox.geop"))),
    ]
}

#[cfg(test)]
mod tests {
    use geop_core_math::scalars::ScalInF64 as S;

    use geop_ops_model::{NamedModel, PartDescription, RefId};
    use std::collections::BTreeMap;

    use crate::registry::testing::{Built, check_valid};

    use super::*;

    fn outputs_dir() -> std::path::PathBuf {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../outputs/parts");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Every named entity's exact geometry, by name: vertex points, edge
    /// curves and face surfaces as their full `Debug` enclosures — equal only
    /// if the two parts agree to the last bit of every interval.
    fn geometry_by_name(part: &NamedModel<S>) -> BTreeMap<String, String> {
        let model = part.topology();
        part.names()
            .iter()
            .filter_map(|(id, name)| {
                let geometry = match id {
                    RefId::Vertex(v) => format!("{:?}", model.get_vertex(v).ok()?.point),
                    RefId::Edge(e) => format!("{:?}", model.get_edge(e).ok()?.curve),
                    RefId::Face(f) => format!("{:?}", model.get_face(f).ok()?.surface),
                    RefId::Solid(_) => return None,
                };
                Some((name.to_string(), geometry))
            })
            .collect()
    }

    /// Builds `program`, writes it and a description of the part it builds
    /// to `outputs/parts/`, reads the program back from its JSON, and
    /// requires the read-back program to be the same program and to build
    /// the very same part: every name, every piece of topology between
    /// names, and every bit of geometry.
    fn build_and_round_trip(name: &str, program: &Program) -> NamedModel<S> {
        let part = program.build_model().unwrap();
        if let Err(report) = check_valid(&part) {
            panic!("{name}: {report}");
        }

        let json = program.to_json().unwrap();
        let dir = outputs_dir();
        std::fs::write(dir.join(format!("{name}.geop")), &json).unwrap();
        let description = PartDescription::of(&part).unwrap();
        std::fs::write(
            dir.join(format!("{name}.part.json")),
            serde_json::to_string_pretty(&description).unwrap(),
        )
        .unwrap();
        geop_core_rasterize::rasterize(part.topology(), 16)
            .unwrap()
            .scene(|_| geop_core_rasterize::debug::Color10::Blue)
            .save_to_file(dir.join(format!("{name}.html")).to_str().unwrap())
            .unwrap();

        let read_back = Program::from_json(&json).unwrap();
        assert_eq!(
            &read_back, program,
            "{name}: JSON round trip changed the program"
        );
        assert_eq!(
            read_back.to_json().unwrap(),
            json,
            "{name}: JSON is not stable"
        );

        let rebuilt = read_back.build_model().unwrap();
        assert_eq!(
            PartDescription::of(&rebuilt).unwrap(),
            description,
            "{name}: the read-back program built a different part"
        );
        assert_eq!(
            geometry_by_name(&rebuilt),
            geometry_by_name(&part),
            "{name}: the read-back program built different geometry"
        );
        part
    }

    /// The examples drawn in the editor build valid parts and round-trip,
    /// as the ones written here do.
    #[test]
    fn drawn_examples_round_trip() {
        let drawn = ["turbine_blade", "cool_flange", "edge_part", "conebox"];
        for (name, program) in all().into_iter().filter(|(n, _)| drawn.contains(n)) {
            build_and_round_trip(name, &program());
        }
    }
}
