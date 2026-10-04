//! The standard parts: every family builds a valid solid, and a screw is
//! placed in a plate as the editor places it.

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{NoFiles, Part};

use super::{StandardPart, parts};
use crate::{Program, operations::regression_tests::check_valid};

/// The part `program`, of the family `part`, builds: one valid solid, its
/// datums and threaded faces named — or what is wrong with it.
fn built(part: &StandardPart, program: &Program) -> Result<Part<S>, String> {
    let built = program
        .build::<S>(&NoFiles)
        .map_err(|e| format!("does not build: {e}"))?;
    check_valid(&built).map_err(|e| format!("is not valid: {e}"))?;
    let solids = built.solid_names();
    if solids.len() != 1 {
        return Err(format!("is not one solid: {solids:?}"));
    }
    for face in &part.threaded {
        built
            .face_id(face)
            .map_err(|_| format!("has no threaded face {face}"))?;
    }
    Ok(built)
}

/// Checks the family placed from `file` builds, at the size its file
/// has, one valid solid.
fn assert_family_builds(file: &str) {
    let part = super::part(file).unwrap();
    if let Err(e) = built(part, &part.program) {
        panic!("{file} {e}");
    }
}

macro_rules! families_build {
    ($($test:ident: $file:literal,)*) => {
        $(
            #[test]
            fn $test() {
                assert_family_builds($file);
            }
        )*

        /// Every family has its test above.
        #[test]
        fn every_family_is_tested() {
            let tested = [$($file),*];
            for part in parts().unwrap() {
                assert!(tested.contains(&part.file), "{} is not tested", part.file);
            }
        }
    };
}

families_build! {
    socket_head_cap_screws_build: "std:iso4762_socket_head_cap_screw.geop",
    button_head_screws_build: "std:iso7380_button_head_screw.geop",
    countersunk_screws_build: "std:iso10642_countersunk_screw.geop",
    hex_head_screws_build: "std:iso4017_hex_head_screw.geop",
    hex_nuts_build: "std:iso4032_hex_nut.geop",
    nylon_insert_nuts_build: "std:iso10511_nylon_insert_nut.geop",
    washers_build: "std:iso7089_washer.geop",
    chamfered_washers_build: "std:iso7090_chamfered_washer.geop",
    dowel_pins_build: "std:iso8734_dowel_pin.geop",
    hex_standoffs_build: "std:hex_standoff.geop",
    ball_bearings_build: "std:ball_bearing.geop",
    tslot_2020_builds: "std:tslot_2020.geop",
    tslot_2040_builds: "std:tslot_2040.geop",
    nema17_steppers_build: "std:nema17_stepper.geop",
}
