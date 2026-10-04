//! The standard parts library: fasteners and purchased components every
//! workspace can place, as read-only files named `std:…` (see
//! [`geop_ops::program::library::is_shared`]) — `std:iso4032_hex_nut.geop`.
//!
//! Each family is one program, generated here in Rust and never saved:
//! its sizes are the rows of its table parameter `size` — `M5`,
//! `M3x10` — its dimensions read from that row by the formulas of its
//! sketches (see [`tables`] for the data, [`drawing`] for sketches drawn
//! by formulas, [`steps`] for how a part is built). So a program placing
//! one picks its size like any parameter of a placed part.
//!
//! Every part turns around, or runs along, the `z` axis, and carries named
//! datums to mate it by: `axis`, and a plane through the origin for the
//! face it sits on — `seat` under a screw's head, `top` of a countersunk
//! screw, `base` of a nut, washer, pin or standoff, `side` of a bearing,
//! `end` of an extrusion, `face` of a motor. A concentric mate on `axis`
//! and a coincident one on that plane place it.
//!
//! Threads are not modelled: a screw's shank and a nut's bore are their
//! nominal diameter, and [`StandardPart::threaded`] names the faces a
//! thread goes on, for it to be recorded on them.
//!
//! Where the files come from is [`WithStandardParts`]: a workspace's own
//! files with the standard parts beside them, which is what
//! [`crate::Workspace`] reads.

mod components;
pub mod drawing;
pub mod drive;
mod fasteners;
pub mod involute;
pub mod steps;
pub mod tables;

#[cfg(test)]
mod tests;

use std::{
    ops::{Deref, DerefMut},
    sync::OnceLock,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_ops::{
    Files, FilesMut,
    parameters::ParameterKind,
    part::{ParamValue, State},
};

use crate::Program;

/// What the standard parts' file names start with.
pub const PREFIX: &str = "std:";

/// A family of standard parts: the file it is placed from, and the
/// program that builds its every size.
pub struct StandardPart {
    /// Its file name, [`PREFIX`] first.
    pub file: &'static str,
    /// What it is, in words: the norm and the name.
    pub title: &'static str,
    /// What a size of it is ordered as, before the size: the norm —
    /// `ISO 4762`, for `ISO 4762 M4x12` — or the catalogue name.
    pub designation: &'static str,
    pub program: Program,
    /// The datum plane through the face it sits on — `seat`, `base`, ...
    /// — which a coincident mate picks; its datum `axis` the concentric
    /// one.
    pub base: &'static str,
    /// The faces a thread goes on or in — a screw's shank, a nut's or a
    /// standoff's bore — by name; none for a part without one.
    pub threaded: Vec<String>,
}

impl StandardPart {
    /// The designation of the part built with the parameter values
    /// `values`: [`StandardPart::designation`] and then each parameter of
    /// the family, in order — a table's row by its name, a number after
    /// its own: `ISO 4762 M4x12`, `T-slot 2020 length 500`.
    pub fn designate(&self, values: &State) -> String {
        let mut words = vec![self.designation.to_string()];
        for parameter in &self.program.parameters.values {
            match (&parameter.kind, values.get(&parameter.name)) {
                (ParameterKind::Table { .. }, Some(ParamValue::Text(row))) => {
                    words.push(row.clone())
                }
                (ParameterKind::Number { .. }, Some(ParamValue::Number(n))) => {
                    words.push(format!("{} {}", parameter.name, n.to_f64()))
                }
                _ => {}
            }
        }
        words.join(" ")
    }
}

/// What the standard part placed from `file` is, built with the
/// parameter values `values`, as a bill of materials lists it — `None`
/// for a file that is no standard part.
pub fn standard(file: &str, values: &State) -> Option<geop_ops_bom::Standard> {
    let family = part(file).ok()?;
    Some(geop_ops_bom::Standard {
        title: family.title.to_string(),
        designation: family.designate(values),
    })
}

/// A built family, and its program as the text a file holds.
struct Entry {
    part: StandardPart,
    text: String,
}

/// Every family, built once: or why one would not build.
fn entries() -> Result<&'static [Entry], &'static str> {
    static ENTRIES: OnceLock<Result<Vec<Entry>, String>> = OnceLock::new();
    let built = ENTRIES.get_or_init(|| {
        let families: [fn() -> GeopResult<StandardPart>; 25] = [
            fasteners::iso4762,
            fasteners::iso7380,
            fasteners::iso10642,
            fasteners::iso4017,
            fasteners::iso4032,
            fasteners::iso10511,
            fasteners::iso7089,
            fasteners::iso7090,
            fasteners::iso8734,
            fasteners::hex_standoff,
            components::ball_bearing,
            components::tslot_2020,
            components::tslot_2040,
            components::nema17,
            drive::spur_gear,
            drive::gt2_pulley_16,
            drive::gt2_pulley_20,
            drive::gt2_pulley_36,
            drive::shaft_collar,
            drive::flange_coupling,
            drive::rack,
            components::linear_rail,
            components::linear_carriage,
            components::servo_sg90,
            components::servo_mg996r,
        ];
        families
            .iter()
            .map(|family| {
                let part = family()?;
                let text = part.program.to_json()?;
                Ok(Entry { part, text })
            })
            .collect::<GeopResult<_>>()
            .map_err(|e: GeopError| format!("building the standard parts: {e}"))
    });
    built.as_deref().map_err(String::as_str)
}

/// Every standard part.
pub fn parts() -> GeopResult<impl Iterator<Item = &'static StandardPart>> {
    Ok(entries().map_err(GeopError::new)?.iter().map(|e| &e.part))
}

/// The standard part placed from `file`.
pub fn part(file: &str) -> GeopResult<&'static StandardPart> {
    parts()?
        .find(|p| p.file == file)
        .ok_or_else(|| no_such_part(file))
}

fn no_such_part(file: &str) -> GeopError {
    let known: Vec<&str> = parts()
        .map(|parts| parts.map(|p| p.file).collect())
        .unwrap_or_default();
    GeopError::new(format!(
        "there is no standard part {file:?}: there are {}",
        known.join(", ")
    ))
}

/// The program files `F`, with the standard parts beside them: every file
/// named [`PREFIX`]`…` is a standard part's — read-only, whatever `F`
/// holds by that name — and every other is `F`'s.
#[derive(Clone, Debug, Default)]
pub struct WithStandardParts<F>(pub F);

impl<F> Deref for WithStandardParts<F> {
    type Target = F;

    fn deref(&self) -> &F {
        &self.0
    }
}

impl<F> DerefMut for WithStandardParts<F> {
    fn deref_mut(&mut self) -> &mut F {
        &mut self.0
    }
}

impl<F: Files> Files for WithStandardParts<F> {
    fn read(&self, path: &str) -> GeopResult<String> {
        if !path.starts_with(PREFIX) {
            return self.0.read(path);
        }
        let entries = entries().map_err(GeopError::new)?;
        entries
            .iter()
            .find(|e| e.part.file == path)
            .map(|e| e.text.clone())
            .ok_or_else(|| no_such_part(path))
    }

    fn list(&self) -> Vec<String> {
        let mut files: Vec<String> = self
            .0
            .list()
            .into_iter()
            .filter(|f| !f.starts_with(PREFIX))
            .collect();
        if let Ok(entries) = entries() {
            files.extend(entries.iter().map(|e| e.part.file.to_string()));
        }
        files
    }
}

/// Writes go to `F`, but never over a standard part: those are read-only,
/// so a write to a [`PREFIX`]`…` name changes nothing.
impl<F: FilesMut> FilesMut for WithStandardParts<F> {
    fn write(&mut self, path: &str, text: Option<String>) {
        if !path.starts_with(PREFIX) {
            self.0.write(path, text);
        }
    }
}
