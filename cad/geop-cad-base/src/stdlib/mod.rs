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
mod fasteners;
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

/// What builds a family, given the file it is placed from.
type Family = fn(&'static str) -> GeopResult<StandardPart>;

/// Every family: the file it is placed from, and what builds it. Listing
/// the files builds nothing; each family is built the first time it is
/// read (see [`entry`]).
const FAMILIES: [(&str, Family); 14] = [
    ("std:iso4762_socket_head_cap_screw.geop", fasteners::iso4762),
    ("std:iso7380_button_head_screw.geop", fasteners::iso7380),
    ("std:iso10642_countersunk_screw.geop", fasteners::iso10642),
    ("std:iso4017_hex_head_screw.geop", fasteners::iso4017),
    ("std:iso4032_hex_nut.geop", fasteners::iso4032),
    ("std:iso10511_nylon_insert_nut.geop", fasteners::iso10511),
    ("std:iso7089_washer.geop", fasteners::iso7089),
    ("std:iso7090_chamfered_washer.geop", fasteners::iso7090),
    ("std:iso8734_dowel_pin.geop", fasteners::iso8734),
    ("std:hex_standoff.geop", fasteners::hex_standoff),
    ("std:ball_bearing.geop", components::ball_bearing),
    ("std:tslot_2020.geop", components::tslot_2020),
    ("std:tslot_2040.geop", components::tslot_2040),
    ("std:nema17_stepper.geop", components::nema17),
];

/// The family placed from `file`, built once, the first time it is asked
/// for: or why it would not build. `None` for a file that is no standard
/// part.
fn entry(file: &str) -> Option<GeopResult<&'static Entry>> {
    static ENTRIES: [OnceLock<Result<Entry, String>>; FAMILIES.len()] =
        [const { OnceLock::new() }; FAMILIES.len()];
    let k = FAMILIES.iter().position(|(f, _)| *f == file)?;
    let (file, family) = FAMILIES[k];
    let built = ENTRIES[k].get_or_init(|| {
        let part = family(file).map_err(|e| format!("building the standard part {file}: {e}"))?;
        let text = part
            .program
            .to_json()
            .map_err(|e| format!("writing the standard part {file}: {e}"))?;
        Ok(Entry { part, text })
    });
    Some(built.as_ref().map_err(|e| GeopError::new(e.clone())))
}

/// Every standard part, each built if it was not yet.
pub fn parts() -> GeopResult<impl Iterator<Item = &'static StandardPart>> {
    let parts = FAMILIES
        .iter()
        .map(|(file, _)| part(file))
        .collect::<GeopResult<Vec<_>>>()?;
    Ok(parts.into_iter())
}

/// The standard part placed from `file`.
pub fn part(file: &str) -> GeopResult<&'static StandardPart> {
    Ok(&entry(file).ok_or_else(|| no_such_part(file))??.part)
}

fn no_such_part(file: &str) -> GeopError {
    let known: Vec<&str> = FAMILIES.iter().map(|(f, _)| *f).collect();
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
        Ok(entry(path).ok_or_else(|| no_such_part(path))??.text.clone())
    }

    fn list(&self) -> Vec<String> {
        let mut files: Vec<String> = self
            .0
            .list()
            .into_iter()
            .filter(|f| !f.starts_with(PREFIX))
            .collect();
        files.extend(FAMILIES.iter().map(|(file, _)| file.to_string()));
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
