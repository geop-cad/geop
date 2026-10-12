//! What is made *of* a part rather than in it: files written from what a
//! program built ([`Exporter`]), and questions answered about it
//! ([`Inspector`]). Files of paper — drawings, flat patterns — are laid out
//! on a [`sheet::Sheet`]. Neither is a step: exporting and inspecting change no
//! program and are nothing to undo.
//!
//! A crate that writes a format, or answers a question, implements one of
//! the traits on a unit struct, as an operation does; an editor offers
//! those registered in its [`Outputs`], by their infos, and knows nothing
//! more of them. The options each takes are its own, sent as JSON.

mod export;
mod inspect;
pub mod sheet;
mod stl;

pub use export::{Content, ErasedExporter, Export, Exporter, ExporterInfo};
pub use inspect::{
    Answer, AnswerAction, Block, Does, ErasedInspector, Inspection, Inspector, InspectorInfo,
    Picks, Row,
};
pub use stl::{STL_QUALITY, Stl, StlOptions, stl_bytes};

use std::{collections::BTreeSet, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::{
    operation::{Build, BuildView},
    parameters::Values,
    placed::{About, about},
    solving::{self, Freedom},
};

/// `text` fit for XML: for an element's content, or an attribute's value
/// between either kind of quotes.
pub fn xml_escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

/// A number as a report shows it — a value, and how far the truth may be
/// from it: the midpoint of an enclosure, and its half-width.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Bounded {
    pub value: f64,
    pub error: f64,
}

impl Bounded {
    /// The enclosure `x`, as shown.
    pub fn of<S: Scalar>(x: S) -> Self {
        let (lo, hi) = (x.lower().to_f64(), x.upper().to_f64());
        Self {
            value: 0.5 * (lo + hi),
            error: 0.5 * (hi - lo),
        }
    }

    /// It times `n`, as shown: a unit's amount for `n` units, or a value
    /// in a unit `n` times smaller.
    pub fn times(self, n: f64) -> Self {
        Self {
            value: self.value * n,
            error: self.error * n,
        }
    }

    /// Whether `exact` lies within the bounds.
    pub fn contains(&self, exact: f64) -> bool {
        (exact - self.value).abs() <= self.error
    }

    /// As a report shows it: its value to `digits` decimals, and `± error`
    /// when the error shows at that precision.
    pub fn show(&self, digits: usize) -> String {
        let value = format!("{:.digits$}", self.value);
        match self.error >= 0.5 * 10f64.powi(-(digits as i32)) {
            true => format!("{value} ± {:.1e}", self.error),
            false => value,
        }
    }

    /// A mass in kilograms as a report shows it: in grams below one
    /// kilogram.
    pub fn show_mass(&self) -> String {
        match self.value < 1.0 {
            true => format!("{} g", self.times(1000.0).show(3)),
            false => format!("{} kg", self.show(3)),
        }
    }
}

/// What a program built, as an exporter or an inspector is given it: the
/// part as far as the program runs, the parts it places included, and the
/// state it ran with — to ask how its parts can move ([`Output::freedom`]).
pub struct Output<'a> {
    build: &'a mut Build,
    state: &'a Values,
    /// The step being edited, if one is: what an export of one step's
    /// result (a drawing) writes unless told which.
    pub editing: Option<&'a str>,
    /// The day it is written, `YYYY-MM-DD`, as the front end says: what a
    /// file is dated with. Empty if not told.
    pub date: &'a str,
}

impl<'a> Output<'a> {
    /// What `build` built, run last with `state`.
    pub fn new(
        build: &'a mut Build,
        state: &'a Values,
        editing: Option<&'a str>,
        date: &'a str,
    ) -> Self {
        Self {
            build,
            state,
            editing,
            date,
        }
    }

    /// The part as far as the program runs.
    pub fn part(&self) -> BuildView<'_> {
        self.build.current()
    }

    /// The part as it was before the step `step`: what a step that
    /// describes an output of the part — a drawing — is of.
    pub fn part_before(&self, step: &str) -> GeopResult<BuildView<'_>> {
        self.build.view_before(step)
    }

    /// What the program says of its part: its file, its material, its
    /// title (see [`About`]).
    pub fn about(&self) -> Rc<About> {
        about(self.part())
    }

    /// The name of the program's file, without folders or `.geop`: what a
    /// file written from it is named after (see [`About::stem`]).
    pub fn stem(&self) -> String {
        self.about().stem().to_string()
    }

    /// How free every placed part is where the state has it, the values
    /// `held` held (see [`solving::freedom`]). It brings the part's
    /// constraints to the states it tries, and back: what an output reads
    /// of the part is as it was.
    pub fn freedom(&mut self, held: &BTreeSet<String>) -> GeopResult<Freedom> {
        solving::freedom(self.build, self.state, held)
    }
}

/// The options of an exporter or inspector that takes none. Like any
/// options, it ignores those it does not take, so a front end may send
/// every exporter the same: a unit `()` would refuse them.
#[derive(Clone, Copy, Debug, Default, Deserialize)]
pub struct NoOptions {}

/// The options `json` gives, as `T`; its defaults where it gives none
/// (`null`).
fn read_options<T: DeserializeOwned + Default>(
    kind: &str,
    json: &serde_json::Value,
) -> GeopResult<T> {
    match json {
        serde_json::Value::Null => Ok(T::default()),
        json => T::deserialize(json)
            .map_err(|e| GeopError::new(format!("the options of `{kind}`: {e}"))),
    }
}

/// The exporters and inspectors an editor offers, each by its kind, in the
/// order they were added.
#[derive(Default)]
pub struct Outputs {
    exporters: Vec<Box<dyn ErasedExporter>>,
    inspectors: Vec<Box<dyn ErasedInspector>>,
}

impl Outputs {
    /// Adds `exporter`. Fails if one of its kind is there already: a front
    /// end could not tell the two apart.
    pub fn exporter<E: Exporter>(mut self, exporter: E) -> GeopResult<Self> {
        if self.exporters.iter().any(|e| e.info().kind == E::INFO.kind) {
            return Err(GeopError::new(format!(
                "exporter `{}` is registered twice",
                E::INFO.kind
            )));
        }
        self.exporters.push(Box::new(exporter));
        Ok(self)
    }

    /// Adds `inspector`. Fails if one of its kind is there already.
    pub fn inspector<I: Inspector>(mut self, inspector: I) -> GeopResult<Self> {
        if self
            .inspectors
            .iter()
            .any(|i| i.info().kind == I::INFO.kind)
        {
            return Err(GeopError::new(format!(
                "inspector `{}` is registered twice",
                I::INFO.kind
            )));
        }
        self.inspectors.push(Box::new(inspector));
        Ok(self)
    }

    /// What each exporter is, in the order they were added.
    pub fn exporters(&self) -> Vec<&'static ExporterInfo> {
        self.exporters.iter().map(|e| e.info()).collect()
    }

    /// What each inspector is, in the order they were added.
    pub fn inspectors(&self) -> Vec<&'static InspectorInfo> {
        self.inspectors.iter().map(|i| i.info()).collect()
    }

    /// The exporter `kind`.
    pub fn get_exporter(&self, kind: &str) -> GeopResult<&dyn ErasedExporter> {
        of_kind(&self.exporters, kind, "exporter", |e| e.info().kind)
    }

    /// The exporter that writes `format` — a file's extension, `stl` —
    /// the first one added if several do.
    pub fn exporter_for(&self, format: &str) -> GeopResult<&dyn ErasedExporter> {
        let format = format.to_ascii_lowercase();
        (self.exporters.iter())
            .find(|e| e.info().formats.contains(&format.as_str()))
            .map(Box::as_ref)
            .ok_or_else(|| {
                let known: Vec<&str> = (self.exporters.iter())
                    .flat_map(|e| e.info().formats.iter().copied())
                    .collect();
                GeopError::new(format!(
                    "no exporter writes `.{format}` files; they write: {}",
                    known.join(", ")
                ))
            })
    }

    /// The inspector `kind`.
    pub fn get_inspector(&self, kind: &str) -> GeopResult<&dyn ErasedInspector> {
        of_kind(&self.inspectors, kind, "inspector", |i| i.info().kind)
    }
}

/// The one of `entries` whose kind, as `kind_of` reads it, is `kind`; if
/// there is none, an error naming the kinds there are, each a `what`.
fn of_kind<'a, T: ?Sized>(
    entries: &'a [Box<T>],
    kind: &str,
    what: &str,
    kind_of: impl Fn(&T) -> &'static str,
) -> GeopResult<&'a T> {
    (entries
        .iter()
        .map(Box::as_ref)
        .find(|&e| kind_of(e) == kind))
    .ok_or_else(|| {
        let known: Vec<&str> = entries.iter().map(Box::as_ref).map(&kind_of).collect();
        GeopError::new(format!(
            "there is no {what} `{kind}`; there are: {}",
            known.join(", ")
        ))
    })
}
