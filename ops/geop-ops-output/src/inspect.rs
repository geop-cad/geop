//! [`Inspector`]: a question answered about what a program built — of the
//! whole part (its mass properties), or of elements picked in it (what
//! they measure).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
};
use serde::{Serialize, de::DeserializeOwned};

use super::{Output, read_options};
use geop_core_editor::editing::{Tone, Visual};
use geop_core_program::element::PickedElement;
use geop_ops_model::Design;

/// What an inspector is to a front end: how it is asked, what it is
/// offered as, and whether it is asked of the whole part or of picks.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct InspectorInfo {
    /// The name it is asked by, and its answers are of: `mass_properties`.
    pub kind: &'static str,
    /// What it is offered as: `Mass`.
    pub label: &'static str,
    /// What it answers, in a sentence.
    pub doc: &'static str,
    /// What it is shown as (see
    /// [`geop_core_editor::editable::OperationInfo::icon`]).
    pub icon: Option<&'static str>,
    /// What it is asked of, if not the whole part: a tool that picks these
    /// in the viewport, and answers again with every pick.
    pub picks: Option<Picks>,
}

/// What an inspector that picks takes: elements of what an editor draws of
/// the kinds `accepts` (see [`geop_core_editor::editing::hit::Scene::pick`]),
/// at most `most` of them — another pick then replaces the oldest. What
/// each is to the inspector, it selects itself (see
/// [`geop_core_editor::selectable`]).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Picks {
    pub accepts: &'static [&'static str],
    pub most: usize,
}

/// A question asked of what a program built. An inspector is a unit
/// struct; what it is told beyond the part is its [`Inspector::Options`].
pub trait Inspector: 'static {
    const INFO: InspectorInfo;
    /// What it is told besides the part, as a front end sends it (JSON);
    /// the defaults where it sends none.
    type Options: DeserializeOwned + Default;
    /// Its answer, as a front end is sent it.
    type Report: Serialize;

    /// The answer about `output` — of the elements `picked`, for one that
    /// picks ([`InspectorInfo::picks`]) — and what to draw of it in the
    /// viewport: the points a distance is measured between.
    fn inspect(
        output: &mut Output,
        picked: &[PickedElement],
        options: Self::Options,
    ) -> GeopResult<(Self::Report, Vec<Visual<Design>>)>;

    /// `report` as a front end shows it: what lets every front end show
    /// every inspector's answer alike. Nothing, by default: the report is
    /// shown as it is.
    fn show(report: &Self::Report) -> Answer {
        let _ = report;
        Answer::default()
    }
}

/// An inspector's answer as a front end shows it, whatever the inspector:
/// text, tables and groups of them, and what to do next — built by the
/// inspector ([`Inspector::show`]), so that a front end knows none.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Answer {
    pub blocks: Vec<Block>,
    /// A plane the answer is of — a planar face or a datum plane picked —
    /// to cut a section view with.
    pub plane: Option<CoordinateSystem<Design>>,
}

/// A part of an [`Answer`].
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Block {
    /// A sentence: a hint, or why something failed.
    Text { text: String, tone: Tone },
    /// Items one under another: what was picked, which solids overlap.
    List { items: Vec<Row> },
    /// Rows of cells, under a head if it has one, over a foot if it has
    /// one: each row's label, if it has one, in the first column.
    Table {
        head: Vec<String>,
        rows: Vec<Row>,
        foot: Vec<Row>,
    },
    /// Blocks under a title, shown open or folded: one solid's mass.
    Group {
        title: String,
        /// Said after the title: what it is made of.
        detail: Option<String>,
        open: bool,
        /// What to light while the title is pointed at.
        light: Vec<PickedElement>,
        blocks: Vec<Block>,
    },
    /// Buttons, side by side.
    Actions { actions: Vec<AnswerAction> },
}

/// A row of a [`Block::Table`], or an item of a [`Block::List`].
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Row {
    pub label: Option<String>,
    /// Its cells, each text of one or more lines.
    pub cells: Vec<String>,
    /// Said in full while it is pointed at.
    pub detail: Option<String>,
    /// What to light while it is pointed at.
    pub light: Vec<PickedElement>,
    pub tone: Tone,
}

impl Row {
    /// The row `label`: `cells`.
    pub fn labelled(label: impl Into<String>, cells: Vec<String>) -> Self {
        Self {
            label: Some(label.into()),
            cells,
            ..Self::default()
        }
    }

    /// The row of `cells`, with no label.
    pub fn of(cells: Vec<String>) -> Self {
        Self {
            cells,
            ..Self::default()
        }
    }
}

/// A button of an [`Answer`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct AnswerAction {
    pub label: String,
    /// What it does.
    pub doc: String,
    /// Whether it is what the answer shown was asked with: lit.
    pub chosen: bool,
    pub does: Does,
}

/// What pressing an [`AnswerAction`] does.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Does {
    /// Asks the inspector again, told `options`.
    Inspect { options: serde_json::Value },
    /// Saves what the exporter `exporter` writes, told `options`.
    Export {
        exporter: &'static str,
        options: serde_json::Value,
    },
}

/// An [`Inspector`] with its types erased, so that inspectors of all kinds
/// can be kept in one [`super::Outputs`]. Every inspector is one, through
/// its value.
pub trait ErasedInspector {
    fn info(&self) -> &'static InspectorInfo;

    /// The answer about `output`, of `picked`, with `options` as sent.
    fn inspect(
        &self,
        output: &mut Output,
        picked: &[PickedElement],
        options: &serde_json::Value,
    ) -> GeopResult<Inspection>;
}

impl<I: Inspector> ErasedInspector for I {
    fn info(&self) -> &'static InspectorInfo {
        &I::INFO
    }

    fn inspect(
        &self,
        output: &mut Output,
        picked: &[PickedElement],
        options: &serde_json::Value,
    ) -> GeopResult<Inspection> {
        let kind = I::INFO.kind;
        let ctx =
            |e: GeopError| e.with_context(format!("inspect(inspector={kind}, picked={picked:?})"));
        let (report, visuals) =
            I::inspect(output, picked, read_options::<I::Options>(kind, options)?).map_err(ctx)?;
        let shown = I::show(&report);
        let report = serde_json::to_value(report)
            .map_err(|e| ctx(GeopError::new(format!("writing the answer: {e}"))))?;
        Ok(Inspection {
            kind,
            report,
            shown,
            visuals,
        })
    }
}

/// An inspector's answer, as an update carries it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Inspection {
    /// The inspector's kind: what `report` is.
    pub kind: &'static str,
    /// The answer as data, for a program to read.
    pub report: serde_json::Value,
    /// The answer as a front end shows it.
    pub shown: Answer,
    /// What to draw of it in the viewport: sent with the tool that picks.
    #[serde(skip)]
    pub visuals: Vec<Visual<Design>>,
}
