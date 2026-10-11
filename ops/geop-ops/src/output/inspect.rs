//! [`Inspector`]: a question answered about what a program built — of the
//! whole part (its mass properties), or of entities picked in it (what
//! they measure).

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Serialize, de::DeserializeOwned};

use super::{Output, read_options};
use crate::{Design, EntityRef, operation::Role, ui::Visual};

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
    /// What it is asked of, if not the whole part: a tool that picks these
    /// in the viewport, and answers again with every pick.
    pub picks: Option<Picks>,
}

/// What an inspector that picks takes: entities filling one of `roles`, at
/// most `most` of them — another pick then replaces the oldest.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Picks {
    pub roles: &'static [Role],
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

    /// The answer about `output` — of the entities `picked`, for one that
    /// picks ([`InspectorInfo::picks`]) — and what to draw of it in the
    /// viewport: the points a distance is measured between.
    fn inspect(
        output: &mut Output,
        picked: &[EntityRef],
        options: Self::Options,
    ) -> GeopResult<(Self::Report, Vec<Visual<Design>>)>;
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
        picked: &[EntityRef],
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
        picked: &[EntityRef],
        options: &serde_json::Value,
    ) -> GeopResult<Inspection> {
        let kind = I::INFO.kind;
        let ctx =
            |e: GeopError| e.with_context(format!("inspect(inspector={kind}, picked={picked:?})"));
        let (report, visuals) =
            I::inspect(output, picked, read_options::<I::Options>(kind, options)?).map_err(ctx)?;
        let report = serde_json::to_value(report)
            .map_err(|e| ctx(GeopError::new(format!("writing the answer: {e}"))))?;
        Ok(Inspection {
            kind,
            report,
            visuals,
        })
    }
}

/// An inspector's answer, as an update carries it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Inspection {
    /// The inspector's kind: what `report` is.
    pub kind: &'static str,
    pub report: serde_json::Value,
    /// What to draw of it in the viewport: sent with the tool that picks.
    #[serde(skip)]
    pub visuals: Vec<Visual<Design>>,
}
