//! How far a tool goes into the solid it is combined with ([`Extent`]): a
//! length, up to the next face, or through all — and what sizes a tool to
//! reach past that solid ([`hull`], [`reach_past`]). An extrude, a revolve
//! and a hole say how far they go this way, and [`crate::Combine`] then
//! stops a tool going up to the next face where [`crate::Tool`] says.

use geop_core_math::{
    geop_error::GeopError, geop_error::GeopResult, primitives::CoordinateSystem, scalars::Scalar,
};
use geop_core_topology::Body;
use geop_ops::ui::Choice;
use geop_ops::{NamedModel, operation::Lookup, parameters::Formula};
use serde::{Deserialize, Serialize};

/// How far one side of a tool goes: a length — an angle, in degrees, for a
/// revolve — plain or a formula of the part's parameters, or as far as the
/// solid it is combined with: up to its next face, or through all of it.
/// Serialized as `{"blind": 20.0}`, `{"blind": "height - 2"}`,
/// `"up_to_next"` or `"through_all"`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Extent {
    Blind(Formula),
    UpToNext,
    ThroughAll,
}

/// Where a side ends: an [`Extent`], its length evaluated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum End {
    Blind(f64),
    UpToNext,
    ThroughAll,
}

impl End {
    /// Its length, if it is blind.
    pub fn length(self) -> Option<f64> {
        match self {
            End::Blind(length) => Some(length),
            End::UpToNext | End::ThroughAll => None,
        }
    }
}

impl Extent {
    /// A side `amount` far: a number, or a formula's text.
    pub fn blind(amount: impl Into<Formula>) -> Self {
        Extent::Blind(amount.into())
    }

    /// Its length, to change, if it is blind.
    pub fn length_mut(&mut self) -> Option<&mut Formula> {
        match self {
            Extent::Blind(length) => Some(length),
            Extent::UpToNext | Extent::ThroughAll => None,
        }
    }

    /// Where it ends, its length as `lookup` reads the parameters.
    pub fn end<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<End> {
        Ok(match self {
            Extent::Blind(length) => End::Blind(length.evaluate(lookup)?),
            Extent::UpToNext => End::UpToNext,
            Extent::ThroughAll => End::ThroughAll,
        })
    }

    /// The ways a side can end, as a dialog offers them — with `none` for a
    /// side which may not be.
    pub fn choices(none: bool) -> Vec<Choice> {
        let mut choices = Vec::new();
        if none {
            choices.push(Choice::new("none", "None"));
        }
        choices.push(Choice::new("blind", "Blind"));
        choices.push(Choice::new("up_to_next", "Up to next"));
        choices.push(Choice::new("through_all", "Through all"));
        choices
    }

    /// How `extent` ends, as [`Extent::choices`] offers it.
    pub fn mode(extent: Option<&Extent>) -> &'static str {
        match extent {
            None => "none",
            Some(Extent::Blind(_)) => "blind",
            Some(Extent::UpToNext) => "up_to_next",
            Some(Extent::ThroughAll) => "through_all",
        }
    }

    /// The extent `mode` chooses, keeping the length of `old` if it was
    /// blind, else `default` far.
    pub fn pick(old: Option<&Extent>, mode: &str, default: f64) -> Option<Extent> {
        match mode {
            "blind" => Some(Extent::Blind(match old {
                Some(Extent::Blind(d)) => d.clone(),
                _ => Formula::Plain(default),
            })),
            "up_to_next" => Some(Extent::UpToNext),
            "through_all" => Some(Extent::ThroughAll),
            _ => None,
        }
    }
}

/// Points whose convex hull holds the solids named `targets`: the control
/// points of all their faces' surfaces — a NURBS surface lies within the
/// hull of its control points — as plain numbers, for sizing a tool that has
/// to reach past them. None for no solids.
pub fn hull<S: Scalar>(
    model: &NamedModel<S>,
    targets: &[String],
) -> GeopResult<Option<Vec<[f64; 3]>>> {
    let topology = model.topology();
    let mut points = Vec::new();
    for target in targets {
        for face in topology.body_faces(Body::Solid(model.solid_id(target)?))? {
            for cp in &topology.get_face(face)?.surface.control_points {
                let w = cp[3].to_f64();
                points.push([0, 1, 2].map(|k| cp[k].to_f64() / w));
            }
        }
    }
    Ok((!points.is_empty()).then_some(points))
}

/// How far along `plane`'s normal — forwards for `sign = 1`, backwards for
/// `-1` — reaches past everything of the solid whose convex hull `hull`
/// spans: a side going up to its next face has to be built at least that
/// long to meet whichever face that is.
pub fn reach_past<S: Scalar>(
    hull: &[[f64; 3]],
    plane: &CoordinateSystem<S>,
    sign: f64,
) -> GeopResult<f64> {
    let o = [0, 1, 2].map(|k| plane.origin()[k].to_f64());
    let w = [0, 1, 2].map(|k| plane.w()[k].to_f64());
    let along = |p: &[f64; 3]| sign * (0..3).map(|k| (p[k] - o[k]) * w[k]).sum::<f64>();
    let ahead = hull.iter().map(along).fold(f64::NEG_INFINITY, f64::max);
    if ahead <= 0.0 {
        return Err(GeopError::new(format!(
            "up to next: nothing of the target lies {} the sketch, so there is no next face",
            if sign > 0.0 { "in front of" } else { "behind" }
        )));
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in hull {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diagonal = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    // Well past the target, so the tool's end is clear of all of it.
    Ok(ahead + diagonal)
}
