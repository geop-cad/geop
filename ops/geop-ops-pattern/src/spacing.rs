//! How the instances of a pattern are spaced — the bodies of a linear or
//! circular pattern, the copies of a placed part — and the fields that set
//! it: [`Spacing`], the [`Row`] of instances it lays out, and the count
//! ([`count_field`], [`whole_count`]).
//!
//! Every number here is a [`NumericInput`]: plain, or a formula of the
//! part's parameters, resolved through whatever reader the step or its
//! form reads the part with.

use geop_core_editor::{
    edit_form::{Edit, EditForm},
    editing::{Choice, Number, Track, Unit, Value},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{numeric::NumericInput, reader::Reader};
use serde::{Deserialize, Serialize};

/// How far apart the instances of a pattern are: each `Step` from the
/// last, or spread evenly over the `Extent` from the first to the last —
/// a length, or an angle in degrees, plain or a formula of the part's
/// parameters. Serialized as `{"step": 2.0}`, `{"extent": 10.0}` or
/// `{"step": {"formula": "pitch"}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Spacing {
    Step(NumericInput),
    Extent(NumericInput),
}

impl Spacing {
    /// Each `step` — a number, or a formula's text — from the last.
    pub fn step(step: impl Into<NumericInput>) -> Self {
        Spacing::Step(step.into())
    }

    /// Spread evenly over `extent` — a number, or a formula's text.
    pub fn extent(extent: impl Into<NumericInput>) -> Self {
        Spacing::Extent(extent.into())
    }

    /// The number it holds, whichever it is.
    pub fn value(&self) -> &NumericInput {
        match self {
            Spacing::Step(v) | Spacing::Extent(v) => v,
        }
    }

    /// The same kind of spacing, holding `value`.
    fn with_value(&self, value: NumericInput) -> Self {
        match self {
            Spacing::Step(_) => Spacing::Step(value),
            Spacing::Extent(_) => Spacing::Extent(value),
        }
    }

    /// `count` instances in a row along a line, this spacing coming to a
    /// length as `reader` reads the parameters. Refuses a spacing that puts
    /// them on top of each other.
    pub fn along(&self, count: usize, reader: &Reader) -> GeopResult<Row> {
        let value = self.value().resolve(reader)?;
        if !value.is_finite() || value == 0.0 {
            return Err(GeopError::new(format!(
                "the spacing {value} puts every copy on top of the bodies"
            )));
        }
        let divisor = match self {
            Spacing::Step(_) => 1,
            Spacing::Extent(_) => (count - 1).max(1),
        };
        Ok(Row {
            count,
            value,
            divisor,
        })
    }

    /// `count` instances in a row round an axis, this spacing coming to an
    /// angle in degrees as `reader` reads the parameters — an extent of a
    /// full turn spreading them over it, the last a step short of the
    /// first. Refuses a spacing that puts them on top of each other, or
    /// brings them round onto the first again.
    pub fn round(&self, count: usize, reader: &Reader) -> GeopResult<Row> {
        let value = self.value().resolve(reader)?;
        if !value.is_finite() || value == 0.0 {
            return Err(GeopError::new(format!(
                "the angle {value}° puts every copy on top of the bodies"
            )));
        }
        let span = |step: f64| step.abs() * (count - 1) as f64;
        let divisor = match self {
            Spacing::Extent(_) if value.abs() == 360.0 => count,
            Spacing::Extent(_) if value.abs() > 360.0 => {
                return Err(GeopError::new(format!(
                    "the pattern spans {value}°, more than a full turn: copies would come round onto each other"
                )));
            }
            Spacing::Extent(_) => (count - 1).max(1),
            Spacing::Step(_) if span(value) >= 360.0 => {
                return Err(GeopError::new(format!(
                    "{} steps of {value}° come round onto the bodies again: spread the instances over 360° instead",
                    count - 1
                )));
            }
            Spacing::Step(_) => 1,
        };
        Ok(Row {
            count,
            value,
            divisor,
        })
    }

    /// Its fields in `f`: which kind — keyed `{key}_mode` — and the number,
    /// keyed `key`, made by `number(label, value)` (see
    /// [`NumericInput::number`]). Switching the kind keeps the number. `set`
    /// puts the spacing set where the arguments keep it.
    pub fn fields<'a, S: Scalar, A, T>(
        &self,
        f: &mut EditForm<'a, S, A, T>,
        key: &str,
        labels: [&str; 2],
        number: impl FnOnce(&str, &NumericInput) -> Number<S>,
        set: impl Fn(&mut A, Spacing) + Clone + 'a,
    ) {
        let (mode, label) = match self {
            Spacing::Step(_) => ("step", labels[0]),
            Spacing::Extent(_) => ("extent", labels[1]),
        };
        let modes = vec![
            Choice::new("step", labels[0]),
            Choice::new("extent", labels[1]),
        ];
        let (value, kind) = (self.value().clone(), self.clone());
        let set_mode = set.clone();
        f.select(
            &format!("{key}_mode"),
            "spacing",
            mode,
            modes,
            false,
            move |edit, mode| {
                let number = value.clone();
                set_mode(
                    edit.args,
                    match mode {
                        "extent" => Spacing::Extent(number),
                        _ => Spacing::Step(number),
                    },
                );
            },
        )
        .numeric(key, number(label, self.value()), move |edit, number| {
            set(edit.args, kind.with_value(number))
        });
    }
}

/// Instances of a pattern in a row: `count` of them, the first where the
/// seed is and the `k`-th `value * k / divisor` on from it — a length, or
/// an angle in degrees (see [`Spacing::along`], [`Spacing::round`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Row {
    pub count: usize,
    value: f64,
    divisor: usize,
}

impl Row {
    /// How far apart two neighbouring instances are, in plain numbers.
    pub fn step(&self) -> f64 {
        self.value / self.divisor as f64
    }

    /// How far instance `k` is from the first, `value * k / divisor`
    /// enclosed in one go rather than accumulated step by step.
    fn distance<S: Scalar>(&self, k: usize) -> GeopResult<S> {
        let distance = S::from_f64(self.value).mul(S::from_i64(k as i64));
        if self.divisor == 1 {
            Ok(distance)
        } else {
            distance.div(S::from_i64(self.divisor as i64))
        }
    }

    /// Where instance `k` is relative to the first, along the unit vector
    /// `unit`: none for the first itself.
    pub fn offset<S: Scalar>(&self, unit: &Vector3<S>, k: usize) -> GeopResult<Option<Vector3<S>>> {
        if k == 0 {
            return Ok(None);
        }
        Ok(Some(unit.prod_scalar(self.distance(k)?)))
    }

    /// The motion turning the first instance to instance `k`, round the
    /// axis through `point` along the unit vector `direction`.
    pub fn turn<S: Scalar>(
        &self,
        point: &Vector3<S>,
        direction: &Vector3<S>,
        k: usize,
    ) -> GeopResult<Pose<S>> {
        // In radians, enclosed in one go.
        let radians = S::from_f64(self.value)
            .mul(S::from_i64(k as i64))
            .mul(S::PI)
            .div(S::from_i64(180 * self.divisor as i64))?;
        Pose::rotation_about(point, direction, radians)
    }
}

/// The count field of a pattern in `f`, keyed `key`, of `count` as
/// `reader` reads the parameters — dragged by `handle` unless it follows a
/// formula — calling `set` with the count it is set to: a formula as
/// typed, a number rounded to a whole one of at least one, the seed alone.
pub fn count_field<'a, S: Scalar, A, T>(
    f: &mut EditForm<'a, S, A, T>,
    key: &str,
    count: &NumericInput,
    reader: &Reader,
    handle: Option<Track<S>>,
    set: impl Fn(Edit<'_, A, T>, NumericInput) + 'a,
) {
    let mut number = Number::numeric(count, "count", reader, Unit::Count)
        .range(1.0, 24.0)
        .handle(handle);
    number.step = 1.0;
    f.form.dialog.number(key, number);
    f.on(key, move |edit, value| {
        if let Some(count) = count_of(&value) {
            set(edit, count);
        }
    });
}

/// What a count field set to `value` makes the count (see [`count_field`]).
fn count_of(value: &Value) -> Option<NumericInput> {
    Some(match value.numeric_input()? {
        NumericInput::Plain(value) if value.is_finite() && value >= 1.0 => {
            NumericInput::Plain(value.round())
        }
        NumericInput::Plain(_) => NumericInput::Plain(1.0),
        formula => formula,
    })
}

/// The number of instances `count` comes to as `reader` reads the
/// parameters: a whole number, at least one — the seed alone. Anything else
/// is refused, saying so.
pub fn whole_count(count: &NumericInput, reader: &Reader) -> GeopResult<usize> {
    let value = count.resolve(reader)?;
    if value.fract() != 0.0 || value < 1.0 {
        return Err(GeopError::new(format!(
            "the count {count} comes to {value}: a pattern has a whole number of instances, at least one — round a formula with round(), floor() or ceil()"
        )));
    }
    Ok(value as usize)
}
