//! [`Dialog`]: the fields an operation shows for a step, in order.
//!
//! A field says what its value *means* — a length, an angle, entities of the
//! part that can fill some [`Role`], things to do — never how to edit it:
//! that is the editor's, the same for every operation (see
//! [`super::StepEditor`]).

use geop_core_math::{scalars::Scalar, vector::Vector3};
use serde::Serialize;

use crate::operation::{EntityRef, Role};

/// How a text reads: plain, a hint, a problem, or good news.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Tone {
    #[default]
    Normal,
    Hint,
    Error,
    Success,
}

/// Something to do or to choose, as an [`Control::Actions`] offers it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Action {
    /// What pressing it sends back, as [`super::Value::Choice`].
    pub value: String,
    pub label: String,
    /// What it does — or, disabled, why it cannot be done now.
    pub title: Option<String>,
    /// Actions of one group are shown together under its name.
    pub group: Option<String>,
    pub enabled: bool,
    /// Shown pressed: the tool in hand, the way chosen.
    pub active: bool,
}

impl Action {
    pub fn new(value: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            value: value.into(),
            label: label.into(),
            title: None,
            group: None,
            enabled: true,
            active: false,
        }
    }

    pub fn title(mut self, title: impl Into<String>) -> Self {
        self.title = Some(title.into());
        self
    }

    pub fn group(mut self, group: impl Into<String>) -> Self {
        self.group = Some(group.into());
        self
    }

    pub fn active(mut self, active: bool) -> Self {
        self.active = active;
        self
    }

    /// Disabled, saying why.
    pub fn disabled(mut self, why: impl Into<String>) -> Self {
        self.enabled = false;
        self.title = Some(why.into());
        self
    }
}

/// One option of a [`Control::Select`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Choice {
    /// What choosing it sends back.
    pub value: String,
    pub label: String,
}

impl Choice {
    pub fn new(value: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            value: value.into(),
            label: label.into(),
        }
    }
}

/// One entry of a [`Control::List`], with its own key: pressing it sends
/// [`super::Value::Press`], removing it `Remove`, editing its value
/// `Number`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ListItem {
    pub key: String,
    pub label: String,
    /// Said about it, after the label.
    pub detail: Option<String>,
    pub tone: Tone,
    pub selected: bool,
    pub removable: bool,
    /// A number to show and edit in place.
    pub value: Option<f64>,
}

impl ListItem {
    pub fn new(key: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            key: key.into(),
            label: label.into(),
            detail: None,
            tone: Tone::Normal,
            selected: false,
            removable: false,
            value: None,
        }
    }
}

/// What a number measures, which decides how it reads and steps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Unit {
    #[default]
    Length,
    /// In degrees.
    Angle,
    /// A fraction of something: `0` its start, `1` its end.
    Fraction,
}

impl Unit {
    /// How far one step of a field without a range goes.
    fn step(self) -> f64 {
        match self {
            Unit::Length => 0.1,
            Unit::Angle => 1.0,
            Unit::Fraction => 0.01,
        }
    }
}

/// Where a number is dragged in the viewport: a handle at `at`, sliding
/// along `direction` — moving it by `direction` adds one to the number.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Track<S: Scalar> {
    pub at: Vector3<S>,
    pub direction: Vector3<S>,
}

/// A number field.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Number<S: Scalar> {
    pub label: String,
    pub value: f64,
    pub unit: Unit,
    /// What a slider offers, not what is valid.
    pub range: Option<[f64; 2]>,
    pub step: f64,
    /// The handle to drag it by, drawn by the editor.
    #[serde(skip)]
    pub handle: Option<Track<S>>,
}

impl<S: Scalar> Number<S> {
    pub fn new(label: impl Into<String>, value: f64, unit: Unit) -> Self {
        Self {
            label: label.into(),
            value,
            unit,
            range: None,
            step: unit.step(),
            handle: None,
        }
    }

    /// With a slider from `min` to `max`, in 200 steps.
    pub fn range(mut self, min: f64, max: f64) -> Self {
        self.range = Some([min, max]);
        self.step = (max - min) / 200.0;
        self
    }

    pub fn handle(mut self, handle: Option<Track<S>>) -> Self {
        self.handle = handle;
        self
    }
}

/// An entity a [`Reference`] holds, and what the editor found it to be.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Picked {
    pub entity: EntityRef,
    /// What it is used as, or why it cannot be.
    pub detail: Option<String>,
    pub tone: Tone,
}

/// Entities of the part a step builds on, picked in the viewport: any
/// entity that can fill one of `roles` — and, with a `scope`, is part of it,
/// like a line of one sketch. One entity, or — `multiple` — any number, a
/// pick adding or taking out one.
///
/// The editor does all of the editing (see [`super::StepEditor`]): arming it
/// for picks, removing and clearing entities, saying what each held is.
/// An operation only ever gets what it holds now, as
/// [`super::Value::Entities`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Reference {
    pub label: String,
    pub roles: Vec<Role>,
    pub scope: Option<EntityRef>,
    pub value: Vec<Picked>,
    pub multiple: bool,
    /// Whether a click in the viewport picks for it now.
    pub armed: bool,
}

impl Reference {
    /// The entities it holds.
    pub fn entities(&self) -> impl Iterator<Item = &EntityRef> {
        self.value.iter().map(|p| &p.entity)
    }
}

/// A dialog primitive.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Control<S: Scalar> {
    Heading {
        text: String,
    },
    Text {
        text: String,
        tone: Tone,
    },
    /// Things to do or choose, as buttons: pressing one sends its value as
    /// [`super::Value::Choice`]. Grouped ones are shown under their group.
    Actions {
        actions: Vec<Action>,
    },
    Checkbox {
        label: String,
        value: bool,
    },
    Number(Number<S>),
    /// One of `options`, by value.
    Select {
        label: String,
        value: String,
        options: Vec<Choice>,
    },
    Reference(Reference),
    List {
        items: Vec<ListItem>,
        /// Shown when there are no items.
        empty: String,
    },
}

/// A control, and the key the events it sends carry.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Field<S: Scalar> {
    pub key: String,
    #[serde(flatten)]
    pub control: Control<S>,
}

/// The fields an operation shows for a step, in order: an ordered
/// dictionary from key to control, serialized as a list, so a UI renders it
/// with one `map` and keys each element by it. Keys are unique.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(transparent, bound = "S: Scalar")]
pub struct Dialog<S: Scalar>(pub Vec<Field<S>>);

impl<S: Scalar> Default for Dialog<S> {
    fn default() -> Self {
        Self(Vec::new())
    }
}

impl<S: Scalar> Dialog<S> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends `control` under `key`.
    pub fn push(&mut self, key: impl Into<String>, control: Control<S>) -> &mut Self {
        let key = key.into();
        debug_assert!(
            self.0.iter().all(|f| f.key != key),
            "dialog key {key:?} used twice"
        );
        self.0.push(Field { key, control });
        self
    }

    /// The field `key`.
    pub fn get(&self, key: &str) -> Option<&Control<S>> {
        self.0.iter().find(|f| f.key == key).map(|f| &f.control)
    }

    /// Every reference field, by key.
    pub(crate) fn references_mut(&mut self) -> impl Iterator<Item = (&str, &mut Reference)> {
        self.0.iter_mut().filter_map(|f| match &mut f.control {
            Control::Reference(r) => Some((f.key.as_str(), r)),
            _ => None,
        })
    }

    /// Every entity its reference fields hold: what the step builds on.
    pub fn picked(&self) -> impl Iterator<Item = &EntityRef> {
        self.0
            .iter()
            .flat_map(|f| match &f.control {
                Control::Reference(r) => r.value.as_slice(),
                _ => &[],
            })
            .map(|p| &p.entity)
    }

    /// The labels of its reference fields that hold nothing yet: what the
    /// step still needs picked before it can be built at all. Until then,
    /// that it does not build is no error — it is waiting.
    pub fn missing(&self) -> Vec<&str> {
        self.0
            .iter()
            .filter_map(|f| match &f.control {
                Control::Reference(r) if r.value.is_empty() => Some(r.label.as_str()),
                _ => None,
            })
            .collect()
    }

    /// Its values in one line, for a list of steps:
    /// `sketch=outline, distance=1.00`.
    pub fn summary(&self) -> String {
        self.0
            .iter()
            .filter_map(|f| {
                let value = match &f.control {
                    Control::Checkbox { label, value } => {
                        (label, if *value { "yes" } else { "no" }.to_string())
                    }
                    Control::Number(n) => (&n.label, format!("{:.2}", n.value)),
                    Control::Select {
                        label,
                        value,
                        options,
                    } => (
                        label,
                        options
                            .iter()
                            .find(|o| &o.value == value)
                            .map_or(value.clone(), |o| o.label.clone()),
                    ),
                    Control::Reference(r) => (
                        &r.label,
                        r.entities()
                            .map(EntityRef::label)
                            .collect::<Vec<_>>()
                            .join(" & "),
                    ),
                    _ => return None,
                };
                Some(format!("{}={}", value.0, value.1))
            })
            .collect::<Vec<_>>()
            .join(", ")
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::scalars::ScalInF64 as S;
    use serde_json::json;

    use super::*;
    use crate::ui::Form;

    /// Fields serialize flat, tagged with their type, as a viewer reads
    /// them; a handle is the editor's to draw, not sent.
    #[test]
    fn fields_serialize_flat() {
        let mut form = Form::<S, (), ()>::new();
        form.number(
            "distance",
            Number::new("distance", 1.0, Unit::Length)
                .range(-1.0, 1.0)
                .handle(Some(Track {
                    at: Vector3::zero(),
                    direction: Vector3::zero(),
                })),
            |_, _| {},
        );
        form.reference(
            "sketch",
            "sketch",
            vec![EntityRef::Sketch { name: "k".into() }],
            &[Role::Sketch],
            None,
            false,
            |_, _| {},
        );
        assert_eq!(
            serde_json::to_value(&form.dialog).unwrap(),
            json!([
                {
                    "key": "distance",
                    "type": "number",
                    "label": "distance",
                    "value": 1.0,
                    "unit": "length",
                    "range": [-1.0, 1.0],
                    "step": 0.01,
                },
                {
                    "key": "sketch",
                    "type": "reference",
                    "label": "sketch",
                    "roles": ["sketch"],
                    "scope": null,
                    "value": [{
                        "entity": {"type": "Sketch", "name": "k"},
                        "detail": null,
                        "tone": "normal",
                    }],
                    "multiple": false,
                    "armed": false,
                },
            ])
        );
    }
}
