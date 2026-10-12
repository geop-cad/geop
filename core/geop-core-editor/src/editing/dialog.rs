//! [`Dialog`]: the fields an operation shows for a step, in order.
//!
//! A field says what its value *means* — a length, an angle, elements of the
//! part of some kinds, things to do — never how to edit it:
//! that is the editor's, the same for every operation (see
//! the step editor, `geop_cad_base::step_editor`).

use geop_core_math::{scalars::Scalar, vector::Vector3};
use serde::Serialize;

use geop_core_program::element::PickedElement;

/// Whether `text`, as a number field holds it, is a formula rather than a
/// plain number.
pub fn is_formula(text: &str) -> bool {
    text.trim().parse::<f64>().is_err()
}

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
    /// The icon it is shown as, with its label as its tooltip — SVG on a
    /// 20 × 20 grid, drawn in the current colour (see
    /// [`crate::editable::OperationInfo::icon`]); none to show the label.
    pub icon: Option<&'static str>,
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
            icon: None,
        }
    }

    /// Shown as `icon`, if one is given (see [`Action::icon`]).
    pub fn icon(mut self, icon: impl Into<Option<&'static str>>) -> Self {
        self.icon = icon.into();
        self
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

/// One option of a [`Control::Select`] or a [`Control::File`].
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
    /// Text to show and edit in place — a value as a formula — sent back
    /// as [`super::Value::Text`].
    pub text: Option<String>,
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
            text: None,
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
    /// How many of something: a whole number.
    Count,
}

impl Unit {
    /// How far one step of a field without a range goes.
    fn step(self) -> f64 {
        match self {
            Unit::Length => 0.1,
            Unit::Angle => 1.0,
            Unit::Fraction => 0.01,
            Unit::Count => 1.0,
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

/// A number field — of a plain number, or of one that may also be given
/// as a formula of the part's parameters (see [`Number::text`]).
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Number<S: Scalar> {
    pub label: String,
    /// The value — of a formula, what it evaluates to now.
    pub value: f64,
    pub unit: Unit,
    /// What a slider offers, not what is valid.
    pub range: Option<[f64; 2]>,
    pub step: f64,
    /// For a field that takes formulas, the value as given — a plain
    /// number or a formula — shown and edited in place of `value`, and sent
    /// back as [`super::Value::Text`].
    pub text: Option<String>,
    /// Why its formula does not evaluate, if it does not.
    pub error: Option<String>,
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
            text: None,
            error: None,
            handle: None,
        }
    }

    /// Whether its value follows a formula.
    pub fn follows_formula(&self) -> bool {
        self.text.as_deref().is_some_and(is_formula)
    }

    /// With a slider from `min` to `max`, in 200 steps — unless it follows
    /// a formula.
    pub fn range(mut self, min: f64, max: f64) -> Self {
        if !self.follows_formula() {
            self.range = Some([min, max]);
            self.step = (max - min) / 200.0;
        }
        self
    }

    /// With `handle` to drag it by — unless it follows a formula.
    pub fn handle(mut self, handle: Option<Track<S>>) -> Self {
        if !self.follows_formula() {
            self.handle = handle;
        }
        self
    }
}

/// An element a [`Reference`] holds, and what the operation says of it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Picked {
    pub element: PickedElement,
    /// What it is used as, or why it cannot be.
    pub detail: Option<String>,
    pub tone: Tone,
}

/// Elements of the part a step builds on, picked in the viewport: any
/// element of one of the kinds `accepts` (see
/// [`geop_core_program::element::Kind::NAME`]). One element, or — `multiple` — any
/// number, a pick adding or taking out one. Which of them the step can use
/// is the operation's to say, in what it holds ([`Picked::tone`]).
///
/// The editor does all of the editing (see `geop_cad_base::step_editor`): arming it
/// for picks, removing and clearing elements. An operation only ever gets
/// what it holds now, as [`super::Value::Elements`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Reference {
    pub label: String,
    pub accepts: Vec<String>,
    pub value: Vec<Picked>,
    pub multiple: bool,
    /// Whether the step needs it to hold something before it can be built;
    /// one that is not is only ever picked for when asked to.
    pub required: bool,
    /// Whether a click in the viewport picks for it now.
    pub armed: bool,
}

impl Reference {
    /// Whether it is waiting for what the step needs: required, and empty.
    pub fn waiting(&self) -> bool {
        self.required && self.value.is_empty()
    }

    /// The elements it holds.
    pub fn elements(&self) -> impl Iterator<Item = &PickedElement> {
        self.value.iter().map(|p| &p.element)
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
    /// Files the step can be saved as, one button per format: pressing
    /// one asks the editor's exporter `exporter` for the file in that
    /// format (the choice's value), written as the step now is — a
    /// drawing — and saves it. It sets nothing.
    Download {
        exporter: String,
        formats: Vec<Choice>,
    },
    Number(Number<S>),
    /// One of `options`, by value — found by typing, if `searchable`.
    Select {
        label: String,
        value: String,
        options: Vec<Choice>,
        searchable: bool,
    },
    /// A file the program reads, by its path relative to the program:
    /// one of `options`, the files of the kinds `accept` (extensions,
    /// without the dot) next to it, or one from elsewhere — the front end
    /// stores a file the user chooses next to the program and sends its
    /// path, as [`super::Value::Choice`] like an option's.
    File {
        label: String,
        value: String,
        options: Vec<Choice>,
        accept: Vec<String>,
    },
    /// A colour, `#rrggbb`, sent back as [`super::Value::Text`].
    Color {
        label: String,
        value: String,
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

    pub fn heading(&mut self, key: &str, text: impl Into<String>) -> &mut Self {
        self.push(key, Control::Heading { text: text.into() })
    }

    pub fn text(&mut self, key: &str, text: impl Into<String>, tone: Tone) -> &mut Self {
        self.push(
            key,
            Control::Text {
                text: text.into(),
                tone,
            },
        )
    }

    /// A list of entries, each with a key of its own.
    pub fn list(&mut self, key: &str, items: Vec<ListItem>, empty: impl Into<String>) -> &mut Self {
        self.push(
            key,
            Control::List {
                items,
                empty: empty.into(),
            },
        )
    }

    /// Files the step can be saved as by the exporter `exporter`, one
    /// button per format in `formats` (see [`Control::Download`]).
    pub fn download(&mut self, key: &str, exporter: &str, formats: Vec<Choice>) -> &mut Self {
        self.push(
            key,
            Control::Download {
                exporter: exporter.into(),
                formats,
            },
        )
    }

    /// Things to do or choose: pressing one sends its value as
    /// [`super::Value::Choice`].
    pub fn actions(&mut self, key: &str, actions: Vec<Action>) -> &mut Self {
        self.push(key, Control::Actions { actions })
    }

    pub fn checkbox(&mut self, key: &str, label: impl Into<String>, value: bool) -> &mut Self {
        self.push(
            key,
            Control::Checkbox {
                label: label.into(),
                value,
            },
        )
    }

    pub fn number(&mut self, key: &str, number: Number<S>) -> &mut Self {
        self.push(key, Control::Number(number))
    }

    /// One of `options`, by value — found by typing, if `searchable`.
    pub fn select(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        options: Vec<Choice>,
        searchable: bool,
    ) -> &mut Self {
        self.push(
            key,
            Control::Select {
                label: label.into(),
                value: value.into(),
                options,
                searchable,
            },
        )
    }

    /// A file the program reads, by its path relative to the program: one
    /// of `files`, or one the user adds from elsewhere (see
    /// [`Control::File`]) — of the kinds `accept`, extensions without the
    /// dot.
    pub fn file(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        files: Vec<String>,
        accept: &[&str],
    ) -> &mut Self {
        let options = (files.into_iter())
            .map(|file| Choice::new(file.clone(), file))
            .collect();
        self.push(
            key,
            Control::File {
                label: label.into(),
                value: value.into(),
                options,
                accept: accept.iter().map(|a| a.to_string()).collect(),
            },
        )
    }

    /// A colour, `#rrggbb`.
    pub fn color(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
    ) -> &mut Self {
        self.push(
            key,
            Control::Color {
                label: label.into(),
                value: value.into(),
            },
        )
    }

    /// Elements of the kinds `accepts` (see [`Reference`]), holding `value`.
    /// Required: see [`Dialog::optional`].
    pub fn reference(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: Vec<Picked>,
        accepts: &[&str],
        multiple: bool,
    ) -> &mut Self {
        self.push(
            key,
            Control::Reference(Reference {
                label: label.into(),
                accepts: accepts.iter().map(|k| k.to_string()).collect(),
                value,
                multiple,
                required: true,
                armed: false,
            }),
        )
    }

    /// Makes the reference field `key` optional: the step builds without
    /// it, and it is only picked for when pressed (see
    /// [`Reference::required`]).
    pub fn optional(&mut self, key: &str) -> &mut Self {
        for (k, reference) in self.references_mut() {
            if k == key {
                reference.required = false;
            }
        }
        self
    }

    /// The field `key`.
    pub fn get(&self, key: &str) -> Option<&Control<S>> {
        self.0.iter().find(|f| f.key == key).map(|f| &f.control)
    }

    /// Every reference field, by key.
    pub fn references_mut(&mut self) -> impl Iterator<Item = (&str, &mut Reference)> {
        self.0.iter_mut().filter_map(|f| match &mut f.control {
            Control::Reference(r) => Some((f.key.as_str(), r)),
            _ => None,
        })
    }

    /// Every element its reference fields hold: what the step builds on.
    pub fn picked(&self) -> impl Iterator<Item = &PickedElement> {
        self.0
            .iter()
            .flat_map(|f| match &f.control {
                Control::Reference(r) => r.value.as_slice(),
                _ => &[],
            })
            .map(|p| &p.element)
    }

    /// The labels of its reference fields that hold nothing yet: what the
    /// step still needs picked before it can be built at all. Until then,
    /// that it does not build is no error — it is waiting.
    pub fn missing(&self) -> Vec<&str> {
        self.0
            .iter()
            .filter_map(|f| match &f.control {
                Control::Reference(r) if r.waiting() => Some(r.label.as_str()),
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
                    Control::Number(n) => (
                        &n.label,
                        match &n.text {
                            Some(text) if n.follows_formula() => text.clone(),
                            _ => format!("{:.2}", n.value),
                        },
                    ),
                    Control::Color { label, value } => (label, value.clone()),
                    Control::Select {
                        label,
                        value,
                        options,
                        ..
                    }
                    | Control::File {
                        label,
                        value,
                        options,
                        ..
                    } => (
                        label,
                        options
                            .iter()
                            .find(|o| &o.value == value)
                            .map_or(value.clone(), |o| o.label.clone()),
                    ),
                    Control::Reference(r) => (
                        &r.label,
                        r.elements()
                            .map(PickedElement::label)
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

    /// Fields serialize flat, tagged with their type, as a viewer reads
    /// them; a handle is the editor's to draw, not sent.
    #[test]
    fn fields_serialize_flat() {
        let mut dialog = Dialog::<S>::new();
        dialog.push(
            "distance",
            Control::Number(
                Number::new("distance", 1.0, Unit::Length)
                    .range(-1.0, 1.0)
                    .handle(Some(Track {
                        at: Vector3::zero(),
                        direction: Vector3::zero(),
                    })),
            ),
        );
        dialog.push(
            "sketch",
            Control::Reference(Reference {
                label: "sketch".into(),
                accepts: vec!["sketch".into()],
                value: vec![Picked {
                    element: PickedElement {
                        target: "k".into(),
                        kind: "sketch".into(),
                        name: String::new(),
                    },
                    detail: None,
                    tone: Tone::Normal,
                }],
                multiple: false,
                required: true,
                armed: false,
            }),
        );
        assert_eq!(
            serde_json::to_value(&dialog).unwrap(),
            json!([
                {
                    "key": "distance",
                    "type": "number",
                    "label": "distance",
                    "value": 1.0,
                    "unit": "length",
                    "range": [-1.0, 1.0],
                    "step": 0.01,
                    "text": null,
                    "error": null,
                },
                {
                    "key": "sketch",
                    "type": "reference",
                    "label": "sketch",
                    "accepts": ["sketch"],
                    "value": [{
                        "element": {"target": "k", "kind": "sketch", "name": ""},
                        "detail": null,
                        "tone": "normal",
                    }],
                    "multiple": false,
                    "required": true,
                    "armed": false,
                },
            ])
        );
    }
}
