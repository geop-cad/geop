//! [`Dialog`]: the fields an operation shows for a step, in order.

use serde::Serialize;

use super::Target;
use crate::operation::EntityRef;

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

/// A button of a [`Control::Buttons`] row, with its own key.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ButtonItem {
    pub key: String,
    pub label: String,
    /// What it does, shown on hovering it.
    pub title: Option<String>,
    /// Shown pressed: the tool in hand.
    pub active: bool,
    pub enabled: bool,
}

impl ButtonItem {
    pub fn new(key: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            key: key.into(),
            label: label.into(),
            title: None,
            active: false,
            enabled: true,
        }
    }

    pub fn title(mut self, title: impl Into<String>) -> Self {
        self.title = Some(title.into());
        self
    }

    pub fn active(mut self, active: bool) -> Self {
        self.active = active;
        self
    }
}

/// One option of a [`Control::Select`].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Choice {
    /// What choosing it sends back.
    pub value: String,
    pub label: String,
    /// Options that do not fit now are shown, but cannot be chosen.
    pub enabled: bool,
    pub title: Option<String>,
    /// Options with the same group are shown together under its name.
    pub group: Option<String>,
}

impl Choice {
    pub fn new(value: impl Into<String>, label: impl Into<String>) -> Self {
        Self {
            value: value.into(),
            label: label.into(),
            enabled: true,
            title: None,
            group: None,
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

/// A dialog primitive.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Control {
    Heading {
        text: String,
    },
    Text {
        text: String,
        tone: Tone,
    },
    /// A row of buttons, each with its own key.
    Buttons {
        buttons: Vec<ButtonItem>,
    },
    Checkbox {
        label: String,
        value: bool,
    },
    /// A number, with a slider over `slider` if given.
    Number {
        label: String,
        value: f64,
        slider: Option<[f64; 2]>,
        step: f64,
    },
    /// One of `options`, by value. Grouped options are shown all at once.
    Select {
        label: String,
        value: String,
        options: Vec<Choice>,
    },
    /// Entities picked in the viewport, of the `targets` kinds: one, or —
    /// `multiple` — any number, a pick adding or removing one. Pressing it
    /// arms it: the editor then picks for it (see
    /// [`super::StepEditor`]), and says so in `armed`.
    Pick {
        label: String,
        value: Vec<EntityRef>,
        targets: Vec<Target>,
        multiple: bool,
        armed: bool,
    },
    List {
        items: Vec<ListItem>,
        /// Shown when there are no items.
        empty: String,
    },
}

/// A control, and the key the events it sends carry.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Field {
    pub key: String,
    #[serde(flatten)]
    pub control: Control,
}

/// The fields an operation shows for a step, in order: an ordered
/// dictionary from key to control, serialized as a list, so a UI renders it
/// with one `map` and keys each element by it. Keys are unique.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
#[serde(transparent)]
pub struct Dialog(pub Vec<Field>);

impl Dialog {
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends `control` under `key`.
    pub fn push(&mut self, key: impl Into<String>, control: Control) -> &mut Self {
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

    /// One button, pressing which sends `key`.
    pub fn button(&mut self, key: &str, label: impl Into<String>) -> &mut Self {
        self.buttons(key, vec![ButtonItem::new(key, label)])
    }

    pub fn buttons(&mut self, key: &str, buttons: Vec<ButtonItem>) -> &mut Self {
        self.push(key, Control::Buttons { buttons })
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

    /// A number with a slider from `min` to `max`, in 200 steps.
    pub fn slider(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: f64,
        min: f64,
        max: f64,
    ) -> &mut Self {
        self.push(
            key,
            Control::Number {
                label: label.into(),
                value,
                slider: Some([min, max]),
                step: (max - min) / 200.0,
            },
        )
    }

    pub fn select(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        options: Vec<Choice>,
    ) -> &mut Self {
        self.push(
            key,
            Control::Select {
                label: label.into(),
                value: value.into(),
                options,
            },
        )
    }

    /// A field picking one entity of the `targets` kinds, or — `multiple` —
    /// any number of them.
    pub fn pick(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: Vec<EntityRef>,
        targets: &[Target],
        multiple: bool,
    ) -> &mut Self {
        self.push(
            key,
            Control::Pick {
                label: label.into(),
                value,
                targets: targets.to_vec(),
                multiple,
                armed: false,
            },
        )
    }

    pub fn list(&mut self, key: &str, items: Vec<ListItem>, empty: impl Into<String>) -> &mut Self {
        self.push(
            key,
            Control::List {
                items,
                empty: empty.into(),
            },
        )
    }

    /// The field `key`.
    pub fn get(&self, key: &str) -> Option<&Control> {
        self.0.iter().find(|f| f.key == key).map(|f| &f.control)
    }

    pub(crate) fn get_mut(&mut self, key: &str) -> Option<&mut Control> {
        self.0
            .iter_mut()
            .find(|f| f.key == key)
            .map(|f| &mut f.control)
    }

    /// Every entity its pick fields hold: what the step builds on.
    pub fn picked(&self) -> impl Iterator<Item = &EntityRef> {
        self.0.iter().flat_map(|f| match &f.control {
            Control::Pick { value, .. } => value.as_slice(),
            _ => &[],
        })
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
                    Control::Number { label, value, .. } => (label, format!("{value:.2}")),
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
                    Control::Pick { label, value, .. } => (
                        label,
                        value
                            .iter()
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
