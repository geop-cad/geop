//! [`Dialog`]: the controls an operation shows for a step, in order.

use serde::Serialize;

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

/// A button of a [`Control::Buttons`] grid, with its own key.
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

/// How a [`Control::Select`] is shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SelectStyle {
    Dropdown,
    /// Every option at once, grouped.
    Radio,
}

/// One entry of a [`Control::List`], with its own key: pressing it sends
/// [`super::DialogValue::Press`], removing it `Remove`, editing its value
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
    Button {
        label: String,
        title: Option<String>,
        /// Shown pressed: a pick waiting for the viewport.
        active: bool,
        enabled: bool,
        primary: bool,
    },
    /// A grid of buttons, each with its own key.
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
    /// One of `options`, by value.
    Select {
        label: String,
        value: String,
        options: Vec<Choice>,
        style: SelectStyle,
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

/// The controls an operation shows for a step, in order: an ordered
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

    pub fn button(&mut self, key: &str, label: impl Into<String>) -> &mut Self {
        self.push(
            key,
            Control::Button {
                label: label.into(),
                title: None,
                active: false,
                enabled: true,
                primary: false,
            },
        )
    }

    /// A button that arms a pick in the viewport, shown pressed while it is
    /// armed.
    pub fn pick_button(&mut self, key: &str, label: impl Into<String>, active: bool) -> &mut Self {
        self.push(
            key,
            Control::Button {
                label: label.into(),
                title: Some("Pick in the viewport".into()),
                active,
                enabled: true,
                primary: false,
            },
        )
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
        style: SelectStyle,
    ) -> &mut Self {
        self.push(
            key,
            Control::Select {
                label: label.into(),
                value: value.into(),
                options,
                style,
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
}
