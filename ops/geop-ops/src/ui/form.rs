//! [`Form`]: what an operation shows for a step — its dialog and the
//! [`Visual`]s it draws — and, for each field, what setting it does.
//!
//! A field is described once: the control it shows and the setter its
//! value goes to, side by side (see [`Form::number`], [`Form::reference`],
//! ...). [`crate::Operation::set`] finds a field's setter by its key, so an
//! operation never matches keys itself. A setter may capture what the form
//! was built from — the step's context, the part before it — for what setting a field
//! implies for others: another sketch picked brings its own axis, a
//! selection picks the construction it fits.

use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar};

use super::{
    Action, Choice, Control, Dialog, ListItem, Number, Picked, Prompt, Reference, Tone, Value,
    Visual,
};
use crate::{
    assembly::Drag,
    operation::{EntityRef, Role},
    parameters::Formula,
    part::State,
};

/// What an operation has in hand while a step is edited (see
/// [`Form::tool`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum InHand {
    /// Nothing: a click selects, and a draggable visual is dragged.
    #[default]
    Nothing,
    /// A tool that clicks — drawing a line, say: clicks go to it rather
    /// than select, and draggable visuals are not dragged.
    Clicks,
    /// A tool that also strokes — trimming, say: besides its clicks, a
    /// press anywhere grabs, and the drag goes to it as a
    /// [`super::CanvasEvent::Stroke`].
    Strokes,
}

/// What a setter edits: the step's arguments, its session, the keys of the
/// visuals selected, and the program's state — the parameters its steps
/// read, which an edit may set: where a placed part is put.
pub struct Edit<'e, A, T> {
    pub args: &'e mut A,
    pub session: &'e mut T,
    pub selection: &'e mut Vec<String>,
    pub state: &'e mut State,
}

/// What setting a field does.
type Setter<'a, A, T> = Box<dyn Fn(Edit<'_, A, T>, Value) + 'a>;

/// What an operation shows for a step: its fields and what setting each
/// does, and what it draws. `A` is its arguments, `T` its session; a form
/// an editor reads (see [`Form::erase`]) has neither.
pub struct Form<'a, S: Scalar, A = (), T = ()> {
    pub dialog: Dialog<S>,
    pub visuals: Vec<Visual<S>>,
    /// A plane to work in, its `u`/`v` plane: the viewer faces it head on,
    /// stops orbiting, and draws a grid on it. Draggable visuals are
    /// dragged in it.
    pub focus: Option<CoordinateSystem<S>>,
    /// What the operation has in hand: clicks — and, for a tool that
    /// strokes, drags — go to it rather than select.
    pub tool: InHand,
    /// The placed parts the step is dragging, by the parameters their poses
    /// are: the editor solves the program with them pulled.
    pub drags: Vec<Drag<S>>,
    /// The joint coordinates the step has set, by the parameters they are:
    /// the editor solves the program with them kept where the state has
    /// them, the parts moving to them.
    pub holds: Vec<String>,
    /// A value asked for in place, in the viewport.
    pub prompt: Option<Prompt<S>>,
    setters: Vec<(String, Setter<'a, A, T>)>,
}

impl<S: Scalar, A, T> Default for Form<'_, S, A, T> {
    fn default() -> Self {
        Self {
            dialog: Dialog::new(),
            visuals: Vec::new(),
            focus: None,
            tool: InHand::Nothing,
            drags: Vec::new(),
            holds: Vec::new(),
            prompt: None,
            setters: Vec::new(),
        }
    }
}

impl<'a, S: Scalar, A, T> Form<'a, S, A, T> {
    pub fn new() -> Self {
        Self::default()
    }

    /// What the field `key` does with `value`: its setter's. Nothing for a
    /// key no field has — a field shown only to be read.
    pub fn set(&self, key: &str, edit: Edit<'_, A, T>, value: Value) {
        if let Some((_, setter)) = self.setters.iter().find(|(k, _)| k == key) {
            setter(edit, value);
        }
    }

    /// The form as an editor reads it: what it shows, without what setting
    /// its fields does — and so borrowing nothing its setters did.
    pub fn erase<'b>(self) -> Form<'b, S> {
        Form {
            dialog: self.dialog,
            visuals: self.visuals,
            focus: self.focus,
            tool: self.tool,
            drags: self.drags,
            holds: self.holds,
            prompt: self.prompt,
            setters: Vec::new(),
        }
    }

    /// `setter` for the key `key`: for a control already in the dialog, or
    /// one of its items — a list's entries each have their own key.
    pub fn on(&mut self, key: impl Into<String>, setter: impl Fn(Edit<'_, A, T>, Value) + 'a) {
        self.setters.push((key.into(), Box::new(setter)));
    }

    /// Appends `control` under `key`, set by `setter`.
    fn field(
        &mut self,
        key: &str,
        control: Control<S>,
        setter: impl Fn(Edit<'_, A, T>, Value) + 'a,
    ) -> &mut Self {
        self.dialog.push(key, control);
        self.on(key, setter);
        self
    }

    pub fn heading(&mut self, key: &str, text: impl Into<String>) -> &mut Self {
        self.dialog
            .push(key, Control::Heading { text: text.into() });
        self
    }

    pub fn text(&mut self, key: &str, text: impl Into<String>, tone: Tone) -> &mut Self {
        self.dialog.push(
            key,
            Control::Text {
                text: text.into(),
                tone,
            },
        );
        self
    }

    /// A list of entries, each with a key of its own for [`Form::on`].
    pub fn list(&mut self, key: &str, items: Vec<ListItem>, empty: impl Into<String>) -> &mut Self {
        self.dialog.push(
            key,
            Control::List {
                items,
                empty: empty.into(),
            },
        );
        self
    }

    /// Things to do or choose; pressing one runs `run` with its value.
    pub fn actions(
        &mut self,
        key: &str,
        actions: Vec<Action>,
        run: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.field(key, Control::Actions { actions }, move |edit, value| {
            if let Value::Choice(choice) = value {
                run(edit, &choice);
            }
        })
    }

    pub fn checkbox(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: bool,
        set: impl Fn(&mut A, bool) + 'a,
    ) -> &mut Self {
        let control = Control::Checkbox {
            label: label.into(),
            value,
        };
        self.field(key, control, move |edit, value| {
            if let Value::Bool(b) = value {
                set(edit.args, b);
            }
        })
    }

    pub fn number(
        &mut self,
        key: &str,
        number: Number<S>,
        set: impl Fn(&mut A, f64) + 'a,
    ) -> &mut Self {
        self.field(key, Control::Number(number), move |edit, value| {
            if let Value::Number(v) = value {
                set(edit.args, v);
            }
        })
    }

    /// A number field that also takes a formula of the part's parameters
    /// (see [`Number::formula`]): text typed sets it — plain if it is a
    /// number, a formula otherwise — and a number slid or dragged sets it
    /// plain. A formula that does not evaluate is kept, as typed, its field
    /// saying why: the step then fails naming it, and the text is there to
    /// be corrected.
    pub fn formula(
        &mut self,
        key: &str,
        number: Number<S>,
        set: impl Fn(&mut A, Formula) + 'a,
    ) -> &mut Self {
        self.field(
            key,
            Control::Number(number),
            move |edit, value| match value {
                Value::Number(v) => set(edit.args, Formula::Plain(v)),
                Value::Text(text) if !text.trim().is_empty() => set(edit.args, Formula::from(text)),
                _ => {}
            },
        )
    }

    /// One of `options`, by value — found by typing, if `searchable`.
    pub fn select(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        options: Vec<Choice>,
        searchable: bool,
        set: impl Fn(&mut A, &str) + 'a,
    ) -> &mut Self {
        let control = Control::Select {
            label: label.into(),
            value: value.into(),
            options,
            searchable,
        };
        self.field(key, control, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit.args, &choice);
            }
        })
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
        set: impl Fn(&mut A, &str) + 'a,
    ) -> &mut Self {
        let control = Control::File {
            label: label.into(),
            value: value.into(),
            options: files
                .into_iter()
                .map(|file| Choice::new(file.clone(), file))
                .collect(),
            accept: accept.iter().map(|a| a.to_string()).collect(),
        };
        self.field(key, control, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit.args, &choice);
            }
        })
    }

    /// Makes the reference field `key` optional: the step builds without
    /// it, and it is only picked for when pressed (see
    /// [`Reference::required`]).
    pub fn optional(&mut self, key: &str) -> &mut Self {
        for (k, reference) in self.dialog.references_mut() {
            if k == key {
                reference.required = false;
            }
        }
        self
    }

    /// A colour, `#rrggbb`.
    pub fn color(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        set: impl Fn(&mut A, &str) + 'a,
    ) -> &mut Self {
        let control = Control::Color {
            label: label.into(),
            value: value.into(),
        };
        self.field(key, control, move |edit, value| {
            if let Value::Text(color) = value {
                set(edit.args, &color);
            }
        })
    }

    /// Entities that can fill one of `roles` — and, with a `scope`, are part
    /// of it (see [`Reference`]); `set` gets what the field holds now.
    #[allow(clippy::too_many_arguments)]
    pub fn reference(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: Vec<EntityRef>,
        roles: &[Role],
        scope: Option<EntityRef>,
        multiple: bool,
        set: impl Fn(Edit<'_, A, T>, Vec<EntityRef>) + 'a,
    ) -> &mut Self {
        let control = Control::Reference(Reference {
            label: label.into(),
            roles: roles.to_vec(),
            scope,
            value: value
                .into_iter()
                .map(|entity| Picked {
                    entity,
                    detail: None,
                    tone: Tone::Normal,
                })
                .collect(),
            multiple,
            required: true,
            armed: false,
        });
        self.field(key, control, move |edit, value| {
            if let Value::Entities(entities) = value {
                set(edit, entities);
            }
        })
    }
}
