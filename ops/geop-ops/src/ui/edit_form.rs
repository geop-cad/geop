//! [`EditForm`]: a [`Form`] built together with what setting each of its
//! fields does — for an operation with many fields, whose events are then
//! answered by the setter of the field they set (see [`EditForm::set`])
//! rather than by matching their keys. Every setter gets the whole
//! [`Edit`]: a field may set the step's arguments or its session.
//!
//! An [`Operation`](geop_core_program::operation::Operation) only ever hands out the
//! form ([`EditForm::into_form`]); the setters stay its own, built again for
//! each event. A setter may capture what the form was built from — the part
//! as the step sees it — for what setting a field implies for others:
//! another sketch picked brings its own axis, a selection picks the
//! construction it fits.

use geop_core_math::scalars::Scalar;
use geop_core_program::editing::{Action, CanvasEvent, Choice, Event, Form, Number, Tone, Value};

use crate::{
    operation::{BuildView, EntityRef, Role, Seen},
    parameters::Formula,
};

/// What a setter edits: the step's arguments, its session, and the keys of
/// the visuals selected.
pub struct Edit<'e, A, T> {
    pub args: &'e mut A,
    pub session: &'e mut T,
    pub selection: &'e mut Vec<String>,
}

/// What a step edited through an [`EditForm`] keeps while it is edited,
/// as its [`geop_core_program::operation::Operation::EditingState`]: its
/// session `T`, and the keys of the visuals selected.
#[derive(Clone, Debug, Default)]
pub struct FormState<T> {
    pub session: T,
    pub selection: Vec<String>,
}

impl<T: Clone> FormState<T> {
    /// The arguments `args` and this state after `event`: a field set by
    /// its setter in the form `form` makes of them as they were, the
    /// selection replaced, and what the pointer or a key did passed to
    /// `canvas`.
    pub fn handle<'f, S: Scalar, A: Clone>(
        &self,
        args: &A,
        event: Event<S>,
        form: impl FnOnce(&A, &Self) -> EditForm<'f, S, A, T>,
        canvas: impl FnOnce(Edit<'_, A, T>, &CanvasEvent<S>),
    ) -> (A, Self) {
        let mut edited = args.clone();
        let mut state = self.clone();
        let edit = Edit {
            args: &mut edited,
            session: &mut state.session,
            selection: &mut state.selection,
        };
        match event {
            Event::Field { key, value } => form(args, self).set(&key, edit, value),
            Event::Select(keys) => *edit.selection = keys,
            Event::Canvas(event) => canvas(edit, &event),
        }
        (edited, state)
    }
}

/// What setting a field does.
type Setter<'a, A, T> = Box<dyn Fn(Edit<'_, A, T>, Value) + 'a>;

/// A [`Form`] and what setting each of its fields does, for the arguments
/// `A` and the session `T`.
pub struct EditForm<'a, S: Scalar, A, T = ()> {
    pub form: Form<S>,
    setters: Vec<(String, Setter<'a, A, T>)>,
}

impl<S: Scalar, A, T> Default for EditForm<'_, S, A, T> {
    fn default() -> Self {
        Self {
            form: Form::default(),
            setters: Vec::new(),
        }
    }
}

impl<'a, S: Scalar, A, T> EditForm<'a, S, A, T> {
    /// The form, as an editor is shown it.
    pub fn into_form(self) -> Form<S> {
        self.form
    }

    /// What the field `key` does with `value`: its setter's. Nothing for a
    /// key no field has — a field shown only to be read.
    pub fn set(&self, key: &str, edit: Edit<'_, A, T>, value: Value) {
        if let Some((_, setter)) = self.setters.iter().find(|(k, _)| k == key) {
            setter(edit, value);
        }
    }

    /// `setter` for the key `key`: for a control already in the dialog, or
    /// one of its items — a list's entries each have their own key.
    pub fn on(&mut self, key: impl Into<String>, setter: impl Fn(Edit<'_, A, T>, Value) + 'a) {
        self.setters.push((key.into(), Box::new(setter)));
    }

    /// Things to do or choose; pressing one runs `run` with its value.
    pub fn actions(
        &mut self,
        key: &str,
        actions: Vec<Action>,
        run: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.actions(key, actions);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                run(edit, &choice);
            }
        });
        self
    }

    pub fn checkbox(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: bool,
        set: impl Fn(Edit<'_, A, T>, bool) + 'a,
    ) -> &mut Self {
        self.form.dialog.checkbox(key, label, value);
        self.on(key, move |edit, value| {
            if let Value::Bool(b) = value {
                set(edit, b);
            }
        });
        self
    }

    pub fn number(
        &mut self,
        key: &str,
        number: Number<S>,
        set: impl Fn(Edit<'_, A, T>, f64) + 'a,
    ) -> &mut Self {
        self.form.dialog.number(key, number);
        self.on(key, move |edit, value| {
            if let Value::Number(v) = value {
                set(edit, v);
            }
        });
        self
    }

    /// A number field that also takes a formula of the part's parameters
    /// (see [`super::formula_number`]): text typed sets it — plain if it is a
    /// number, a formula otherwise — and a number slid or dragged sets it
    /// plain. A formula that does not evaluate is kept, as typed, its field
    /// saying why: the step then fails naming it, and the text is there to
    /// be corrected.
    pub fn formula(
        &mut self,
        key: &str,
        number: Number<S>,
        set: impl Fn(Edit<'_, A, T>, Formula) + 'a,
    ) -> &mut Self {
        self.form.dialog.number(key, number);
        self.on(key, move |edit, value| {
            if let Some(formula) = Formula::from_value(&value) {
                set(edit, formula);
            }
        });
        self
    }

    /// One of `options`, by value — found by typing, if `searchable`.
    pub fn select(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        options: Vec<Choice>,
        searchable: bool,
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        (self.form.dialog).select(key, label, value, options, searchable);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit, &choice);
            }
        });
        self
    }

    /// A file the program reads (see [`geop_core_program::editing::Dialog::file`]).
    pub fn file(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        files: Vec<String>,
        accept: &[&str],
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.file(key, label, value, files, accept);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit, &choice);
            }
        });
        self
    }

    /// A colour, `#rrggbb`.
    pub fn color(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.color(key, label, value);
        self.on(key, move |edit, value| {
            if let Value::Text(color) = value {
                set(edit, &color);
            }
        });
        self
    }

    /// One entity, of the kind `kind` makes of a name — a face, a solid, a
    /// sketch — that can fill one of `roles`, held by its name `name`
    /// (empty for none); `set` gets the name of the one picked last, or an
    /// empty one.
    pub fn pick_one(
        &mut self,
        key: &str,
        label: impl Into<String>,
        kind: fn(String) -> EntityRef,
        name: &str,
        roles: &[Role],
        set: impl Fn(Edit<'_, A, T>, String) + 'a,
    ) -> &mut Self {
        let value = EntityRef::of_name(kind, name);
        self.reference(
            key,
            label,
            value,
            roles,
            None,
            false,
            move |edit, picked| {
                set(
                    edit,
                    EntityRef::last_name(kind, &picked).unwrap_or_default(),
                )
            },
        )
    }

    /// Entities of the kind `kind` makes of a name that can fill one of
    /// `roles`, held by their names `names`; `set` gets the names of those
    /// picked.
    pub fn pick_many(
        &mut self,
        key: &str,
        label: impl Into<String>,
        kind: fn(String) -> EntityRef,
        names: &[String],
        roles: &[Role],
        set: impl Fn(Edit<'_, A, T>, Vec<String>) + 'a,
    ) -> &mut Self {
        let value = EntityRef::of_names(kind, names);
        self.reference(key, label, value, roles, None, true, move |edit, picked| {
            set(edit, EntityRef::names_of(kind, &picked))
        })
    }

    /// The sketch named `name` (empty for none) of those `view` has, in the
    /// field `key`, labelled the same — or, before there is any sketch, a
    /// hint to draw one; `set` gets the name of the one picked last, or an
    /// empty one.
    pub fn pick_sketch(
        &mut self,
        view: &BuildView,
        key: &str,
        name: &str,
        set: impl Fn(Edit<'_, A, T>, String) + 'a,
    ) -> &mut Self {
        if self.no_sketch_yet(view, key) {
            return self;
        }
        self.pick_one(key, key, EntityRef::sketch, name, &[Role::Sketch], set)
    }

    /// The sketches named `names` of those `view` has, as
    /// [`Self::pick_sketch`]; `set` gets the names of those picked.
    pub fn pick_sketches(
        &mut self,
        view: &BuildView,
        key: &str,
        names: &[String],
        set: impl Fn(Edit<'_, A, T>, Vec<String>) + 'a,
    ) -> &mut Self {
        if self.no_sketch_yet(view, key) {
            return self;
        }
        self.pick_many(key, key, EntityRef::sketch, names, &[Role::Sketch], set)
    }

    /// Whether `view` has no sketch yet, and if so, a hint to draw one in
    /// place of the field `key`.
    fn no_sketch_yet(&mut self, view: &BuildView, key: &str) -> bool {
        let none = Seen::new(*view).sketch_names().is_empty();
        if none {
            (self.form.dialog).text(key, "No sketch yet — add one first.", Tone::Hint);
        }
        none
    }

    /// Entities that can fill one of `roles` — and, with a `scope`, are part
    /// of it (see [`geop_core_program::editing::Reference`]); `set` gets what the
    /// field holds now.
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
        (self.form.dialog).reference(key, label, value, roles, scope, multiple);
        self.on(key, move |edit, value| {
            if let Value::Entities(entities) = value {
                set(edit, entities);
            }
        });
        self
    }
}
