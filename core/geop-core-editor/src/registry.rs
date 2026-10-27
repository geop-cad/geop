//! [`Registry`]: everything an editor offers, gathered in one builder that
//! each operation crate adds to with its one `register` function — the
//! operations a step can be, how what they build is drawn and listed
//! ([`Drawer`]), the exporters and inspectors ([`crate::output`]), the
//! panels ([`Panel`]), the tools ([`Tool`]), what settles the program's
//! state after every edit ([`Settle`]) and the examples
//! ([`ExampleSet`]).
//!
//! The [`crate::editor::Editor`] names none of them: what it shows and does
//! beyond editing steps is what its registry holds.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::PickedElement,
    operation_registry::OperationRegistry,
    program::Program,
    reader::{Reader, Reads},
    values::StateValues,
};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    editable::{Editable, Editors, Listed},
    editing::Pull,
    editor::{Changed, EditorContext, Rerun},
    output::{Exporter, Inspector, Outputs},
    tools::{Tool, ToolInfo, pick::PickTool},
    view::{PartView, annotations_of, listed_by_steps},
};

/// What a crate draws of the part, and lists of it: the targets it
/// defines. A part is drawn by every drawer of the registry in turn, into
/// one [`PartView`] — the parts placed in it as instances of components,
/// never one mesh per placement (see [`crate::view::ViewInstance`]).
pub trait Drawer: 'static {
    /// Draws what of the part `reader` reads is this crate's into `view`.
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()>;

    /// What of the part is this crate's, to list: each drawn item shown
    /// (`visible: Some(true)`) — the editor hides what is hidden — and
    /// each one that is not drawn with `visible: None`.
    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>>;

    /// The names of what it draws only to be picked: shown only while a
    /// field waiting for a pick could pick it (see
    /// [`crate::editing::Reference::accepts`]) — none, unless it says
    /// otherwise.
    fn hidden(&self, _reader: &Reader) -> Vec<String> {
        Vec::new()
    }
}

/// What each drawer of a registry drew last ([`Registry::draw`]), kept by
/// what it read ([`Reads`]): a part shown again with other sketches — a
/// sketch opened, or closed — draws its sketches again, not its faces.
#[derive(Default)]
pub struct Drawings(RefCell<BTreeMap<usize, Vec<Drawn>>>);

/// What a drawer drew, and what it read to draw it.
type Drawn = (Reads, Rc<PartView>);

/// How many drawings of each drawer [`Drawings`] keeps: of the part before
/// the step edited, the part with it, and those of the last steps opened.
const KEPT_DRAWINGS: usize = 4;

impl Drawings {
    /// What the drawer `drawer` draws of the part `reader` reads, by
    /// `draw`: as it drew it last of what reads the same.
    fn drawn(
        &self,
        drawer: usize,
        reader: &Reader,
        draw: impl FnOnce(&Reader) -> GeopResult<PartView>,
    ) -> GeopResult<Rc<PartView>> {
        let mut kept = self.0.borrow_mut();
        let kept = kept.entry(drawer).or_default();
        let entry = match kept.iter().position(|(reads, _)| reader.reads_same(reads)) {
            Some(k) => kept.remove(k),
            None => {
                let (drawn, reads) = reader.recorded(draw);
                (reads, Rc::new(drawn?))
            }
        };
        let drawn = entry.1.clone();
        kept.push(entry);
        if kept.len() > KEPT_DRAWINGS {
            kept.remove(0);
        }
        Ok(drawn)
    }
}

/// An entry of the structure of the part drawn, by name: what
/// [`Drawer::structure`] lists.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StructureItem {
    /// What it is: `datum`, `sketch`, `solid`, `face`, `part`, or what a
    /// step lists it as.
    pub kind: String,
    /// What a list calls the things of its kind: `Datums`.
    pub kind_label: String,
    /// What it is shown by — and shown and hidden by, its key.
    pub name: String,
    /// What it is called, if not its name: a placed part after its file
    /// too, `base: part2`.
    pub label: Option<String>,
    /// Whether it is shown, if it is drawn at all: what an editor's
    /// visibility switches. None for what a step lists, which is not drawn.
    pub visible: Option<bool>,
    /// What it refers to: the elements a mate is between, lit while it is
    /// pointed at.
    pub elements: Vec<PickedElement>,
}

impl StructureItem {
    /// The item `name`, of the kind `kind` listed as `kind_label`, drawn and
    /// shown.
    pub fn drawn(kind: &str, kind_label: &str, name: &str) -> Self {
        Self {
            kind: kind.to_string(),
            kind_label: kind_label.to_string(),
            name: name.to_string(),
            label: None,
            visible: Some(true),
            elements: Vec::new(),
        }
    }

    /// The same item, called `label` (see [`Self::label`]).
    pub fn labelled(self, label: String) -> Self {
        Self {
            label: Some(label),
            ..self
        }
    }
}

/// A view of the program a front end shows beside the steps, and the
/// commands it is sent from there ([`crate::editor::Command::Panel`]): the
/// program's parameters, the joints of an assembly. A front end looks its
/// component up by [`Panel::KEY`].
pub trait Panel: 'static {
    /// The panel's key: what its state is under in
    /// [`crate::editor::ProgramState::panels`], and what its commands name.
    const KEY: &'static str;
    /// What it shows.
    type State: Serialize;
    /// What it is sent.
    type Command: DeserializeOwned;

    /// What it shows of `program`, the part `part` reads being what the
    /// program built as far as it runs.
    fn state(&self, program: &Program, part: &Reader) -> Self::State;

    /// Does `command`, saying what it changed.
    fn command(&self, ctx: &mut EditorContext, command: Self::Command) -> GeopResult<Changed>;

    /// Told once an edit — after a command changed the program or the step
    /// being edited, once it has run again and settled — never for every
    /// move of the pointer: where a question about the whole of what was
    /// built is asked. Nothing, unless it says otherwise.
    fn after_edit(&self, _ctx: &mut EditorContext) {}
}

/// A [`Panel`] with its types erased, to keep panels of every kind in one
/// registry.
pub trait ErasedPanel {
    fn key(&self) -> &'static str;
    fn state(&self, program: &Program, part: &Reader) -> GeopResult<serde_json::Value>;
    fn command(&self, ctx: &mut EditorContext, payload: serde_json::Value) -> GeopResult<Changed>;
    fn after_edit(&self, ctx: &mut EditorContext);
}

impl<P: Panel> ErasedPanel for P {
    fn key(&self) -> &'static str {
        P::KEY
    }

    fn state(&self, program: &Program, part: &Reader) -> GeopResult<serde_json::Value> {
        serde_json::to_value(Panel::state(self, program, part))
            .map_err(|e| GeopError::new(format!("the state of the panel `{}`: {e}", P::KEY)))
    }

    fn command(&self, ctx: &mut EditorContext, payload: serde_json::Value) -> GeopResult<Changed> {
        let command = serde_json::from_value(payload)
            .map_err(|e| GeopError::new(format!("a command of the panel `{}`: {e}", P::KEY)))?;
        Panel::command(self, ctx, command)
    }

    fn after_edit(&self, ctx: &mut EditorContext) {
        Panel::after_edit(self, ctx)
    }
}

/// What settles a program's state after every edit: moves what the steps
/// say a solve moves — where placed parts are — so that the constraints
/// they declare hold. At most one per registry; with none, the state is
/// kept as given. When to settle, and what of the program runs first, is
/// the editor's.
pub trait Settle: 'static {
    /// The values of `rerun.program`'s state it moves, and where to: first
    /// towards what `pulls` pull — every value free to give way — or else,
    /// where the constraints do not hold, the state of the step `own`
    /// being edited first. None where nothing needs to move.
    fn settle(
        &self,
        rerun: &mut Rerun,
        pulls: &[Pull],
        own: Option<&str>,
    ) -> GeopResult<StateValues>;

    /// The names of the state it solves for in the part `reader` reads:
    /// once the whole program has run, the rest of the state goes.
    fn solvable(&self, reader: &Reader) -> GeopResult<Vec<String>>;
}

/// An example by name, with what makes it: a program, or the files of one
/// made of several. Listing examples builds none.
pub type Example<T> = (&'static str, fn() -> T);

/// The files of an example made of several, by path: the first is the one
/// edited.
pub type ExampleFiles = Vec<(&'static str, Program)>;

/// Example programs, and examples made of several files, that an editor
/// offers to load ([`crate::editor::Command::LoadExample`]).
#[derive(Default)]
pub struct ExampleSet {
    pub programs: Vec<Example<Program>>,
    pub workspaces: Vec<Example<ExampleFiles>>,
}

/// Makes a tool to take in hand.
type MakeTool = Rc<dyn Fn() -> Box<dyn Tool>>;

/// Everything an editor offers: see the module docs.
#[derive(Default)]
pub struct Registry {
    editors: Editors,
    operations: OperationRegistry,
    drawers: Vec<Rc<dyn Drawer>>,
    outputs: Outputs,
    panels: Vec<Rc<dyn ErasedPanel>>,
    tools: Vec<(ToolInfo, MakeTool)>,
    settler: Option<Rc<dyn Settle>>,
    examples: Vec<ExampleSet>,
}

impl Registry {
    /// Adds the operation `operation`, in the order an editor offers its
    /// group's operations. Fails for a second of the same kind.
    pub fn operation<O: Editable>(mut self, operation: O) -> GeopResult<Self> {
        let operation = Rc::new(operation);
        self.editors.add(operation.clone())?;
        self.operations = self.operations.with_erased(operation)?;
        Ok(self)
    }

    /// Adds `drawer`, drawing after those already there.
    pub fn drawer<D: Drawer>(mut self, drawer: D) -> GeopResult<Self> {
        self.drawers.push(Rc::new(drawer));
        Ok(self)
    }

    /// Adds `exporter`, offered after those already there.
    pub fn exporter<E: Exporter>(mut self, exporter: E) -> GeopResult<Self> {
        self.outputs = self.outputs.exporter(exporter)?;
        Ok(self)
    }

    /// Adds `inspector`, offered after those already there.
    pub fn inspector<I: Inspector>(mut self, inspector: I) -> GeopResult<Self> {
        self.outputs = self.outputs.inspector(inspector)?;
        Ok(self)
    }

    /// Adds `panel`. Fails for a second of the same key.
    pub fn panel<P: Panel>(mut self, panel: P) -> GeopResult<Self> {
        if self.panels.iter().any(|p| p.key() == P::KEY) {
            return Err(GeopError::new(format!(
                "the panel `{}` is registered twice",
                P::KEY
            )));
        }
        self.panels.push(Rc::new(panel));
        Ok(self)
    }

    /// Adds `tool`, offered as `info` says: what is taken in hand by its
    /// key is a copy of it, as it is now. Fails for a second of the same
    /// key, and for a key a pick tool's could be.
    pub fn tool<T: Tool + Clone>(mut self, info: ToolInfo, tool: T) -> GeopResult<Self> {
        let key = info.key;
        if self.tools.iter().any(|(i, _)| i.key == key) || key.starts_with(PickTool::PREFIX) {
            return Err(GeopError::new(format!(
                "the tool `{key}` is registered twice"
            )));
        }
        self.tools
            .push((info, Rc::new(move || Box::new(tool.clone()))));
        Ok(self)
    }

    /// Makes `settler` what settles the program's state. Fails if there is
    /// one already: two would move the same state each its own way.
    pub fn settler<Scalar: Settle>(mut self, settler: Scalar) -> GeopResult<Self> {
        if self.settler.is_some() {
            return Err(GeopError::new("a settler is registered twice"));
        }
        self.settler = Some(Rc::new(settler));
        Ok(self)
    }

    /// Adds the examples `set`. Fails for an example of a name already
    /// there.
    pub fn examples(mut self, set: ExampleSet) -> GeopResult<Self> {
        let names = (set.programs.iter().map(|(name, _)| name))
            .chain(set.workspaces.iter().map(|(name, _)| name));
        for name in names {
            let mut known =
                (self.example_names().chain(self.workspace_example_names())).map(|n| n == *name);
            if known.any(|same| same) {
                return Err(GeopError::new(format!(
                    "the example `{name}` is registered twice"
                )));
            }
        }
        self.examples.push(set);
        Ok(self)
    }

    /// The operations, as a program runner runs them.
    pub fn operations(&self) -> OperationRegistry {
        self.operations.clone()
    }

    /// The operations, and how each is edited.
    pub fn editors(&self) -> &Editors {
        &self.editors
    }

    /// What each operation is, in the order they were added.
    pub fn infos(&self) -> Vec<Listed> {
        self.editors.infos()
    }

    /// The exporters and inspectors.
    pub fn outputs(&self) -> &Outputs {
        &self.outputs
    }

    /// The exporters and inspectors, without the rest.
    pub fn into_outputs(self) -> Outputs {
        self.outputs
    }

    /// The part `reader` reads, as every drawer draws it, with what its
    /// steps annotate it with: what a drawer drew before of what reads the
    /// same, as `drawings` kept it.
    pub fn draw(&self, reader: &Reader, drawings: &Drawings) -> GeopResult<PartView> {
        let mut view = PartView::new();
        for (k, drawer) in self.drawers.iter().enumerate() {
            let drawn = drawings.drawn(k, reader, |reader| {
                let mut drawn = PartView::new();
                drawer.draw(reader, &mut drawn)?;
                Ok(drawn)
            })?;
            view.add(&drawn);
        }
        view.annotations = annotations_of(reader);
        Ok(view.finish())
    }

    /// What the part `reader` reads has, as every drawer lists it, each
    /// drawn item shown unless `hidden` names it — and whatever its steps
    /// list.
    pub fn structure(&self, reader: &Reader, hidden: &[String]) -> GeopResult<Vec<StructureItem>> {
        let mut items = Vec::new();
        for drawer in &self.drawers {
            items.extend(drawer.structure(reader)?);
        }
        for item in &mut items {
            if item.visible.is_some() {
                item.visible = Some(!hidden.contains(&item.name));
            }
        }
        items.extend(
            listed_by_steps(reader)
                .into_iter()
                .map(|listed| StructureItem {
                    kind_label: plural(&listed.kind),
                    kind: listed.kind,
                    name: listed.name,
                    label: None,
                    visible: None,
                    elements: listed.elements,
                }),
        );
        Ok(items)
    }

    /// What the drawers draw of the part `reader` reads only to be picked
    /// (see [`Drawer::hidden`]).
    pub fn hidden(&self, reader: &Reader) -> Vec<String> {
        (self.drawers.iter())
            .flat_map(|drawer| drawer.hidden(reader))
            .collect()
    }

    /// The panels, in the order they were added.
    pub fn panels(&self) -> impl Iterator<Item = &dyn ErasedPanel> {
        self.panels.iter().map(Rc::as_ref)
    }

    /// The panel `key`.
    pub fn get_panel(&self, key: &str) -> GeopResult<&dyn ErasedPanel> {
        (self.panels())
            .find(|panel| panel.key() == key)
            .ok_or_else(|| GeopError::new(format!("there is no panel `{key}`")))
    }

    /// The tools added, as a toolbar offers them, in the order they were
    /// added.
    pub fn tools(&self) -> Vec<&ToolInfo> {
        self.tools.iter().map(|(info, _)| info).collect()
    }

    /// A new tool `key`, to take in hand: one added, or, for
    /// `pick:<inspector>`, the pick tool of that inspector.
    pub fn tool_in_hand(&self, key: &str) -> GeopResult<Box<dyn Tool>> {
        if let Some(inspector) = key.strip_prefix(PickTool::PREFIX) {
            let info = self.outputs.get_inspector(inspector)?.info();
            return Ok(Box::new(PickTool::new(info)?));
        }
        let (_, make) = (self.tools.iter())
            .find(|(info, _)| info.key == key)
            .ok_or_else(|| GeopError::new(format!("there is no tool `{key}`")))?;
        Ok(make())
    }

    /// What settles the program's state, if anything does.
    pub fn get_settler(&self) -> Option<&dyn Settle> {
        self.settler.as_deref()
    }

    /// The names of the example programs.
    pub fn example_names(&self) -> impl Iterator<Item = &'static str> + '_ {
        (self.examples.iter()).flat_map(|set| set.programs.iter().map(|(name, _)| *name))
    }

    /// The example programs, each by name with what makes it: listing them
    /// builds none.
    pub fn example_programs(&self) -> Vec<Example<Program>> {
        (self.examples.iter())
            .flat_map(|set| set.programs.iter().copied())
            .collect()
    }

    /// The names of the examples made of several files.
    pub fn workspace_example_names(&self) -> impl Iterator<Item = &'static str> + '_ {
        (self.examples.iter()).flat_map(|set| set.workspaces.iter().map(|(name, _)| *name))
    }

    /// The example program `name`, made.
    pub fn example(&self, name: &str) -> GeopResult<Program> {
        (self.examples.iter())
            .flat_map(|set| &set.programs)
            .find(|(n, _)| *n == name)
            .map(|(_, make)| make())
            .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))
    }

    /// The files of the example of several files `name`, made.
    pub fn workspace_example(&self, name: &str) -> GeopResult<ExampleFiles> {
        (self.examples.iter())
            .flat_map(|set| &set.workspaces)
            .find(|(n, _)| *n == name)
            .map(|(_, make)| make())
            .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))
    }
}

/// What a list calls the things of the kind `kind` a step lists: `mate`
/// as `Mates`.
fn plural(kind: &str) -> String {
    let words = kind.replace('_', " ");
    let mut chars = words.chars();
    match chars.next() {
        Some(first) => format!("{}{}s", first.to_uppercase(), chars.as_str()),
        None => String::new(),
    }
}
