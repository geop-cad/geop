//! Parts placed in a part: the [`Library`] a run builds them with — another
//! program file's part, as a [`Component`] — placed as the instances of
//! [`geop_core_program::instance`].
//!
//! Where an instance is, is the program's state: the pose state of its
//! [`Placement`], or a fixed motion after another instance, as a pattern's
//! copy goes. The parts placed in a placed part are the placing program's
//! to move too: their poses are its state, behind the instance's name
//! (`bolt/nut.pose`), and the placed part is run with them. A component is
//! built once for every state it is run with — a part with no state of its
//! own once, however often it is placed — and drawn once
//! ([`Component::drawn`]).

use std::{
    cell::OnceCell,
    collections::{BTreeSet, HashMap},
    rc::Rc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, TriangleFace},
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    instance::{Factor, INSTANCE_SEPARATOR, Subpart, instance_path, split_path},
    part::{Input, Part, Reader, State},
    target::TargetReference,
};
use geop_ops_model::{Design, SolidMeshes, read_model};
use geop_ops_constraints::{solve::Mobility, unknown::Unknowns};
use geop_ops_parameters::{
    BuildPart, Files, Parameters, PartReader, Settings, StateValue, StateValues, compose,
    read_about,
};
use geop_ops_view::PartView;

/// Where a program being built finds the parts it places, besides the files
/// its steps read: the given target [`library`].
pub trait Library: Files {
    /// The part the program in `file` builds — `file` relative to the
    /// program being built — with its parameters `overrides` instead of
    /// its own, run with the values `state` gives of its state instead of
    /// its own.
    fn component(
        &self,
        file: &str,
        overrides: &Settings,
        state: &StateValues,
    ) -> GeopResult<Rc<Component>>;
}

/// The library a run places parts from, as given to it: equal only to
/// itself, so a step reading it is rebuilt when it is given another.
#[derive(Clone)]
pub struct GivenLibrary(pub Rc<dyn Library>);

impl PartialEq for GivenLibrary {
    fn eq(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

/// The library a run places parts from, defined by the given step.
pub fn library() -> TargetReference<GivenLibrary> {
    TargetReference::new("library")
}

/// Read whole.
impl Kind for GivenLibrary {
    const NAME: &'static str = "library";
}

impl HasElements<GivenLibrary> for GivenLibrary {
    type Element<'a> = &'a dyn Library;
    const SINGLETON: Option<&'static str> = Some("library");

    fn element(&self, name: &str) -> GeopResult<&dyn Library> {
        Ok(whole(self, name)?.0.as_ref())
    }
}

/// Defines `library` as the run's [`library`]: in the given step.
pub fn define_library<I: Input, S: State>(
    part: &mut Part<I, S>,
    library: &GivenLibrary,
) -> GeopResult<()> {
    part.define_target(&self::library(), library, |_, library| Ok(library.clone()))
}

/// The library a run of `reader` places parts from: an error, for a program
/// that is built without one.
pub fn read_library<'a, I: Input, S: State>(
    reader: &Reader<'a, I, S>,
) -> GeopResult<&'a dyn Library> {
    if !reader.defines(&library()) {
        return Err(GeopError::new(
            "this program is built without any other files: there is no library to place parts from",
        ));
    }
    reader.retrieve_element(&ElementReference::whole(library()))
}

/// How a component's part is drawn: what the application composes of what
/// each crate draws.
pub type Draw = fn(&PartReader) -> GeopResult<PartView>;

/// The part another program file builds, as its run left it: what an
/// [`Instance`] places.
pub struct Component {
    /// The program file, as the library names files.
    pub file: String,
    /// Its own file and every file it places, directly or through others:
    /// what placing it must not form a cycle with, and what changing makes
    /// it stale.
    pub files: BTreeSet<String>,
    /// What tells this build apart from every other: a viewer keeps a part's
    /// view by it, so that it is sent once however often it is drawn.
    pub key: String,
    /// The parameters its program declares, which a program placing it
    /// can give values of its own.
    pub parameters: Parameters,
    pub part: BuildPart,
    draw: Draw,
    drawn: OnceCell<Rc<PartView>>,
}

impl std::fmt::Debug for Component {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Component({})", self.key)
    }
}

impl Component {
    pub fn new(
        file: String,
        files: BTreeSet<String>,
        key: String,
        parameters: Parameters,
        part: BuildPart,
        draw: Draw,
    ) -> Self {
        Self {
            file,
            files,
            key,
            parameters,
            part,
            draw,
            drawn: OnceCell::new(),
        }
    }

    /// The part as its program left it.
    pub fn reader(&self) -> PartReader<'_> {
        self.part.current()
    }

    /// What its part is ordered as (see
    /// [`geop_ops_parameters::About::designation`]).
    pub fn designation(&self) -> Option<String> {
        read_about(&self.reader()).ok()?.designation()
    }

    /// The part as drawn where it is placed: drawn once, the first time it
    /// is asked for — without its sketches, which are what it was drawn
    /// with, its own.
    pub fn drawn(&self) -> GeopResult<Rc<PartView>> {
        if let Some(view) = self.drawn.get() {
            return Ok(view.clone());
        }
        let mut view = (self.draw)(&self.reader())?;
        view.sketches.clear();
        let view = Rc::new(view);
        Ok(self.drawn.get_or_init(|| view).clone())
    }
}

impl Subpart<Design, StateValue> for Component {
    fn part(&self) -> &BuildPart {
        &self.part
    }

    fn into_any(self: Rc<Self>) -> Rc<dyn std::any::Any> {
        self
    }
}

/// A part placed in a part (see [`geop_core_program::instance`]).
pub type Instance = geop_core_program::instance::Instance<Design, StateValue>;

/// Where an [`Instance`] is, in the part it is placed in.
pub type Placement = geop_core_program::instance::Placement<StateValue>;

/// The component `instance` places: every instance of a part places one.
pub fn component_of(instance: &Instance) -> GeopResult<Rc<Component>> {
    (instance.component.clone().into_any().downcast::<Component>())
        .map_err(|_| GeopError::new("a placed part is no component of a library"))
}

/// A part placed in the part read, however deep: its instance, its pose in
/// the part read, and the state that pose is, if it is the part read's to
/// move.
pub struct Located<'a> {
    pub instance: &'a Instance,
    pub component: Rc<Component>,
    pub world: Pose<Design>,
    /// Where it is in the part it is placed in.
    pub local: Pose<Design>,
    pub state: Option<String>,
}

/// The part placed at `path` — `bolt`, or `asm/bolt` for one placed in a
/// placed part — of the part `reader` reads, where the state has it.
pub fn locate<'a>(reader: &PartReader<'a>, path: &str) -> GeopResult<Located<'a>> {
    let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
    let (outer, name) = split_path(path);
    let level = match outer.is_empty() {
        true => reader.clone(),
        false => reader.inside(outer).with_context(&ctx)?,
    };
    let instance = level.instance(name).with_context(&ctx)?;
    let factors = reader.placement(path).with_context(&ctx)?;
    let outside = match outer.is_empty() {
        true => 0,
        false => reader.placement(outer).with_context(&ctx)?.len(),
    };
    let state = match (factors.last(), instance.is_copy()) {
        (Some(Factor::State { state, .. }), false) => Some(state.clone()),
        _ => None,
    };
    let own = factors.get(outside..).unwrap_or_default();
    Ok(Located {
        instance,
        component: component_of(instance).with_context(&ctx)?,
        world: compose(&factors, &mut |_, pose| pose),
        local: compose(own, &mut |_, pose| pose),
        state,
    })
}

/// The state that moves the part placed at `path` — as [`Located::state`]
/// has it, if the part read moves it at all. Reads no state: which state
/// moves a part does not depend on where it is.
pub fn moved_by(reader: &PartReader, path: &str) -> GeopResult<Option<String>> {
    let mut level = reader.clone();
    let mut prefix = String::new();
    let mut state = None;
    for name in path.split(INSTANCE_SEPARATOR) {
        let instance = level.instance(name)?;
        match &instance.placement {
            Placement::State(own) => state = Some(instance_path(&prefix, own)),
            Placement::After { .. } => return Ok(None),
        }
        prefix = instance_path(&prefix, name);
        level = instance.reader();
    }
    Ok(state)
}

/// A part placed in the part a reader reads, however deep.
pub struct Placed<'a> {
    pub name: String,
    pub located: Located<'a>,
    /// The part it is placed in, by index among them all, if not the
    /// part read itself.
    pub parent: Option<usize>,
}

/// Every part placed in the part `reader` reads, and in those, however
/// deep: each before those placed in it, in the order the steps placed
/// them.
pub fn placed_parts<'a>(reader: &PartReader<'a>) -> GeopResult<Vec<Placed<'a>>> {
    fn within<'a>(
        outer: &PartReader<'a>,
        level: &PartReader<'a>,
        prefix: &str,
        parent: Option<usize>,
        out: &mut Vec<Placed<'a>>,
    ) -> GeopResult<()> {
        for (name, instance) in level.instances() {
            let path = instance_path(prefix, name);
            let located = locate(outer, &path)?;
            out.push(Placed {
                name: path.clone(),
                located,
                parent,
            });
            let index = Some(out.len() - 1);
            within(outer, &instance.reader(), &path, index, out)?;
        }
        Ok(())
    }
    let mut out = Vec::new();
    within(reader, reader, "", None, &mut out)?;
    Ok(out)
}

/// Every solid of the part `reader` reads, meshed `quality` fine: its own,
/// and those of every part placed in it, however deep — each where it is,
/// named behind its name in the part read: `pin/extrude(pin)`. A part
/// placed many times is meshed once.
pub fn solid_meshes(reader: &PartReader, quality: usize) -> GeopResult<SolidMeshes<Design>> {
    let own = |reader: &PartReader| read_model(reader)?.solid_meshes(quality);
    let mut out = own(reader)?;
    let mut meshed: HashMap<*const Component, SolidMeshes<Design>> = HashMap::new();
    for placed in placed_parts(reader)? {
        let component = &placed.located.component;
        let meshes = match meshed.entry(Rc::as_ptr(component)) {
            std::collections::hash_map::Entry::Occupied(meshes) => meshes.into_mut(),
            std::collections::hash_map::Entry::Vacant(slot) => {
                slot.insert(own(&component.reader())?)
            }
        };
        let motion = placed.located.world.motion();
        for (name, triangles) in meshes.iter() {
            let moved = (triangles.iter())
                .map(|t| TriangleFace {
                    a: motion.apply(&t.a),
                    b: motion.apply(&t.b),
                    c: motion.apply(&t.c),
                    normal: motion.rotate(&t.normal),
                    vertex_normals: t.vertex_normals.map(|n| n.map(|v| motion.rotate(&v))),
                })
                .collect();
            out.push((instance_path(&placed.name, name), moved));
        }
    }
    Ok(out)
}

/// Draws every part placed in the part `reader` reads, however deep, into
/// `view`, where the state has it: each as its own part's view, which
/// leaves out the parts placed in it — they are drawn here already.
///
/// One is drawn fixed that no solve moves: a copy, or one a step holds
/// fixed.
pub fn draw_instances(reader: &PartReader, view: &mut PartView) -> GeopResult<()> {
    let unknowns = Unknowns::<StateValue>::of(reader)?;
    for placed in placed_parts(reader)? {
        let located = placed.located;
        let pose = located.world;
        let origin = geop_core_math::primitives::CoordinateSystem::world_at(
            geop_core_math::vector::Vector3::zero(),
        );
        view.instances.push(geop_ops_view::ViewInstance {
            frame: pose.motion().apply_frame(&origin)?,
            name: placed.name,
            component: located.component.key.clone(),
            pose,
            view: located.component.drawn()?,
            fixed: (located.state.as_ref())
                .and_then(|state| unknowns.get(state))
                .is_none_or(|unknown| unknown.mobility == Mobility::Fixed),
            state: located.state,
        });
    }
    Ok(())
}
