//! Parts placed in a part: the [`Library`] a run builds them with — another
//! program file's part, as a [`Component`] — and the [`Instance`]s that
//! place them, each a target of its own ([`instance`]).
//!
//! Where an instance is, is the program's state: the pose state of its
//! [`Placement`], or a fixed motion after another instance, as a pattern's
//! copy goes. The parts placed in a placed part are the placing program's
//! to move too: their poses are its state, behind the instance's name
//! (`bolt/nut.pose`), and start where the placed part's own program has
//! them. So a component is built once, whatever moves in it, and drawn
//! once ([`Component::drawn`]) however often it is placed.

use std::{
    cell::OnceCell,
    collections::{BTreeSet, HashMap},
    rc::Rc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, TriangleFace},
    scalars::Scalar,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::{Input, Part, Reader, State},
    target::TargetReference,
};
use geop_ops_model::{Design, SolidMeshes, read_model};
use geop_ops_parameters::{
    BuildPart, Files, Parameters, PartReader, Settings, StateValue, read_about,
};
use geop_ops_view::{INSTANCE_SEPARATOR, PartView};

/// Where a program being built finds the parts it places, besides the files
/// its steps read: the given target [`library`].
pub trait Library: Files {
    /// The part the program in `file` builds — `file` relative to the
    /// program being built — with its parameters `overrides` instead of
    /// its own.
    fn component(&self, file: &str, overrides: &Settings) -> GeopResult<Rc<Component>>;
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

/// Where an [`Instance`] is, in the part it is placed in.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement {
    /// Where the pose state `state` puts it: moved by solving the mates,
    /// and by dragging it.
    State(String),
    /// `motion` after where the instance `of` of the same part is: a copy
    /// of a pattern, which goes with the part it copies.
    After { of: String, motion: Pose<Design> },
}

/// A part placed in a part: a [`Component`], where its [`Placement`] says.
#[derive(Clone, Debug)]
pub struct Instance {
    pub component: Rc<Component>,
    pub placement: Placement,
    /// Whether nothing moves it — no mate, no drag but its own step's.
    pub fixed: bool,
}

/// Equal when it places the very same build, the same way: a component
/// built again counts as changed, however alike.
impl PartialEq for Instance {
    fn eq(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.component, &other.component)
            && self.placement == other.placement
            && self.fixed == other.fixed
    }
}

/// What the target of an instance is named, before the instance's name.
const PREFIX: &str = "instance:";

/// The instance `name`: the part the step of that id places.
pub fn instance(name: &str) -> TargetReference<Instance> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// The name of the instance whose target is named `target`, if it is one.
pub fn instance_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}

/// The path of the part placed as `name` in the part placed at `outer`:
/// `asm/bolt`.
pub fn instance_path(outer: &str, name: &str) -> String {
    format!("{outer}{INSTANCE_SEPARATOR}{name}")
}

/// Read whole.
impl Kind for Instance {
    const NAME: &'static str = "instance";
}

impl HasElements<Instance> for Instance {
    type Element<'a> = &'a Instance;

    fn element(&self, name: &str) -> GeopResult<&Instance> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

/// The instance `name` of the part `reader` reads.
pub fn read_instance<'a, I: Input, S: State>(
    reader: &Reader<'a, I, S>,
    name: &str,
) -> GeopResult<&'a Instance> {
    reader.retrieve_element(&ElementReference::whole(instance(name)))
}

/// The pose a state holds, if it holds one.
fn pose_in(value: StateValue) -> Option<Pose<Design>> {
    match value {
        StateValue::Pose(pose) => Some(pose),
        StateValue::Number(_) => None,
    }
}

/// One part of the instances a path names: the instances it reads, and
/// where they are.
trait Level<'a> {
    fn instance(&self, name: &str) -> GeopResult<&'a Instance>;

    /// The state the pose state `state` of this part is, as the part read
    /// names it, and the pose it holds there, if any.
    fn pose(&self, state: &str) -> (String, Option<Pose<Design>>);
}

/// The part read itself: its instances, and its state.
struct Top<'r, 'a>(&'r PartReader<'a>);

impl<'a> Level<'a> for Top<'_, 'a> {
    fn instance(&self, name: &str) -> GeopResult<&'a Instance> {
        read_instance(self.0, name)
    }

    fn pose(&self, state: &str) -> (String, Option<Pose<Design>>) {
        (state.to_string(), pose_in(self.0.retrieve_state(state)))
    }
}

/// A part placed — however deep — in the part read, at `prefix`: its
/// instances are its own, where they are the part read's state, behind
/// the prefix, or else its own.
struct Nested<'r, 'a> {
    outer: &'r PartReader<'a>,
    inner: PartReader<'a>,
    prefix: String,
}

impl<'a> Level<'a> for Nested<'_, 'a> {
    fn instance(&self, name: &str) -> GeopResult<&'a Instance> {
        read_instance(&self.inner, name)
    }

    fn pose(&self, state: &str) -> (String, Option<Pose<Design>>) {
        let outer = format!("{}{state}", self.prefix);
        let pose = pose_in(self.outer.retrieve_state(&outer))
            .or_else(|| pose_in(self.inner.retrieve_state(state)));
        (outer, pose)
    }
}

/// How the pose of each state a placement reads enters it, computed in `T`:
/// as it is, or moved by a step to differentiate by it. Given the state's
/// name and the pose it holds.
pub type PoseOf<'f, T> = dyn FnMut(&str, Pose<Design>) -> Pose<T> + 'f;

/// The instance `name` of `level`, and where it is in its part, in `T` —
/// each pose state it reads entering as `pose_of` has it. A state that
/// holds no pose — before anything solved for it — puts it at the origin.
fn place<'a, T: Scalar>(
    level: &impl Level<'a>,
    name: &str,
    pose_of: &mut PoseOf<'_, T>,
) -> GeopResult<(&'a Instance, Pose<T>)> {
    let instance = level.instance(name)?;
    let pose = match &instance.placement {
        Placement::State(state) => {
            let (state, pose) = level.pose(state);
            pose_of(&state, pose.unwrap_or_else(Pose::identity))
        }
        Placement::After { of, motion } => {
            let ctx = |e: GeopError| e.with_context(format!("{name} goes after {of}"));
            motion
                .cast()
                .compose(&place(level, of, pose_of).with_context(&ctx)?.1)
        }
    };
    Ok((instance, pose))
}

/// A part placed in the part read, however deep: its instance, its pose in
/// the part read, and the state that pose is, if it is the part read's to
/// move.
pub struct Located<'a, T: Scalar = Design> {
    pub instance: &'a Instance,
    pub world: Pose<T>,
    /// Where it is in the part it is placed in.
    pub local: Pose<T>,
    pub state: Option<String>,
}

/// The part placed at `path` — `bolt`, or `asm/bolt` for one placed in a
/// placed part — of the part `reader` reads.
pub fn locate<'a>(reader: &PartReader<'a>, path: &str) -> GeopResult<Located<'a>> {
    locate_in(reader, path, &mut |_, pose| pose)
}

/// [`locate`] in `T`, each pose state the placement reads entering as
/// `pose_of` has it — as it is, or moved by a step, to differentiate where
/// the part is by that state.
pub fn locate_in<'a, T: Scalar>(
    reader: &PartReader<'a>,
    path: &str,
    pose_of: &mut PoseOf<'_, T>,
) -> GeopResult<Located<'a, T>> {
    let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
    let mut names = path.split(INSTANCE_SEPARATOR);
    let first = names.next().unwrap_or_default();
    let (mut instance, mut local) = place(&Top(reader), first, pose_of).with_context(&ctx)?;
    let mut world = local;
    let mut prefix = format!("{first}{INSTANCE_SEPARATOR}");
    let mut state = state_of(instance, "");
    for name in names {
        let level = Nested {
            outer: reader,
            inner: instance.component.reader(),
            prefix: prefix.clone(),
        };
        (instance, local) = place(&level, name, pose_of).with_context(&ctx)?;
        world = world.compose(&local);
        state = state.and(state_of(instance, &prefix));
        prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
    }
    Ok(Located {
        instance,
        world,
        local,
        state,
    })
}

/// The state that moves `instance`, placed behind `prefix`: its pose's
/// state, if it has one. A part moves only with its own state, and those
/// of the parts placed flexibly around it: a copy's parts go with the copy.
fn state_of(instance: &Instance, prefix: &str) -> Option<String> {
    match &instance.placement {
        Placement::State(state) => Some(format!("{prefix}{state}")),
        Placement::After { .. } => None,
    }
}

/// The state that moves the part placed at `path` — as [`Located::state`]
/// has it, if the part read moves it at all. Reads no state: which state
/// moves a part does not depend on where it is.
pub fn moved_by(reader: &PartReader, path: &str) -> GeopResult<Option<String>> {
    let mut names = path.split(INSTANCE_SEPARATOR);
    let first = names.next().unwrap_or_default();
    let mut placed = read_instance(reader, first)?;
    let mut state = state_of(placed, "");
    let mut prefix = format!("{first}{INSTANCE_SEPARATOR}");
    for name in names {
        placed = read_instance(&placed.component.reader(), name)?;
        state = state.and(state_of(placed, &prefix));
        prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
    }
    Ok(state)
}

/// The part placed at `path` in the part `reader` reads, as its own
/// program left it: the part itself, for an empty path. Reads no state:
/// what a placed part is does not depend on where it is.
pub fn component_at<'a>(reader: &PartReader<'a>, path: &str) -> GeopResult<PartReader<'a>> {
    if path.is_empty() {
        return Ok(reader.clone());
    }
    let mut inside = reader.clone();
    for name in path.split(INSTANCE_SEPARATOR) {
        inside = read_instance(&inside, name)?.component.reader();
    }
    Ok(inside)
}

/// The part placed at `path` in the part `reader` reads, as its own
/// program left it, and where it is: the part itself, at no pose, for an
/// empty path.
pub fn part_at<'a>(
    reader: &PartReader<'a>,
    path: &str,
) -> GeopResult<(PartReader<'a>, Option<Pose<Design>>)> {
    if path.is_empty() {
        return Ok((reader.clone(), None));
    }
    let located = locate(reader, path)?;
    Ok((located.instance.component.reader(), Some(located.world)))
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
        for target in level.targets_of::<Instance>() {
            let Some(name) = instance_name(target.name()) else {
                continue;
            };
            let inner = level.retrieve_element(&ElementReference::whole(target.clone()))?;
            let path = format!("{prefix}{name}");
            let located = locate(outer, &path)?;
            out.push(Placed {
                name: path.clone(),
                located,
                parent,
            });
            let index = Some(out.len() - 1);
            let prefix = format!("{path}{INSTANCE_SEPARATOR}");
            within(outer, &inner.component.reader(), &prefix, index, out)?;
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
        let component = &placed.located.instance.component;
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
pub fn draw_instances(reader: &PartReader, view: &mut PartView) -> GeopResult<()> {
    for placed in placed_parts(reader)? {
        let located = placed.located;
        let pose = located.world;
        let origin = geop_core_math::primitives::CoordinateSystem::world_at(
            geop_core_math::vector::Vector3::zero(),
        );
        view.instances.push(geop_ops_view::ViewInstance {
            frame: pose.motion().apply_frame(&origin)?,
            name: placed.name,
            component: located.instance.component.key.clone(),
            pose,
            view: located.instance.component.drawn()?,
            state: located.state,
            fixed: located.instance.fixed,
        });
    }
    Ok(())
}
