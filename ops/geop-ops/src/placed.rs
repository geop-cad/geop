//! Parts placed in a part: the [`Library`] a run builds them with — another
//! program file's part, as a [`Component`] — and the [`Instance`]s that
//! place them, each a target of its own ([`instance`]).
//!
//! Where an instance is, is the program's state: the pose parameter of its
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
    part::{Retrieve, View},
    target::TargetReference,
};

use crate::{
    Design,
    model::{ParamValue, SolidMeshes, model},
    operation::{Build, EntityRef, INSTANCE_SEPARATOR, Input, Seen},
    parameters::{Parameters, Values},
    ui::PartView,
};

/// The name of the input a run finds its [`Library`] under.
pub const LIBRARY: &str = "library";

/// Where a program being built finds the parts it places, and the files its
/// steps read.
pub trait Library {
    /// The program being built's own file, as the library names files.
    fn file(&self) -> String;

    /// The part the program in `file` builds — `file` relative to the
    /// program being built — with its parameters `overrides` instead of
    /// its own.
    fn component(&self, file: &str, overrides: &Values) -> GeopResult<Rc<Component>>;

    /// The files the program being built could place or read, as it would
    /// name them: every file but its own.
    fn files(&self) -> Vec<String>;

    /// The file `file`, relative to the program being built: its path, as
    /// the library names files, and its text.
    fn read(&self, file: &str) -> GeopResult<(String, String)>;
}

/// The library a run was given: none, for a program that places nothing.
pub fn library<'a>(r: &mut impl Retrieve<'a, Input, ParamValue>) -> GeopResult<Rc<dyn Library>> {
    match r.retrieve_input(LIBRARY) {
        Some(Input::Library(library)) => Ok(library),
        _ => Err(GeopError::new(
            "this program is built without any other files: there is no library to place parts from",
        )),
    }
}

/// The name of the input a run finds its [`About`] under.
pub const ABOUT: &str = "about";

/// What the program a run builds says of its part besides its steps: the
/// file it is in, as the library names files, and the parameters it
/// declares — its colour, material, title and designation.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct About {
    pub file: String,
    pub parameters: Parameters,
}

/// What a part whose material is not given is weighed as: water, 1000
/// kg/m³.
pub const UNGIVEN_DENSITY: f64 = 1000.0;

/// What a part is made of, as it is weighed.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialOf {
    pub name: String,
    /// kg/m³.
    pub density: f64,
    /// Whether no material was given, and it is weighed as water.
    pub assumed: bool,
}

impl About {
    /// What its part is made of: its material — or, not given, water, and
    /// said so.
    pub fn material(&self) -> MaterialOf {
        match &self.parameters.material {
            Some(material) => MaterialOf {
                name: material.name.clone(),
                density: material.density,
                assumed: false,
            },
            None => MaterialOf {
                name: "Water (no material given)".into(),
                density: UNGIVEN_DENSITY,
                assumed: true,
            },
        }
    }

    /// The name of its file, without folders or `.geop`: what a file
    /// written from its part is named after — `part` if that leaves
    /// nothing.
    pub fn stem(&self) -> &str {
        let file = self.file.rsplit(['/', ':']).next().unwrap_or_default();
        match file.trim_end_matches(".geop") {
            "" => "part",
            stem => stem,
        }
    }

    /// What its part is called where it is listed: its title, else its
    /// file's [`About::stem`].
    pub fn name(&self) -> &str {
        self.parameters.title.as_deref().unwrap_or(self.stem())
    }
}

/// What the program of the part `view` shows says of it: nothing, for a
/// part built without a program — an operation's test.
pub fn about(view: View<'_, Input, ParamValue>) -> Rc<About> {
    match view.retrieve_input(ABOUT) {
        Some(Input::About(about)) => about,
        _ => Rc::default(),
    }
}

/// The values of the parameters the program of the part `view` shows
/// declares, as it was built with them.
pub fn declared_values(view: View<'_, Input, ParamValue>) -> Values {
    (about(view).parameters.values.iter())
        .filter_map(|parameter| match view.retrieve_input(&parameter.name)? {
            Input::Value(value) => Some((parameter.name.clone(), value)),
            Input::Library(_) | Input::About(_) => None,
        })
        .collect()
}

/// What the part `view` shows is ordered as, with the values it was built
/// with (see [`Parameters::designate`]): none for a part made rather than
/// bought.
pub fn designation(view: View<'_, Input, ParamValue>) -> Option<String> {
    about(view).parameters.designate(&declared_values(view))
}

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
    pub part: Build,
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
        part: Build,
    ) -> Self {
        Self {
            file,
            files,
            key,
            parameters,
            part,
            drawn: OnceCell::new(),
        }
    }

    /// The part as its program left it.
    pub fn view(&self) -> View<'_, Input, ParamValue> {
        self.part.current()
    }

    /// What its part is ordered as (see [`designation`]).
    pub fn designation(&self) -> Option<String> {
        designation(self.view())
    }

    /// The part as drawn where it is placed: drawn once, the first time it
    /// is asked for — without its sketches, which are what it was drawn
    /// with, its own.
    pub fn drawn(&self) -> GeopResult<Rc<PartView>> {
        if let Some(view) = self.drawn.get() {
            return Ok(view.clone());
        }
        let mut view = PartView::of(&Seen::new(self.view()))?;
        view.sketches.clear();
        let view = Rc::new(view);
        Ok(self.drawn.get_or_init(|| view).clone())
    }
}

/// Where an [`Instance`] is, in the part it is placed in.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement {
    /// Where the pose state `parameter` puts it: moved by solving the
    /// mates, and by dragging it.
    Parameter(String),
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

/// The pose a state holds, if it holds one.
fn pose_in(value: ParamValue) -> Option<Pose<Design>> {
    match value {
        ParamValue::Pose(pose) => Some(pose),
        _ => None,
    }
}

/// One part of the instances a path names: the instances it reads, and
/// where they are.
trait Level<'a> {
    fn instance(&mut self, name: &str) -> GeopResult<&'a Instance>;

    /// The state the parameter `parameter` of this part is, as the part
    /// read names it, and the pose it holds there, if any.
    fn pose(&mut self, parameter: &str) -> (String, Option<Pose<Design>>);
}

/// The part read itself: its instances, and its state.
struct Top<'r, R>(&'r mut R);

impl<'a, R: Retrieve<'a, Input, ParamValue>> Level<'a> for Top<'_, R> {
    fn instance(&mut self, name: &str) -> GeopResult<&'a Instance> {
        self.0.retrieve_target(&instance(name))
    }

    fn pose(&mut self, parameter: &str) -> (String, Option<Pose<Design>>) {
        (
            parameter.to_string(),
            pose_in(self.0.retrieve_state(parameter)),
        )
    }
}

/// A part placed — however deep — in the part read, at `prefix`: its
/// instances are its own, where they are the part read's state, behind
/// the prefix, or else its own.
struct Nested<'r, 'a, R> {
    outer: &'r mut R,
    view: View<'a, Input, ParamValue>,
    prefix: String,
}

impl<'a, R: Retrieve<'a, Input, ParamValue>> Level<'a> for Nested<'_, 'a, R> {
    fn instance(&mut self, name: &str) -> GeopResult<&'a Instance> {
        self.view.retrieve_target(&instance(name))
    }

    fn pose(&mut self, parameter: &str) -> (String, Option<Pose<Design>>) {
        let outer = format!("{}{parameter}", self.prefix);
        let pose = pose_in(self.outer.retrieve_state(&outer))
            .or_else(|| pose_in(self.view.retrieve_state(parameter)));
        (outer, pose)
    }
}

/// How the pose of each state a placement reads enters it, computed in `T`:
/// as it is, or moved by a step to differentiate by it. Given the state's
/// name and the pose it holds.
pub type PoseOf<'f, T> = dyn FnMut(&str, Pose<Design>) -> Pose<T> + 'f;

/// The instance `name` of `level`, and where it is in its part, in `T` —
/// each pose state it reads entering as `pose_of` has it. A parameter that
/// holds no pose — before anything solved for it — puts it at the origin.
fn place<'a, T: Scalar>(
    level: &mut impl Level<'a>,
    name: &str,
    pose_of: &mut PoseOf<'_, T>,
) -> GeopResult<(&'a Instance, Pose<T>)> {
    let instance = level.instance(name)?;
    let pose = match &instance.placement {
        Placement::Parameter(parameter) => {
            let (state, pose) = level.pose(parameter);
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
/// the part read, and the parameter that pose is, if it is the part read's
/// to move.
pub struct Located<'a, T: Scalar = Design> {
    pub instance: &'a Instance,
    pub world: Pose<T>,
    /// Where it is in the part it is placed in.
    pub local: Pose<T>,
    pub parameter: Option<String>,
}

/// The part placed at `path` — `bolt`, or `asm/bolt` for one placed in a
/// placed part — read through `r`.
pub fn locate<'a, R: Retrieve<'a, Input, ParamValue>>(
    r: &mut R,
    path: &str,
) -> GeopResult<Located<'a>> {
    locate_in(r, path, &mut |_, pose| pose)
}

/// [`locate`] in `T`, each pose state the placement reads entering as
/// `pose_of` has it — as it is, or moved by a step, to differentiate where
/// the part is by that state.
pub fn locate_in<'a, T: Scalar, R: Retrieve<'a, Input, ParamValue>>(
    r: &mut R,
    path: &str,
    pose_of: &mut PoseOf<'_, T>,
) -> GeopResult<Located<'a, T>> {
    let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
    let mut names = path.split(INSTANCE_SEPARATOR);
    let first = names.next().unwrap_or_default();
    let (mut instance, mut local) = place(&mut Top(r), first, pose_of).with_context(&ctx)?;
    let mut world = local;
    let mut prefix = format!("{first}{INSTANCE_SEPARATOR}");
    let mut parameter = parameter_of(instance, "");
    for name in names {
        let view = instance.component.view();
        let mut level = Nested {
            outer: &mut *r,
            view,
            prefix: prefix.clone(),
        };
        (instance, local) = place(&mut level, name, pose_of).with_context(&ctx)?;
        world = world.compose(&local);
        parameter = parameter.and(parameter_of(instance, &prefix));
        prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
    }
    Ok(Located {
        instance,
        world,
        local,
        parameter,
    })
}

/// The state that moves `instance`, placed behind `prefix`: its pose's
/// parameter, if it has one. A part moves only with its own parameter, and
/// those of the parts placed flexibly around it: a copy's parts go with the
/// copy.
fn parameter_of(instance: &Instance, prefix: &str) -> Option<String> {
    match &instance.placement {
        Placement::Parameter(parameter) => Some(format!("{prefix}{parameter}")),
        Placement::After { .. } => None,
    }
}

/// The state that moves the part placed at `path` — as [`Located::parameter`]
/// has it, if the part read moves it at all — read through `r`. Reads no
/// state: which state moves a part does not depend on where it is.
pub fn moved_by<'a>(
    r: &mut impl Retrieve<'a, Input, ParamValue>,
    path: &str,
) -> GeopResult<Option<String>> {
    let mut names = path.split(INSTANCE_SEPARATOR);
    let first = names.next().unwrap_or_default();
    let mut placed = r.retrieve_target(&instance(first))?;
    let mut parameter = parameter_of(placed, "");
    let mut prefix = format!("{first}{INSTANCE_SEPARATOR}");
    for name in names {
        placed = placed.component.view().retrieve_target(&instance(name))?;
        parameter = parameter.and(parameter_of(placed, &prefix));
        prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
    }
    Ok(parameter)
}

/// The part placed in the part read that `entity` is an entity of — the
/// innermost, however deep, by its path: `hinge/pin` — its instance, and
/// the entity as that part names it. None for an entity of the part read
/// itself. Reads no state: what a placed part is does not depend on where
/// it is.
pub fn within<'a>(
    r: &mut impl Retrieve<'a, Input, ParamValue>,
    entity: &EntityRef,
) -> Option<(String, &'a Instance, EntityRef)> {
    let mut found: Option<(String, &'a Instance)> = None;
    let mut rest = entity.clone();
    while let Some((name, inner)) = rest.split_instance() {
        let placed = match &found {
            None => r.retrieve_target(&instance(&name)),
            Some((_, outer)) => outer.component.view().retrieve_target(&instance(&name)),
        };
        let Ok(placed) = placed else {
            break;
        };
        let path = match &found {
            None => name,
            Some((outer, _)) => format!("{outer}{INSTANCE_SEPARATOR}{name}"),
        };
        found = Some((path, placed));
        rest = inner;
    }
    found.map(|(path, placed)| (path, placed, rest))
}

/// The part an entity is of, where it is, and the entity as that part
/// names it (see [`owner`]).
pub type Owner<'a> = (View<'a, Input, ParamValue>, Option<Pose<Design>>, EntityRef);

/// The part `entity` is an entity of — the part `view` shows, or one
/// placed in it, however deep, as its own program left it — where it is in
/// the part `view` shows (`None` for that part itself), and the entity as
/// that part names it.
pub fn owner<'a>(view: View<'a, Input, ParamValue>, entity: &EntityRef) -> GeopResult<Owner<'a>> {
    match within(&mut view.clone(), entity) {
        None => Ok((view, None, entity.clone())),
        Some((path, placed, local)) => {
            let located = locate(&mut view.clone(), &path)?;
            Ok((placed.component.view(), Some(located.world), local))
        }
    }
}

/// A part placed in the part a view shows, however deep.
pub struct Placed<'a> {
    pub name: String,
    pub located: Located<'a>,
    /// The part it is placed in, by index among them all, if not the
    /// part shown itself.
    pub parent: Option<usize>,
}

/// Every part placed in the part `view` shows, and in those, however deep:
/// each before those placed in it, in the order the steps placed them.
pub fn placed_parts(view: View<'_, Input, ParamValue>) -> GeopResult<Vec<Placed<'_>>> {
    fn within<'a>(
        outer: View<'a, Input, ParamValue>,
        level: View<'a, Input, ParamValue>,
        prefix: &str,
        parent: Option<usize>,
        out: &mut Vec<Placed<'a>>,
    ) -> GeopResult<()> {
        for (target, inner) in level.targets_of::<Instance>() {
            let Some(name) = instance_name(target) else {
                continue;
            };
            let path = format!("{prefix}{name}");
            let located = locate(&mut outer.clone(), &path)?;
            out.push(Placed {
                name: path.clone(),
                located,
                parent,
            });
            let index = Some(out.len() - 1);
            let prefix = format!("{path}{INSTANCE_SEPARATOR}");
            within(outer, inner.component.view(), &prefix, index, out)?;
        }
        Ok(())
    }
    let mut out = Vec::new();
    within(view, view, "", None, &mut out)?;
    Ok(out)
}

/// Every solid of the part `view` shows, meshed `quality` fine: its own,
/// and those of every part placed in it, however deep — each where it is,
/// named behind its name in the part shown: `pin/extrude(pin)`. A part
/// placed many times is meshed once.
pub fn solid_meshes(
    view: View<'_, Input, ParamValue>,
    quality: usize,
) -> GeopResult<SolidMeshes<Design>> {
    // A part with no model has no solids.
    let own = |view: View<'_, Input, ParamValue>| match view.retrieve_target(&model()) {
        Ok(own) => own.solid_meshes(quality),
        Err(_) => Ok(Vec::new()),
    };
    let mut out = own(view)?;
    let mut meshed: HashMap<*const Component, SolidMeshes<Design>> = HashMap::new();
    for placed in placed_parts(view)? {
        let component = &placed.located.instance.component;
        let meshes = match meshed.entry(Rc::as_ptr(component)) {
            std::collections::hash_map::Entry::Occupied(meshes) => meshes.into_mut(),
            std::collections::hash_map::Entry::Vacant(slot) => slot.insert(own(component.view())?),
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
            out.push((format!("{}{INSTANCE_SEPARATOR}{name}", placed.name), moved));
        }
    }
    Ok(out)
}
