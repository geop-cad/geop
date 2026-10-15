//! Parts placed in a part: the part another program file builds, placed
//! as a node of the part placing it (see [`geop_core_program::placed`]),
//! and drawn as a part is drawn ([`draw`]).
//!
//! Where an instance is, is the program's state: the pose state of its
//! [`Placement`], or a fixed motion after another instance, as a pattern's
//! copy goes. The parts placed in a placed part are the placing program's
//! to move too: their poses are its state, behind the instance's name
//! (`bolt/nut.pose`), and the placed part is built with them. A placed
//! part is built from its program, its inputs and the state under its
//! name, and drawn as it is built ([`define_placed`]): moving it builds and
//! draws nothing again, and a change in it builds and draws again only
//! what reads it. Equal builds — every screw of a size — are one.

use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::StepContext,
    placed::{INSTANCE_SEPARATOR, PlacedArgs, instance_path, place, split_path},
    reader::Reader,
    target::{Target, TargetReference},
    values::{Design, Settings},
};
use std::{
    hash::{Hash, Hasher},
    rc::Rc,
};

use geop_core_editor::view::{PartView, annotations_of};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, TriangleFace},
};
use geop_ops_constraints::{solve::Mobility, unknown::Unknowns};
use geop_ops_datums::read_datums;
use geop_ops_model::{SolidMeshes, read_model};
use geop_ops_parameters::{
    Factor, Instance, Placement, compose, define_instance, instance, instances, placement,
    read_properties,
};
use geop_ops_sketch::PlacedSketch;

/// The part `reader` reads, drawn: its model, its datums — the origin
/// first — its sketches, what its steps draw on it and the parts placed in
/// it, in its colour.
pub(crate) fn draw(reader: &Reader) -> GeopResult<PartView> {
    let mut view = PartView::new();
    read_model(reader)?.draw("model", &mut view)?;
    read_datums(reader)?.draw(&mut view)?;
    for target in reader.targets_of::<PlacedSketch<Design>>() {
        let name = target.name().to_string();
        let reference = ElementReference::<_, PlacedSketch<Design>>::whole(target);
        reader
            .retrieve_element(&reference)?
            .draw(&name, &mut view)?;
    }
    view.annotations = annotations_of(reader);
    draw_instances(reader, &mut view)?;
    view.color = read_properties(reader)?.color().map(str::to_string);
    Ok(view.finish())
}

/// A placed part as drawn where it is placed: its own part's view, without
/// its sketches, which are what it was drawn with. Its key is what the view
/// shows, hashed: a viewer keeps a view by it, so that it is sent once
/// however often the part placing it is drawn, and parts drawn alike —
/// every screw of a size — share it.
#[derive(Clone, Debug)]
pub struct Drawn {
    pub key: String,
    pub view: Rc<PartView>,
}

/// The drawing of the same part.
impl PartialEq for Drawn {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key
    }
}

/// Each placed part's own.
impl Target for Drawn {}

/// Read whole.
impl Kind for Drawn {
    const NAME: &'static str = "drawn";
}

impl HasElements<Drawn> for Drawn {
    type Element<'a> = &'a Drawn;

    fn element(&self, name: &str) -> GeopResult<&Drawn> {
        whole(self, name)
    }
}

/// The drawing of the instance `name`.
fn drawn_target(name: &str) -> TargetReference<Drawn> {
    TargetReference::new(format!("drawn:{name}"))
}

/// Places the part the program in `file` builds as the instance `name`,
/// its parameters set to `inputs`, at `placement` (see
/// [`geop_core_program::placed::place`]), and draws it ([`draw`]): again
/// only when what it is drawn from changed. A step of the part placed that
/// fails fails this, after it is placed and drawn as far as it builds.
pub fn define_placed(
    ctx: &mut StepContext,
    name: &str,
    file: &str,
    inputs: &Settings,
    placement: Placement,
) -> GeopResult<()> {
    let placed = PlacedArgs {
        file: file.to_string(),
        inputs: inputs.clone(),
    };
    let loaded = place(ctx, name, &placed);
    let instance = Instance {
        placement,
        file: file.to_string(),
        inputs: inputs.clone(),
    };
    define_instance(ctx, name, &instance)?;
    let drawn = ctx.produce(&drawn_target(name), &name.to_string(), |reader, name| {
        let inside = reader.placed_part(name)?;
        let mut view = draw(&inside)?;
        view.sketches.clear();
        let shown = serde_json::to_vec(&view)
            .map_err(|e| GeopError::new(format!("the drawing of `{name}`: {e}")))?;
        let mut key = std::hash::DefaultHasher::new();
        shown.hash(&mut key);
        Ok(Drawn {
            key: format!("{:016x}", key.finish()),
            view: Rc::new(view),
        })
    });
    // A part that failed to load is drawn as far as it was built, or not at
    // all: why it failed is what to report, not that it is not drawn.
    loaded.and(drawn)
}

/// The drawing of the part placed at `path` in the part `reader` reads.
pub fn drawn<'a>(reader: &Reader<'a>, path: &str) -> GeopResult<&'a Drawn> {
    let (outer, name) = split_path(path);
    let level = match outer.is_empty() {
        true => reader.clone(),
        false => reader.placed_part(outer)?,
    };
    (level.retrieve_element(&ElementReference::whole(drawn_target(name))))
        .map_err(|e| e.with_context(format!("the drawing of the placed part {path:?}")))
}

/// A part placed in the part read, however deep: its instance, its part,
/// its pose in the part read, and the state that pose is, if it is the
/// part read's to move.
pub struct Located<'a> {
    pub instance: &'a Instance,
    pub part: Reader<'a>,
    pub world: Pose<Design>,
    /// Where it is in the part it is placed in.
    pub local: Pose<Design>,
    pub state: Option<String>,
}

/// The part placed at `path` — `bolt`, or `asm/bolt` for one placed in a
/// placed part — of the part `reader` reads, where the state has it.
pub fn locate<'a>(reader: &Reader<'a>, path: &str) -> GeopResult<Located<'a>> {
    let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
    let (outer, name) = split_path(path);
    let level = match outer.is_empty() {
        true => reader.clone(),
        false => reader.placed_part(outer).with_context(&ctx)?,
    };
    let instance = instance(&level, name).with_context(&ctx)?;
    let part = level.placed_part(name).with_context(&ctx)?;
    let factors = placement(reader, path).with_context(&ctx)?;
    let outside = match outer.is_empty() {
        true => 0,
        false => placement(reader, outer).with_context(&ctx)?.len(),
    };
    // A copy is moved by what it copies, not by a state of its own.
    let state = match (&instance.placement, factors.last()) {
        (Placement::State(_), Some(Factor::State { state, .. })) => Some(state.clone()),
        _ => None,
    };
    let own = factors.get(outside..).unwrap_or_default();
    Ok(Located {
        instance,
        part,
        world: compose(&factors, &mut |_, pose| pose),
        local: compose(own, &mut |_, pose| pose),
        state,
    })
}

/// The state that moves the part placed at `path` — as [`Located::state`]
/// has it, if the part read moves it at all. Reads no state: which state
/// moves a part does not depend on where it is.
pub fn moved_by(reader: &Reader, path: &str) -> GeopResult<Option<String>> {
    let mut level = reader.clone();
    let mut prefix = String::new();
    let mut state = None;
    for name in path.split(INSTANCE_SEPARATOR) {
        match &instance(&level, name)?.placement {
            Placement::State(own) => state = Some(instance_path(&prefix, own)),
            Placement::After { .. } => return Ok(None),
        }
        prefix = instance_path(&prefix, name);
        level = level.placed_part(name)?;
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
pub fn placed_parts<'a>(reader: &Reader<'a>) -> GeopResult<Vec<Placed<'a>>> {
    fn within<'a>(
        outer: &Reader<'a>,
        level: &Reader<'a>,
        prefix: &str,
        parent: Option<usize>,
        out: &mut Vec<Placed<'a>>,
    ) -> GeopResult<()> {
        for (name, _) in instances(level)? {
            let path = instance_path(prefix, name);
            let located = locate(outer, &path)?;
            let inside = located.part.clone();
            out.push(Placed {
                name: path.clone(),
                located,
                parent,
            });
            let index = Some(out.len() - 1);
            within(outer, &inside, &path, index, out)?;
        }
        Ok(())
    }
    let mut out = Vec::new();
    within(reader, reader, "", None, &mut out)?;
    Ok(out)
}

/// Every solid of the part `reader` reads, meshed `quality` fine: its own,
/// and those of every part placed in it, however deep — each where it is,
/// named behind its name in the part read: `pin/extrude(pin)`.
pub fn solid_meshes(reader: &Reader, quality: usize) -> GeopResult<SolidMeshes<Design>> {
    let own = |reader: &Reader| read_model(reader)?.solid_meshes(quality);
    let mut out = own(reader)?;
    for placed in placed_parts(reader)? {
        let meshes = own(&placed.located.part)?;
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
pub(crate) fn draw_instances(reader: &Reader, view: &mut PartView) -> GeopResult<()> {
    let unknowns = Unknowns::of(reader)?;
    for placed in placed_parts(reader)? {
        let located = placed.located;
        let pose = located.world;
        let origin = geop_core_math::primitives::CoordinateSystem::world_at(
            geop_core_math::vector::Vector3::zero(),
        );
        let drawn = drawn(reader, &placed.name)?;
        view.instances.push(geop_core_editor::view::ViewInstance {
            frame: pose.motion().apply_frame(&origin)?,
            name: placed.name,
            component: drawn.key.clone(),
            pose,
            view: drawn.view.clone(),
            fixed: (located.state.as_ref())
                .and_then(|state| unknowns.get(state))
                .is_none_or(|unknown| unknown.mobility == Mobility::Fixed),
            state: located.state,
        });
    }
    Ok(())
}
