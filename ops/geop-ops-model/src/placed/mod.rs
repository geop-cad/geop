//! Parts placed in a part: the part another program file builds, placed
//! as a node of the part placing it (see [`geop_core_program::placed`]) by
//! the step that places it, and drawn as a part is drawn ([`Drawn`]) —
//! read here, wherever they are placed ([`placed_parts`], [`locate`]), with
//! their solids ([`solids`]) and the bill of materials they make ([`bom`]).
//!
//! Where an instance is, is the program's state: the pose state of its
//! [`Placement`], or a fixed motion after another instance, as a pattern's
//! copy goes. The parts placed in a placed part are the placing program's
//! to move too: their poses are its state, behind the instance's name
//! (`/bolt:nut.pose`), and the placed part is built with them. A placed
//! part is built from its program, its inputs and the state under its
//! name, and drawn as it is built: moving it builds and draws nothing
//! again, and a change in it builds and draws again only what reads it.
//! Equal builds — every screw of a size — are one.

pub mod bom;
pub mod solids;

use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    placed::{INSTANCE_SEPARATOR, Located as LocatedName, instance_path, split_instance_path},
    reader::Reader,
    target::{Target, TargetReference},
};
use std::rc::Rc;

use geop_core_editor::view::PartView;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, TriangleFace},
};

use crate::{
    SolidMeshes,
    parameters::{Factor, Instance, Placement, compose, instance, instances, placement},
    read_model,
};

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

/// The drawing of the instance `name`: what the step placing it produces.
pub fn drawn_target(name: &str) -> TargetReference<Drawn> {
    TargetReference::new(format!("drawn:{name}"))
}

/// The drawing of the part placed at `path` in the part `reader` reads.
pub fn drawn<'a>(reader: &Reader<'a>, path: &str) -> GeopResult<&'a Drawn> {
    let (outer, name) = split_instance_path(path);
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
    pub world: Pose,
    /// Where it is in the part it is placed in.
    pub local: Pose,
    pub state: Option<String>,
}

/// The part placed at `path` — `bolt`, or `asm/bolt` for one placed in a
/// placed part — of the part `reader` reads, where the state has it.
pub fn locate<'a>(reader: &Reader<'a>, path: &str) -> GeopResult<Located<'a>> {
    let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
    let (outer, name) = split_instance_path(path);
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
            Placement::State(own) => state = Some(LocatedName::in_instance(&prefix, own).key()),
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
/// named behind its name in the part read: `/pin:extrude(pin)`.
pub fn solid_meshes(reader: &Reader, quality: usize) -> GeopResult<SolidMeshes> {
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
            out.push((LocatedName::in_instance(&placed.name, name).key(), moved));
        }
    }
    Ok(out)
}
