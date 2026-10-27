//! Placing a part in a part ([`define_placed`]): the part another program
//! file builds, placed as a node of the part placing it (see
//! [`geop_core_program::placed`]) and drawn as a part is drawn ([`draw`]) —
//! and every placed part drawn where the state has it
//! ([`draw_instances`]). What a placed part is to the steps and outputs
//! that read it is [`geop_ops_model::placed`]'s.
//!
//! A placed part is built from its program, its inputs and the state under
//! its name, and drawn as it is built: moving it builds and draws nothing
//! again, and a change in it builds and draws again only what reads it.
//! Equal builds — every screw of a size — are one.

use geop_core_program::{
    element::ElementReference,
    part::StepContext,
    placed::{PlacedArgs, place},
    reader::Reader,
    values::Settings,
};
use std::{
    hash::{Hash, Hasher},
    rc::Rc,
};

use geop_core_editor::view::{PartView, annotations_of};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_rasterize::Rasterizer;
use geop_lib_constraints::{solve::Mobility, unknown::Unknowns};
use geop_ops_model::{
    datums::read_datums,
    parameters::{Instance, Placement, define_instance, read_properties},
    placed::{Drawn, drawn, drawn_target, placed_parts},
    read_model,
    sketch::PlacedSketch,
};

/// The part `reader` reads, drawn: its model, its datums — the origin
/// first — its sketches, what its steps draw on it and the parts placed in
/// it, in its colour.
pub(crate) fn draw(reader: &Reader) -> GeopResult<PartView> {
    let mut view = PartView::new();
    read_model(reader)?.draw("model", &mut Rasterizer::default(), &mut view)?;
    read_datums(reader)?.draw(&mut view)?;
    for target in reader.targets_of::<PlacedSketch>() {
        let name = target.name().to_string();
        let reference = ElementReference::<_, PlacedSketch>::whole(target);
        reader
            .retrieve_element(&reference)?
            .draw(&name, &mut view)?;
    }
    view.annotations = annotations_of(reader);
    draw_instances(reader, &mut view)?;
    view.color = read_properties(reader)?.color().map(str::to_string);
    Ok(view.finish())
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
    // Where it is before it is built: a part reading the parts beside it in
    // context reads where it is itself.
    let instance = Instance {
        placement,
        file: file.to_string(),
        inputs: inputs.clone(),
    };
    define_instance(ctx, name, &instance)?;
    let loaded = place(ctx, name, &placed);
    let drawn = ctx.produce(&drawn_target(name), &name.to_string(), |reader, name| {
        // Drawn once for every instance of one build: every screw of a size.
        let inside = reader.placed_part(name)?;
        let drawn = inside.shared("drawn", |inside| {
            let mut view = draw(inside)?;
            view.sketches.clear();
            let shown = serde_json::to_vec(&view)
                .map_err(|e| GeopError::new(format!("the drawing of `{name}`: {e}")))?;
            let mut key = std::hash::DefaultHasher::new();
            shown.hash(&mut key);
            Ok(Drawn {
                key: format!("{:016x}", key.finish()),
                view: Rc::new(view),
            })
        })?;
        Ok((*drawn).clone())
    });
    // A part that failed to load is drawn as far as it was built, or not at
    // all: why it failed is what to report, not that it is not drawn.
    loaded.and(drawn)
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
            color: None,
        });
    }
    Ok(())
}
