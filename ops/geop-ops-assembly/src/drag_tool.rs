//! The drag tool ([`DragTool`]): in hand while no step is edited, it drags
//! any placed part as far as the program's mates let it.

use geop_core_editor::{
    editing::{Dialog, Layout, Presentation, Pull, Shape, StepEditEvent, Style, Visual},
    editor::{Changed, EditorContext},
    tools::{Tool, ToolInfo},
};
use geop_core_math::{geop_error::GeopResult, vector::Vector3};
use geop_core_program::program::Program;

use crate::{
    icons,
    solving::{self, Report},
};

/// The drag tool, in hand: the placed part the pointer is over, and the one
/// being dragged.
#[derive(Clone, Default)]
pub struct DragTool {
    hover: Option<String>,
    grab: Option<Grabbed>,
    /// How the solve of its last drag went.
    dragged: Option<Report>,
}

/// A placed part being dragged by the drag tool.
#[derive(Clone)]
struct Grabbed {
    /// Its name, as drawn.
    name: String,
    /// The state its pose is.
    parameter: String,
    /// The point grabbed, in its own frame: where the pointer was, a free
    /// choice, so sharp.
    local: Vector3,
    /// The plane it is dragged in: through the point grabbed, facing the
    /// eye — `(point, normal)`.
    plane: (Vector3, Vector3),
    /// The program when it was grabbed: what undoing the drag goes back to.
    before: Program,
}

impl DragTool {
    /// How an editor offers it, and its key.
    pub const INFO: ToolInfo = ToolInfo {
        key: "drag",
        label: "Drag",
        doc: "Drag placed parts as far as their mates let them",
        icon: Some(icons::DRAG),
    };

    /// How the solve of its last drag went: whether the constraints hold,
    /// and how hard it was to get there.
    pub fn dragged(&self) -> Option<&Report> {
        self.dragged.as_ref()
    }
}

impl Tool for DragTool {
    /// An event in the viewport: a hover finds the placed part a press
    /// would grab, a drag pulls the point grabbed towards the pointer — in
    /// the plane through it facing the eye — and solves the program's mates
    /// with that pull, every part that is not fixed free to give way. A
    /// drag, once released, is one edit to undo.
    fn handle(&mut self, ctx: &mut EditorContext, event: &StepEditEvent) -> GeopResult<Changed> {
        let part_at = |ctx: &EditorContext, pointer| {
            let view = ctx.drawn()?;
            let (instance, t) = view.part_to_drag(pointer)?;
            let parameter = instance.state.clone()?;
            Some((instance.name.clone(), parameter, instance.pose, t))
        };
        Ok(match event {
            StepEditEvent::Hover { pointer, .. } => {
                self.hover = part_at(ctx, pointer).map(|(name, ..)| name);
                Changed::Nothing
            }
            StepEditEvent::Leave => {
                self.hover = None;
                Changed::Nothing
            }
            StepEditEvent::Drag { from, to, done, .. } => {
                if self.grab.is_none()
                    && let Some((name, parameter, pose, t)) = part_at(ctx, from)
                {
                    let at = from.ray.at(t);
                    self.grab = Some(Grabbed {
                        name,
                        parameter,
                        local: pose.inverse().apply(&at).sharpen(),
                        plane: (at, *from.ray.dir()),
                        before: ctx.program().clone(),
                    });
                }
                let Some(grab) = &self.grab else {
                    return Ok(Changed::Nothing);
                };
                let (point, normal) = &grab.plane;
                if let Some((_, target)) = to.ray.intersect_plane(point, normal) {
                    ctx.run_all();
                    let pull = Pull::Point {
                        parameter: grab.parameter.clone(),
                        local: grab.local,
                        target: target.sharpen(),
                    };
                    let mut rerun = ctx.rerun();
                    let state = rerun.program.state.clone();
                    if let Ok((moved, report)) = solving::solve(&mut rerun, &state, &[pull], None) {
                        ctx.move_state(moved);
                        self.dragged = Some(report);
                    }
                }
                if *done && let Some(grab) = self.grab.take() {
                    ctx.undoable(grab.before);
                }
                Changed::Run
            }
            _ => Changed::Nothing,
        })
    }

    /// What it shows: the part it would drag, or drags, lit — and that a
    /// press grabs it.
    fn presentation(&self) -> Presentation {
        let lit = (self.grab.as_ref().map(|g| &g.name)).or(self.hover.as_ref());
        Presentation {
            dialog: Dialog::new(),
            layout: Layout::default(),
            visuals: (lit.map(|name| {
                let shape = Shape::Instance { name: name.clone() };
                Visual::new("drag", shape, Style::Hover)
            }))
            .into_iter()
            .collect(),
            highlights: Vec::new(),
            pickable: Vec::new(),
            focus: None,
            sheet: None,
            grab: lit.is_some(),
            prompt: None,
            gizmo: None,
            hidden: Vec::new(),
        }
    }
}
