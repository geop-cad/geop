//! [`Explode`]: one move of an exploded view — placed parts moved off
//! where they are assembled, along a direction and turned about it, a
//! line trailing each from where it was. A sequence of moves is a sequence
//! of steps: each moves its parts on from where the ones before left them,
//! and seeking the timeline through them takes the assembly apart move by
//! move. What a move is to the scene and to a drawing is
//! [`geop_ops_model::placed::explode`]'s: the state and the mates stay as
//! assembled.
//!
//! In its dialog, a click on a placed part takes it in or out of the move,
//! and dragging one of the parts moved — or the distance's handle — slides
//! them along the direction.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{CanvasEvent, Choice, ListItem, Number, Shape, Style, Tone, Track, Unit, Visual},
    part_operation,
    part_operation::PartOperation,
    view::{Annotations, ViewAnnotation, annotations},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_ops_model::{
    mate_entity::MateEntity,
    placed::{
        drawn,
        explode::{Explosion, exploded, explosion, explosions},
        locate, placed_parts,
    },
};
use serde::{Deserialize, Serialize};

/// Moves placed parts of the part off where they are assembled — see the
/// module docs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Explode;

/// Which way a move goes: along one of the part's axes, or along the `z`
/// axis of the frame something picked gives — a face's normal, an edge's
/// direction, a datum's axis (see [`MateEntity::frame`]), where its part
/// is assembled.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    X,
    Y,
    #[default]
    Z,
    Along(MateEntity),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExplodeArgs {
    /// The placed parts moved, by their paths: `bolt`, or `asm/bolt` for
    /// one placed in a placed part.
    #[serde(default)]
    pub parts: Vec<String>,
    #[serde(default)]
    pub direction: Direction,
    /// Against the direction.
    #[serde(default)]
    pub reversed: bool,
    /// How far along it.
    #[serde(default)]
    pub distance: f64,
    /// How far turned about it, through the middle of the parts moved, in
    /// degrees.
    #[serde(default)]
    pub angle: f64,
    /// Whether a line trails each part from where it was.
    #[serde(default = "trailed")]
    pub trails: bool,
}

fn trailed() -> bool {
    true
}

impl Default for ExplodeArgs {
    fn default() -> Self {
        ExplodeArgs {
            parts: Vec::new(),
            direction: Direction::Z,
            reversed: false,
            distance: 0.0,
            angle: 0.0,
            trails: true,
        }
    }
}

/// The unit direction a move of `args` goes in, in the part `reader` reads.
fn direction(reader: &Reader, args: &ExplodeArgs) -> GeopResult<Vector3> {
    let axis = |k: usize| Vector3::axis(k);
    let along = match &args.direction {
        Direction::X => axis(0),
        Direction::Y => axis(1),
        Direction::Z => axis(2),
        Direction::Along(entity) => *entity.frame_in_world(reader)?.w(),
    };
    Ok(if args.reversed { along.neg() } else { along })
}

/// The middle of the box of the parts `parts` — of their own boxes'
/// middles — where the moves `before` leave them in the part `reader`
/// reads: what a move turns them about. A free choice, so sharp.
fn middle(reader: &Reader, parts: &[String], before: &[&Explosion]) -> GeopResult<Vector3> {
    let mut bounds: Option<[Scalar; 3]> = None;
    for part in parts {
        let at = centre_of(reader, part, before)?;
        let at = at.to_array();
        bounds = Some(match bounds {
            Some(b) => std::array::from_fn(|k| b[k].union(at[k])),
            None => at,
        });
    }
    let bounds = bounds.ok_or_else(|| GeopError::new("no part to move"))?;
    Ok(Vector3::from_array(bounds.map(|b| b.midpoint())))
}

/// The middle of the box of the part placed at `path`, where the moves
/// `before` leave it in the part `reader` reads.
fn centre_of(reader: &Reader, path: &str, before: &[&Explosion]) -> GeopResult<Vector3> {
    let located = locate(reader, path)?;
    let centre = drawn(reader, path)?.view.extent.center;
    Ok(exploded(before, path, &located.world).apply(&centre))
}

/// Checks `args` name parts to move, each placed in the part `reader`
/// reads.
fn check(reader: &Reader, args: &ExplodeArgs) -> GeopResult<()> {
    if args.parts.is_empty() {
        return Err(GeopError::new(
            "click the placed parts to move: none is chosen yet",
        ));
    }
    for part in &args.parts {
        if locate(reader, part).is_err() {
            return Err(GeopError::new(format!(
                "{part:?} is no part placed so far: move one an earlier step placed"
            )));
        }
    }
    Ok(())
}

/// The move `args` makes in the part `reader` reads, after the moves the
/// steps before made: the generator of its [`explosion`].
fn plan(reader: &Reader, args: &ExplodeArgs) -> GeopResult<Explosion> {
    check(reader, args)?;
    let along = direction(reader, args)?;
    let offset = along.prod_scalar(Scalar::from_f64(args.distance));
    let slide = Pose::identity().with_position(offset);
    // Not turned at all, nothing is turned: an exact turn of nothing
    // composed would round every pose a hair outward.
    let motion = match args.angle == 0.0 {
        true => slide,
        false => {
            let before = explosions(reader);
            let middle = middle(reader, &args.parts, &before)?;
            let radians = Scalar::from_f64(args.angle).degrees_to_radians()?;
            slide.compose(&Pose::rotation_about(&middle, &along, radians)?)
        }
    };
    Ok(Explosion {
        parts: args.parts.clone(),
        motion,
    })
}

/// The lines trailing each part the step `step` moves, as `args` asks:
/// from the middle of its box before the move to after it.
fn trails(reader: &Reader, (step, args): &(String, ExplodeArgs)) -> GeopResult<Annotations> {
    if !args.trails {
        return Ok(Annotations::default());
    }
    let moved = reader.retrieve_element(&ElementReference::whole(explosion(step)))?;
    let before: Vec<&Explosion> = (explosions(reader).into_iter())
        .filter(|e| !std::ptr::eq(*e, moved))
        .collect();
    let lines = (moved.parts.iter())
        .map(|part| {
            let from = centre_of(reader, part, &before)?;
            Ok(ViewAnnotation {
                name: format!("explode({step},{part})"),
                label: format!("trail of {part}"),
                polyline: vec![from, moved.motion.apply(&from)],
            })
        })
        .collect::<GeopResult<Vec<_>>>()?;
    Ok(Annotations(lines))
}

/// What the dialog keeps while a move is edited: the part the pointer is
/// over, and the distance when a drag of the parts began.
#[derive(Clone, Debug, Default)]
pub struct ExplodeSession {
    hover: Option<String>,
    grabbed: Option<f64>,
    /// Whether the direction is to be picked, before anything is.
    along: bool,
}

/// The key of the visual of the part moved `part`, to drag.
fn moved_key(part: &str) -> String {
    format!("move:{part}")
}

/// The placed part of the part `context` edits the pointer `pointer` is
/// over, the nearest of them.
fn part_at(context: &Context, pointer: &geop_core_editor::editing::Pointer) -> Option<String> {
    let scene = context.scene?;
    let reader = context.built.as_ref().unwrap_or(&context.before);
    (placed_parts(reader).ok()?.into_iter())
        .filter_map(|placed| Some((scene.hit_instance(&placed.name, pointer)?, placed.name)))
        .min_by(|a, b| a.0.to_f64().total_cmp(&b.0.to_f64()))
        .map(|(_, name)| name)
}

part_operation!(Explode(ExplodeArgs));

impl PartOperation for Explode {
    const KIND: &'static str = "explode";
    const INFO: OperationInfo = OperationInfo::new(
        "Explode",
        "Move placed parts off where they are assembled, along a direction and turned about it — one move of an exploded view, a line trailing each part.",
        OperationGroup::Assembly,
        OperationTier::Small,
    )
    .icon(crate::icons::EXPLODE);
    type Args = ExplodeArgs;
    type Session = ExplodeSession;

    /// Nothing moved yet: straight up, trailed.
    fn new_args(_: &Context) -> ExplodeArgs {
        ExplodeArgs::default()
    }

    /// The move, and the lines trailing its parts.
    fn run(ctx: &mut StepContext, args: &ExplodeArgs) -> GeopResult<()> {
        let step = ctx.step().to_string();
        let context = with_context!("explode({step}, parts={:?})", args.parts);
        ctx.produce(&explosion(&step), args, plan)
            .with_context(context)?;
        let trailed = (step.clone(), args.clone());
        ctx.produce(&annotations(&step), &trailed, trails)
            .with_context(context)
    }

    /// The parts moved, which way, how far and how far turned, and whether
    /// they trail lines; the parts lit, to drag.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ExplodeArgs,
        editing: &FormState<ExplodeSession>,
    ) -> EditForm<'a, ExplodeArgs, ExplodeSession> {
        let mut f = EditForm::<ExplodeArgs, ExplodeSession>::default();
        let before = &context.before;
        f.form.dialog.text(
            "hint",
            "Click the placed parts to move, then drag one of them, or the arrow, along the direction.",
            Tone::Hint,
        );
        let items = (args.parts.iter())
            .map(|part| {
                let key = format!("part:{part}");
                let moved = part.clone();
                f.on(key.clone(), move |Edit { args, .. }, value| {
                    if matches!(value, geop_core_editor::editing::Value::Remove) {
                        args.parts.retain(|p| *p != moved);
                    }
                });
                let mut item = ListItem::new(key, part.clone());
                item.removable = true;
                item
            })
            .collect();
        f.form
            .dialog
            .list("parts", items, "None yet: click a placed part.");

        let way = match &args.direction {
            _ if editing.session.along => "along",
            Direction::X => "x",
            Direction::Y => "y",
            Direction::Z => "z",
            Direction::Along(_) => "along",
        };
        let choices = vec![
            Choice::new("z", "up (z)"),
            Choice::new("x", "along x"),
            Choice::new("y", "along y"),
            Choice::new("along", "along what is picked"),
        ];
        f.select(
            "direction",
            "direction",
            way,
            choices,
            false,
            |Edit { args, session, .. }, way| {
                session.along = way == "along";
                args.direction = match way {
                    "x" => Direction::X,
                    "y" => Direction::Y,
                    "z" => Direction::Z,
                    // Kept as it is until something is picked.
                    _ => args.direction.clone(),
                };
            },
        );
        if way == "along" {
            let along = match &args.direction {
                Direction::Along(entity) => Some(entity),
                _ => None,
            };
            f.pick_one("along", "along", before, along, |e, entity| {
                e.args.direction = Direction::Along(entity);
                e.session.along = false;
            });
        }
        f.checkbox(
            "reversed",
            "the other way",
            args.reversed,
            |Edit { args, .. }, reversed| args.reversed = reversed,
        );
        // The handle, at the parts' middle where this move leaves it,
        // sliding along the direction.
        let shown = context.built.as_ref().unwrap_or(before);
        let handle = (|| -> GeopResult<Track> {
            let along = direction(shown, args)?;
            let all = explosions(shown);
            let at = middle(shown, &args.parts, &all)?;
            Ok(Track {
                at,
                direction: along,
            })
        })()
        .ok();
        f.number(
            "distance",
            Number::new("distance", args.distance, Unit::Length).handle(handle),
            |Edit { args, .. }, distance| args.distance = distance,
        );
        f.number(
            "angle",
            Number::new("turned", args.angle, Unit::Angle),
            |Edit { args, .. }, angle| args.angle = angle,
        );
        f.checkbox(
            "trails",
            "trail lines",
            args.trails,
            |Edit { args, .. }, trails| args.trails = trails,
        );
        for part in &args.parts {
            let shape = Shape::Instance { name: part.clone() };
            f.form
                .visuals
                .push(Visual::new(moved_key(part), shape, Style::Selected).draggable());
        }
        if let Some(hover) = &editing.session.hover
            && !args.parts.contains(hover)
        {
            let shape = Shape::Instance {
                name: hover.clone(),
            };
            f.form
                .visuals
                .push(Visual::new("hover", shape, Style::Hover));
        }
        f
    }

    /// A hover lights the part a click takes in or out of the move; a drag
    /// of a part moved slides the parts along the direction, from the
    /// distance it began at.
    fn canvas(context: &Context, edit: Edit<'_, ExplodeArgs, ExplodeSession>, event: &CanvasEvent) {
        let Edit { args, session, .. } = edit;
        match event {
            CanvasEvent::Hover { pointer, .. } => session.hover = part_at(context, pointer),
            CanvasEvent::Leave | CanvasEvent::Cancel => session.hover = None,
            CanvasEvent::Click { pointer, .. } => {
                if let Some(part) = part_at(context, pointer) {
                    match args.parts.iter().position(|p| *p == part) {
                        Some(k) => {
                            args.parts.remove(k);
                        }
                        None => args.parts.push(part),
                    }
                }
            }
            CanvasEvent::Move {
                key,
                from,
                to,
                done,
                ..
            } if key.starts_with("move:") => {
                let shown = context.built.as_ref().unwrap_or(&context.before);
                let Ok(along) = direction(shown, args) else {
                    return;
                };
                let start = *session.grabbed.get_or_insert(args.distance);
                // Where the pointer is: a free choice, made by whoever moved
                // it.
                let slid = to.sub(from).prod_dot(&along).to_f64();
                args.distance = start + slid;
                if *done {
                    session.grabbed = None;
                }
            }
            _ => {}
        }
    }
}
