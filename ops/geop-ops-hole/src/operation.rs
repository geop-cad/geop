//! [`Hole`]: drill holes into a planar face at points on it, the way a hole
//! wizard does — simple, counterbored, countersunk or tapped, sized by ISO
//! tables or by hand. [`Thread`]: put a thread on a cylindrical face,
//! recorded or modelled.

use geop_ops::operation::Input;
use geop_ops::parameters::expressions;
use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    editing::{Choice, Event, Form, Number, Shape, Style, Tone, Unit, Value, Visual},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
    part::Retrieve,
};
use geop_core_sketch::CurveKind;
use geop_core_topology::{Body, FaceId, SolidId};
use geop_ops::{
    Design, NamedModel, Namer, ParamValue, model,
    operation::{
        Aspects, Build, BuildReader, EntityRef, Lookup, ModelStep, ResolveEntity, Role, Scope,
        Seen, build_model, frame_along,
    },
    parameters::Formula,
    ui::formula_number,
};
use geop_ops_booleans::{
    Combine, Extent, Tool,
    boolean::{BooleanOp, Reach},
    extent::{hull, reach_past},
};
use serde::{Deserialize, Serialize};

use crate::{
    cosmetic::{CosmeticThread, Threads, draw_threads, threads},
    hole::{Head, HoleDepth, HoleShape, hole_tool},
    iso::{Fit, MetricSize, SIZES, metric},
    thread::{ThreadPlacement, Wall, thread_tool},
};

/// What a hole is for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoleKind {
    /// A plain drilled hole: a screw's clearance hole, or any diameter.
    #[default]
    Simple,
    /// Sunk for a socket head cap screw's head.
    Counterbore,
    /// Sunk 90° for a countersunk screw's head.
    Countersink,
    /// Drilled to the tap drill and threaded: a cosmetic thread recorded on
    /// its wall.
    Tapped,
}

impl HoleKind {
    const ALL: [(HoleKind, &'static str, &'static str); 4] = [
        (HoleKind::Simple, "simple", "Simple"),
        (HoleKind::Counterbore, "counterbore", "Counterbore"),
        (HoleKind::Countersink, "countersink", "Countersink"),
        (HoleKind::Tapped, "tapped", "Tapped"),
    ];

    fn value(self) -> &'static str {
        Self::ALL.iter().find(|k| k.0 == self).expect("listed").1
    }
}

/// How a hole is sized: by an ISO metric screw — `size` `M6`, and for a
/// clearance hole how loose a `fit` — or by hand.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Standard {
    Iso {
        size: String,
        #[serde(default)]
        fit: Fit,
    },
    /// The drill's `diameter`; for a counterbore or countersink the head's
    /// diameter on the face, and for a counterbore its depth.
    Custom {
        diameter: f64,
        #[serde(default)]
        head_diameter: f64,
        #[serde(default)]
        head_depth: f64,
    },
}

/// Drills holes into the planar face `face` at each of `points`, for the
/// operation `H`: every hole a solid of revolution around the face's
/// normal through its point, cut out of the face's solid in one boolean.
/// The result replaces the solid and is named `hole(H)`.
///
/// A point is a sketch's point, a datum point or a vertex, lying on the
/// face; a sketch stands for its points that no line, arc or spline runs
/// through — points placed on their own, and circles' centres.
///
/// What each hole leaves is named after its point `X` — `K,p3` for the
/// point `p3` of the sketch `K`, else the point's name: its wall
/// `hole(H,X,wall,q0)` .. `q3` by quarter turn, a counterbore's
/// `hole(H,X,counterbore,q)` and `hole(H,X,shoulder,q)`, a countersink's
/// `hole(H,X,countersink,q)`, a blind hole's bottom `hole(H,X,bottom,q)` or
/// drill point `hole(H,X,point,q)` (see `crate::hole`); what the boolean
/// creates is named `combine(H,X,...)`. A tapped hole's cosmetic thread is
/// the part's thread `hole(H,X,thread)` (see [`threads`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Hole;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HoleArgs {
    /// The planar face to drill into.
    pub face: String,
    /// Where the holes go.
    pub points: Vec<EntityRef>,
    pub kind: HoleKind,
    pub standard: Standard,
    /// How deep: blind, measured from the face to the end of the drill's
    /// full diameter; up to where it comes out of the solid again; or
    /// through all of it.
    pub end: Extent,
    /// A blind hole ends in a drill's 118° point rather than flat.
    #[serde(default)]
    pub drill_point: bool,
    /// How far a tapped hole is threaded from the face: all the way, if
    /// none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thread_length: Option<f64>,
}

impl HoleArgs {
    /// The shape across the holes.
    pub fn shape(&self) -> GeopResult<HoleShape> {
        match &self.standard {
            Standard::Iso { size, fit } => {
                let size = metric(size)?;
                let clearance = size.clearance(*fit);
                Ok(match self.kind {
                    HoleKind::Simple => HoleShape {
                        diameter: clearance,
                        head: Head::None,
                        thread: None,
                    },
                    HoleKind::Counterbore => HoleShape {
                        diameter: clearance,
                        head: Head::Counterbore {
                            diameter: size.counterbore,
                            depth: size.diameter,
                        },
                        thread: None,
                    },
                    HoleKind::Countersink => HoleShape {
                        diameter: clearance,
                        head: Head::Countersink {
                            diameter: size.countersink.ok_or_else(|| {
                                GeopError::new(format!(
                                    "ISO 15065 gives no countersink for {}: it stops at M20",
                                    size.name
                                ))
                            })?,
                        },
                        thread: None,
                    },
                    HoleKind::Tapped => HoleShape {
                        diameter: size.tap_drill,
                        head: Head::None,
                        thread: Some(size),
                    },
                })
            }
            Standard::Custom {
                diameter,
                head_diameter,
                head_depth,
            } => {
                let head = match self.kind {
                    HoleKind::Simple => Head::None,
                    HoleKind::Counterbore => Head::Counterbore {
                        diameter: *head_diameter,
                        depth: *head_depth,
                    },
                    HoleKind::Countersink => Head::Countersink {
                        diameter: *head_diameter,
                    },
                    HoleKind::Tapped => {
                        return Err(GeopError::new(
                            "a tapped hole is sized by its ISO thread: pick an ISO size",
                        ));
                    }
                };
                Ok(HoleShape {
                    diameter: *diameter,
                    head,
                    thread: None,
                })
            }
        }
    }

    /// The dimensions the holes are made with, as the dialog says them.
    fn describe(&self) -> GeopResult<String> {
        let shape = self.shape()?;
        let mut text = match shape.thread {
            Some(size) => format!(
                "tap drill Ø{}, thread {}",
                shape.diameter,
                size.designation()
            ),
            None => format!("Ø{}", shape.diameter),
        };
        match shape.head {
            Head::None => {}
            Head::Counterbore { diameter, depth } => {
                text += &format!(", counterbore Ø{diameter} × {depth} deep")
            }
            Head::Countersink { diameter } => text += &format!(", countersink Ø{diameter} × 90°"),
        }
        Ok(text)
    }
}

/// The hole centres `points` stand for, as `lookup` has them: each by the
/// name its hole's entities are scoped by, and where it is. A sketch stands
/// for its points no line, arc or spline runs through.
fn centres<S: Scalar>(
    lookup: &mut (impl Lookup<S> + ?Sized),
    points: &[EntityRef],
) -> GeopResult<Vec<(String, Vector3<S>)>> {
    let mut found = Vec::new();
    for point in points {
        let ctx = with_context!("the hole centre {point}");
        match point {
            EntityRef::Sketch { name } => {
                let placed = lookup.sketch(name).with_context(ctx)?;
                let sketch = &placed.sketch;
                let on_curves: BTreeSet<_> = sketch
                    .curves
                    .values()
                    .filter(|c| !matches!(c.kind, CurveKind::Circle { .. }))
                    .flat_map(|c| c.points())
                    .collect();
                let geometry = sketch.enclose::<S>().with_context(ctx)?;
                let before = found.len();
                for (&id, p) in &sketch.points {
                    if !p.fixed && !on_curves.contains(&id) {
                        found.push((
                            format!("{name},{id}"),
                            placed.plane.uv_to_xyz(&geometry.points[&id]),
                        ));
                    }
                }
                if found.len() == before {
                    return Err(GeopError::new(format!(
                        "sketch {name:?} has no points of its own to drill at: place points in it, or pick them one by one"
                    )));
                }
            }
            _ => {
                let at = Aspects::of(point, lookup)
                    .with_context(ctx)?
                    .point
                    .ok_or_else(|| GeopError::new(format!("{point} is not a point")))?;
                let scope = match point {
                    EntityRef::SketchPoint { sketch, point } => format!("{sketch},{point}"),
                    other => other.label(),
                };
                found.push((scope, at));
            }
        }
    }
    let mut seen = BTreeSet::new();
    for (scope, _) in &found {
        if !seen.insert(scope) {
            return Err(GeopError::new(format!("the point {scope} is picked twice")));
        }
    }
    Ok(found)
}

/// The circle of `diameter` around `centre` in the plane normal to
/// `normal`, as a polyline: what a hole looks like on its face.
fn circle<S: Scalar>(centre: Vector3<S>, normal: &Vector3<S>, diameter: f64) -> Option<Shape<S>> {
    let frame = frame_along(centre, normal).ok()?;
    let r = diameter / 2.0;
    let points = (0..=48)
        .map(|k| {
            let a = k as f64 / 48.0 * std::f64::consts::TAU;
            frame
                .origin()
                .add(&frame.u().prod_scalar(S::from_f64(r * a.cos())))
                .add(&frame.v().prod_scalar(S::from_f64(r * a.sin())))
        })
        .collect();
    Some(Shape::Polyline { points })
}

/// The choices of the ISO sizes.
fn size_choices() -> Vec<Choice> {
    SIZES.iter().map(|s| Choice::new(s.name, s.name)).collect()
}

/// Where the holes go: the solid they are drilled into, by name, and each
/// hole's scope and frame — at its centre on the face, `z` the face's
/// normal, out of the solid — as `lookup` has them.
struct Sites<S: Scalar> {
    target: String,
    holes: Vec<(String, CoordinateSystem<S>)>,
}

/// The [`Sites`] of the holes `args` drills.
fn sites<S: Scalar>(
    lookup: &mut (impl Lookup<S> + ?Sized),
    args: &HoleArgs,
) -> GeopResult<Sites<S>> {
    if args.face.is_empty() {
        return Err(GeopError::new("pick the planar face to drill into"));
    }
    let named = lookup.model();
    let face = named.face_id(&args.face)?;
    let model = named.topology();
    let plane = (model.get_face(face)?.surface.as_plane())?.ok_or_else(|| {
        GeopError::new(format!(
            "face {:?} is not planar: holes are drilled into a planar face",
            args.face
        ))
    })?;
    let target = solid_of(named, face, &args.face, "holes are drilled into a solid")?;
    if args.points.is_empty() {
        return Err(GeopError::new("pick the points to drill at"));
    }
    let mut holes = Vec::new();
    for (scope, at) in centres(lookup, &args.points)? {
        if !plane.signed_distance(&at).could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "the point {scope} does not lie on face {:?}: holes are drilled at points on the face",
                args.face
            )));
        }
        holes.push((scope, frame_along(at, &plane.normal)?));
    }
    Ok(Sites { target, holes })
}

/// The name of the solid `face`, named `name`, is a face of — refused, as
/// `why` says, for a face standing on its own.
fn solid_of<S: Scalar>(
    model: &NamedModel<S>,
    face: FaceId,
    name: &str,
    why: &str,
) -> GeopResult<String> {
    let Body::Solid(solid) = model.topology().body_of_face(face)? else {
        return Err(GeopError::new(format!(
            "face {name:?} stands on its own: {why}"
        )));
    };
    Ok(model.name(solid)?.to_string())
}

impl ModelStep for Hole {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &HoleArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("hole({step}, {args:?})");
        let namer = Namer::new("hole", step)?;
        let shape = args.shape().with_context(ctx)?;
        let (sites, blind) = {
            let mut scope = Scope { model, reader };
            let sites = sites(&mut scope, args).with_context(ctx)?;
            (sites, args.end.end(&mut scope).with_context(ctx)?.length())
        };
        let target = sites.target;
        let hull = match blind {
            Some(_) => None,
            None => hull(model, std::slice::from_ref(&target)).with_context(ctx)?,
        };
        let mut tools = Vec::new();
        for (scope, axes) in &sites.holes {
            let depth = match (blind, &hull) {
                (Some(depth), _) => HoleDepth {
                    depth,
                    point: args.drill_point,
                },
                (None, Some(hull)) => HoleDepth {
                    // Into the solid: against the face's normal.
                    depth: reach_past(hull, axes, -1.0).with_context(ctx)?,
                    point: false,
                },
                (None, None) => {
                    return Err(GeopError::new(format!("solid {target:?} has no faces")))
                        .with_context(ctx);
                }
            };
            let named = namer.scoped(scope);
            let solid = hole_tool(model, &named, &named.name(&["tool"]), axes, &shape, depth)
                .with_context(ctx)?;
            tools.push(Tool {
                solid,
                up_to_next: (args.end == Extent::UpToNext)
                    .then(|| (named.name(&["top", "q0"]), named.name(&["bottom", "q0"]))),
                scope: Some(scope.clone()),
            });
        }
        let combine = Combine::With {
            op: BooleanOp::Difference,
            target,
        };
        match combine.apply(model, &namer, step, &tools).with_context(ctx)? {
            Reach::Stopped(()) => Ok(()),
            Reach::RunsOn(e) => Err(e.with_context(
                "up to next: a hole does not come out of the solid all round — part of it runs on; drill it blind or through all",
            ))
            .with_context(ctx),
        }
    }
}

/// What a tapped hole's threads are recorded from: the step, its
/// arguments, where each hole was drilled and how far it is threaded.
type Tapped = (
    String,
    HoleArgs,
    Vec<(String, CoordinateSystem<Design>)>,
    Option<f64>,
);

/// Records the cosmetic thread of every hole a tapped hole drilled, on its
/// wall in the model the hole left.
fn tap(
    mut threads: Threads<Design>,
    reader: &mut BuildReader,
    (step, args, holes, length): &Tapped,
) -> GeopResult<Threads<Design>> {
    let ctx = with_context!("the threads of hole({step})");
    let namer = Namer::new("hole", step)?;
    let shape = args.shape().with_context(ctx)?;
    let Some(size) = shape.thread else {
        return Ok(threads);
    };
    let model = reader.retrieve_target(&model()).with_context(ctx)?;
    let result = model.solid_id(&namer.root()).with_context(ctx)?;
    for hole in holes {
        let thread = tapped_thread(model, result, hole, shape.diameter / 2.0, size, *length)
            .with_context(ctx)?;
        threads.add(namer.scoped(&hole.0).name(&["thread"]), thread)?;
    }
    Ok(threads)
}

impl Operation for Hole {
    const INFO: OperationInfo = OperationInfo::new(
        "hole",
        "Hole",
        "Drill simple, counterbored, countersunk or tapped holes at points on a planar face, sized by ISO tables or by hand.",
        OperationGroup::Features,
        OperationTier::Big,
    );
    type Args = HoleArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut HoleArgs) -> Vec<&mut String> {
        expressions(args.end.length_mut())
    }

    /// No face or point yet; a simple M6 clearance hole, 10 deep.
    fn new_args(_: &Context<Input, ParamValue>) -> HoleArgs {
        HoleArgs {
            face: String::new(),
            points: Vec::new(),
            kind: HoleKind::Simple,
            standard: Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
            end: Extent::blind(10.0),
            drill_point: false,
            thread_length: None,
        }
    }

    /// Drills the holes, then — tapped — records their threads, where the
    /// part before the step put them and on the walls the holes left.
    fn run(step: &str, part: &mut Build, args: &HoleArgs) -> GeopResult<()> {
        let tapped = args.shape().is_ok_and(|shape| shape.thread.is_some());
        let planned = tapped.then(|| {
            let mut seen = Seen::new(part.current());
            let holes = sites(&mut seen, args)?.holes;
            let length = args.thread_length.or(args.end.end(&mut seen)?.length());
            Ok::<_, GeopError>((step.to_string(), args.clone(), holes, length))
        });
        build_model::<Self>(part, step, args)?;
        match planned {
            Some(planned) => {
                part.revise_or_start(&threads(), &planned?, tap)?;
                draw_threads(part, step)
            }
            None => Ok(()),
        }
    }

    /// The face and the points, picked; the kind of hole; its standard,
    /// size and fit — or its diameters by hand — and what they come to; how
    /// deep. Each hole drawn as a circle of its diameter on the face.
    fn presentation(context: &Context<Input, ParamValue>, args: &HoleArgs, _: &()) -> Form<Design> {
        let mut seen = Seen::new(context.before);
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let face = EntityRef::of_names(EntityRef::face, std::slice::from_ref(&args.face));
        (dialog.reference("face", "face", face, &[Role::Plane], None, false)).reference(
            "points",
            "points",
            args.points.clone(),
            &[Role::Point, Role::Sketch],
            None,
            true,
        );
        let kinds = HoleKind::ALL
            .iter()
            .map(|&(_, value, label)| Choice::new(value, label));
        dialog.select("kind", "type", args.kind.value(), kinds.collect(), false);
        let standard = match args.standard {
            Standard::Iso { .. } => "iso",
            Standard::Custom { .. } => "custom",
        };
        let standards = vec![
            Choice::new("iso", "ISO metric"),
            Choice::new("custom", "Custom"),
        ];
        dialog.select("standard", "standard", standard, standards, false);
        match &args.standard {
            Standard::Iso { size, fit } => {
                dialog.select("size", "size", size.clone(), size_choices(), true);
                if args.kind != HoleKind::Tapped {
                    let fits = Fit::ALL.iter().map(|f| Choice::new(f.value(), f.label()));
                    dialog.select("fit", "fit", fit.value(), fits.collect(), false);
                }
            }
            Standard::Custom {
                diameter,
                head_diameter,
                head_depth,
            } => {
                let length = |label: &str, value: f64, max: f64| {
                    Number::new(label, value, Unit::Length).range(0.0, max)
                };
                dialog.number("diameter", length("diameter", *diameter, 50.0));
                if matches!(args.kind, HoleKind::Counterbore | HoleKind::Countersink) {
                    dialog.number(
                        "head_diameter",
                        length("head diameter", *head_diameter, 80.0),
                    );
                }
                if args.kind == HoleKind::Counterbore {
                    dialog.number("head_depth", length("head depth", *head_depth, 50.0));
                }
            }
        }
        match args.describe() {
            Ok(text) => dialog.text("dimensions", text, Tone::Hint),
            Err(e) => dialog.text("dimensions", e.root_message(), Tone::Error),
        };
        let end = Extent::mode(Some(&args.end));
        dialog.select("end", "end", end, Extent::choices(false), false);
        if let Extent::Blind(depth) = &args.end {
            let depth = formula_number("depth", depth, &mut seen, Unit::Length);
            (dialog.number("depth", depth.range(0.0, 100.0))).checkbox(
                "drill_point",
                "drill point (118°)",
                args.drill_point,
            );
        }
        if args.kind == HoleKind::Tapped {
            let full = (args.end.end(&mut seen).ok()).and_then(|end| end.length());
            let length = args.thread_length.or(full);
            let all = args.thread_length.is_none();
            dialog.checkbox("full_thread", "thread all the way", all);
            if let (Some(length), Some(_)) = (length, args.thread_length) {
                let number = Number::new("thread length", length, Unit::Length).range(0.0, 100.0);
                dialog.number("thread_length", number);
            }
        }
        // Each hole, drawn on the face.
        let face = EntityRef::face(args.face.clone());
        if let (Ok(shape), Ok(plane)) = (args.shape(), face.resolve_plane(&mut seen))
            && let Ok(centres) = centres(&mut seen, &args.points)
        {
            let outer = match shape.head {
                Head::None => shape.diameter,
                Head::Counterbore { diameter, .. } | Head::Countersink { diameter } => diameter,
            };
            for (k, (_, at)) in centres.iter().enumerate() {
                for (j, d) in [shape.diameter, outer].into_iter().enumerate() {
                    if j == 1 && d == shape.diameter {
                        continue;
                    }
                    if let Some(c) = circle(*at, plane.w(), d) {
                        let visual = Visual::new(format!("hole{k}.{j}"), c, Style::Draft);
                        form.visuals.push(visual);
                    }
                }
            }
        }
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &HoleArgs,
        _: &(),
        event: Event<Design>,
    ) -> (HoleArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("face", Value::Entities(picked)) => {
                args.face =
                    (EntityRef::names_of(EntityRef::face, &picked).pop()).unwrap_or_default();
            }
            ("points", Value::Entities(picked)) => args.points = picked,
            ("kind", Value::Choice(value)) => {
                if let Some(&(kind, ..)) = HoleKind::ALL.iter().find(|k| k.1 == value) {
                    args.kind = kind;
                }
            }
            ("standard", Value::Choice(value)) => {
                let shape = args.shape().ok();
                match (value.as_str(), &args.standard) {
                    ("custom", Standard::Iso { .. }) => {
                        // Starting from what the ISO size made.
                        let (head_diameter, head_depth) = match shape.map(|s| s.head) {
                            Some(Head::Counterbore { diameter, depth }) => (diameter, depth),
                            Some(Head::Countersink { diameter }) => (diameter, 0.0),
                            _ => (0.0, 0.0),
                        };
                        args.standard = Standard::Custom {
                            diameter: shape.map_or(6.0, |s| s.diameter),
                            head_diameter,
                            head_depth,
                        };
                    }
                    ("iso", Standard::Custom { .. }) => {
                        args.standard = Standard::Iso {
                            size: "M6".into(),
                            fit: Fit::Normal,
                        }
                    }
                    _ => {}
                }
            }
            ("size", Value::Choice(value)) => {
                if let Standard::Iso { size, .. } = &mut args.standard {
                    *size = value;
                }
            }
            ("fit", Value::Choice(value)) => {
                if let (Standard::Iso { fit, .. }, Some(new)) =
                    (&mut args.standard, Fit::from_value(&value))
                {
                    *fit = new;
                }
            }
            ("diameter", Value::Number(v)) => {
                if let Standard::Custom { diameter, .. } = &mut args.standard {
                    *diameter = v;
                }
            }
            ("head_diameter", Value::Number(v)) => {
                if let Standard::Custom { head_diameter, .. } = &mut args.standard {
                    *head_diameter = v;
                }
            }
            ("head_depth", Value::Number(v)) => {
                if let Standard::Custom { head_depth, .. } = &mut args.standard {
                    *head_depth = v;
                }
            }
            ("end", Value::Choice(value)) => {
                if let Some(end) = Extent::pick(Some(&args.end), &value, 10.0) {
                    args.end = end;
                }
            }
            ("depth", value) => {
                if let Some(depth) = Formula::from_value(&value) {
                    args.end = Extent::Blind(depth);
                }
            }
            ("drill_point", Value::Bool(b)) => args.drill_point = b,
            ("full_thread", Value::Bool(b)) => {
                let full = (args.end.end(&mut Seen::new(context.before)).ok())
                    .and_then(|end| end.length());
                args.thread_length = if b { None } else { Some(full.unwrap_or(10.0)) };
            }
            ("thread_length", Value::Number(l)) => args.thread_length = Some(l),
            _ => {}
        }
        (args, ())
    }
}

/// The cosmetic thread of the tapped hole `(X, axes)` drilled at `axes`,
/// of `radius` — on its wall in `solid` of `model` — the thread of `size`
/// for the hole's scope `X`: from the face down, `length` far — a blind
/// hole as deep as it was drilled, unless told otherwise — or, if none, all
/// the way.
fn tapped_thread<S: Scalar>(
    model: &NamedModel<S>,
    solid: SolidId,
    (scope, axes): &(String, CoordinateSystem<S>),
    radius: f64,
    size: &MetricSize,
    length: Option<f64>,
) -> GeopResult<CosmeticThread<S>> {
    let ctx = with_context!("the thread of the hole at {scope}");
    let into = axes.w().neg();
    let radius = S::from_f64(radius);
    let cylinder = geop_core_geometry::shape::Cylinder {
        axis: geop_core_geometry::shape::Axis::try_new(*axes.origin(), into)?,
        radius,
    };
    let wall = Wall::at(model, solid, &cylinder, axes.origin())
        .with_context(ctx)?
        .ok_or_else(|| GeopError::new(format!("the hole at {scope} left no wall to thread")))?;
    // Measured into the solid from the face.
    let entry =
        (axes.origin().sub(&wall.cylinder.axis.point)).prod_dot(&wall.cylinder.axis.direction);
    let placement = wall
        .placement((entry, into), radius, length)
        .with_context(ctx)?;
    placement.check_fits(size, &wall.face_name(model))?;
    placement.cosmetic(size, wall.face_name(model))
}

/// Puts an ISO metric thread on the cylindrical face `face` — a shaft's or
/// a hole's, the whole wall it is part of — for the operation `T`. From one
/// end of the wall, `length` far or all of it: from its open end — where a
/// shaft ends, where a hole was drilled from — or, if both or neither are
/// open, from one of them; from the other if `reversed`.
///
/// Recorded, it is the part's cosmetic thread `thread(T)` (see
/// [`threads`]), which drawings and the viewer read; the solid is left as it is. Modelled, its profile is
/// swept along its helix and cut out of the solid (see `crate::thread`),
/// which is replaced by `thread(T)`; the cut faces are named
/// `thread(T,c0,q)` .. `thread(T,c3,q)` for each quarter turn `q`, and what
/// the boolean creates `combine(T,...)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Thread;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThreadArgs {
    /// A cylindrical face of the wall to thread.
    pub face: String,
    /// The ISO metric size: `M6`.
    pub size: String,
    /// How far it runs: the whole wall, if none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length: Option<f64>,
    /// Start from the other end of the wall.
    #[serde(default)]
    pub reversed: bool,
    /// Cut the thread's profile into the solid, rather than record it.
    #[serde(default)]
    pub modelled: bool,
}

impl ThreadArgs {
    /// Where the thread runs in `model`, on the wall of its face.
    fn placement<S: Scalar>(
        &self,
        model: &NamedModel<S>,
    ) -> GeopResult<(Wall<S>, ThreadPlacement<S>)> {
        if self.face.is_empty() {
            return Err(GeopError::new("pick the cylindrical face to thread"));
        }
        let wall = Wall::of(model, model.face_id(&self.face)?)?;
        let start = wall.start(self.reversed);
        let placement = wall.placement(start, wall.cylinder.radius, self.length)?;
        Ok((wall, placement))
    }
}

impl ModelStep for Thread {
    /// Cuts the thread's profile into the solid: a modelled thread.
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &ThreadArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("thread({step}, {args:?})");
        let namer = Namer::new("thread", step)?;
        let size = metric(&args.size).with_context(ctx)?;
        let (wall, placement) = args.placement(model).with_context(ctx)?;
        placement.check_fits(size, &args.face).with_context(ctx)?;
        let face = *(wall.faces.first())
            .ok_or_else(|| GeopError::new("the wall has no faces"))
            .with_context(ctx)?;
        let why = "a modelled thread is cut into a solid";
        let target = solid_of(model, face, &args.face, why).with_context(ctx)?;
        let tool = thread_tool(
            model,
            &namer,
            &namer.name(&["tool"]),
            size,
            &placement,
            &wall,
        )
        .with_context(ctx)?;
        let combine = Combine::With {
            op: BooleanOp::Difference,
            target,
        };
        (combine.apply(model, &namer, step, &[Tool::whole(tool)]))
            .and_then(Reach::stopped)
            .with_context(ctx)
    }
}

/// Records the cosmetic thread `thread(T)` of the step `T`, on the wall of
/// its face in the model as it is.
fn record(
    mut threads: Threads<Design>,
    reader: &mut BuildReader,
    (step, args): &(String, ThreadArgs),
) -> GeopResult<Threads<Design>> {
    let ctx = with_context!("thread({step}, {args:?})");
    let namer = Namer::new("thread", step)?;
    let size = metric(&args.size).with_context(ctx)?;
    let model = reader.retrieve_target(&model()).with_context(ctx)?;
    let (_, placement) = args.placement(model).with_context(ctx)?;
    placement.check_fits(size, &args.face).with_context(ctx)?;
    let thread = placement.cosmetic(size, args.face.clone())?;
    threads.add(namer.root(), thread).with_context(ctx)?;
    Ok(threads)
}

impl Operation for Thread {
    const INFO: OperationInfo = OperationInfo::new(
        "thread",
        "Thread",
        "Put an ISO metric thread on a cylindrical face: recorded as a cosmetic thread, or modelled.",
        OperationGroup::Features,
        OperationTier::Small,
    );
    type Args = ThreadArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// No face yet; M6, recorded, the whole wall.
    fn new_args(_: &Context<Input, ParamValue>) -> ThreadArgs {
        ThreadArgs {
            face: String::new(),
            size: "M6".into(),
            length: None,
            reversed: false,
            modelled: false,
        }
    }

    /// Modelled, cut into the solid; recorded, one more of the part's
    /// threads.
    fn run(step: &str, part: &mut Build, args: &ThreadArgs) -> GeopResult<()> {
        if args.modelled {
            build_model::<Self>(part, step, args)
        } else {
            part.revise_or_start(&threads(), &(step.to_string(), args.clone()), record)?;
            draw_threads(part, step)
        }
    }

    /// The face, picked; the size; how far; which end; whether modelled.
    /// The thread drawn as its helix.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &ThreadArgs,
        _: &(),
    ) -> Form<Design> {
        let seen = Seen::new(context.before);
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let face = EntityRef::of_names(EntityRef::face, std::slice::from_ref(&args.face));
        (dialog.reference("face", "face", face, &[Role::Round], None, false)).select(
            "size",
            "size",
            args.size.clone(),
            size_choices(),
            true,
        );
        let placed = args.placement(seen.model());
        if let Ok(size) = metric(&args.size) {
            let text = format!(
                "{}: Ø{} major, Ø{:.3} minor",
                size.designation(),
                size.diameter,
                size.minor_diameter()
            );
            dialog.text("dimensions", text, Tone::Hint);
        }
        let problem = match &placed {
            Ok((wall, p)) => (metric(&args.size))
                .and_then(|size| p.check_fits(size, &wall.face_name(seen.model())))
                .err()
                .map(|e| e.root_message().to_string()),
            Err(e) => Some(e.root_message().to_string()),
        };
        if let Some(problem) = problem
            && !args.face.is_empty()
        {
            dialog.text("problem", problem, Tone::Error);
        }
        dialog.checkbox("full_length", "whole face", args.length.is_none());
        if let Some(length) = args.length {
            let number = Number::new("length", length, Unit::Length).range(0.0, 100.0);
            dialog.number("length", number);
        }
        (dialog.checkbox("reversed", "start from the other end", args.reversed)).checkbox(
            "modelled",
            "modelled",
            args.modelled,
        );
        if let (Ok((_, placement)), Ok(size)) = (&placed, metric(&args.size))
            && let Ok(thread) = placement.cosmetic(size, args.face.clone())
            && let Ok(points) = thread.polyline()
        {
            let visual = Visual::new("helix", Shape::Polyline { points }, Style::Draft);
            form.visuals.push(visual);
        }
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &ThreadArgs,
        _: &(),
        event: Event<Design>,
    ) -> (ThreadArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("face", Value::Entities(picked)) => {
                args.face =
                    (EntityRef::names_of(EntityRef::face, &picked).pop()).unwrap_or_default();
            }
            ("size", Value::Choice(size)) => args.size = size,
            ("full_length", Value::Bool(b)) => args.length = if b { None } else { Some(10.0) },
            ("length", Value::Number(l)) => args.length = Some(l),
            ("reversed", Value::Bool(b)) => args.reversed = b,
            ("modelled", Value::Bool(b)) => args.modelled = b,
            _ => {}
        }
        (args, ())
    }
}
