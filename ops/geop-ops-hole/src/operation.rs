//! [`Hole`]: drill holes into a planar face at points on it, the way a hole
//! wizard does — simple, counterbored, countersunk or tapped, sized by ISO
//! tables or by hand. [`Thread`]: put a thread on a cylindrical face,
//! recorded or modelled.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::CurveKind;
use geop_core_topology::Body;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Aspects, EntityRef, Operation, Role, frame_along},
    ui::{Choice, Form, Number, Shape, Style, Tone, Unit, Visual},
};
use geop_ops_booleans::{Combine, Tool, boolean::NOTHING_STOPS};
use geop_ops_extrude_revolve::{
    Extent,
    operation::{hull, reach_past},
};
use serde::{Deserialize, Serialize};

use crate::{
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
/// drill point `hole(H,X,point,q)` (see [`crate::hole`]); what the boolean
/// creates is named `combine(H,X,...)`. A tapped hole's cosmetic thread is
/// the part's thread `hole(H,X,thread)`.
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

/// The hole centres `points` stand for in `part`: each by the name its
/// hole's entities are scoped by, and where it is. A sketch stands for its
/// points no line, arc or spline runs through.
fn centres<S: Scalar>(
    part: &Part<S>,
    points: &[EntityRef],
) -> GeopResult<Vec<(String, Vector3<S>)>> {
    let mut found = Vec::new();
    for point in points {
        let ctx = with_context!("the hole centre {point}");
        match point {
            EntityRef::Sketch { name } => {
                let placed = part.sketch(part.sketch_id(name).with_context(ctx)?)?;
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
                let at = Aspects::of(point, part)
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

/// The faces of a reference field, by name: none, if it holds no face.
fn face_name(picked: &[EntityRef]) -> String {
    match picked {
        [EntityRef::Face { name }] => name.clone(),
        _ => String::new(),
    }
}

/// A face by name, as a reference field holds it.
fn face_ref(name: &str) -> Vec<EntityRef> {
    if name.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Face { name: name.into() }]
    }
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

impl Operation for Hole {
    type Args = HoleArgs;
    type Session = ();

    /// No face or point yet; a simple M6 clearance hole, 10 deep.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> HoleArgs {
        HoleArgs {
            face: String::new(),
            points: Vec::new(),
            kind: HoleKind::Simple,
            standard: Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
            end: Extent::Blind(10.0),
            drill_point: false,
            thread_length: None,
        }
    }

    /// The face and the points, picked; the kind of hole; its standard,
    /// size and fit — or its diameters by hand — and what they come to; how
    /// deep. Each hole drawn as a circle of its diameter on the face.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &HoleArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, HoleArgs> {
        let before = context.before;
        let mut f = Form::<S, HoleArgs>::new();
        f.reference(
            "face",
            "face",
            face_ref(&args.face),
            &[Role::Plane],
            None,
            false,
            |e, picked| e.args.face = face_name(&picked),
        );
        f.reference(
            "points",
            "points",
            args.points.clone(),
            &[Role::Point, Role::Sketch],
            None,
            true,
            |e, picked| e.args.points = picked,
        );
        f.select(
            "kind",
            "type",
            args.kind.value(),
            HoleKind::ALL
                .iter()
                .map(|&(_, value, label)| Choice::new(value, label))
                .collect(),
            false,
            |args, value| {
                if let Some(&(kind, ..)) = HoleKind::ALL.iter().find(|k| k.1 == value) {
                    args.kind = kind;
                }
            },
        );
        let standard = match args.standard {
            Standard::Iso { .. } => "iso",
            Standard::Custom { .. } => "custom",
        };
        f.select(
            "standard",
            "standard",
            standard,
            vec![
                Choice::new("iso", "ISO metric"),
                Choice::new("custom", "Custom"),
            ],
            false,
            |args, value| {
                let shape = args.shape().ok();
                match (value, &args.standard) {
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
            },
        );
        match &args.standard {
            Standard::Iso { size, fit } => {
                f.select(
                    "size",
                    "size",
                    size.clone(),
                    size_choices(),
                    true,
                    |args, value| {
                        if let Standard::Iso { size, .. } = &mut args.standard {
                            *size = value.to_string();
                        }
                    },
                );
                if args.kind != HoleKind::Tapped {
                    f.select(
                        "fit",
                        "fit",
                        fit.value(),
                        Fit::ALL
                            .iter()
                            .map(|f| Choice::new(f.value(), f.label()))
                            .collect(),
                        false,
                        |args, value| {
                            if let (Standard::Iso { fit, .. }, Some(new)) =
                                (&mut args.standard, Fit::from_value(value))
                            {
                                *fit = new;
                            }
                        },
                    );
                }
            }
            Standard::Custom {
                diameter,
                head_diameter,
                head_depth,
            } => {
                f.number(
                    "diameter",
                    Number::new("diameter", *diameter, Unit::Length).range(0.0, 50.0),
                    |args, v| {
                        if let Standard::Custom { diameter, .. } = &mut args.standard {
                            *diameter = v;
                        }
                    },
                );
                if matches!(args.kind, HoleKind::Counterbore | HoleKind::Countersink) {
                    f.number(
                        "head_diameter",
                        Number::new("head diameter", *head_diameter, Unit::Length).range(0.0, 80.0),
                        |args, v| {
                            if let Standard::Custom { head_diameter, .. } = &mut args.standard {
                                *head_diameter = v;
                            }
                        },
                    );
                }
                if args.kind == HoleKind::Counterbore {
                    f.number(
                        "head_depth",
                        Number::new("head depth", *head_depth, Unit::Length).range(0.0, 50.0),
                        |args, v| {
                            if let Standard::Custom { head_depth, .. } = &mut args.standard {
                                *head_depth = v;
                            }
                        },
                    );
                }
            }
        }
        match args.describe() {
            Ok(text) => f.text("dimensions", text, Tone::Hint),
            Err(e) => f.text("dimensions", e.root_message(), Tone::Error),
        };
        let mode = match args.end {
            Extent::Blind(_) => "blind",
            Extent::UpToNext => "up_to_next",
            Extent::ThroughAll => "through_all",
        };
        f.select(
            "end",
            "end",
            mode,
            vec![
                Choice::new("blind", "Blind"),
                Choice::new("up_to_next", "Up to next"),
                Choice::new("through_all", "Through all"),
            ],
            false,
            |args, value| {
                args.end = match (value, args.end) {
                    ("blind", Extent::Blind(d)) => Extent::Blind(d),
                    ("blind", _) => Extent::Blind(10.0),
                    ("up_to_next", _) => Extent::UpToNext,
                    ("through_all", _) => Extent::ThroughAll,
                    (_, end) => end,
                }
            },
        );
        if let Extent::Blind(depth) = args.end {
            f.number(
                "depth",
                Number::new("depth", depth, Unit::Length).range(0.0, 100.0),
                |args, d| args.end = Extent::Blind(d),
            );
            f.checkbox(
                "drill_point",
                "drill point (118°)",
                args.drill_point,
                |args, b| args.drill_point = b,
            );
        }
        if args.kind == HoleKind::Tapped {
            let full = match args.end {
                Extent::Blind(depth) => Some(depth),
                _ => None,
            };
            let length = args.thread_length.or(full);
            f.checkbox(
                "full_thread",
                "thread all the way",
                args.thread_length.is_none(),
                move |args, b| {
                    args.thread_length = if b { None } else { Some(full.unwrap_or(10.0)) }
                },
            );
            if let (Some(length), Some(_)) = (length, args.thread_length) {
                f.number(
                    "thread_length",
                    Number::new("thread length", length, Unit::Length).range(0.0, 100.0),
                    |args, l| args.thread_length = Some(l),
                );
            }
        }
        // Each hole, drawn on the face.
        if let (Ok(shape), Ok(plane)) = (
            args.shape(),
            EntityRef::Face {
                name: args.face.clone(),
            }
            .resolve_plane(before),
        ) && let Ok(centres) = centres(before, &args.points)
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
                        f.visuals
                            .push(Visual::new(format!("hole{k}.{j}"), c, Style::Draft));
                    }
                }
            }
        }
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &HoleArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("hole({operation_id}, {args:?})");
        let namer = Namer::new("hole", operation_id)?;
        let shape = args.shape().with_context(ctx)?;
        if args.face.is_empty() {
            return Err(GeopError::new("pick the planar face to drill into")).with_context(ctx);
        }
        let face = part.face_id(&args.face).with_context(ctx)?;
        let model = part.topology();
        let plane = model
            .get_face(face)?
            .surface
            .as_plane()
            .with_context(ctx)?
            .ok_or_else(|| {
                GeopError::new(format!(
                    "face {:?} is not planar: holes are drilled into a planar face",
                    args.face
                ))
            })
            .with_context(ctx)?;
        let Body::Solid(solid) = model.body_of_face(face)? else {
            return Err(GeopError::new(format!(
                "face {:?} stands on its own: holes are drilled into a solid",
                args.face
            )))
            .with_context(ctx);
        };
        let target = part
            .name_of(solid)
            .ok_or_else(|| GeopError::new(format!("{solid} has no name")))?
            .to_string();
        if args.points.is_empty() {
            return Err(GeopError::new("pick the points to drill at")).with_context(ctx);
        }
        let centres = centres(&part, &args.points).with_context(ctx)?;
        for (scope, at) in &centres {
            if !plane.signed_distance(at).could_be_equal(S::ZERO) {
                return Err(GeopError::new(format!(
                    "the point {scope} does not lie on face {:?}: holes are drilled at points on the face",
                    args.face
                )))
                .with_context(ctx);
            }
        }
        let blind = match args.end {
            Extent::Blind(depth) => Some(depth),
            Extent::UpToNext | Extent::ThroughAll => None,
        };
        let hull = match blind {
            Some(_) => None,
            None => hull(&part, std::slice::from_ref(&target)).with_context(ctx)?,
        };
        let mut tools = Vec::new();
        let mut placed = Vec::new();
        for (scope, at) in &centres {
            let axes = frame_along(*at, &plane.normal).with_context(ctx)?;
            let depth = match (blind, &hull) {
                (Some(depth), _) => HoleDepth {
                    depth,
                    point: args.drill_point,
                },
                (None, Some(hull)) => HoleDepth {
                    // Into the solid: against the face's normal.
                    depth: reach_past(hull, &axes, -1.0).with_context(ctx)?,
                    point: false,
                },
                (None, None) => {
                    return Err(GeopError::new(format!("solid {target:?} has no faces")))
                        .with_context(ctx);
                }
            };
            let named = namer.scoped(scope);
            let solid = hole_tool(
                &mut part,
                &named,
                &named.name(&["tool"]),
                &axes,
                &shape,
                depth,
            )
            .with_context(ctx)?;
            tools.push(Tool {
                solid,
                up_to_next: (args.end == Extent::UpToNext)
                    .then(|| (named.name(&["top", "q0"]), named.name(&["bottom", "q0"]))),
                scope: Some(scope.clone()),
            });
            placed.push((scope, axes));
        }
        Combine::Difference {
            target: target.clone(),
        }
        .apply(&mut part, &namer, operation_id, &tools)
        .map_err(|e| {
            if e.root_message().starts_with(NOTHING_STOPS) {
                e.with_context(
                    "up to next: a hole does not come out of the solid all round — part of it runs on; drill it blind or through all",
                )
            } else {
                e
            }
        })
        .with_context(ctx)?;
        if let Some(size) = shape.thread {
            let result = part.solid_id(&namer.root()).with_context(ctx)?;
            for (scope, axes) in placed {
                record_tapped_thread(&mut part, &namer, result, scope, &axes, &shape, size, args)
                    .with_context(ctx)?;
            }
        }
        Ok(part)
    }
}

/// Records the cosmetic thread of the tapped hole drilled at `axes` — its
/// wall in `solid` — as the thread `hole(H,X,thread)` for the scope `X`:
/// from the face down, as far as `args` says, or all the way.
#[allow(clippy::too_many_arguments)]
fn record_tapped_thread<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: geop_core_topology::SolidId,
    scope: &str,
    axes: &CoordinateSystem<S>,
    shape: &HoleShape,
    size: &MetricSize,
    args: &HoleArgs,
) -> GeopResult<()> {
    let ctx = with_context!("the thread of the hole at {scope}");
    let into = axes.w().neg();
    let radius = S::from_f64(shape.diameter / 2.0);
    let cylinder = geop_core_geometry::shape::Cylinder {
        axis: geop_core_geometry::shape::Axis::try_new(*axes.origin(), into)?,
        radius,
    };
    let wall = Wall::at(part, solid, &cylinder, axes.origin())
        .with_context(ctx)?
        .ok_or_else(|| GeopError::new(format!("the hole at {scope} left no wall to thread")))?;
    // Measured into the solid from the face.
    let entry = axes
        .origin()
        .sub(&wall.cylinder.axis.point)
        .prod_dot(&wall.cylinder.axis.direction);
    let runs = if wall
        .cylinder
        .axis
        .direction
        .prod_dot(&into)
        .definitely_greater(S::ZERO)
    {
        wall.to.sub(entry)
    } else {
        entry.sub(wall.from)
    };
    // A blind hole is threaded as deep as it was drilled, unless told.
    let length = match (args.thread_length, args.end) {
        (Some(length), _) => length,
        (None, Extent::Blind(depth)) => depth,
        (None, _) => runs.to_f64(),
    };
    if S::from_f64(length).definitely_greater(runs) {
        return Err(GeopError::new(format!(
            "the thread runs {length} deep, but the hole at {scope} only {}",
            runs.to_f64()
        )));
    }
    let placement = ThreadPlacement {
        start: *axes.origin(),
        direction: into,
        radius,
        length,
        internal: true,
    };
    placement.check_fits(size, &wall.face_name(part))?;
    let thread = placement.cosmetic(size, wall.face_name(part))?;
    part.add_thread(namer.scoped(scope).name(&["thread"]), thread)
}

/// Puts an ISO metric thread on the cylindrical face `face` — a shaft's or
/// a hole's, the whole wall it is part of — for the operation `T`. From one
/// end of the wall, `length` far or all of it: from its open end — where a
/// shaft ends, where a hole was drilled from — or, if both or neither are
/// open, from one of them; from the other if `reversed`.
///
/// Recorded, it is the part's cosmetic thread `thread(T)`, which drawings
/// and the viewer read; the solid is left as it is. Modelled, its profile is
/// swept along its helix and cut out of the solid (see [`crate::thread`]),
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
    /// Where the thread runs in `part`, on the wall of its face.
    fn placement<S: Scalar>(&self, part: &Part<S>) -> GeopResult<(Wall<S>, ThreadPlacement<S>)> {
        if self.face.is_empty() {
            return Err(GeopError::new("pick the cylindrical face to thread"));
        }
        let wall = Wall::of(part, part.face_id(&self.face)?)?;
        let runs = wall.length();
        let length = self.length.unwrap_or(runs.to_f64());
        if S::from_f64(length).definitely_greater(runs) {
            return Err(GeopError::new(format!(
                "the thread runs {length} far, but face {:?} only {}",
                self.face,
                runs.to_f64()
            )));
        }
        let (start, direction) = wall.start(self.reversed);
        let placement = ThreadPlacement {
            start: wall.axis_point(start),
            direction,
            radius: wall.cylinder.radius,
            length,
            internal: wall.internal,
        };
        Ok((wall, placement))
    }
}

impl Operation for Thread {
    type Args = ThreadArgs;
    type Session = ();

    /// No face yet; M6, recorded, the whole wall.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> ThreadArgs {
        ThreadArgs {
            face: String::new(),
            size: "M6".into(),
            length: None,
            reversed: false,
            modelled: false,
        }
    }

    /// The face, picked; the size; how far; which end; whether modelled.
    /// The thread drawn as its helix.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &ThreadArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ThreadArgs> {
        let before = context.before;
        let mut f = Form::<S, ThreadArgs>::new();
        f.reference(
            "face",
            "face",
            face_ref(&args.face),
            &[Role::Round],
            None,
            false,
            |e, picked| e.args.face = face_name(&picked),
        );
        f.select(
            "size",
            "size",
            args.size.clone(),
            size_choices(),
            true,
            |args, v| args.size = v.to_string(),
        );
        let placed = args.placement(before);
        if let Ok(size) = metric(&args.size) {
            let text = format!(
                "{}: Ø{} major, Ø{:.3} minor",
                size.designation(),
                size.diameter,
                size.minor_diameter()
            );
            f.text("dimensions", text, Tone::Hint);
        }
        let problem = match &placed {
            Ok((wall, p)) => metric(&args.size)
                .and_then(|size| p.check_fits(size, &wall.face_name(before)))
                .err()
                .map(|e| e.root_message().to_string()),
            Err(e) => Some(e.root_message().to_string()),
        };
        if let Some(problem) = problem
            && !args.face.is_empty()
        {
            f.text("problem", problem, Tone::Error);
        }
        f.checkbox(
            "full_length",
            "whole face",
            args.length.is_none(),
            |args, b| args.length = if b { None } else { Some(10.0) },
        );
        if let Some(length) = args.length {
            f.number(
                "length",
                Number::new("length", length, Unit::Length).range(0.0, 100.0),
                |args, l| args.length = Some(l),
            );
        }
        f.checkbox(
            "reversed",
            "start from the other end",
            args.reversed,
            |args, b| args.reversed = b,
        );
        f.checkbox("modelled", "modelled", args.modelled, |args, b| {
            args.modelled = b
        });
        if let (Ok((_, placement)), Ok(size)) = (&placed, metric(&args.size))
            && let Ok(thread) = placement.cosmetic(size, args.face.clone())
            && let Ok(helix) = thread.helix()
        {
            let n = 8 * (helix.control_points.len() / 2) as i64;
            let points: GeopResult<Vec<_>> = (0..=n)
                .map(|i| helix.evaluate(S::from_ratio(i, n)?))
                .collect();
            if let Ok(points) = points {
                f.visuals.push(Visual::new(
                    "helix",
                    Shape::Polyline { points },
                    Style::Draft,
                ));
            }
        }
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ThreadArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("thread({operation_id}, {args:?})");
        let namer = Namer::new("thread", operation_id)?;
        let size = metric(&args.size).with_context(ctx)?;
        let (wall, placement) = args.placement(&part).with_context(ctx)?;
        placement.check_fits(size, &args.face).with_context(ctx)?;
        if !args.modelled {
            let thread = placement.cosmetic(size, args.face.clone())?;
            part.add_thread(namer.root(), thread).with_context(ctx)?;
            return Ok(part);
        }
        let model = part.topology();
        let Body::Solid(solid) = model.body_of_face(wall.faces[0])? else {
            return Err(GeopError::new(format!(
                "face {:?} stands on its own: a modelled thread is cut into a solid",
                args.face
            )))
            .with_context(ctx);
        };
        let target = part
            .name_of(solid)
            .ok_or_else(|| GeopError::new(format!("{solid} has no name")))?
            .to_string();
        let tool = thread_tool(&mut part, &namer, &namer.name(&["tool"]), size, &placement)
            .with_context(ctx)?;
        Combine::Difference { target }
            .apply(
                &mut part,
                &namer,
                operation_id,
                &[Tool {
                    solid: tool,
                    up_to_next: None,
                    scope: None,
                }],
            )
            .with_context(ctx)?;
        Ok(part)
    }
}
