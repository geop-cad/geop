//! [`Route`]: lay a bundle of wires between connectors, through clips.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role, frame_along},
    part::{Cable, CutWire},
    ui::{Action, Choice, Form, ListItem, Number, Shape, Style, Tone, Unit, Value, Visual},
};
use geop_ops_extrude_revolve::{
    common::{Profile, arc2, sqrt2_over_2},
    path_sweep::{Control, sweep_along},
    sweep::SweepLoop,
};
use serde::{Deserialize, Serialize};

use crate::{
    route::{Measure, RoutePath, Waypoint},
    wire::{GAUGES, Wire, WireSize, bundle_diameter},
};

/// Routes a bundle of wires through points picked in order — from the
/// connector it starts at, through clips, to the connector it ends at —
/// as a tangent-continuous chain of lines and arcs (see [`crate::route`]),
/// and sweeps the bundle along it into a solid named `route(R)` for the
/// operation `R`. A route that bends tighter than the bundle may is
/// refused, naming the waypoints each bend lies between, its radius and
/// the radius allowed.
///
/// What the bundle is cut from is recorded on the part as the cable
/// `route(R)` (see [`geop_ops::part::Cable`]): every wire with its cut
/// length, the route's length and a service loop at each end.
///
/// The points may be of parts placed in the part: the route follows them
/// wherever their mates put them, rerouted whenever the program runs.
///
/// The solid is named like any path sweep (see
/// [`geop_ops_extrude_revolve::path_sweep::sweep_along`]): the profile is a
/// circle of four arcs `c0`–`c3` meeting at `p0`–`p3`, the path's curves
/// and joints are named as [`RoutePath`] says — `route(R,c0,a1)` is a
/// quarter of the bundle's skin along the first arc after the second
/// waypoint — and its ends are `route(R,start)` and `route(R,end)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Route;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteArgs {
    /// What the route runs through, in order: points, coordinate systems
    /// (along their `z` axis: out of a connector at the start, into one at
    /// the end) and circular edges (through their centre, along their
    /// axis).
    #[serde(default)]
    pub through: Vec<EntityRef>,
    /// The wires bundled.
    pub wires: Vec<Wire>,
    /// The fraction of the bundle's section its wires fill (see
    /// [`bundle_diameter`]).
    pub fill: f64,
    /// The smallest radius the bundle may bend with, in bundle diameters.
    pub bend_factor: f64,
    /// How much longer than the route each wire is cut at each end.
    #[serde(default)]
    pub service_loop: f64,
}

/// What editing a route keeps: the wire selected.
#[derive(Clone, Debug, Default)]
pub struct RouteSession {
    pub selected: Option<usize>,
}

/// What a step of `args` routes: the bundle's diameter, the smallest
/// radius it may bend with, and the path through the points of `part`.
struct Plan<S: Scalar> {
    diameter: f64,
    min_bend_radius: f64,
    path: RoutePath<S>,
    measure: Measure<S>,
}

impl RouteArgs {
    fn plan<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Plan<S>> {
        let diameter = bundle_diameter(&self.wires, self.fill)?;
        // Any tighter, and the bundle would fold into itself.
        if !(self.bend_factor > 0.5 && self.bend_factor.is_finite()) {
            return Err(GeopError::new(format!(
                "route: a bend factor of {}: the bundle cannot bend tighter than its own radius, half its diameter",
                self.bend_factor
            )));
        }
        if !(self.service_loop >= 0.0 && self.service_loop.is_finite()) {
            return Err(GeopError::new(format!(
                "route: a service loop of {}: it is a length, zero or more",
                self.service_loop
            )));
        }
        let waypoints = self
            .through
            .iter()
            .enumerate()
            .map(|(i, entity)| Waypoint::of(part, entity, i))
            .collect::<GeopResult<Vec<_>>>()?;
        let path = RoutePath::through(&waypoints)?;
        let measure = path.measure()?;
        Ok(Plan {
            diameter,
            min_bend_radius: self.bend_factor * diameter,
            path,
            measure,
        })
    }
}

impl<S: Scalar> Plan<S> {
    /// Refuses a route bending tighter than the bundle may, saying where.
    fn check_bends(&self, bend_factor: f64) -> GeopResult<()> {
        let tight = self.measure.too_tight(self.min_bend_radius);
        if tight.is_empty() {
            return Ok(());
        }
        let bends: Vec<String> = tight
            .iter()
            .map(|b| {
                format!(
                    "with radius {:.3} {}",
                    b.radius.to_f64(),
                    self.path.places[b.curve]
                )
            })
            .collect();
        Err(GeopError::new(format!(
            "route: the bundle, {:.3} across, may bend no tighter than radius {:.3} ({bend_factor} × its diameter), but it bends {}: move the points apart, or give the route room to turn",
            self.diameter,
            self.min_bend_radius,
            bends.join(", and ")
        )))
    }
}

/// A circle of radius `r` around the origin of a plane: four quarter arcs,
/// counter-clockwise.
fn circle<S: Scalar>(r: f64) -> GeopResult<Vec<geop_core_geometry::nurb_curve::NurbCurve2D<S>>> {
    let at = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
    let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
    (0..4)
        .map(|i| {
            let (a, b) = (q[i], q[(i + 1) % 4]);
            arc2(
                at(a.0, a.1),
                at(a.0 + b.0, a.1 + b.1),
                at(b.0, b.1),
                sqrt2_over_2(),
            )
        })
        .collect()
}

/// The gauges a wire's size is chosen from, by `awg:<gauge>`, and a
/// diameter of its own.
fn size_choices(size: &WireSize) -> Vec<Choice> {
    let mut gauges: Vec<i32> = (10..=30).step_by(2).collect();
    if let Some(g) = size.gauge()
        && !gauges.contains(&g)
    {
        gauges.push(g);
        gauges.sort();
    }
    gauges
        .into_iter()
        .map(|g| Choice::new(format!("awg:{g}"), format!("AWG {g}")))
        .chain([Choice::new("diameter", "diameter")])
        .collect()
}

fn size_value(size: &WireSize) -> String {
    match size {
        WireSize::Awg { gauge, .. } => format!("awg:{gauge}"),
        WireSize::Diameter(_) => "diameter".into(),
    }
}

/// How a wire reads in the list: its size and colour — and, built, the
/// length to cut it to.
fn wire_detail(wire: &Wire, cut: Option<f64>) -> String {
    let size = match (wire.size, wire.size.diameter()) {
        (WireSize::Awg { gauge, .. }, Ok(d)) => format!("AWG {gauge}, {d:.2} across"),
        (WireSize::Diameter(_), Ok(d)) => format!("{d:.2} across"),
        (_, Err(e)) => e.root_message().to_string(),
    };
    match cut {
        Some(cut) => format!("{size} · {} · cut {cut:.1}", wire.colour),
        None => format!("{size} · {}", wire.colour),
    }
}

/// Samples along each curve of the route, to draw it by.
const SAMPLES_PER_CURVE: usize = 16;

impl Operation for Route {
    type Args = RouteArgs;
    type Session = RouteSession;

    /// Nothing picked yet; one hookup wire, filling three quarters of the
    /// bundle, bent no tighter than five diameters.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> RouteArgs {
        RouteArgs {
            through: Vec::new(),
            wires: vec![Wire::new("w1")],
            fill: 0.75,
            bend_factor: 5.0,
            service_loop: 0.0,
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &RouteArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("route({operation_id}, {args:?})");
        let namer = Namer::new("route", operation_id)?;
        let plan = args.plan(&part).with_context(ctx)?;
        plan.check_bends(args.bend_factor).with_context(ctx)?;

        // The bundle's section, square to the route where it starts.
        let chain = &plan.path.chain;
        let start = chain.joints()?[0];
        let plane = frame_along(start, &chain.curves[0].tangent(S::ZERO)?).with_context(ctx)?;
        let section = SweepLoop::plain(Profile::closed(circle(plan.diameter / 2.0)?));
        let name = namer.root();
        sweep_along(&mut part, &namer, Some(&name), chain, &plane, &[section], &Control::default()).with_context(ctx)?;

        let length = plan.measure.length;
        let extra = S::from_f64(2.0 * args.service_loop);
        let wires = args
            .wires
            .iter()
            .map(|wire| {
                Ok(CutWire {
                    name: wire.name.clone(),
                    colour: wire.colour.clone(),
                    diameter: wire.size.diameter()?,
                    gauge: wire.size.gauge(),
                    cut_length: length.add(extra),
                })
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let cable = Cable {
            length,
            diameter: plan.diameter,
            min_bend_radius: plan.min_bend_radius,
            tightest_bend: plan.measure.tightest().map(|b| b.radius),
            wires,
        };
        part.add_cable(name, cable).with_context(ctx)?;
        Ok(part)
    }

    /// The points to run through, picked in order; the wires, each
    /// selected to set its size and colour; the bundle's fill, bend factor
    /// and service loop; what was built — and the route drawn, its points
    /// numbered and its bends too tight marked.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &RouteArgs,
        session: &RouteSession,
        _: &[String],
    ) -> Form<'a, S, RouteArgs, RouteSession> {
        let mut f = Form::<S, RouteArgs, RouteSession>::new();
        f.reference(
            "through",
            "through",
            args.through.clone(),
            &[Role::Point, Role::Circle],
            None,
            true,
            |edit, picked| edit.args.through = picked,
        );
        f.text(
            "through_hint",
            "Pick the connector the route starts at, the clips it runs through and the connector it ends at, in order. A coordinate system's z axis is the way the cable leaves it; a circular edge is passed through along its axis.",
            Tone::Hint,
        );

        let cable = context
            .built
            .and_then(|part| part.cable(&format!("route({})", context.id)).ok());
        f.heading("wires_heading", "Wires");
        let items = args
            .wires
            .iter()
            .enumerate()
            .map(|(i, wire)| {
                let key = format!("wire:{i}");
                f.on(key.clone(), move |edit, value| match value {
                    Value::Press => {
                        edit.session.selected = (edit.session.selected != Some(i)).then_some(i);
                    }
                    Value::Remove if i < edit.args.wires.len() => {
                        edit.args.wires.remove(i);
                        edit.session.selected = None;
                    }
                    Value::Text(name) => {
                        if let Some(wire) = edit.args.wires.get_mut(i) {
                            wire.name = name;
                        }
                    }
                    _ => {}
                });
                let cut = cable
                    .and_then(|c| c.wires.get(i))
                    .map(|w| w.cut_length.to_f64());
                let mut item = ListItem::new(key, wire.name.clone());
                item.detail = Some(wire_detail(wire, cut));
                item.tone = match wire.size.diameter() {
                    Ok(_) => Tone::Normal,
                    Err(_) => Tone::Error,
                };
                item.selected = session.selected == Some(i);
                item.removable = true;
                item.text = Some(wire.name.clone());
                item
            })
            .collect();
        f.list("wires", items, "None yet: add one.");
        f.actions(
            "add_wire",
            vec![Action::new("add", "Add wire").title("Add a wire to the bundle, and select it.")],
            |edit, _| {
                let n = edit.args.wires.len();
                edit.args.wires.push(Wire::new(format!("w{}", n + 1)));
                edit.session.selected = Some(n);
            },
        );
        if let Some((i, wire)) = session
            .selected
            .and_then(|i| args.wires.get(i).map(|w| (i, w)))
        {
            f.select(
                "wire_size",
                format!("{} size", wire.name),
                size_value(&wire.size),
                size_choices(&wire.size),
                false,
                move |args, choice| {
                    let Some(wire) = args.wires.get_mut(i) else {
                        return;
                    };
                    let insulation = match wire.size {
                        WireSize::Awg { insulation, .. } => insulation,
                        WireSize::Diameter(_) => 0.3,
                    };
                    if let Some(gauge) = choice
                        .strip_prefix("awg:")
                        .and_then(|g| g.parse().ok())
                        .filter(|g| GAUGES.contains(g))
                    {
                        wire.size = WireSize::Awg { gauge, insulation };
                    } else if choice == "diameter"
                        && let Ok(d) = wire.size.diameter()
                    {
                        wire.size = WireSize::Diameter(d);
                    }
                },
            );
            match wire.size {
                WireSize::Awg { insulation, .. } => {
                    f.number(
                        "wire_insulation",
                        Number::new("insulation", insulation, Unit::Length).range(0.0, 1.0),
                        move |args, v| {
                            if let Some(WireSize::Awg { insulation, .. }) =
                                args.wires.get_mut(i).map(|w| &mut w.size)
                            {
                                *insulation = v;
                            }
                        },
                    );
                }
                WireSize::Diameter(d) => {
                    f.number(
                        "wire_diameter",
                        Number::new("diameter", d, Unit::Length).range(0.1, 20.0),
                        move |args, v| {
                            if let Some(wire) = args.wires.get_mut(i) {
                                wire.size = WireSize::Diameter(v);
                            }
                        },
                    );
                }
            }
            f.color(
                "wire_colour",
                "colour",
                wire.colour.clone(),
                move |args, c| {
                    if let Some(wire) = args.wires.get_mut(i) {
                        wire.colour = c.to_string();
                    }
                },
            );
        }

        f.heading("bundle_heading", "Bundle");
        f.number(
            "fill",
            Number::new("fill factor", args.fill, Unit::Fraction).range(0.3, 1.0),
            |args, v| args.fill = v,
        );
        f.number(
            "bend_factor",
            Number::new("min bend radius / diameter", args.bend_factor, Unit::Count)
                .range(1.0, 20.0),
            |args, v| args.bend_factor = v,
        );
        f.number(
            "service_loop",
            Number::new("service loop per end", args.service_loop, Unit::Length).range(0.0, 200.0),
            |args, v| args.service_loop = v,
        );
        if let Some(cable) = cable {
            let tightest = match cable.tightest_bend {
                Some(r) => format!("tightest bend radius {:.2}", r.to_f64()),
                None => "straight".into(),
            };
            f.text(
                "report",
                format!(
                    "Route {:.1} long · bundle {:.2} across · {tightest}, at least {:.2} allowed",
                    cable.length.to_f64(),
                    cable.diameter,
                    cable.min_bend_radius
                ),
                Tone::Success,
            );
        }
        draw(&mut f, context.before, args);
        f
    }
}

/// The route drawn: its points numbered, as what it says about them counts
/// them, and its centre line, the bends too tight marked as failed.
fn draw<S: Scalar>(f: &mut Form<'_, S, RouteArgs, RouteSession>, part: &Part<S>, args: &RouteArgs) {
    let waypoints: Vec<Waypoint<S>> = args
        .through
        .iter()
        .enumerate()
        .filter_map(|(i, entity)| Waypoint::of(part, entity, i).ok())
        .collect();
    for (i, waypoint) in waypoints.iter().enumerate() {
        f.visuals.push(Visual::new(
            format!("point:{i}"),
            Shape::Label {
                at: waypoint.point,
                text: (i + 1).to_string(),
                offset: Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(2.0)]),
            },
            Style::Reference,
        ));
    }
    let Ok(plan) = args.plan(part) else {
        return;
    };
    let tight: Vec<usize> = plan
        .measure
        .too_tight(plan.min_bend_radius)
        .iter()
        .map(|b| b.curve)
        .collect();
    for (k, curve) in plan.path.chain.curves.iter().enumerate() {
        let points = (0..=SAMPLES_PER_CURVE)
            .map(|j| curve.evaluate(S::from_ratio(j as i64, SAMPLES_PER_CURVE as i64)?))
            .collect::<GeopResult<Vec<_>>>();
        let Ok(points) = points else { continue };
        let style = if tight.contains(&k) {
            Style::Failed
        } else {
            Style::Guide
        };
        f.visuals.push(Visual::new(
            format!("route:{k}"),
            Shape::Polyline { points },
            style,
        ));
    }
}
