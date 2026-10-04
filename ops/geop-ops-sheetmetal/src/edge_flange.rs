//! [`EdgeFlange`]: a flange bent up from a straight edge of a sheet-metal
//! body.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Choice, Form, Number, Unit},
};
use geop_ops_extrude_revolve::common::{line2, start_point};
use serde::{Deserialize, Serialize};

use crate::{
    sheet::{Bend, BendFrame, Flat, FlatEdge, Sheet, straight},
    thicken::thicken,
};

/// What a flange's length is measured from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LengthReference {
    /// Where the outsides of the sheet and the flange would meet were the
    /// bend sharp.
    #[default]
    OuterSharp,
    /// Where their insides would meet.
    InnerSharp,
    /// Where the bend ends: the flange's flat part only.
    Tangent,
}

/// Where a flange's bend sits relative to the edge it is bent from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlangePosition {
    /// Inside the sheet's old outline: the outside of the flange where the
    /// edge was — the sheet set back by the bend.
    #[default]
    MaterialInside,
    /// The inside of the flange where the edge was.
    MaterialOutside,
    /// The bend starts at the edge: everything added outside.
    BendOutside,
}

/// Bends a flange up from the straight edge `edge` of a sheet-metal body,
/// for the operation `F`: the body is consumed and rebuilt as
/// `edge_flange(F)`, every face, edge and vertex it had keeping its name
/// except the edge, which the flange replaces.
///
/// `edge` is an edge of a flat face of the body on its outline (where the
/// face meets a wall across the thickness), and the flange turns `angle`
/// degrees towards that face's side, `length` long as `reference` says,
/// with its bend's inner radius `radius` (the body's own by default), set
/// as `position` says. It runs along the edge from `offset_start` after
/// the edge's start to `offset_end` before its end (the edge running with
/// its face on its left, seen from outside); where it is narrower than the
/// edge, the body's relief is cut beside the bend. An end of the edge that
/// meets a bend already there keeps the body's corner gap from it.
///
/// What it builds is named `edge_flange(F,...)`: the bend `bend`, the
/// flange's flat face `flange` with its edges `flange,line` where the bend
/// ends, `flange,side0`, `flange,end` and `flange,side1`; where the bend
/// starts on the body `line`, the rest of the edge `before` and `after`,
/// and the reliefs
/// `relief0,in`, `relief0,across`, `relief0,out` (and `relief1,...`) — each
/// a face's name with `a`/`b` added, an edge's or a wall's as
/// [`crate::thicken`] says.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EdgeFlange;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeFlangeArgs {
    /// The edge, of a flat face of a sheet-metal body.
    pub edge: String,
    /// How far the flange turns, in degrees: more than 0, less than 180.
    pub angle: f64,
    pub length: f64,
    #[serde(default)]
    pub reference: LengthReference,
    #[serde(default)]
    pub position: FlangePosition,
    /// The bend's inner radius, if not the body's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    #[serde(default)]
    pub offset_start: f64,
    #[serde(default)]
    pub offset_end: f64,
}

impl Operation for EdgeFlange {
    type Args = EdgeFlangeArgs;
    type Session = ();

    /// No edge yet: a right angle, half a unit long, material inside.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> EdgeFlangeArgs {
        EdgeFlangeArgs {
            edge: String::new(),
            angle: 90.0,
            length: 0.5,
            reference: LengthReference::default(),
            position: FlangePosition::default(),
            radius: None,
            offset_start: 0.0,
            offset_end: 0.0,
        }
    }

    /// The edge, picked; angle and length; what the length is measured from
    /// and where the bend sits; the radius, if its own; and how far in
    /// from the edge's ends.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &EdgeFlangeArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, EdgeFlangeArgs> {
        let mut f = Form::<S, EdgeFlangeArgs>::new();
        let edge = (!args.edge.is_empty())
            .then(|| EntityRef::Edge {
                name: args.edge.clone(),
            })
            .into_iter()
            .collect();
        f.reference("edge", "edge", edge, &[Role::Line], None, false, |e, picked| {
            e.args.edge = match picked.as_slice() {
                [EntityRef::Edge { name }] => name.clone(),
                _ => String::new(),
            }
        });
        f.number(
            "angle",
            Number::new("angle", args.angle, Unit::Angle).range(0.0, 180.0),
            |args, a| args.angle = a,
        );
        f.number(
            "length",
            Number::new("length", args.length, Unit::Length).range(0.0, 5.0),
            |args, l| args.length = l,
        );
        f.select(
            "reference",
            "measured from",
            match args.reference {
                LengthReference::OuterSharp => "outer_sharp",
                LengthReference::InnerSharp => "inner_sharp",
                LengthReference::Tangent => "tangent",
            },
            vec![
                Choice::new("outer_sharp", "Outer virtual sharp"),
                Choice::new("inner_sharp", "Inner virtual sharp"),
                Choice::new("tangent", "Bend tangent"),
            ],
            false,
            |args, choice| {
                args.reference = match choice {
                    "inner_sharp" => LengthReference::InnerSharp,
                    "tangent" => LengthReference::Tangent,
                    _ => LengthReference::OuterSharp,
                }
            },
        );
        f.select(
            "position",
            "position",
            match args.position {
                FlangePosition::MaterialInside => "material_inside",
                FlangePosition::MaterialOutside => "material_outside",
                FlangePosition::BendOutside => "bend_outside",
            },
            vec![
                Choice::new("material_inside", "Material inside"),
                Choice::new("material_outside", "Material outside"),
                Choice::new("bend_outside", "Bend outside"),
            ],
            false,
            |args, choice| {
                args.position = match choice {
                    "material_outside" => FlangePosition::MaterialOutside,
                    "bend_outside" => FlangePosition::BendOutside,
                    _ => FlangePosition::MaterialInside,
                }
            },
        );
        f.checkbox(
            "own_radius",
            "own bend radius",
            args.radius.is_some(),
            |args, on| args.radius = on.then_some(args.radius.unwrap_or(0.1)),
        );
        if let Some(radius) = args.radius {
            f.number(
                "radius",
                Number::new("bend radius", radius, Unit::Length).range(0.0, 1.0),
                |args, r| args.radius = Some(r),
            );
        }
        f.number(
            "offset_start",
            Number::new("offset at start", args.offset_start, Unit::Length).range(0.0, 5.0),
            |args, o| args.offset_start = o,
        );
        f.number(
            "offset_end",
            Number::new("offset at end", args.offset_end, Unit::Length).range(0.0, 5.0),
            |args, o| args.offset_end = o,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &EdgeFlangeArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("edge_flange({operation_id}, {args:?})");
        let namer = Namer::new("edge_flange", operation_id)?;
        let (solid, mut sheet, (f, k, toward_b)) = part
            .solid_names()
            .into_iter()
            .find_map(|solid| {
                let sheet = part.body_data::<Sheet<S>>(&solid)?;
                let at = sheet.edge_named(&args.edge)?;
                Some((solid, sheet.clone(), at))
            })
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{:?} is no edge of a sheet-metal body's flat face on its outline: a flange is bent from one of those, on a body built by a base flange",
                    args.edge
                ))
            })
            .with_context(ctx)?;
        add_flange(&mut sheet, &namer, f, k, toward_b, args).with_context(ctx)?;
        let folded = sheet.folded().with_context(ctx)?;
        let id = part.solid_id(&solid)?;
        part.assemble_sheet(&[Body::Solid(id)], &[])
            .with_context(ctx)?;
        let name = namer.root();
        thicken(&mut part, &folded, &name, &|n| n).with_context(ctx)?;
        part.set_body_data(&name, sheet).with_context(ctx)?;
        Ok(part)
    }
}

/// Adds the flange `args` describes to `sheet`, bent from edge `k` of flat
/// `f`'s outline towards its B side if `toward_b` (see [`EdgeFlange`]).
fn add_flange<S: Scalar>(
    sheet: &mut Sheet<S>,
    namer: &Namer,
    f: usize,
    k: usize,
    toward_b: bool,
    args: &EdgeFlangeArgs,
) -> GeopResult<()> {
    let rules = sheet.rules.clone();
    let flat = &sheet.flats[f];
    let edge = &flat.outer[k];
    let name = edge.key();
    let [a, b] = edge.line().ok_or_else(|| {
        GeopError::new(format!("edge {name} is not straight: a flange is bent along a straight edge"))
    })??;
    if sheet.is_bent(&name) {
        return Err(GeopError::new(format!("edge {name} is already bent")));
    }
    if !(args.angle > 0.0 && args.angle < 180.0) {
        return Err(GeopError::new(format!(
            "a flange turns by more than 0 and less than 180 degrees, not {}",
            args.angle
        )));
    }
    let radius = args.radius.unwrap_or(rules.bend_radius);
    if !(radius.is_finite() && radius > 0.0) {
        return Err(GeopError::new(format!("the bend radius must be positive, not {radius}")));
    }
    for (what, offset) in [("start", args.offset_start), ("end", args.offset_end)] {
        if !(offset.is_finite() && offset >= 0.0) {
            return Err(GeopError::new(format!(
                "the offset at the {what} must not be negative, not {offset}"
            )));
        }
    }
    let n = flat.outer.len();
    let (prev, next) = ((k + n - 1) % n, (k + 1) % n);
    let s = S::from_f64;
    let t = s(rules.thickness);
    let r = s(radius);
    let angle = s(args.angle).mul(S::PI).div(s(180.0))?;
    let half = angle.div(S::TWO)?;
    let half_tan = half.sin().div(half.cos())?;
    let set_back = match args.position {
        FlangePosition::MaterialInside => Some(r.add(t).mul(half_tan)),
        FlangePosition::MaterialOutside => Some(r.mul(half_tan)),
        FlangePosition::BendOutside => None,
    };
    let length = s(args.length).sub(match args.reference {
        LengthReference::OuterSharp => r.add(t).mul(half_tan),
        LengthReference::InnerSharp => r.mul(half_tan),
        LengthReference::Tangent => S::ZERO,
    });
    if !length.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "a flange {} long leaves nothing flat after its bend ({length:?})",
            args.length
        )));
    }

    // Along the edge `s`, into the flat `h`.
    let edge_length = b.sub(&a).norm();
    let tau = b.sub(&a).normalize()?;
    let inward = Vector2::from_array([tau[1].neg(), tau[0]]);
    let at = |along: S, into: S| a.add(&tau.prod_scalar(along)).add(&inward.prod_scalar(into));
    let d = set_back.unwrap_or(S::ZERO);
    let relief = rules.relief_size().map(s);
    let depth = relief.map(|w| d.add(w));

    // Each end: how far in the flange starts, whether it reaches the
    // corner, and the neighbour there.
    let gap = |neighbour: usize, end: &str| -> GeopResult<f64> {
        let key = flat.outer[neighbour].key();
        if !sheet.is_bent(&key) {
            return Ok(0.0);
        }
        if rules.corner_gap > 0.0 {
            Ok(rules.corner_gap)
        } else {
            Err(GeopError::new(format!(
                "edge {name} meets the bent edge {key} at its {end}, and the corner gap is 0: the two bends would touch"
            )))
        }
    };
    let inset_start = args.offset_start + gap(prev, "start")?;
    let inset_end = args.offset_end + gap(next, "end")?;
    let (s0, s1) = (s(inset_start), edge_length.sub(s(inset_end)));
    if !s1.sub(s0).definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "edge {name} is {edge_length:?} long, too short for a flange set in {inset_start} at its start and {inset_end} at its end"
        )));
    }

    // A set-back bend narrower than its edge lies where the sheet beside
    // it still is: only a relief keeps the two apart.
    if set_back.is_some() && relief.is_none() && !(inset_start == 0.0 && inset_end == 0.0) {
        return Err(GeopError::new(format!(
            "a flange on part of edge {name}, set back into the sheet, needs a relief beside it: with a tear, the bend would touch the sheet beside it — use a rectangular relief, run the flange along the whole edge, or bend it outside"
        )));
    }

    // What the flange cuts out of the flat — the strip it is set back by,
    // the reliefs — must lie inside it: no corner of the flat in it.
    let full = (inset_start == 0.0, inset_end == 0.0);
    let notched = relief.is_some() && !(full.0 && full.1);
    let height = if notched { depth.expect("a relief has a depth") } else { d };
    if height.definitely_greater(S::ZERO) {
        let w = if notched { relief.expect("a relief") } else { S::ZERO };
        let lo = if full.0 { S::ZERO } else { s0.sub(w) };
        let hi = if full.1 { edge_length } else { s1.add(w) };
        for e in std::iter::once(&flat.outer).chain(&flat.holes).flatten() {
            let p = start_point(&e.curve)?;
            if p.could_be_equal(&a) || p.could_be_equal(&b) {
                continue;
            }
            let (along, into) = (p.sub(&a).prod_dot(&tau), p.sub(&a).prod_dot(&inward));
            if along.could_be_greater(lo)
                && along.could_be_less(hi)
                && into.could_be_greater(S::ZERO)
                && into.could_be_less(height)
            {
                return Err(GeopError::new(format!(
                    "a flange on edge {name} cuts {height:?} into flat {}, through its corner where {} starts: make the flange narrower or shorter, or bend it outside",
                    flat.name.root(),
                    e.key()
                )));
            }
        }
    }

    // The new edges in place of the edge, as the points they start at.
    let mut run: Vec<(Vector2<S>, Namer)> = Vec::new();
    let mut corner_start = None;
    let mut corner_end = None;
    if inset_start == 0.0 {
        match set_back {
            Some(d) => {
                let corner = trim_point(flat, prev, at(S::ZERO, d), d, &name, "start")?;
                corner_start = Some(corner);
                run.push((corner, namer.scoped("line")));
            }
            None => run.push((a, namer.scoped("line"))),
        }
    } else {
        run.push((a, namer.scoped("before")));
        match (relief, depth) {
            (Some(w), Some(depth)) => {
                fits(s0.sub(w), &name, "start")?;
                let relief = namer.scoped("relief0");
                run.push((at(s0.sub(w), S::ZERO), relief.scoped("in")));
                run.push((at(s0.sub(w), depth), relief.scoped("across")));
                run.push((at(s0, depth), relief.scoped("out")));
            }
            _ => {}
        }
        run.push((at(s0, d), namer.scoped("line")));
    }
    let line_start = run.last().expect("the flange's line").0;
    let line_end;
    if inset_end == 0.0 {
        match set_back {
            Some(d) => {
                let corner = trim_point(flat, next, at(edge_length, d), d, &name, "end")?;
                corner_end = Some(corner);
                line_end = corner;
            }
            None => line_end = b,
        }
    } else {
        line_end = at(s1, d);
        match (relief, depth) {
            (Some(w), Some(depth)) => {
                fits(edge_length.sub(s1.add(w)), &name, "end")?;
                let relief = namer.scoped("relief1");
                run.push((line_end, relief.scoped("in")));
                run.push((at(s1, depth), relief.scoped("across")));
                run.push((at(s1.add(w), depth), relief.scoped("out")));
                run.push((at(s1.add(w), S::ZERO), namer.scoped("after")));
            }
            _ => run.push((line_end, namer.scoped("after"))),
        }
    }
    let end_point = if inset_end == 0.0 { line_end } else { b };
    let mut edges = Vec::new();
    for (i, (from, name)) in run.iter().enumerate() {
        let to = run.get(i + 1).map_or(end_point, |(p, _)| *p);
        edges.push(straight(*from, to, name.clone())?);
    }

    // The flange: its line is the body's, run the other way; it reaches out
    // across it, away from the flat.
    let out = inward.neg().prod_scalar(length);
    let flange = namer.scoped("flange");
    let (c_a, c_b) = (line_start, line_end);
    let outer = vec![
        straight(c_b, c_a, flange.scoped("line"))?,
        straight(c_a, c_a.add(&out), flange.scoped("side0"))?,
        straight(c_a.add(&out), c_b.add(&out), flange.scoped("end"))?,
        straight(c_b.add(&out), c_b, flange.scoped("side1"))?,
    ];
    let place = flat.place.clone();
    let frame = BendFrame::new(&place, [c_a, c_b], angle, r, t, toward_b)?;
    let child = Flat {
        place: frame.child_placement(&place),
        outer,
        holes: Vec::new(),
        name: flange.clone(),
    };

    let flat = &mut sheet.flats[f];
    if let Some(corner) = corner_start {
        trim(&mut flat.outer[prev], None, Some(corner))?;
    }
    if let Some(corner) = corner_end {
        trim(&mut flat.outer[next], Some(corner), None)?;
    }
    flat.outer.splice(k..k + 1, edges);
    sheet.flats.push(child);
    sheet.bends.push(Bend {
        parent: f,
        parent_edge: namer.scoped("line").root(),
        child: sheet.flats.len() - 1,
        child_edge: flange.scoped("line").root(),
        angle,
        radius: r,
        toward_b,
        name: namer.scoped("bend"),
    });
    Ok(())
}

/// Checks that a relief leaves `left` of the edge `name` beside it, at its
/// `end`.
fn fits<S: Scalar>(left: S, name: &str, end: &str) -> GeopResult<()> {
    if left.definitely_greater(S::ZERO) {
        Ok(())
    } else {
        Err(GeopError::new(format!(
            "the relief at the {end} of edge {name} does not fit beside it: offset the flange further, or use a tear"
        )))
    }
}

/// Where the flange's set-back line meets the straight edge `neighbour` of
/// `flat` at the `end` of the edge `name` it is bent from: `corner`, a set
/// back `d` along it — the neighbour must meet the edge at a right angle,
/// and be longer than that.
fn trim_point<S: Scalar>(
    flat: &Flat<S>,
    neighbour: usize,
    corner: Vector2<S>,
    d: S,
    name: &str,
    end: &str,
) -> GeopResult<Vector2<S>> {
    let edge = &flat.outer[neighbour];
    let refuse = |why: &str| {
        GeopError::new(format!(
            "a flange set back along all of edge {name} cuts back the edge {} at its {end}, which {why}: offset the flange from that end, or bend it outside",
            edge.key()
        ))
    };
    let [p, q] = edge.line().ok_or_else(|| refuse("is not straight"))??;
    let this = &flat.outer[flat
        .outer
        .iter()
        .position(|e| e.key() == name)
        .expect("the edge is in its flat")];
    let [a, b] = this.line().expect("the edge is straight")?;
    let along = q.sub(&p).normalize()?;
    if !along.prod_dot(&b.sub(&a).normalize()?).could_be_equal(S::ZERO) {
        return Err(refuse("does not meet it at a right angle"));
    }
    if !q.sub(&p).norm().definitely_greater(d) {
        return Err(refuse("is shorter than the set back"));
    }
    Ok(corner)
}

/// `edge` with its start or end moved to the given point.
fn trim<S: Scalar>(
    edge: &mut FlatEdge<S>,
    start: Option<Vector2<S>>,
    end: Option<Vector2<S>>,
) -> GeopResult<()> {
    let [p, q] = edge
        .line()
        .ok_or_else(|| GeopError::new(format!("edge {} is not straight", edge.key())))??;
    edge.curve = line2(start.unwrap_or(p), end.unwrap_or(q))?;
    Ok(())
}
