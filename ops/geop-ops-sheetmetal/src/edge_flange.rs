//! [`EdgeFlange`]: a flange bent up from a straight edge of a sheet-metal
//! body.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    parameters::Formula,
    ui::{Choice, Form, Number, Unit},
};
use geop_ops_extrude_revolve::common::{line2, start_point};
use serde::{Deserialize, Serialize};

use crate::sheet::{Bend, BendFrame, Flat, FlatEdge, Sheet, SheetMetalRules, straight};

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

/// What a flange does at an end of its edge where a flange already stands
/// on the edge beside it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Corner {
    /// It stops the body's corner gap short of the other flange's bend.
    #[default]
    Open,
    /// Its bend stops the corner gap short of the other's, and its flat
    /// reaches on, past its bend, to the corner gap from the other flange's
    /// inside: the corner closed. For two flanges at right angles, turning
    /// the same way, of one radius, on edges at right angles.
    Closed,
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
/// meets a bend already there keeps the body's corner gap from it — or,
/// with a closed `corner`, its flat reaches on to the other flange.
///
/// What it builds is named `edge_flange(F,...)`: the bend `bend`, the
/// flange's flat face `flange` with its edges `flange,line` where the bend
/// ends, `flange,side0`, `flange,end` and `flange,side1` — and where a
/// closed corner carries its flat past its bend, `flange,corner0` or
/// `flange,corner1` on from its line; where the bend
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
    /// How far the flange turns, in degrees: more than 0, less than 180 —
    /// a number, or a formula of the part's parameters.
    pub angle: Formula,
    /// How long it is, measured from its `reference`: a number, or a
    /// formula of the part's parameters.
    pub length: Formula,
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
    #[serde(default)]
    pub corner: Corner,
}

impl Operation for EdgeFlange {
    type Args = EdgeFlangeArgs;
    type Session = ();

    /// No edge yet: a right angle, half a unit long, material inside.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> EdgeFlangeArgs {
        EdgeFlangeArgs {
            edge: String::new(),
            angle: Formula::Plain(90.0),
            length: Formula::Plain(0.5),
            reference: LengthReference::default(),
            position: FlangePosition::default(),
            radius: None,
            offset_start: 0.0,
            offset_end: 0.0,
            corner: Corner::Open,
        }
    }

    /// The edge, picked; angle and length; what the length is measured from
    /// and where the bend sits; the radius, if its own; and how far in
    /// from the edge's ends.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
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
        f.reference(
            "edge",
            "edge",
            edge,
            &[Role::Line],
            None,
            false,
            |e, picked| {
                e.args.edge = match picked.as_slice() {
                    [EntityRef::Edge { name }] => name.clone(),
                    _ => String::new(),
                }
            },
        );
        let inputs = context.before.inputs();
        f.formula(
            "angle",
            Number::formula("angle", &args.angle, inputs, Unit::Angle).range(0.0, 180.0),
            |args, a| args.angle = a,
        );
        f.formula(
            "length",
            Number::formula("length", &args.length, inputs, Unit::Length).range(0.0, 5.0),
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
        f.select(
            "corner",
            "corner",
            match args.corner {
                Corner::Open => "open",
                Corner::Closed => "closed",
            },
            vec![Choice::new("open", "Open"), Choice::new("closed", "Closed")],
            false,
            |args, choice| {
                args.corner = match choice {
                    "closed" => Corner::Closed,
                    _ => Corner::Open,
                }
            },
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
        let (solid, mut sheet, (f, k, toward_b)) =
            Sheet::with_edge(&part, &args.edge, "a flange").with_context(ctx)?;
        let angle = args.angle.evaluate(&mut part).with_context(ctx)?;
        let length = args.length.evaluate(&mut part).with_context(ctx)?;
        let shape = args.shape(&sheet.rules, angle, length).with_context(ctx)?;
        add_flange(&mut sheet, &namer, f, k, toward_b, &shape).with_context(ctx)?;
        sheet
            .replace(&mut part, &solid, &namer.root())
            .with_context(ctx)?;
        Ok(part)
    }
}

/// What a flange is, worked out from the arguments of the operation that
/// adds it: how far it turns and at what inner radius, how far its bend is
/// set back into the sheet (`None`: it starts at the edge), how long its
/// flat is, how far in from the edge's ends it runs, and what it does at a
/// corner.
pub(crate) struct FlangeShape<S: Scalar> {
    pub angle: S,
    pub radius: S,
    pub set_back: Option<S>,
    pub flat: S,
    pub offset_start: f64,
    pub offset_end: f64,
    pub corner: Corner,
}

/// Checks that the offsets from the edge's ends are not negative.
pub(crate) fn check_offsets(offset_start: f64, offset_end: f64) -> GeopResult<()> {
    for (what, offset) in [("start", offset_start), ("end", offset_end)] {
        if !(offset.is_finite() && offset >= 0.0) {
            return Err(GeopError::new(format!(
                "the offset at the {what} must not be negative, not {offset}"
            )));
        }
    }
    Ok(())
}

impl EdgeFlangeArgs {
    /// The flange these arguments describe, on a body of `rules`, turning
    /// `angle` degrees and `length` long — the values its formulas came to.
    fn shape<S: Scalar>(
        &self,
        rules: &SheetMetalRules,
        angle: f64,
        length: f64,
    ) -> GeopResult<FlangeShape<S>> {
        if !(angle > 0.0 && angle < 180.0) {
            return Err(GeopError::new(format!(
                "a flange turns by more than 0 and less than 180 degrees, not {angle}"
            )));
        }
        let radius = self.radius.unwrap_or(rules.bend_radius);
        if !(radius.is_finite() && radius > 0.0) {
            return Err(GeopError::new(format!(
                "the bend radius must be positive, not {radius}"
            )));
        }
        check_offsets(self.offset_start, self.offset_end)?;
        let s = S::from_f64;
        let t = s(rules.thickness);
        let r = s(radius);
        let angle = s(angle).mul(S::PI).div(s(180.0))?;
        let half = angle.div(S::TWO)?;
        let half_tan = half.sin().div(half.cos())?;
        let set_back = match self.position {
            FlangePosition::MaterialInside => Some(r.add(t).mul(half_tan)),
            FlangePosition::MaterialOutside => Some(r.mul(half_tan)),
            FlangePosition::BendOutside => None,
        };
        let flat = s(length).sub(match self.reference {
            LengthReference::OuterSharp => r.add(t).mul(half_tan),
            LengthReference::InnerSharp => r.mul(half_tan),
            LengthReference::Tangent => S::ZERO,
        });
        if !flat.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "a flange {length} long leaves nothing flat after its bend ({flat:?})"
            )));
        }
        Ok(FlangeShape {
            angle,
            radius: r,
            set_back,
            flat,
            offset_start: self.offset_start,
            offset_end: self.offset_end,
            corner: self.corner,
        })
    }
}

/// Adds the flange `shape` describes to `sheet`, bent from edge `k` of flat
/// `f`'s outline towards its B side if `toward_b` (see [`EdgeFlange`]).
pub(crate) fn add_flange<S: Scalar>(
    sheet: &mut Sheet<S>,
    namer: &Namer,
    f: usize,
    k: usize,
    toward_b: bool,
    shape: &FlangeShape<S>,
) -> GeopResult<()> {
    let rules = sheet.rules.clone();
    let flat = &sheet.flats[f];
    let edge = &flat.outer[k];
    let name = edge.key();
    let [a, b] = edge.line().ok_or_else(|| {
        GeopError::new(format!(
            "edge {name} is not straight: a flange is bent along a straight edge"
        ))
    })??;
    if sheet.is_bent(&name) {
        return Err(GeopError::new(format!("edge {name} is already bent")));
    }
    if sheet.layout()?.changed.contains(&name) {
        return Err(GeopError::new(format!(
            "edge {name} has been cut: a flange is bent along an edge as a flange or base flange built it — flange before cutting"
        )));
    }
    let s = S::from_f64;
    let t = s(rules.thickness);
    let (angle, r, set_back, length) = (shape.angle, shape.radius, shape.set_back, shape.flat);
    let n = flat.outer.len();
    let (prev, next) = ((k + n - 1) % n, (k + 1) % n);

    // Along the edge `s`, into the flat `h`.
    let edge_length = b.sub(&a).norm();
    let tau = b.sub(&a).normalize()?;
    let inward = Vector2::from_array([tau[1].neg(), tau[0]]);
    let at = |along: S, into: S| {
        a.add(&tau.prod_scalar(along))
            .add(&inward.prod_scalar(into))
    };
    let d = set_back.unwrap_or(S::ZERO);
    let relief = rules.relief_size().map(s);
    let depth = relief.map(|w| d.add(w));

    // Each end: how far in from the corner the flange starts if a bend
    // meets it there, and how far its flat reaches on past its bend if it
    // closes the corner.
    let corner = |neighbour: usize, end: &str, offset: f64| -> GeopResult<(f64, Option<S>)> {
        let key = flat.outer[neighbour].key();
        if !sheet.is_bent(&key) {
            return Ok((0.0, None));
        }
        if rules.corner_gap <= 0.0 {
            return Err(GeopError::new(format!(
                "edge {name} meets the bent edge {key} at its {end}, and the corner gap is 0: the two bends would touch"
            )));
        }
        if shape.corner == Corner::Open || offset > 0.0 {
            return Ok((rules.corner_gap, None));
        }
        let refuse = |why: &str| {
            GeopError::new(format!(
                "a closed corner at the {end} of edge {name}, where it meets {key}: {why} — close only corners of two flanges at right angles, turning the same way, of one radius, on edges at right angles"
            ))
        };
        let other = sheet
            .bends
            .iter()
            .find(|b| b.parent == f && b.parent_edge == key)
            .ok_or_else(|| refuse("that edge is where a bend ends, not a flange beside it"))?;
        let right = S::PI.div(S::TWO)?;
        if !(other.angle.could_be_equal(right) && angle.could_be_equal(right)) {
            return Err(refuse("the flanges do not both turn by 90 degrees"));
        }
        if other.toward_b != toward_b {
            return Err(refuse("the flanges turn opposite ways"));
        }
        if !other.radius.could_be_equal(r) {
            return Err(refuse("the bends' radii differ"));
        }
        let [p, q] = flat.outer[neighbour]
            .line()
            .ok_or_else(|| refuse("that edge is not straight"))??;
        if !q
            .sub(&p)
            .normalize()?
            .prod_dot(&b.sub(&a).normalize()?)
            .could_be_equal(S::ZERO)
        {
            return Err(refuse("the edges do not meet at a right angle"));
        }
        // The other flange's inside lies its radius short of where this
        // bend starts, less the gap: however each is set back.
        Ok((rules.corner_gap, Some(other.radius)))
    };
    let (gap_start, close_start) = corner(prev, "start", shape.offset_start)?;
    let (gap_end, close_end) = corner(next, "end", shape.offset_end)?;
    let inset_start = shape.offset_start + gap_start;
    let inset_end = shape.offset_end + gap_end;
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
    let height = if notched {
        depth.expect("a relief has a depth")
    } else {
        d
    };
    if height.definitely_greater(S::ZERO) {
        let w = if notched {
            relief.expect("a relief")
        } else {
            S::ZERO
        };
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
        if let (Some(w), Some(depth)) = (relief, depth) {
            fits(s0.sub(w), &name, "start")?;
            let relief = namer.scoped("relief0");
            run.push((at(s0.sub(w), S::ZERO), relief.scoped("in")));
            run.push((at(s0.sub(w), depth), relief.scoped("across")));
            run.push((at(s0, depth), relief.scoped("out")));
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
    // Where a closed corner carries the flat on past the bend, along the
    // line it starts at.
    let e_a = close_start.map_or(c_a, |x| c_a.sub(&tau.prod_scalar(x)));
    let e_b = close_end.map_or(c_b, |x| c_b.add(&tau.prod_scalar(x)));
    let mut outer = vec![straight(c_b, c_a, flange.scoped("line"))?];
    if close_start.is_some() {
        outer.push(straight(c_a, e_a, flange.scoped("corner0"))?);
    }
    outer.push(straight(e_a, e_a.add(&out), flange.scoped("side0"))?);
    outer.push(straight(
        e_a.add(&out),
        e_b.add(&out),
        flange.scoped("end"),
    )?);
    outer.push(straight(e_b.add(&out), e_b, flange.scoped("side1"))?);
    if close_end.is_some() {
        outer.push(straight(e_b, c_b, flange.scoped("corner1"))?);
    }
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
    if !along
        .prod_dot(&b.sub(&a).normalize()?)
        .could_be_equal(S::ZERO)
    {
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
