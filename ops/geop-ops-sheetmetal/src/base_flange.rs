//! [`BaseFlange`]: the first piece of a sheet-metal body — a plate from a
//! sketch's area, or a bent strip from a chain of lines and arcs.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_sketch::{CurveKind, ProfileLoop, Shape};
use geop_ops::{
    Context, Library, Namer, Part, PlacedSketch,
    operation::{EntityRef, Operation, Role},
    parameters::Formula,
    ui::{Choice, Form, Number, Tone, Unit},
};
use geop_ops_extrude_revolve::operation::shape_loops;
use serde::{Deserialize, Serialize};

use crate::{
    sheet::{Bend, BendFrame, Flat, FlatEdge, Placement, Relief, Sheet, SheetMetalRules, straight},
    thicken::thicken,
};

/// Starts a sheet-metal body from a sketch, for the operation `B`: a solid
/// named `base_flange(B)`, with the sheet-metal `rules` every later flange
/// and the flat pattern of it follow.
///
/// - A sketch's **area** becomes a plate `thickness` thick, from the
///   sketch's plane along its normal (against it, if `flip`). Its faces are
///   `base_flange(B,plate,a)` on the sketch's plane and
///   `base_flange(B,plate,b)` across, and the edge of the area swept by
///   piece `X` of sketch `K` (`c3`, `c3#1`) gives the wall
///   `base_flange(B,K,X)` and the edges `base_flange(B,K,X,a)` /
///   `base_flange(B,K,X,b)` around it.
/// - A sketch's **chain** of lines and arcs becomes a bent strip, its lines
///   flat and every corner between two of them a bend of the inner radius
///   `bend_radius` — the lines are where the faces would meet if the bends
///   were sharp — every arc, tangent to its lines, a bend of its own radius.
///   The chain is the strip's A side, the material on its right (seen from
///   the sketch's normal; its left if `flip`), and it runs `depth` along the
///   sketch's normal. Line `c3` of sketch `K` gives the flat faces
///   `base_flange(B,K,c3,a)` / `(...,b)`, its edges at either end of the
///   strip `base_flange(B,K,c3,start)` / `(...,end)` and where it ends at
///   point `p2` `base_flange(B,K,c3,p2)`; the bend at corner `p2` is
///   `base_flange(B,K,p2,a)` / `(...,b)`, the one of arc `c4`
///   `base_flange(B,K,c4,...)`.
///
/// Every edge `E` named so has its wall `E` if it lies on the sheet's
/// outline, and the vertex it starts at is `E,v,a` / `E,v,b` (see
/// [`crate::thicken`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BaseFlange;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BaseFlangeArgs {
    /// The sketch: one area, or one chain of lines and arcs.
    pub sketch: String,
    pub rules: SheetMetalRules,
    /// How far a chain's strip runs along the sketch's normal: a number,
    /// or a formula of the part's parameters.
    pub depth: Formula,
    /// Put the material on the other side: of the sketch's plane, for an
    /// area; of the chain, for a chain.
    #[serde(default)]
    pub flip: bool,
}

impl Operation for BaseFlange {
    type Args = BaseFlangeArgs;
    type Session = ();

    /// The newest sketch, with the default rules, a unit deep.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> BaseFlangeArgs {
        BaseFlangeArgs {
            sketch: before.sketch_names().pop().unwrap_or_default(),
            rules: SheetMetalRules::default(),
            depth: Formula::Plain(1.0),
            flip: false,
        }
    }

    /// The sketch, picked; for a chain how deep; whether flipped; and the
    /// rules.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &BaseFlangeArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, BaseFlangeArgs> {
        let before = context.before;
        let mut f = Form::<S, BaseFlangeArgs>::new();
        if before.sketches().next().is_none() {
            f.text("sketch", "No sketch yet — add one first.", Tone::Hint);
        } else {
            let value = (!args.sketch.is_empty())
                .then(|| EntityRef::Sketch {
                    name: args.sketch.clone(),
                })
                .into_iter()
                .collect();
            f.reference(
                "sketch",
                "sketch",
                value,
                &[Role::Sketch],
                None,
                false,
                |edit, picked| {
                    edit.args.sketch = match picked.as_slice() {
                        [EntityRef::Sketch { name }] => name.clone(),
                        _ => String::new(),
                    }
                },
            );
        }
        let chain = before
            .sketch_id(&args.sketch)
            .and_then(|id| before.sketch(id))
            .and_then(|placed| placed.sketch.shape())
            .is_ok_and(|shape| matches!(shape, Shape::Chain(_)));
        if chain {
            f.formula(
                "depth",
                Number::formula("depth", &args.depth, before.inputs(), Unit::Length)
                    .range(0.0, 10.0),
                |args, d| args.depth = d,
            );
        }
        f.checkbox("flip", "flip side", args.flip, |args, b| args.flip = b);
        rules_fields(&mut f, &args.rules);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &BaseFlangeArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("base_flange({operation_id}, {args:?})");
        let namer = Namer::new("base_flange", operation_id)?;
        args.rules.check().with_context(ctx)?;
        let placed = part
            .sketch(part.sketch_id(&args.sketch).with_context(ctx)?)?
            .clone();
        let sheet = match placed.sketch.shape().with_context(ctx)? {
            Shape::Region(region) => {
                plate(&namer, &args.sketch, &placed, Shape::Region(region), args)
            }
            Shape::Chain(chain) => {
                let depth = args.depth.evaluate(&mut part).with_context(ctx)?;
                strip(&namer, &args.sketch, &placed, &chain, depth, args)
            }
        }
        .with_context(ctx)?;
        let solid = namer.root();
        thicken(
            &mut part,
            &sheet.folded().with_context(ctx)?,
            &solid,
            &|n| n,
        )
        .with_context(ctx)?;
        part.set_body_data(&solid, sheet).with_context(ctx)?;
        Ok(part)
    }
}

/// The fields of the sheet-metal rules.
fn rules_fields<'a, S: Scalar>(f: &mut Form<'a, S, BaseFlangeArgs>, rules: &SheetMetalRules) {
    f.number(
        "thickness",
        Number::new("thickness", rules.thickness, Unit::Length).range(0.0, 1.0),
        |args, t| args.rules.thickness = t,
    );
    f.number(
        "bend_radius",
        Number::new("bend radius", rules.bend_radius, Unit::Length).range(0.0, 1.0),
        |args, r| args.rules.bend_radius = r,
    );
    f.number(
        "k_factor",
        Number::new("K-factor", rules.k_factor, Unit::Fraction).range(0.0, 1.0),
        |args, k| args.rules.k_factor = k,
    );
    f.select(
        "relief",
        "relief",
        match rules.relief {
            Relief::Rectangular => "rectangular",
            Relief::Tear => "tear",
        },
        vec![
            Choice::new("rectangular", "Rectangular"),
            Choice::new("tear", "Tear"),
        ],
        false,
        |args, choice| {
            args.rules.relief = match choice {
                "tear" => Relief::Tear,
                _ => Relief::Rectangular,
            }
        },
    );
    if rules.relief == Relief::Rectangular {
        f.number(
            "relief_ratio",
            Number::new("relief ratio", rules.relief_ratio, Unit::Fraction).range(0.0, 2.0),
            |args, r| args.rules.relief_ratio = r,
        );
    }
    f.number(
        "corner_gap",
        Number::new("corner gap", rules.corner_gap, Unit::Length).range(0.0, 1.0),
        |args, g| args.rules.corner_gap = g,
    );
}

/// The sketch's area as one flat plate: on the sketch's plane, its B side
/// along the plane's normal — or, flipped, its B side on the plane and its
/// A side behind it.
fn plate<S: Scalar>(
    namer: &Namer,
    sketch_name: &str,
    placed: &PlacedSketch<S>,
    region: Shape,
    args: &BaseFlangeArgs,
) -> GeopResult<Sheet<S>> {
    let sketch = &placed.sketch;
    let geometry = sketch.enclose::<S>()?;
    let loops = shape_loops(sketch_name, sketch, &geometry, region)?;
    let plane = &placed.plane;
    let n = *plane.w();
    let origin = if args.flip {
        plane
            .origin()
            .sub(&n.prod_scalar(S::from_f64(args.rules.thickness)))
    } else {
        *plane.origin()
    };
    let place = Placement {
        origin,
        e1: *plane.u(),
        e2: *plane.v(),
        n,
    };
    let edges = |lp: &geop_ops_extrude_revolve::sweep::SweepLoop<S>| -> Vec<FlatEdge<S>> {
        lp.profile
            .curves
            .iter()
            .zip(&lp.profile.curve_names)
            .map(|(curve, name)| FlatEdge {
                curve: curve.clone(),
                name: namer.scoped(name),
            })
            .collect()
    };
    let mut loops = loops.iter().map(edges);
    let outer = loops
        .next()
        .ok_or_else(|| GeopError::new("the sketch has no area"))?;
    Ok(Sheet {
        rules: args.rules.clone(),
        flats: vec![Flat {
            place,
            outer,
            holes: loops.collect(),
            name: namer.scoped("plate"),
        }],
        bends: Vec::new(),
        cuts: Vec::new(),
    })
}

/// A piece of a chain, from `a` to `b` in sketch coordinates: a line, or
/// an arc turning by `sweep` (counter-clockwise if positive); named after
/// its curve, and its ends after their points.
struct Piece<S: Scalar> {
    a: Vector2<S>,
    b: Vector2<S>,
    sweep: Option<S>,
    name: String,
    start: String,
    end: String,
}

impl<S: Scalar> Piece<S> {
    fn reversed(self) -> Self {
        Self {
            a: self.b,
            b: self.a,
            sweep: self.sweep.map(S::neg),
            name: self.name,
            start: self.end,
            end: self.start,
        }
    }
}

/// How a strip turns from one line to the next: through a sharp corner
/// at a point, bent at the default radius, or along an arc.
struct Turn<S: Scalar> {
    name: String,
    angle: S,
    radius: S,
    toward_b: bool,
    /// How far each line's flat stops short of the corner.
    setback: S,
}

/// The sketch's chain of lines and arcs as a bent strip, `depth` deep (see
/// [`BaseFlange`]).
fn strip<S: Scalar>(
    namer: &Namer,
    sketch_name: &str,
    placed: &PlacedSketch<S>,
    chain: &ProfileLoop,
    depth: f64,
    args: &BaseFlangeArgs,
) -> GeopResult<Sheet<S>> {
    let sketch = &placed.sketch;
    let geometry = sketch.enclose::<S>()?;
    let mut pieces = Vec::new();
    for edge in &chain.edges {
        let curve = &sketch.curves[&edge.curve];
        let point = |id| {
            geometry
                .points
                .get(&id)
                .copied()
                .ok_or_else(|| GeopError::new(format!("the sketch has no point {id}")))
        };
        let (a, b, sweep) = match curve.kind {
            CurveKind::Line { start, end } => (start, end, None),
            CurveKind::Arc { start, end, .. } => (
                start,
                end,
                Some(geometry.params.get(&edge.curve).copied().ok_or_else(|| {
                    GeopError::new(format!("the sketch has no sweep for arc {}", edge.curve))
                })?),
            ),
            _ => {
                return Err(GeopError::new(format!(
                    "{} is neither a line nor an arc: a sheet-metal strip is drawn with lines, and arcs for its bends",
                    edge.curve
                )));
            }
        };
        let piece = Piece {
            a: point(a)?,
            b: point(b)?,
            sweep,
            name: edge.curve.to_string(),
            start: a.to_string(),
            end: b.to_string(),
        };
        pieces.push(if edge.reversed {
            piece.reversed()
        } else {
            piece
        });
    }
    if args.flip {
        pieces = pieces.into_iter().rev().map(Piece::reversed).collect();
    }
    if !(depth.is_finite() && depth > 0.0) {
        return Err(GeopError::new(format!(
            "the strip's depth must be positive, not {depth}"
        )));
    }

    let plane = &placed.plane;
    let w = *plane.w();
    let t = S::from_f64(args.rules.thickness);
    let direction = |p: &Piece<S>| -> GeopResult<Vector3<S>> {
        plane
            .u()
            .prod_scalar(p.b[0].sub(p.a[0]))
            .add(&plane.v().prod_scalar(p.b[1].sub(p.a[1])))
            .normalize()
    };
    // The lines, and the turn from each to the next.
    let mut lines: Vec<&Piece<S>> = Vec::new();
    let mut turns: Vec<Turn<S>> = Vec::new();
    let mut arc: Option<&Piece<S>> = None;
    for piece in &pieces {
        if piece.sweep.is_none() {
            if let Some(&last) = lines.last() {
                turns.push(turn(
                    last,
                    piece,
                    arc.take(),
                    &direction,
                    &w,
                    t,
                    &args.rules,
                )?);
            }
            lines.push(piece);
            continue;
        }
        if lines.is_empty() || arc.is_some() {
            return Err(GeopError::new(format!(
                "arc {} does not lie between two lines: a sheet-metal strip starts and ends flat, and bends from one line to the next",
                piece.name
            )));
        }
        arc = Some(piece);
    }
    if let Some(arc) = arc {
        return Err(GeopError::new(format!(
            "the strip ends in arc {}: a sheet-metal strip starts and ends flat",
            arc.name
        )));
    }

    let depth = S::from_f64(depth);
    let sketch_namer = namer.scoped(sketch_name);
    let mut flats: Vec<Flat<S>> = Vec::new();
    let mut bends: Vec<Bend<S>> = Vec::new();
    let mut x0 = S::ZERO;
    let mut place = {
        let e1 = direction(lines[0])?;
        Placement {
            origin: plane.uv_to_xyz(&lines[0].a),
            n: e1.prod_cross(&w),
            e1,
            e2: w,
        }
    };
    for (i, line) in lines.iter().enumerate() {
        let before = i.checked_sub(1).map_or(S::ZERO, |j| turns[j].setback);
        let after = turns.get(i).map_or(S::ZERO, |turn| turn.setback);
        let length = line.b.sub(&line.a).norm().sub(before).sub(after);
        if !length.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "line {} is too short for its bends: {:?} of it is left flat",
                line.name, length
            )));
        }
        let x1 = x0.add(length);
        let p = |x: S, y: S| Vector2::from_array([x, y]);
        let flat = sketch_namer.scoped(&line.name);
        let outer = vec![
            straight(p(x0, S::ZERO), p(x1, S::ZERO), flat.scoped("start"))?,
            straight(p(x1, S::ZERO), p(x1, depth), flat.scoped(&line.end))?,
            straight(p(x1, depth), p(x0, depth), flat.scoped("end"))?,
            straight(p(x0, depth), p(x0, S::ZERO), flat.scoped(&line.start))?,
        ];
        flats.push(Flat {
            place: place.clone(),
            outer,
            holes: Vec::new(),
            name: flat.clone(),
        });
        if let Some(turn) = turns.get(i) {
            let frame = BendFrame::new(
                &place,
                [p(x1, S::ZERO), p(x1, depth)],
                turn.angle,
                turn.radius,
                t,
                turn.toward_b,
            )?;
            let next = sketch_namer.scoped(&lines[i + 1].name);
            bends.push(Bend {
                parent: i,
                parent_edge: flat.scoped(&line.end).root(),
                child: i + 1,
                child_edge: next.scoped(&lines[i + 1].start).root(),
                angle: turn.angle,
                radius: turn.radius,
                toward_b: turn.toward_b,
                name: sketch_namer.scoped(&turn.name),
            });
            place = frame.child_placement(&place);
        }
        x0 = x1;
    }
    Ok(Sheet {
        rules: args.rules.clone(),
        flats,
        bends,
        cuts: Vec::new(),
    })
}

/// How the strip turns from line `from` to line `to`: along `arc`, which
/// must be tangent to both, or else through the sharp corner between them,
/// bent at the default radius.
fn turn<S: Scalar>(
    from: &Piece<S>,
    to: &Piece<S>,
    arc: Option<&Piece<S>>,
    direction: &dyn Fn(&Piece<S>) -> GeopResult<Vector3<S>>,
    w: &Vector3<S>,
    t: S,
    rules: &SheetMetalRules,
) -> GeopResult<Turn<S>> {
    let (d0, d1) = (direction(from)?, direction(to)?);
    let n = d0.prod_cross(w);
    let side = d1.prod_dot(&n);
    let toward_b = if side.definitely_greater(S::ZERO) {
        true
    } else if side.definitely_less(S::ZERO) {
        false
    } else {
        return Err(GeopError::new(format!(
            "lines {} and {} could run straight on: join them into one line, or draw a corner between them",
            from.name, to.name
        )));
    };
    let cos = d0.prod_dot(&d1);
    if !cos.definitely_greater(S::ONE.neg()) {
        return Err(GeopError::new(format!(
            "line {} could fold straight back onto line {}: a bend turns by less than 180 degrees",
            to.name, from.name
        )));
    }
    let Some(arc) = arc else {
        let angle = cos.acos()?;
        let radius = S::from_f64(rules.bend_radius);
        let outer = if toward_b { radius.add(t) } else { radius };
        let half = angle.div(S::TWO)?;
        return Ok(Turn {
            name: from.end.clone(),
            angle,
            radius,
            toward_b,
            setback: outer.mul(half.sin().div(half.cos())?),
        });
    };
    // An arc tangent to both lines: it starts where the first ends, along
    // it, and ends where the next starts, along that.
    let sweep = arc.sweep.expect("an arc has a sweep");
    let angle = sweep.abs();
    let half = sweep.div(S::TWO)?;
    let chord = arc.b.sub(&arc.a);
    let r_a = chord.norm().div(S::TWO.mul(half.sin().abs()))?;
    let chord = chord.normalize()?;
    let turned = |by: S| {
        let (c, s) = (by.cos(), by.sin());
        Vector2::from_array([
            chord[0].mul(c).sub(chord[1].mul(s)),
            chord[0].mul(s).add(chord[1].mul(c)),
        ])
    };
    let along = |d: Vector2<S>, line: &Piece<S>| -> GeopResult<bool> {
        let l = line.b.sub(&line.a).normalize()?;
        Ok(d.prod_cross(&l).could_be_equal(S::ZERO) && d.prod_dot(&l).definitely_greater(S::ZERO))
    };
    if !(from.b.could_be_equal(&arc.a) && arc.b.could_be_equal(&to.a)) {
        return Err(GeopError::new(format!(
            "arc {} does not join lines {} and {}",
            arc.name, from.name, to.name
        )));
    }
    if !(along(turned(half.neg()), from)? && along(turned(half), to)?) {
        return Err(GeopError::new(format!(
            "arc {} is not tangent to lines {} and {}: a bend runs on smoothly from one flat to the next",
            arc.name, from.name, to.name
        )));
    }
    let radius = if toward_b { r_a.sub(t) } else { r_a };
    if !radius.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "arc {} has a radius of {:?}, no more than the thickness on the inside of its bend",
            arc.name, r_a
        )));
    }
    Ok(Turn {
        name: arc.name.clone(),
        angle,
        radius,
        toward_b,
        setback: S::ZERO,
    })
}
