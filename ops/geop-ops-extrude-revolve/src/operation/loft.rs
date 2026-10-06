//! [`Loft`]: a body through the profiles of several sketches.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::{PointId, Shape};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Tone},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

use super::{extrude::sketch_profile, paths_field};
use crate::loft::{Section, loft, mark};

/// Lofts through the profiles of two or more sketches, in order, into a
/// solid named `loft(L)` for the operation `L` — each sketch's one area, a
/// single loop — kept as a new body or combined with another solid (see
/// [`Combine`]). Or, as a face ([`LoftArgs::face`]), into faces standing on
/// their own: through the sketches' loops, or through open chains of curves
/// — between two curves, the ruled surface joining them.
///
/// Two profiles are joined by ruled walls, more by walls running smoothly
/// through them all (see [`crate::loft`]);
/// profiles of different numbers of curves are matched up by halving the
/// longest curves of the ones with fewer. Sketch points on the profiles'
/// loops picked among the [`LoftArgs::matches`] are lofted into each other:
/// each profile's first with the others' first, its second with their
/// second, and so on — the curve such a point lies on split there, its
/// second half named `X#P`.
///
/// Or guide curves ([`LoftArgs::guides`]) say which points match: sketches
/// whose one chain of curves runs from a point of the first profile through
/// every other to the last. Each guide's point on each profile is matched,
/// as a joint `K,G` of the profile `K` for the guide `G`, and between
/// the profiles the walls follow the guides (see [`crate::loft`]) rather
/// than run straight.
///
/// Named after the first sketch's elements — `X` a piece of a curve and `P`
/// a joint of the sketch `K` — and the sketches `K`, `M`, ... themselves:
///
/// - `loft(L,K,X)`: the wall from `X` — `loft(L,K,X,M>N)` for the one
///   between the sketches `M` and `N`, when there are more than two;
/// - `loft(L,K,X,M)`: the curve of the sketch `M` the wall `X` meets there;
/// - `loft(L,K,P)` (or `loft(L,K,P,M>N)`) / `loft(L,K,P,M)`: the edge from
///   `P` between two profiles, and its vertex on `M`;
/// - `loft(L,start)` / `loft(L,end)`: the caps, the first and last profile.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Loft;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoftArgs {
    /// The sketches to loft through, in order.
    pub profiles: Vec<String>,
    /// Points that correspond across the profiles: sketch points on the
    /// profiles' loops — corners, or points along curves — each profile's
    /// in the order picked, the `k`-th of every profile lofted into each
    /// other.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub matches: Vec<EntityRef>,
    /// Sketches or 3-D sketches whose curves guide the loft from the first
    /// profile to the last — at most three — instead of matching points.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub guides: Vec<EntityRef>,
    /// Loft into faces standing on their own, rather than a solid.
    #[serde(default)]
    pub face: bool,
    /// Keep the solid as a new body, or combine it with another solid.
    #[serde(default)]
    pub combine: Combine,
}

impl Operation for Loft {
    type Args = LoftArgs;
    type Session = ();

    /// Through the two newest sketches, joined to the newest solid if there
    /// is one.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> LoftArgs {
        let sketches = before.sketch_names();
        LoftArgs {
            profiles: sketches[sketches.len().saturating_sub(2)..].to_vec(),
            matches: Vec::new(),
            guides: Vec::new(),
            face: false,
            combine: Combine::new_for(before),
        }
    }

    /// The sketches, picked in order; the points matched on them, or the
    /// guide curves; whether a face; and how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &LoftArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, LoftArgs> {
        let before = context.before;
        let mut f = Form::<S, LoftArgs>::new();
        if before.sketches().next().is_none() {
            f.text("profiles", "No sketch yet — add two first.", Tone::Hint);
        } else {
            let value = args
                .profiles
                .iter()
                .map(|name| EntityRef::Sketch { name: name.clone() })
                .collect();
            f.reference(
                "profiles",
                "profiles",
                value,
                &[Role::Sketch],
                None,
                true,
                |edit, picked| {
                    edit.args.profiles = picked
                        .into_iter()
                        .filter_map(|entity| match entity {
                            EntityRef::Sketch { name } => Some(name),
                            _ => None,
                        })
                        .collect();
                },
            );
            f.reference(
                "matches",
                "matching points",
                args.matches.clone(),
                &[Role::Point],
                None,
                true,
                |edit, picked| {
                    edit.args.matches = picked
                        .into_iter()
                        .filter(|entity| matches!(entity, EntityRef::SketchPoint { .. }))
                        .collect();
                },
            );
            f.optional("matches");
            paths_field(
                &mut f,
                before,
                "guides",
                "guide curves",
                &args.guides,
                |args, guides| args.guides = guides,
            );
        }
        f.checkbox("face", "face", args.face, |args, b| args.face = b);
        if !args.face {
            args.combine.show(&mut f, before, |args| &mut args.combine);
        }
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &LoftArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("loft({operation_id}, {args:?})");
        let namer = Namer::new("loft", operation_id)?;
        let sections = args
            .profiles
            .iter()
            .map(|name| section(&part, name, args.face, &matched_points(args, name)?))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        let guides = args
            .guides
            .iter()
            .map(|guide| {
                if let EntityRef::Sketch { name } = guide
                    && args.profiles.contains(name)
                {
                    return Err(GeopError::new(format!(
                        "loft: the guide {name:?} is one of the profiles: a guide is a curve of its own"
                    )));
                }
                guide.resolve_chain(&part)
            })
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        if args.face {
            loft(&mut part, &namer, None, &sections, &guides).with_context(ctx)?;
            return Ok(part);
        }
        let name = args.combine.built_name(&namer);
        let built = loft(&mut part, &namer, Some(&name), &sections, &guides).with_context(ctx)?;
        let tool = Tool {
            solid: built.solid.expect("lofted as a solid"),
            up_to_next: None,
            scope: None,
        };
        args.combine
            .apply(&mut part, &namer, operation_id, &[tool])
            .with_context(ctx)?;
        Ok(part)
    }
}

/// The points of the profile `name` among `args.matches`, in the order
/// picked: an error for a point of a sketch that is no profile.
fn matched_points(args: &LoftArgs, name: &str) -> GeopResult<Vec<PointId>> {
    let mut found = Vec::new();
    for entity in &args.matches {
        let EntityRef::SketchPoint { sketch, point } = entity else {
            return Err(GeopError::new(format!(
                "loft: a matching point has to be a sketch point, not {entity}"
            )));
        };
        if !args.profiles.contains(sketch) {
            return Err(GeopError::new(format!(
                "loft: the matching point {point} is of the sketch {sketch:?}, which is no profile"
            )));
        }
        if sketch == name {
            found.push(*point);
        }
    }
    Ok(found)
}

/// The sketch `name` of `part` as a section: its one loop — or, for a face,
/// its one open chain if it encloses nothing — with a joint at each of its
/// points `matched`.
fn section<S: Scalar>(
    part: &Part<S>,
    name: &str,
    face: bool,
    matched: &[PointId],
) -> GeopResult<Section<S>> {
    let ctx = with_context!("profile {name:?}");
    let placed = part.sketch(part.sketch_id(name).with_context(ctx)?)?;
    let sketch = &placed.sketch;
    let geometry = sketch.enclose::<S>().with_context(ctx)?;
    let shape = if face {
        sketch.shape()
    } else {
        sketch.region().map(Shape::Region)
    }
    .with_context(ctx)?;
    let (lp, closed) = match shape {
        Shape::Region(region) if region.holes.is_empty() => (region.outer, true),
        Shape::Region(_) => {
            return Err(GeopError::new(format!(
                "loft: the profile {name:?} has holes, but a loft runs through single loops"
            )));
        }
        Shape::Chain(chain) => (chain, false),
    };
    let mut profile = sketch_profile(
        name,
        lp.to_nurbs(sketch, &geometry).with_context(ctx)?,
        closed,
    );
    let mut joints = Vec::new();
    for &point in matched {
        let at = geometry.points.get(&point).ok_or_else(|| {
            GeopError::new(format!(
                "loft: the matching point {point} is no point of the sketch"
            ))
        })?;
        let (marked, joint) = mark(&profile, at, &format!("{name},{point}"))
            .with_context(with_context!("the matching point {point}"))
            .with_context(ctx)?;
        profile = marked;
        joints.push(joint);
    }
    Ok(Section {
        plane: placed.plane.clone(),
        profile,
        name: name.to_string(),
        matched: joints,
    })
}
