//! [`Loft`]: a body through the profiles of several sketches.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::{PointId, Shape};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{
        BuildReader, EntityRef, Lookup, PartOperation, ResolveEntity, Role, Scope, Seen,
        StepContext,
    },
    part_operation,
    ui::{EditForm, FormState},
};
use serde::{Deserialize, Serialize};

use super::Output;
use crate::{
    common::Profile,
    loft::{Section, loft, mark},
};

/// Lofts through the profiles of two or more sketches, in order, into a
/// solid named `loft(L)` for the operation `L` — each sketch's one area, a
/// single loop — kept as a new body or combined with another solid (see
/// [`Combine`](geop_ops_booleans::Combine)). Or, as faces ([`Output::Faces`]), into faces standing on
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
    /// A solid, kept as a new body or combined with another, or faces.
    #[serde(default)]
    pub output: Output,
}

part_operation!(Loft(LoftArgs));

impl PartOperation for Loft {
    const INFO: OperationInfo = OperationInfo::new(
        "loft",
        "Loft",
        "Build a solid through the areas of several sketches, or faces through their curves.",
        OperationGroup::Solid,
        OperationTier::Small,
    )
    .icon(crate::icons::LOFT);
    type Args = LoftArgs;
    type Session = ();

    /// Through the two newest sketches, joined to the newest solid if there
    /// is one.
    fn new_args(context: &StepContext) -> LoftArgs {
        let seen = Seen::new(context.before);
        let sketches = seen.sketch_names();
        LoftArgs {
            profiles: sketches
                .get(sketches.len().saturating_sub(2)..)
                .unwrap_or_default()
                .to_vec(),
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::new_for(seen.model()),
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &LoftArgs,
    ) -> GeopResult<()> {
        let mut scope = Scope {
            model: &mut *model,
            reader,
        };
        let namer = Namer::new("loft", step)?;
        let sections = args
            .profiles
            .iter()
            .map(|name| section(&mut scope, name, &args.output, &matched_points(args, name)?))
            .collect::<GeopResult<Vec<_>>>()?;
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
                guide.resolve_chain(&mut scope)
            })
            .collect::<GeopResult<Vec<_>>>()?;
        (args.output).build_whole(model, &namer, step, |model, name| {
            loft(model, &namer, name, &sections, &guides)
        })
    }
    /// The sketches, picked in order; the points matched on them, or the
    /// guide curves; whether a face; and how to combine.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &LoftArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, LoftArgs> {
        let view = &context.before;
        let mut f = EditForm::<Design, LoftArgs>::default();
        f.pick_sketches(view, "profiles", &args.profiles, |edit, names| {
            edit.args.profiles = names
        });
        let matches = args.matches.clone();
        let guides = args.guides.clone();
        let label = "matching points";
        f.reference(
            "matches",
            label,
            matches,
            &[Role::Point],
            None,
            true,
            |edit, picked| {
                edit.args.matches = (picked.into_iter())
                    .filter(|entity| matches!(entity, EntityRef::SketchPoint { .. }))
                    .collect();
            },
        );
        f.form.dialog.optional("matches");
        let label = "guide curves";
        f.reference(
            "guides",
            label,
            guides,
            &[Role::Path],
            None,
            true,
            |edit, picked| edit.args.guides = picked,
        );
        f.form.dialog.optional("guides");
        let output = |args: &mut LoftArgs, output| args.output = output;
        (args.output).fields(&mut f, Seen::new(context.before).model(), false, output);
        f
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

/// The sketch `name`, as `lookup` has it, as a section: its one loop — or,
/// lofted into faces, its one open chain if it encloses nothing (see
/// [`Output::shape`]) — with a joint at each of its points `matched`.
fn section<S: Scalar>(
    lookup: &mut (impl Lookup<S> + ?Sized),
    name: &str,
    output: &Output,
    matched: &[PointId],
) -> GeopResult<Section<S>> {
    let ctx = with_context!("profile {name:?}");
    let placed = lookup.sketch(name).with_context(ctx)?;
    let sketch = &placed.sketch;
    let geometry = sketch.enclose::<S>().with_context(ctx)?;
    let shape = output.shape(sketch).with_context(ctx)?;
    let (lp, closed) = match shape {
        Shape::Region(region) if region.holes.is_empty() => (region.outer, true),
        Shape::Region(_) => {
            return Err(GeopError::new(format!(
                "loft: the profile {name:?} has holes, but a loft runs through single loops"
            )));
        }
        Shape::Chain(chain) => (chain, false),
    };
    let mut profile = Profile::of_pieces(
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
