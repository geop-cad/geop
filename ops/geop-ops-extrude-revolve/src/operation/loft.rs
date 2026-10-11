//! [`Loft`]: a body through the profiles of several sketches.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_core_sketch::{PointId, Shape};
use geop_ops::operation::Input;
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{
        Build, BuildReader, EntityRef, Lookup, ModelStep, ResolveEntity, Role, Scope, Seen,
        build_model,
    },
};
use serde::{Deserialize, Serialize};

use super::{Output, extrude::sketch_profile, sketch_field};
use crate::loft::{Section, loft, mark};

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

impl Operation for Loft {
    const INFO: OperationInfo = OperationInfo::new(
        "loft",
        "Loft",
        "Build a solid through the areas of several sketches, or faces through their curves.",
        OperationGroup::Solid,
        OperationTier::Small,
    );
    type Args = LoftArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// Through the two newest sketches, joined to the newest solid if there
    /// is one.
    fn new_args(context: &Context<Input, ParamValue>) -> LoftArgs {
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

    fn run(step: &str, part: &mut Build, args: &LoftArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The sketches, picked in order; the points matched on them, or the
    /// guide curves; whether a face; and how to combine.
    fn presentation(context: &Context<Input, ParamValue>, args: &LoftArgs, _: &()) -> Form<Design> {
        let view = &context.before;
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        sketch_field(dialog, view, "profiles", &args.profiles, true);
        let matches = args.matches.clone();
        let guides = args.guides.clone();
        (dialog.reference(
            "matches",
            "matching points",
            matches,
            &[Role::Point],
            None,
            true,
        ))
        .optional("matches")
        .reference("guides", "guide curves", guides, &[Role::Path], None, true)
        .optional("guides");
        args.output.show(dialog, false);
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &LoftArgs,
        _: &(),
        event: Event<Design>,
    ) -> (LoftArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("profiles", Value::Entities(picked)) => {
                args.profiles = EntityRef::names_of(EntityRef::sketch, &picked);
            }
            ("matches", Value::Entities(picked)) => {
                args.matches = (picked.into_iter())
                    .filter(|entity| matches!(entity, EntityRef::SketchPoint { .. }))
                    .collect();
            }
            ("guides", Value::Entities(picked)) => args.guides = picked,
            (key, value) => {
                (args.output).set(key, &value, Seen::new(context.before).model());
            }
        }
        (args, ())
    }
}

impl ModelStep for Loft {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &LoftArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("loft({step}, {args:?})");
        let mut scope = Scope {
            model: &mut *model,
            reader,
        };
        let namer = Namer::new("loft", step)?;
        let sections = args
            .profiles
            .iter()
            .map(|name| section(&mut scope, name, &args.output, &matched_points(args, name)?))
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
                guide.resolve_chain(&mut scope)
            })
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        (args.output)
            .build_whole(model, &namer, step, |model, name| {
                loft(model, &namer, name, &sections, &guides)
            })
            .with_context(ctx)
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
