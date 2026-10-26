//! [`Loft`]: a body through the profiles of several sketches.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    with_context,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_core_sketch::{PointId, Shape};
use geop_ops_model::sketch::{PlacedSketch, SketchRef, chain_of_pieces};
use geop_ops_model::{ModelOperation, NamedModel, Namer, Point, build_model, read_model};
use serde::{Deserialize, Serialize};

use geop_core_math::vector::Vector3;
use geop_lib_sweep::{Apex, Section, loft, mark};

use crate::sweeping::{Output, PathRef};

/// Lofts through the profiles of two or more sketches, in order, into a
/// solid named `loft(L)` for the operation `L` — each sketch's one area, a
/// single loop — kept as a new body or combined with another solid (see
/// [`Combine`](geop_ops_model::Combine)). Or, as faces ([`Output::Faces`]), into faces standing on
/// their own: through the sketches' loops, or through open chains of curves
/// — between two curves, the ruled surface joining them.
///
/// Two profiles are joined by ruled walls, more by walls running smoothly
/// through them all (see [`geop_lib_sweep::loft`]). A sketch of a single
/// point, first or last, is a point the loft comes to: the walls run into
/// it, as a cone into its apex, the vertex `loft(L,K)` for the sketch `K`. The profiles' corners
/// — the sketch points where their curves meet at an angle — are lofted
/// into each other, in order round the loops, so every profile with
/// corners needs as many: a D and a triangle are refused, naming both. A
/// profile without corners, a circle, is lined up as a whole. Between two
/// corners, the curves are cut alike, each where another profile has a
/// joint at the same fraction of the way along: a piece `X` cut gives
/// `X#cut1`, `X#cut2`, ..., with the joints `X@cut1`, .... Sketch points on
/// the profiles' loops picked among the [`LoftArgs::matches`] say instead
/// which points are lofted into each other: each profile's first with the
/// others' first, its second with their second, and so on — the curve such
/// a point lies on split there, its second half named `X#P`.
///
/// Or guide curves ([`LoftArgs::guides`]) say which points match: sketches
/// whose one chain of curves runs from a point of the first profile through
/// every other to the last. Each guide's point on each profile is matched,
/// as a joint `K,G` of the profile `K` for the guide `G`, and between
/// the profiles the walls follow the guides (see [`geop_lib_sweep::loft`]) rather
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
    pub profiles: Vec<SketchRef>,
    /// Points that correspond across the profiles: sketch points on the
    /// profiles' loops — corners, or points along curves — each profile's
    /// in the order picked, the `k`-th of every profile lofted into each
    /// other.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub matches: Vec<ElementReference<PlacedSketch, Point>>,
    /// Sketches or 3-D sketches whose curves guide the loft from the first
    /// profile to the last — at most three — instead of matching points.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub guides: Vec<PathRef>,
    /// A solid, kept as a new body or combined with another, or faces.
    #[serde(default)]
    pub output: Output,
}

part_operation!(Loft(LoftArgs));

impl PartOperation for Loft {
    const KIND: &'static str = "loft";
    const INFO: OperationInfo = OperationInfo::new(
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
    fn new_args(context: &Context) -> LoftArgs {
        let before = &context.before;
        let sketches = before.targets_of::<PlacedSketch>();
        LoftArgs {
            profiles: (sketches
                .get(sketches.len().saturating_sub(2)..)
                .unwrap_or_default())
            .iter()
            .cloned()
            .map(SketchRef::whole)
            .collect(),
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::new_for(&read_model(before).unwrap_or_default()),
        }
    }

    fn run(ctx: &mut StepContext, args: &LoftArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }
    /// The sketches, picked in order; the points matched on them, or the
    /// guide curves; whether a face; and how to combine.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &LoftArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, LoftArgs> {
        let before = &context.before;
        let mut f = EditForm::<LoftArgs>::default();
        f.pick(
            "profiles",
            "profiles",
            before,
            &args.profiles,
            true,
            |edit, profiles| edit.args.profiles = profiles,
        )
        .pick(
            "matches",
            "matching points",
            before,
            &args.matches,
            true,
            |edit, matches| edit.args.matches = matches,
        )
        .pick(
            "guides",
            "guide curves",
            before,
            &args.guides,
            true,
            |edit, guides| edit.args.guides = guides,
        );
        f.form.dialog.optional("matches");
        f.form.dialog.optional("guides");
        let output = |args: &mut LoftArgs, output| args.output = output;
        (args.output).fields(&mut f, before, false, output);
        f
    }
}

impl ModelOperation for Loft {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &LoftArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("loft", step)?;
        // A sketch of a single point, first or last, is a point the loft
        // comes to.
        let last = args.profiles.len().saturating_sub(1);
        let mut apexes = [None, None];
        let mut sections = Vec::new();
        for (k, profile) in args.profiles.iter().enumerate() {
            let placed = reader.retrieve_element(profile)?;
            if let Some(point) = single_point(placed)? {
                let end = match k {
                    0 => 0,
                    k if k == last => 1,
                    _ => {
                        return Err(GeopError::new(format!(
                            "loft: the sketch {:?} is a point, between profiles: a loft comes to a point only at its start or end",
                            placed.name
                        )));
                    }
                };
                apexes[end] = Some(Apex {
                    point,
                    name: placed.name.to_string(),
                });
                continue;
            }
            sections.push(section(
                reader,
                profile,
                &args.output,
                &matched_points(args, profile)?,
            )?);
        }
        let guides = (args.guides.iter())
            .map(|guide| {
                if args.profiles.iter().any(|profile| guide.is_sketch(profile)) {
                    return Err(GeopError::new(format!(
                        "loft: the guide {guide:?} is one of the profiles: a guide is a curve of its own"
                    )));
                }
                guide.resolve(reader)
            })
            .collect::<GeopResult<Vec<_>>>()?;
        (args.output).build_whole(model, &namer, step, |model, name| {
            loft(model, &namer, name, &sections, &apexes, &guides)
        })
    }
}

/// The points of the profile `profile` among `args.matches`, in the order
/// picked: an error for a point of a sketch that is no profile.
fn matched_points(args: &LoftArgs, profile: &SketchRef) -> GeopResult<Vec<PointId>> {
    let mut found = Vec::new();
    for point in &args.matches {
        let sketch = point.target();
        if !args.profiles.iter().any(|p| p.target() == sketch) {
            return Err(GeopError::new(format!(
                "loft: the matching point {} is of the sketch {:?}, which is no profile",
                point.name(),
                sketch.name()
            )));
        }
        if sketch == profile.target() {
            found.push(point.name().parse()?);
        }
    }
    Ok(found)
}

/// Where the sketch `placed` is a single point and draws nothing else, the
/// point in space; none for a sketch that draws curves.
fn single_point(placed: &PlacedSketch) -> GeopResult<Option<Vector3>> {
    let sketch = &placed.sketch;
    if sketch.curves.values().any(|c| !c.construction) || sketch.points.len() != 1 {
        return Ok(None);
    }
    let geometry = sketch.enclose()?;
    Ok(geometry
        .points
        .values()
        .next()
        .map(|at| placed.plane.uv_to_xyz(at)))
}

/// The sketch `profile`, as `reader` reads it, as a section: its one loop
/// — or, lofted into faces, its one open chain if it encloses nothing (see
/// [`Output::shape`]) — with a joint at each of its points `matched`.
fn section(
    reader: &Reader,
    profile: &SketchRef,
    output: &Output,
    matched: &[PointId],
) -> GeopResult<Section> {
    let ctx = with_context!("profile {:?}", profile.target().name());
    let placed = reader.retrieve_element(profile).with_context(ctx)?;
    let name = &placed.name;
    let sketch = &placed.sketch;
    let geometry = sketch.enclose().with_context(ctx)?;
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
    let mut profile = chain_of_pieces(
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
