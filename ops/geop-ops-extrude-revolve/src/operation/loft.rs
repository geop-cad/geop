//! [`Loft`]: a body through the profiles of several sketches.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_sketch::Shape;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Tone},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

use super::extrude::sketch_profile;
use crate::loft::{Section, loft};

/// Lofts through the profiles of two or more sketches, in order, into a
/// solid named `loft(L)` for the operation `L` — each sketch's one area, a
/// single loop — kept as a new body or combined with another solid (see
/// [`Combine`]). Or, as a face ([`LoftArgs::face`]), into faces standing on
/// their own: through the sketches' loops, or through open chains of curves
/// — between two curves, the ruled surface joining them.
///
/// Consecutive profiles are joined by ruled walls (see [`crate::loft`]);
/// profiles of different numbers of curves are matched up by halving the
/// longest curves of the ones with fewer.
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
            face: false,
            combine: Combine::new_for(before),
        }
    }

    /// The sketches, picked in order; whether a face; and how to combine.
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
            .map(|name| section(&part, name, args.face))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        if args.face {
            loft(&mut part, &namer, None, &sections).with_context(ctx)?;
            return Ok(part);
        }
        let name = args.combine.built_name(&namer);
        let built = loft(&mut part, &namer, Some(&name), &sections).with_context(ctx)?;
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

/// The sketch `name` of `part` as a section: its one loop — or, for a face,
/// its one open chain if it encloses nothing.
fn section<S: Scalar>(part: &Part<S>, name: &str, face: bool) -> GeopResult<Section<S>> {
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
    Ok(Section {
        plane: placed.plane.clone(),
        profile: sketch_profile(
            name,
            lp.to_nurbs(sketch, &geometry).with_context(ctx)?,
            closed,
        ),
        name: name.to_string(),
    })
}
