//! [`interference_report`]: which solids a part shows overlap — its own
//! with each other, with those of the parts placed in it, and those
//! between each other — by how much, and which only touch.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_core_topology::{
    SolidId,
    distance::{Feature, PlacedFace, closest_points},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use serde::Serialize;

use crate::{
    Bounded,
    bodies::{PlacedSolid, apart, hull, placed_solids},
};

/// How two solids meet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Contact {
    /// They share a volume.
    Overlap,
    /// They meet, along a face, an edge or at a point, sharing no volume.
    Touch,
}

/// Two solids that meet, named as the part shown names them.
#[derive(Clone, Debug, Serialize)]
pub struct Interference {
    pub a: String,
    pub b: String,
    pub contact: Contact,
    /// The volume they share, in mm³, for an overlap.
    pub volume: Option<Bounded>,
}

/// A pair that could not be checked, and why.
#[derive(Clone, Debug, Serialize)]
pub struct Unchecked {
    pub a: String,
    pub b: String,
    pub error: String,
}

/// Every pair of solids a part shows that overlap or touch; those apart
/// are not listed.
#[derive(Clone, Debug, Serialize)]
pub struct InterferenceReport {
    /// How many solids there are, and so pairs: `n (n - 1) / 2`.
    pub solids: usize,
    pub found: Vec<Interference>,
    pub unchecked: Vec<Unchecked>,
}

/// Which solids of `part` — its own, and those of the parts placed in it,
/// where they are — overlap, and by how much, and which only touch.
///
/// A pair whose boxes are definitely apart is apart. Any other pair is
/// intersected, as copies in a part of their own: a solid left is their
/// overlap, measured by its volume. Nothing left, the two touch if the
/// least distance between their faces could be zero (see
/// [`closest_points`]), and are apart otherwise.
pub fn interference_report<S: Scalar>(part: &Part<S>) -> GeopResult<InterferenceReport> {
    let solids = placed_solids(part)?;
    let boxes = solids
        .iter()
        .map(|s| s.bounding_box())
        .collect::<GeopResult<Vec<_>>>()?;
    let mut found = Vec::new();
    let mut unchecked = Vec::new();
    for i in 0..solids.len() {
        for j in i + 1..solids.len() {
            if apart(&boxes[i], &boxes[j]) {
                continue;
            }
            let (a, b) = (&solids[i], &solids[j]);
            match check_pair(a, b) {
                Ok(Some((contact, volume))) => found.push(Interference {
                    a: a.name.clone(),
                    b: b.name.clone(),
                    contact,
                    volume,
                }),
                Ok(None) => {}
                Err(e) => unchecked.push(Unchecked {
                    a: a.name.clone(),
                    b: b.name.clone(),
                    error: e.to_string(),
                }),
            }
        }
    }
    Ok(InterferenceReport {
        solids: solids.len(),
        found,
        unchecked,
    })
}

/// How `a` and `b` meet, if they do, and the volume they share.
#[allow(clippy::type_complexity)]
fn check_pair<S: Scalar>(
    a: &PlacedSolid<S>,
    b: &PlacedSolid<S>,
) -> GeopResult<Option<(Contact, Option<Bounded>)>> {
    let ctx = |e: GeopError| e.with_context(format!("checking {} against {}", a.name, b.name));
    let mut scratch = Part::<S>::new();
    let copy_a = copy_solid(&mut scratch, a, "a").with_context(&ctx)?;
    let copy_b = copy_solid(&mut scratch, b, "b").with_context(&ctx)?;
    let namer = Namer::new("interference", "overlap").with_context(&ctx)?;
    let overlap = boolean(
        &mut scratch,
        &namer,
        copy_a,
        copy_b,
        BooleanOp::Intersection,
        RemeshParams::default(),
    )
    .with_context(&ctx)?;
    if let Some(solid) = overlap {
        let volume = scratch
            .topology()
            .mass_properties(solid, S::ONE)
            .with_context(&ctx)?
            .volume;
        return Ok(Some((Contact::Overlap, Some(Bounded::of(volume)))));
    }
    Ok(touch(a, b).with_context(&ctx)?.then_some((Contact::Touch, None)))
}

/// Whether some face of `a` could be at distance zero from some face of
/// `b` — of those pairs whose boxes are not definitely apart.
fn touch<S: Scalar>(a: &PlacedSolid<S>, b: &PlacedSolid<S>) -> GeopResult<bool> {
    let faces = |s: &PlacedSolid<'_, S>| -> GeopResult<Vec<_>> {
        let model = s.part.topology();
        model
            .solid_faces(s.solid)?
            .into_iter()
            .map(|face| {
                let surface = &model.get_face(face)?.surface;
                let points = surface
                    .control_points
                    .iter()
                    .map(|cp| s.place_control_point(cp))
                    .collect::<GeopResult<Vec<_>>>()?;
                Ok((face, hull(&points)))
            })
            .collect()
    };
    let (fa, fb) = (faces(a)?, faces(b)?);
    for (face_a, box_a) in &fa {
        for (face_b, box_b) in &fb {
            if apart(box_a, box_b) {
                continue;
            }
            let closest = closest_points(
                &Feature::Face(PlacedFace::new(a.part.topology(), *face_a, a.pose)?),
                &Feature::Face(PlacedFace::new(b.part.topology(), *face_b, b.pose)?),
            )?;
            if closest.distance.could_be_equal(S::ZERO) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// A copy of `solid` in `scratch`, where it is placed, each of its entities
/// named as it is behind `prefix`. Refuses a solid of more than one shell —
/// one with a void — which a copy does not yet carry over.
fn copy_solid<S: Scalar>(
    scratch: &mut Part<S>,
    solid: &PlacedSolid<S>,
    prefix: &str,
) -> GeopResult<SolidId> {
    let model = solid.part.topology();
    let shells = &model.get_solid(solid.solid)?.shells;
    if shells.len() != 1 {
        return Err(GeopError::new(format!(
            "the solid {} has {} shells — a void inside it — and interference is checked for \
             solids of one shell only",
            solid.name,
            shells.len()
        )));
    }
    let faces = model.solid_faces(solid.solid)?;
    let (spec, sources) = model.body_spec(&faces, true)?;
    let spec = match &solid.pose {
        Some(pose) => spec.placed(pose),
        None => spec,
    };
    let named = |id: geop_ops::RefId| -> GeopResult<String> {
        let name = solid
            .part
            .name_of(id)
            .ok_or_else(|| GeopError::new(format!("{id} of {} has no name", solid.name)))?;
        Ok(format!("{prefix}.{name}"))
    };
    let names = BodyNames {
        vertices: sources
            .vertices
            .iter()
            .map(|&v| named(v.into()))
            .collect::<GeopResult<_>>()?,
        edges: sources
            .edges
            .iter()
            .map(|&e| named(e.into()))
            .collect::<GeopResult<_>>()?,
        faces: sources
            .faces
            .iter()
            .map(|&f| named(f.into()))
            .collect::<GeopResult<_>>()?,
        solid: Some(prefix.to_string()),
    };
    let built = scratch.build_body(spec, names)?;
    built
        .solid
        .ok_or_else(|| GeopError::new(format!("the copy of {} is no solid", solid.name)))
}

