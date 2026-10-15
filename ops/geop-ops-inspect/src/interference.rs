//! [`interference_report`]: which solids a part shows overlap — its own
//! with each other, with those of the parts placed in it, and those
//! between each other — by how much, and which only touch.

use geop_core_editor::editing::Tone;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector, Vector3},
};
use geop_core_program::{element::PickedElement, reader::Reader, values::Design};
use geop_core_topology::{
    Body, SolidId,
    distance::{Feature, PlacedFace, closest_points},
};
use geop_ops_assembly::solids::{PlacedSolid, placed_solids};
use geop_ops_model::{BodyNames, NamedModel, Namer};
use geop_ops_output::{Answer, Block, Bounded, Row};

use crate::mass::drawn;
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use serde::Serialize;

/// An axis-aligned box: each coordinate the interval it spans.
pub type Box3 = Vector3<Design>;

/// The box of `points`: each coordinate the union of theirs.
fn hull(points: &[Vector3<Design>]) -> Box3 {
    points
        .iter()
        .copied()
        .reduce(|a, b| a.union(&b))
        .unwrap_or_else(Vector3::everything)
}

/// Whether two boxes are definitely apart along some axis.
fn apart(a: &Box3, b: &Box3) -> bool {
    (0..3).any(|k| {
        a[k].upper().definitely_less(b[k].lower()) || b[k].upper().definitely_less(a[k].lower())
    })
}

/// The control point `cp` of a surface of `solid`, as a point, where the
/// solid is placed.
fn place_control_point(solid: &PlacedSolid, cp: &Vector<Design, 4>) -> GeopResult<Box3> {
    let p = cp.dehomogenized()?;
    Ok(match &solid.pose {
        Some(pose) => pose.apply(&p),
        None => p,
    })
}

/// The box around the faces `faces` of `solid`: around their surfaces'
/// control points — each surface lies in their hull.
pub fn bounding_box(solid: &PlacedSolid, faces: &[geop_core_topology::FaceId]) -> GeopResult<Box3> {
    let model = solid.model.topology();
    let mut points = Vec::new();
    for &face in faces {
        for cp in &model.get_face(face)?.surface.control_points {
            points.push(place_control_point(solid, cp)?);
        }
    }
    Ok(hull(&points))
}

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
    /// What is drawn of the two, to light.
    #[serde(skip)]
    pub drawn: Vec<PickedElement>,
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

impl InterferenceReport {
    /// As the inspect panel shows it: each pair that meets, lit while
    /// pointed at, how — and each that could not be checked, why.
    pub fn shown(&self) -> Answer {
        let mut blocks = Vec::new();
        if self.found.is_empty() && self.unchecked.is_empty() {
            let plural = if self.solids == 1 { "" } else { "s" };
            blocks.push(Block::Text {
                text: format!("No interference among {} solid{plural}.", self.solids),
                tone: Tone::Hint,
            });
        }
        let mut items: Vec<Row> = (self.found.iter())
            .map(|f| {
                let (how, tone) = match f.contact {
                    Contact::Overlap => {
                        let volume = f.volume.map_or("?".into(), |v| v.show(3));
                        (format!("overlap {volume} mm³"), Tone::Error)
                    }
                    Contact::Touch => ("touch".into(), Tone::Normal),
                };
                Row {
                    light: f.drawn.clone(),
                    tone,
                    ..Row::of(vec![format!("{} × {}: {how}", f.a, f.b)])
                }
            })
            .collect();
        items.extend(self.unchecked.iter().map(|u| Row {
            detail: Some(u.error.clone()),
            tone: Tone::Hint,
            ..Row::of(vec![format!("{} × {}: could not be checked", u.a, u.b)])
        }));
        if !items.is_empty() {
            blocks.push(Block::List { items });
        }
        Answer {
            blocks,
            plane: None,
        }
    }
}

/// Which solids of the part `reader` reads — its own, and those of the parts
/// placed in it, where they are — overlap, and by how much, and which only
/// touch.
///
/// A pair whose boxes are definitely apart is apart. Any other pair is
/// intersected, as copies in a part of their own: a solid left is their
/// overlap, measured by its volume. Nothing left, the two touch if the
/// least distance between their faces could be zero (see
/// [`closest_points`]), and are apart otherwise.
pub fn interference_report(reader: &Reader) -> GeopResult<InterferenceReport> {
    let solids = placed_solids(reader)?;
    let boxes = (solids.iter())
        .map(|s| bounding_box(s, &s.model.topology().solid_faces(s.solid)?))
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
                    drawn: [drawn(a), drawn(b)].concat(),
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
fn check_pair(a: &PlacedSolid, b: &PlacedSolid) -> GeopResult<Option<(Contact, Option<Bounded>)>> {
    let ctx = |e: GeopError| e.with_context(format!("checking {} against {}", a.name, b.name));
    let mut scratch = NamedModel::<Design>::new();
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
            .mass_properties(solid)
            .with_context(&ctx)?
            .volume;
        return Ok(Some((Contact::Overlap, Some(Bounded::of(volume)))));
    }
    Ok(touch(a, b)
        .with_context(&ctx)?
        .then_some((Contact::Touch, None)))
}

/// Whether some face of `a` could be at distance zero from some face of
/// `b` — of those pairs whose boxes are not definitely apart.
fn touch(a: &PlacedSolid, b: &PlacedSolid) -> GeopResult<bool> {
    let faces = |s: &PlacedSolid| -> GeopResult<Vec<_>> {
        (s.model.topology().solid_faces(s.solid)?.into_iter())
            .map(|face| Ok((face, bounding_box(s, &[face])?)))
            .collect()
    };
    let (fa, fb) = (faces(a)?, faces(b)?);
    for (face_a, box_a) in &fa {
        for (face_b, box_b) in &fb {
            if apart(box_a, box_b) {
                continue;
            }
            let closest = closest_points(
                &Feature::Face(PlacedFace::new(a.model.topology(), *face_a, a.pose)?),
                &Feature::Face(PlacedFace::new(b.model.topology(), *face_b, b.pose)?),
            )?;
            if closest.distance.could_be_equal(Design::ZERO) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// A copy of `solid` in `scratch`, where it is placed, each of its entities
/// named as it is behind `prefix`. Refuses a solid of more than one shell —
/// one with a void — which the boolean is not asked to intersect.
fn copy_solid(
    scratch: &mut NamedModel<Design>,
    solid: &PlacedSolid,
    prefix: &str,
) -> GeopResult<SolidId> {
    let shells = &solid.model.topology().get_solid(solid.solid)?.shells;
    if shells.len() != 1 {
        return Err(GeopError::new(format!(
            "the solid {} has {} shells — a void inside it — and interference is checked for \
             solids of one shell only",
            solid.name,
            shells.len()
        )));
    }
    let (spec, names) = solid.model.body_record(Body::Solid(solid.solid))?;
    let spec = match &solid.pose {
        Some(pose) => spec.placed(pose),
        None => spec,
    };
    let names = BodyNames {
        solid: Some(prefix.to_string()),
        ..names.renamed(|name| format!("{prefix}.{name}"))
    };
    let built = scratch.build_body(spec, names)?;
    built
        .solid
        .ok_or_else(|| GeopError::new(format!("the copy of {} is no solid", solid.name)))
}
