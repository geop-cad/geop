//! [`clearance_report`]: which solids of a part come nearer each other than
//! a clearance — its own, and those of the parts placed in it — and the
//! least distance between them, where it is attained.
//!
//! The least distance between two solids is the least between their faces,
//! each pair's from the branch and bound of [`closest_points`]: an
//! enclosure, not the best of samples. Pairs whose boxes are further apart
//! than the clearance are never looked at (the sweep of
//! [`candidate_pairs`]), nor pairs of faces whose boxes are further apart
//! than the least distance found so far; and a pair of the same two shapes
//! at the same relative pose — every screw beside its nut — is measured
//! once, as interference checks it once.

use std::collections::HashMap;

use geop_core_editor::editing::{Shape as Drawn, Style, Tone, Visual};
use geop_core_editor::output::{Answer, AnswerAction, Block, Bounded, Does, Row};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::PickedElement, reader::Reader};
use geop_core_topology::distance::{Closest, Feature, PlacedFace, closest_points};
use geop_ops_model::placed::solids::{PlacedSolid, placed_solids};
use serde::{Deserialize, Serialize};

use crate::interference::{
    Box3, ByPose, LocalBoxes, LocalPoses, Shape, Work, bounding_box, candidate_pairs, corners, hull,
};
use crate::mass::drawn;

/// What a clearance check is told: the least distance two solids should
/// keep, in mm.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct ClearanceOptions {
    pub clearance: f64,
}

impl Default for ClearanceOptions {
    /// A millimetre.
    fn default() -> Self {
        Self { clearance: 1.0 }
    }
}

/// The clearances offered as buttons, in mm.
const OFFERED: [f64; 4] = [0.5, 1.0, 2.0, 5.0];

/// Two solids nearer each other than the clearance.
#[derive(Clone, Debug, Serialize)]
pub struct Near {
    pub a: String,
    pub b: String,
    /// The least distance between them, in mm: zero for two that touch or
    /// overlap.
    pub distance: Bounded,
    /// Where it is attained: a point on each, where they are placed.
    pub witness: [Vector3; 2],
    /// What is drawn of the two, to light.
    #[serde(skip)]
    pub drawn: Vec<PickedElement>,
}

/// A pair that could not be measured, and why.
#[derive(Clone, Debug, Serialize)]
pub struct Unmeasured {
    pub a: String,
    pub b: String,
    pub error: String,
}

/// Every pair of solids a part shows that is nearer than a clearance.
#[derive(Clone, Debug, Serialize)]
pub struct ClearanceReport {
    pub clearance: f64,
    pub solids: usize,
    /// The pairs nearer than the clearance, the nearest first.
    pub near: Vec<Near>,
    pub unmeasured: Vec<Unmeasured>,
    /// What finding them took: [`Work::booleans`] counts the pairs of
    /// solids measured.
    pub work: Work,
}

impl ClearanceReport {
    /// As the inspect panel shows it: the clearances to ask with, then each
    /// pair nearer, lit while pointed at, how near — and each that could
    /// not be measured, why.
    pub fn shown(&self) -> Answer {
        let actions = (OFFERED.iter())
            .map(|&clearance| AnswerAction {
                label: format!("{clearance} mm"),
                doc: format!("Every pair of solids nearer than {clearance} mm"),
                chosen: clearance == self.clearance,
                does: Does::Inspect {
                    options: serde_json::json!({ "clearance": clearance }),
                },
            })
            .collect();
        let mut blocks = vec![Block::Actions { actions }];
        if self.near.is_empty() && self.unmeasured.is_empty() {
            blocks.push(Block::Text {
                text: format!(
                    "No two of {} solids are nearer than {} mm.",
                    self.solids, self.clearance
                ),
                tone: Tone::Hint,
            });
        }
        let mut items: Vec<Row> = (self.near.iter())
            .map(|n| Row {
                light: n.drawn.clone(),
                tone: Tone::Error,
                ..Row::of(vec![format!(
                    "{} × {}: {} mm",
                    n.a,
                    n.b,
                    n.distance.show(3)
                )])
            })
            .collect();
        items.extend(self.unmeasured.iter().map(|u| Row {
            detail: Some(u.error.clone()),
            tone: Tone::Hint,
            ..Row::of(vec![format!("{} × {}: could not be measured", u.a, u.b)])
        }));
        if !items.is_empty() {
            blocks.push(Block::List { items });
        }
        Answer {
            blocks,
            plane: None,
        }
    }

    /// What it draws: a line between the points of each pair nearer than
    /// the clearance.
    pub fn visuals(&self) -> Vec<Visual> {
        (self.near.iter().enumerate())
            .map(|(k, near)| {
                let [a, b] = near.witness;
                Visual::new(
                    format!("clearance-{k}"),
                    Drawn::Polyline { points: vec![a, b] },
                    Style::Selected,
                )
            })
            .collect()
    }
}

/// Which solids of the part `reader` reads, its own and those of the parts
/// placed in it, are nearer each other than `options.clearance`, and how
/// near (see the module's docs).
pub fn clearance_report(reader: &Reader, options: ClearanceOptions) -> GeopResult<ClearanceReport> {
    let clearance = options.clearance;
    if !clearance.is_finite() || clearance < 0.0 {
        return Err(GeopError::new(format!(
            "a clearance of {clearance} mm: give a distance of zero or more"
        )));
    }
    let solids = placed_solids(reader)?;
    let mut local = LocalBoxes::default();
    // Half the clearance on every side: two boxes this far apart are as far
    // apart as the clearance.
    let half = Scalar::from_f64(clearance)
        .mul(Scalar::from_f64(0.5))
        .union(Scalar::ZERO);
    let pad = Vector3::from_array([half.neg().union(half); 3]);
    let boxes = (solids.iter())
        .map(|s| Ok(local.placed(s)?.add(&pad)))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut work = Work::default();
    let pairs = candidate_pairs(&boxes, &mut work.boxes_compared);
    let mut poses = LocalPoses::default();
    let mut measured: HashMap<(Shape, Shape), ByPose<Option<Closest>>> = HashMap::new();
    let mut near = Vec::new();
    let mut unmeasured = Vec::new();
    let limit = Scalar::from_f64(clearance);
    for (i, j) in pairs {
        work.candidates += 1;
        let (a, b) = (&solids[i], &solids[j]);
        let relative = poses.relative(reader, a, b)?;
        let seen = measured.entry((Shape::of(a), Shape::of(b))).or_default();
        let closest = match seen.iter().find(|(pose, _)| *pose == relative) {
            Some((_, closest)) => closest.clone(),
            None => {
                work.booleans += 1;
                let closest = least_distance(a, b, relative.as_ref(), limit).map_err(|e| {
                    e.with_context(format!("measuring {} against {}", a.name, b.name))
                        .to_string()
                });
                seen.push((relative, closest.clone()));
                closest
            }
        };
        match closest {
            Ok(Some(closest)) if closest.distance.lower().definitely_less(limit) => {
                // Measured in the frame of `a`: placed where `a` is.
                let place = |p: &Vector3| a.pose.as_ref().map_or(*p, |pose| pose.apply(p));
                near.push(Near {
                    a: a.name.clone(),
                    b: b.name.clone(),
                    distance: Bounded::of(closest.distance),
                    witness: [place(&closest.a), place(&closest.b)],
                    drawn: [drawn(a), drawn(b)].concat(),
                })
            }
            Ok(_) => {}
            Err(error) => unmeasured.push(Unmeasured {
                a: a.name.clone(),
                b: b.name.clone(),
                error,
            }),
        }
    }
    near.sort_by(|x, y| x.distance.value.total_cmp(&y.distance.value));
    Ok(ClearanceReport {
        clearance,
        solids: solids.len(),
        near,
        unmeasured,
        work,
    })
}

/// The least distance between the faces of `a`, where its part has it, and
/// of `b` placed at `relative` to it — none, if every pair of faces is
/// further apart than `limit`. Pairs of faces are measured nearest box first, and a pair whose
/// boxes are definitely further apart than the least distance found, or
/// than `limit`, is not measured.
fn least_distance(
    a: &PlacedSolid,
    b: &PlacedSolid,
    relative: Option<&Pose>,
    limit: Scalar,
) -> GeopResult<Option<Closest>> {
    let faces = |s: &PlacedSolid, pose: Option<&Pose>| -> GeopResult<Vec<_>> {
        let unplaced = PlacedSolid {
            pose: None,
            name: s.name.clone(),
            instance: s.instance.clone(),
            reader: s.reader.clone(),
            model: std::borrow::Cow::Borrowed(&*s.model),
            solid: s.solid,
        };
        (s.model.topology().solid_faces(s.solid)?.into_iter())
            .map(|face| {
                let own = bounding_box(&unplaced, &[face])?;
                let placed = match pose {
                    None => own,
                    Some(pose) => hull(&corners(&own).map(|c| pose.apply(&c))),
                };
                Ok((face, placed))
            })
            .collect()
    };
    let (fa, fb) = (faces(a, None)?, faces(b, relative)?);
    let mut pairs: Vec<(Scalar, usize, usize)> = Vec::new();
    for (i, (_, box_a)) in fa.iter().enumerate() {
        for (j, (_, box_b)) in fb.iter().enumerate() {
            pairs.push((box_gap(box_a, box_b), i, j));
        }
    }
    pairs.sort_by(|x, y| x.0.to_f64().total_cmp(&y.0.to_f64()));
    let mut best: Option<Closest> = None;
    for (gap, i, j) in pairs {
        let bound = best
            .as_ref()
            .map_or(limit, |b| b.distance.upper().min(limit));
        if gap.definitely_greater(bound) {
            continue;
        }
        let closest = closest_points(
            &Feature::Face(PlacedFace::new(a.model.topology(), fa[i].0, None)?),
            &Feature::Face(PlacedFace::new(
                b.model.topology(),
                fb[j].0,
                relative.copied(),
            )?),
        )?;
        let nearer = best
            .as_ref()
            .is_none_or(|b| (closest.distance.upper()).definitely_less(b.distance.upper()));
        if nearer {
            best = Some(closest);
        }
    }
    Ok(best)
}

/// A lower bound on the distance between two boxes: the length of the gap
/// between them along each axis, zero where they overlap.
fn box_gap(a: &Box3, b: &Box3) -> Scalar {
    let mut sum = Scalar::ZERO;
    for k in 0..3 {
        let gap = (b[k].lower().sub(a[k].upper()))
            .max(a[k].lower().sub(b[k].upper()))
            .max(Scalar::ZERO)
            .lower();
        sum = sum.add(gap.mul(gap));
    }
    // The root of a sharp sum of squares: never refused.
    sum.sqrt().map_or(Scalar::ZERO, Scalar::lower)
}
