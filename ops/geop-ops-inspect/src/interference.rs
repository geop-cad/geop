//! [`interference_report`]: which solids a part shows overlap — its own
//! with each other, with those of the parts placed in it, and those
//! between each other — by how much, and which only touch.

use geop_core_editor::editing::Tone;
use geop_core_editor::output::{Answer, Block, Bounded, Row};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::{Vector, Vector3},
};
use geop_core_program::{
    element::PickedElement,
    placed::{INSTANCE_SEPARATOR, instance_path},
    reader::Reader,
};
use geop_core_topology::{
    Body, SolidId,
    distance::{Feature, PlacedFace, closest_points},
};
use geop_ops_model::placed::{
    locate,
    solids::{PlacedSolid, placed_solids},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};
use std::collections::HashMap;

use crate::mass::drawn;
use geop_lib_booleans::{RemeshParams, boolean};
use geop_ops_model::BooleanOp;
use serde::Serialize;

/// An axis-aligned box: each coordinate the interval it spans.
pub type Box3 = Vector3;

/// The box of `points`: each coordinate the union of theirs.
pub(crate) fn hull(points: &[Vector3]) -> Box3 {
    points
        .iter()
        .copied()
        .reduce(|a, b| a.union(&b))
        .unwrap_or_else(Vector3::everything)
}

/// Whether two boxes are definitely apart along some axis.
pub(crate) fn apart(a: &Box3, b: &Box3) -> bool {
    (0..3).any(|k| {
        a[k].upper().definitely_less(b[k].lower()) || b[k].upper().definitely_less(a[k].lower())
    })
}

/// The control point `cp` of a surface of `solid`, as a point, where the
/// solid is placed.
fn place_control_point(solid: &PlacedSolid, cp: &Vector<4>) -> GeopResult<Box3> {
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
    /// What finding them took.
    pub work: Work,
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

/// The work a report took: what the broad phase compared and what the
/// narrow phase ran — counted, not timed (see AGENTS.md "Count the work").
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct Work {
    /// Pairs of boxes compared by the sweep along `x`.
    pub boxes_compared: usize,
    /// Pairs whose boxes could meet: each looked at closely, or found
    /// among those already looked at.
    pub candidates: usize,
    /// Booleans run: one per distinct pair of shapes and relative pose
    /// (see [`Checked`]).
    pub booleans: usize,
}

/// Which solids of the part `reader` reads — its own, and those of the parts
/// placed in it, where they are — overlap, and by how much, and which only
/// touch.
///
/// A pair whose boxes are definitely apart is apart; the pairs whose boxes
/// could meet are found by sweeping the boxes along `x` ([`candidate_pairs`]),
/// not by comparing every pair. Any other pair is intersected, as copies in
/// a part of their own: a solid left is their overlap, measured by its
/// volume. Nothing left, the two touch if the least distance between their
/// faces could be zero (see [`closest_points`]), and are apart otherwise.
///
/// A pair is checked in the frame of its first solid, the second moved
/// relative to it; a pair of the same two shapes at the same relative pose
/// — every screw in its nut — is checked once ([`Checked`]).
pub fn interference_report(reader: &Reader) -> GeopResult<InterferenceReport> {
    let solids = placed_solids(reader)?;
    let mut local = LocalBoxes::default();
    let boxes = (solids.iter())
        .map(|s| local.placed(s))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut work = Work::default();
    let pairs = candidate_pairs(&boxes, &mut work.boxes_compared);
    let mut checked = Checked::default();
    let mut poses = LocalPoses::default();
    let mut found = Vec::new();
    let mut unchecked = Vec::new();
    for (i, j) in pairs {
        work.candidates += 1;
        let (a, b) = (&solids[i], &solids[j]);
        let relative = poses.relative(reader, a, b)?;
        match checked.check(a, b, relative, &mut work.booleans) {
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
    Ok(InterferenceReport {
        solids: solids.len(),
        found,
        unchecked,
        work,
    })
}

/// A solid's shape, whatever instance shows it: the model it is a solid of
/// — one for every instance of one build, every screw of a size — and the
/// solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct Shape(*const NamedModel, SolidId);

impl Shape {
    pub(crate) fn of(solid: &PlacedSolid) -> Self {
        Shape(std::ptr::from_ref(&*solid.model), solid.solid)
    }
}

/// The boxes of shapes in their own part's frame, each computed once
/// however often its part is placed.
#[derive(Default)]
pub(crate) struct LocalBoxes(HashMap<Shape, Box3>);

impl LocalBoxes {
    /// The box of `solid` where it is placed: the box of its shape's box's
    /// corners moved there, which encloses the shape moved.
    pub(crate) fn placed(&mut self, solid: &PlacedSolid) -> GeopResult<Box3> {
        let shape = Shape::of(solid);
        let own = match self.0.get(&shape) {
            Some(own) => *own,
            None => {
                let unplaced = PlacedSolid {
                    pose: None,
                    name: solid.name.clone(),
                    instance: solid.instance.clone(),
                    reader: solid.reader.clone(),
                    model: std::borrow::Cow::Borrowed(&*solid.model),
                    solid: solid.solid,
                };
                let faces = solid.model.topology().solid_faces(solid.solid)?;
                let own = bounding_box(&unplaced, &faces)?;
                self.0.insert(shape, own);
                own
            }
        };
        Ok(match &solid.pose {
            None => own,
            Some(pose) => hull(&corners(&own).map(|c| pose.apply(&c))),
        })
    }
}

/// The eight corners of `b`.
pub(crate) fn corners(b: &Box3) -> [Vector3; 8] {
    std::array::from_fn(|k| {
        Vector3::from_array(std::array::from_fn(|axis| match (k >> axis) & 1 {
            0 => b[axis].lower(),
            _ => b[axis].upper(),
        }))
    })
}

/// The pairs `(i, j)`, `i < j`, of `boxes` that are not definitely apart,
/// in order: found by sweeping along `x` — the boxes sorted by where they
/// start, each compared only with those started before it that have not
/// definitely ended — counting each comparison in `compared`.
pub fn candidate_pairs(boxes: &[Box3], compared: &mut usize) -> Vec<(usize, usize)> {
    let start = |k: usize| boxes[k][0].lower();
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    // The starts are sharp numbers, so this is a total order.
    order.sort_by(|&a, &b| {
        if start(a).definitely_less(start(b)) {
            std::cmp::Ordering::Less
        } else if start(b).definitely_less(start(a)) {
            std::cmp::Ordering::Greater
        } else {
            a.cmp(&b)
        }
    });
    let mut active: Vec<usize> = Vec::new();
    let mut pairs = Vec::new();
    for &k in &order {
        // Every box after this one starts where it does or later: one that
        // has ended before it starts meets none of them.
        active.retain(|&j| !boxes[j][0].upper().definitely_less(start(k)));
        for &j in &active {
            *compared += 1;
            if !apart(&boxes[j], &boxes[k]) {
                pairs.push((j.min(k), j.max(k)));
            }
        }
        active.push(k);
    }
    pairs.sort_unstable();
    pairs
}

/// Where each placed part is in the part it is placed in, each found once.
#[derive(Default)]
pub(crate) struct LocalPoses(HashMap<String, Pose>);

impl LocalPoses {
    /// The pose of `b` relative to `a`, in the part `reader` reads: none
    /// for two solids of one part. It is composed of the poses below the
    /// part both are placed in — not of their poses in the part read, which
    /// would round differently wherever that part is — so that two pairs
    /// placed alike within copies of one part come out the same, bound for
    /// bound.
    pub(crate) fn relative(
        &mut self,
        reader: &Reader,
        a: &PlacedSolid,
        b: &PlacedSolid,
    ) -> GeopResult<Option<Pose>> {
        let (path_a, path_b) = (levels(&a.instance), levels(&b.instance));
        let common = (path_a.iter().zip(&path_b))
            .take_while(|(x, y)| x == y)
            .count();
        let below_a = self.below(reader, &path_a[common..])?;
        let below_b = self.below(reader, &path_b[common..])?;
        Ok(match (below_a, below_b) {
            (None, b) => b,
            (Some(a), None) => Some(a.inverse()),
            (Some(a), Some(b)) => Some(a.inverse().compose(&b)),
        })
    }

    /// The pose of the last of `paths` in the part the first is placed in:
    /// each in the one before composed. None for no paths.
    fn below(&mut self, reader: &Reader, paths: &[String]) -> GeopResult<Option<Pose>> {
        let mut pose: Option<Pose> = None;
        for path in paths {
            let local = match self.0.get(path) {
                Some(local) => *local,
                None => {
                    let local = locate(reader, path)?.local;
                    self.0.insert(path.clone(), local);
                    local
                }
            };
            pose = Some(match pose {
                None => local,
                Some(outer) => outer.compose(&local),
            });
        }
        Ok(pose)
    }
}

/// The paths of the parts `instance` is placed in, outermost first, it
/// last: `a`, `a/b`, `a/b/c`. None for the part's own.
fn levels(instance: &str) -> Vec<String> {
    let mut paths: Vec<String> = Vec::new();
    for name in instance.split(INSTANCE_SEPARATOR).filter(|n| !n.is_empty()) {
        let outer = paths.last().map_or("", String::as_str);
        paths.push(instance_path(outer, name));
    }
    paths
}

/// How a pair of solids meets: whether they overlap, by what volume, or
/// only touch — or none, apart.
type Meeting = Option<(Contact, Option<Bounded>)>;

/// The pairs checked so far, by their shapes and the pose of the second
/// relative to the first: a pair of the same shapes at the same relative
/// pose meets the same way, and is not checked again.
///
/// Poses are compared exactly, bound for bound: a pose equal only to
/// within its width is another pose.
#[derive(Default)]
struct Checked(HashMap<(Shape, Shape), Checks>);

/// What was found of one pair of shapes, at each relative pose checked
/// (none: placed as they are).
type Checks = Vec<(Option<Pose>, Result<Meeting, String>)>;

impl Checked {
    /// How `a` and `b` meet: checked, counting the boolean in `booleans`,
    /// or as found before for the same shapes at the same relative pose.
    fn check(
        &mut self,
        a: &PlacedSolid,
        b: &PlacedSolid,
        relative: Option<Pose>,
        booleans: &mut usize,
    ) -> GeopResult<Meeting> {
        let seen = self.0.entry((Shape::of(a), Shape::of(b))).or_default();
        if let Some((_, meeting)) = seen.iter().find(|(pose, _)| *pose == relative) {
            return meeting.clone().map_err(GeopError::new);
        }
        *booleans += 1;
        let first = PlacedSolid {
            pose: None,
            name: a.name.clone(),
            instance: a.instance.clone(),
            reader: a.reader.clone(),
            model: std::borrow::Cow::Borrowed(&*a.model),
            solid: a.solid,
        };
        let second = PlacedSolid {
            pose: relative,
            name: b.name.clone(),
            instance: b.instance.clone(),
            reader: b.reader.clone(),
            model: std::borrow::Cow::Borrowed(&*b.model),
            solid: b.solid,
        };
        let meeting = check_pair(&first, &second).map_err(|e| e.to_string());
        seen.push((relative, meeting.clone()));
        meeting.map_err(GeopError::new)
    }
}

/// How `a` and `b` meet, if they do, and the volume they share.
fn check_pair(a: &PlacedSolid, b: &PlacedSolid) -> GeopResult<Meeting> {
    let ctx = |e: GeopError| e.with_context(format!("checking {} against {}", a.name, b.name));
    let mut scratch = NamedModel::new();
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
            if closest.distance.could_be_equal(Scalar::ZERO) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// A copy of `solid` in `scratch`, where it is placed, each of its entities
/// named as it is behind `prefix`. Refuses a solid of more than one shell —
/// one with a void — which the boolean is not asked to intersect.
fn copy_solid(scratch: &mut NamedModel, solid: &PlacedSolid, prefix: &str) -> GeopResult<SolidId> {
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
