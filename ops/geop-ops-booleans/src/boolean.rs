//! Boolean operations on two solids, built on top of [`remesh`].
//!
//! The algorithm is deliberately simple, and it is simple *because* remesh
//! has already done the hard part. After remeshing, every face of either
//! solid lies wholly inside the other, wholly outside it, or exactly on its
//! boundary — no face straddles, because every intersection curve has been
//! imprinted and every face it crossed has been split. That turns a boolean
//! into a per-face classification followed by a keep/drop table.
//!
//! 1. [`remesh`] the two solids against each other.
//! 2. Classify each face with [`classify_face`], by taking a point strictly
//!    inside its trimmed region and asking where it sits relative to the
//!    other solid.
//! 3. Keep the faces the operator wants (see [`BooleanOp::keeps`]), reversing
//!    them where the operator needs the material on the other side.
//! 4. Assemble the survivors into a new solid and discard everything else.
//!
//! [`boolean_up_to_next`] is the same with one more step between 3 and 4:
//! of the second solid, a tool, it keeps only the first piece its start
//! reaches on the side of the first solid's boundary the operator wants.

use std::collections::{HashMap, HashSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
    vector::Vector3,
};
use geop_core_topology::{
    CoedgeGeometry, EdgeId, FaceId, Model, ShellId, SolidId, VertexId,
    boundary::BoundaryType,
    contains::{
        face::{PointClassification as FacePoint, face_contains, face_interior_point_where},
        shell::{PointClassification as ShellPoint, shell_contains},
    },
};
use geop_ops::{Namer, Part};
use serde::{Deserialize, Serialize};

use crate::remesh::remesh::{RemeshParams, remesh};

/// Which boolean to perform.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanOp {
    /// Everything in either solid.
    Union,
    /// Only what is in both.
    Intersection,
    /// `solid_a` with `solid_b` removed.
    Difference,
}

/// Where one solid's face sits relative to the *other* solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClassification {
    /// Strictly inside the other solid.
    Inside,
    /// Strictly outside it.
    Outside,
    /// On its boundary, with the two surfaces' normals pointing the same way
    /// — the two solids touch and lie on the same side of the shared patch.
    OnSameNormal,
    /// On its boundary, with the normals opposed — the solids meet along the
    /// patch from opposite sides.
    OnOppositeNormal,
}

/// What to do with a classified face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Keep {
    /// Keep it as it is.
    AsIs,
    /// Keep it with its normal flipped — the operator wants the material on
    /// the other side of this patch.
    Reversed,
    /// Drop it.
    Drop,
}

impl BooleanOp {
    /// Whether to keep a face classified as `class`, and with which
    /// orientation. `from_a` says which solid the face came from, because the
    /// two are not symmetric for [`BooleanOp::Difference`], and because a
    /// coincident patch must be kept exactly once rather than from both.
    ///
    /// The coincident rules are the only subtle ones. A patch shared by both
    /// solids with **matching** normals bounds the same material on the same
    /// side, so union and intersection each keep exactly one copy (`a`'s, by
    /// convention) and difference deletes it — the material behind it is
    /// removed from both sides at once. A patch shared with **opposing**
    /// normals is the reverse: it separates the two solids, so union and
    /// intersection drop it (it is interior to the result) while difference
    /// keeps `a`'s copy, since that is exactly the surface where `a` is left
    /// open by removing `b`.
    fn keeps(self, class: FaceClassification, from_a: bool) -> Keep {
        use FaceClassification::*;
        match (self, class) {
            // Union: the result is bounded by whatever is outside the other.
            (BooleanOp::Union, Outside) => Keep::AsIs,
            (BooleanOp::Union, Inside) => Keep::Drop,
            (BooleanOp::Union, OnSameNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
            (BooleanOp::Union, OnOppositeNormal) => Keep::Drop,

            // Intersection: bounded by whatever is inside the other.
            (BooleanOp::Intersection, Inside) => Keep::AsIs,
            (BooleanOp::Intersection, Outside) => Keep::Drop,
            (BooleanOp::Intersection, OnSameNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
            (BooleanOp::Intersection, OnOppositeNormal) => Keep::Drop,

            // Difference (a - b) = a intersected with the complement of b, so
            // `a`'s faces behave as for intersection-with-the-outside, and
            // `b`'s surviving faces are the ones inside `a`, turned around to
            // face into the cavity they now bound.
            (BooleanOp::Difference, Outside) if from_a => Keep::AsIs,
            (BooleanOp::Difference, Inside) if from_a => Keep::Drop,
            (BooleanOp::Difference, Inside) => Keep::Reversed,
            (BooleanOp::Difference, Outside) => Keep::Drop,
            (BooleanOp::Difference, OnSameNormal) => Keep::Drop,
            (BooleanOp::Difference, OnOppositeNormal) => {
                if from_a {
                    Keep::AsIs
                } else {
                    Keep::Drop
                }
            }
        }
    }
}

/// Fixed seed for the ray casting behind every containment query here. Both
/// `face_contains` and `shell_contains` retry until they find a ray grazing
/// nothing, so their answers are seed-independent; a constant keeps a boolean
/// reproducible run to run.
const SEED: u64 = 0xB001_EA47_0000_0001;

/// `solid_a` combined with `solid_b` under `op`, as a new solid in `model`.
///
/// `Ok(None)` means the result is **empty**, which is an answer rather than a
/// failure: intersecting two solids that do not overlap, or subtracting a
/// solid that wholly contains the first, legitimately leaves nothing. Callers
/// that treat "no solid" as an error would reject the majority of scenes in
/// this crate's own test set.
///
/// Both input solids are consumed either way: their faces are transferred to
/// the result or deleted, so neither id is valid afterwards.
///
/// The result is named `namer`'s root name, and everything the boolean
/// creates on the way is named as [`crate::naming`] describes; every face,
/// edge and vertex it keeps keeps its name.
pub fn boolean<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid_a: SolidId,
    solid_b: SolidId,
    op: BooleanOp,
    params: RemeshParams<S>,
) -> GeopResult<Option<SolidId>> {
    combine(part, namer, solid_a, solid_b, op, None, params)
}

/// A tool going up to the next face — the second operand of a boolean — by
/// the names its start and end faces had before it (see [`crate::naming`]),
/// and whether only the piece of it taken is kept, `alone`, as a solid of
/// its own, rather than combined with the first operand.
#[derive(Clone, Copy, Debug)]
struct UpToNext<'a> {
    start: &'a str,
    end: &'a str,
    alone: bool,
}

/// Like [`boolean`] — `target` combined with `tool` — but "up to next":
/// only the first piece of `tool` its face `start` reaches is combined.
///
/// For a [`BooleanOp::Union`] that is the first piece of the tool outside
/// the target — growing from `start` until it runs into the target; for a
/// [`BooleanOp::Difference`] or [`BooleanOp::Intersection`], the first
/// piece inside it — cutting, or keeping, from where the tool enters the
/// target until it comes out of it again. The tool has to be long enough to
/// reach past whatever stops it: if the piece also reaches its face `end`,
/// nothing stopped it, and that is an error. `start` and `end` are the
/// names those faces had before the boolean (see [`crate::naming`]).
pub fn boolean_up_to_next<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    target: SolidId,
    tool: SolidId,
    op: BooleanOp,
    (start, end): (&str, &str),
    params: RemeshParams<S>,
) -> GeopResult<Option<SolidId>> {
    let up_to_next = UpToNext {
        start,
        end,
        alone: false,
    };
    combine(part, namer, target, tool, op, Some(up_to_next), params)
}

/// The first piece of `tool` its face `start` reaches outside `stops` — up
/// to the next face of any of them — as a solid of its own, named
/// `namer`'s root, `stops` left as they are: what a new body going up to the
/// next face is. Fails, as [`boolean_up_to_next`] does, if nothing stops
/// the tool before its face `end`.
///
/// The tool is cut by a copy of the solids it stops at, so that only the
/// copy is imprinted and consumed.
pub fn piece_up_to_next<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    tool: SolidId,
    stops: &[SolidId],
    (start, end): (&str, &str),
    params: RemeshParams<S>,
) -> GeopResult<SolidId> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "piece_up_to_next(name={}, tool={tool}, stops={stops:?})",
            namer.root()
        ))
    };
    let mut faces = Vec::new();
    for &solid in stops {
        faces.extend(part.topology().solid_faces(solid).with_context(&ctx)?);
    }
    let (spec, sources) = part.topology().body_spec(&faces, true).with_context(&ctx)?;
    let copied = |ids: Vec<geop_ops::RefId>| -> GeopResult<Vec<String>> {
        ids.into_iter()
            .map(|id| {
                let name = part
                    .name_of(id)
                    .ok_or_else(|| GeopError::new(format!("{id} has no name")))?;
                Ok(namer.name(&["copy", name]))
            })
            .collect()
    };
    let names = geop_ops::BodyNames {
        vertices: copied(sources.vertices.iter().map(|&v| v.into()).collect())?,
        edges: copied(sources.edges.iter().map(|&e| e.into()).collect())?,
        faces: copied(sources.faces.iter().map(|&f| f.into()).collect())?,
        solid: Some(namer.name(&["copy"])),
    };
    let copy = part.build_body(spec, names).with_context(&ctx)?;
    let copy = copy.solid.expect("built as a solid");
    // As a join of the tool to the copy, picking only the piece — the copy
    // first, as in every other combination with a tool.
    let up_to_next = UpToNext {
        start,
        end,
        alone: true,
    };
    combine(
        part,
        namer,
        copy,
        tool,
        BooleanOp::Union,
        Some(up_to_next),
        params,
    )
    .with_context(&ctx)?
    .ok_or_else(|| ctx(GeopError::new("up to next: nothing of the tool is left")))
}

/// [`boolean`], or with `up_to_next` [`boolean_up_to_next`] — or, with the
/// tool kept alone, [`piece_up_to_next`].
fn combine<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid_a: SolidId,
    solid_b: SolidId,
    op: BooleanOp,
    up_to_next: Option<UpToNext>,
    params: RemeshParams<S>,
) -> GeopResult<Option<SolidId>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "boolean(name={}, solid_a={solid_a}, solid_b={solid_b}, op={op:?}, up_to_next={up_to_next:?})",
            namer.root()
        ))
    };

    let origins = remesh(part, namer, solid_a, solid_b, params).with_context(&ctx)?;
    let model = part.topology();

    let faces_a = model.solid_faces(solid_a).with_context(&ctx)?;
    let faces_b = model.solid_faces(solid_b).with_context(&ctx)?;

    let mut decisions: HashMap<FaceId, (FaceClassification, Keep)> = HashMap::new();
    let mut keep: Vec<FaceId> = Vec::new();
    let mut reverse: Vec<FaceId> = Vec::new();
    for (faces, from_a, other) in [(&faces_a, true, solid_b), (&faces_b, false, solid_a)] {
        for &face_id in faces {
            let class = classify_face(model, face_id, other, params)
                .with_context(&ctx)
                .with_context(&|e: GeopError| {
                    e.with_context(format!("classifying face {face_id}"))
                })?;
            decisions.insert(face_id, (class, op.keeps(class, from_a)));
        }
    }
    if let Some(UpToNext { start, end, alone }) = up_to_next {
        let tool: HashSet<FaceId> = faces_b.iter().copied().collect();
        // The piece of the kind the operator takes from the target, and
        // what of the target the tool leaves untouched — but for an
        // intersection, which keeps only what both hold, or a piece kept
        // alone.
        let (wanted_inside, keep_untouched) = match op {
            BooleanOp::Union => (false, !alone),
            BooleanOp::Difference => (true, true),
            BooleanOp::Intersection => (true, false),
        };
        let reached = reach_up_to_next(
            model,
            &tool,
            (wanted_inside, keep_untouched),
            &origins.faces,
            (start, end),
            &mut decisions,
        )
        .with_context(&ctx)?;
        if alone {
            keep_alone(&reached, &mut decisions);
        }
    }
    // In the order the faces were classified, so the result's shell lists
    // its faces the same way every run.
    for &face_id in faces_a.iter().chain(&faces_b) {
        match decisions[&face_id].1 {
            Keep::AsIs => keep.push(face_id),
            Keep::Reversed => {
                keep.push(face_id);
                reverse.push(face_id);
            }
            Keep::Drop => {}
        }
    }
    check_closed(part, &decisions).with_context(&ctx)?;

    for &face_id in &reverse {
        part.reverse_face(face_id).with_context(&ctx)?;
    }

    part.assemble_solid(&[solid_a.into(), solid_b.into()], &keep, namer.root())
        .with_context(&ctx)
}

/// Narrows `decisions` down to the piece of the tool (the faces `tool`) that
/// its start reaches first, see [`boolean_up_to_next`].
///
/// The target's boundary cuts the tool's material into pieces, alternately
/// outside and inside the target. Each piece is bounded by tool faces —
/// all on one side of the target, the side its classification says — and
/// by target faces inside the tool, each of which bounds two pieces: the
/// inside one on its material's side, the outside one on the other. (Remesh
/// split every face along every crossing, so no face straddles two pieces.)
/// Faces sharing an edge bound the same piece if their material lies on the
/// same side of the target, which makes the pieces connected sets of face
/// *sides*, and two pieces neighbours when a target face lies between them.
///
/// From the piece bounded by the start face, the piece wanted is the first
/// of the kind wanted — outside the target, growing until it runs into the
/// target, or inside it, cutting or keeping from where it enters the target
/// until it leaves it. That is the start's own piece, if it is of that
/// kind, or else the pieces next to it. The tool's other pieces are
/// dropped, and the target's faces between them kept if `keep_untouched`,
/// as if the tool had not been there, or else dropped too.
fn reach_up_to_next<S: Scalar>(
    model: &Model<S>,
    tool: &HashSet<FaceId>,
    (wanted_inside, keep_untouched): (bool, bool),
    origins: &HashMap<FaceId, String>,
    (start, end): (&str, &str),
    decisions: &mut HashMap<FaceId, (FaceClassification, Keep)>,
) -> GeopResult<Reached> {
    use FaceClassification::*;
    // Which side of the target the material behind a tool face lies on.
    let inside = |class: FaceClassification| matches!(class, Inside | OnSameNormal);
    let mut tool_faces: Vec<FaceId> = tool.iter().copied().collect();
    tool_faces.sort_by_key(|f| f.0);
    let mut between: Vec<FaceId> = decisions
        .iter()
        .filter(|(face, (class, _))| !tool.contains(face) && *class == Inside)
        .map(|(&face, _)| face)
        .collect();
    between.sort_by_key(|f| f.0);
    // Nodes: every tool face, then each target face between pieces by its
    // inner side, then by its outer side.
    let (n, m) = (tool_faces.len(), between.len());
    let (inner, outer) = (|j: usize| n + j, |j: usize| n + m + j);
    let tool_index: HashMap<FaceId, usize> = tool_faces
        .iter()
        .enumerate()
        .map(|(i, &f)| (f, i))
        .collect();
    let between_index: HashMap<FaceId, usize> =
        between.iter().enumerate().map(|(j, &f)| (f, j)).collect();
    let class = |face: &FaceId| decisions[face].0;

    let mut users: HashMap<EdgeId, Vec<FaceId>> = HashMap::new();
    for &face in tool_faces.iter().chain(&between) {
        for coedge in model.iterate_face_coedges(face) {
            if let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry {
                users.entry(edge).or_default().push(face);
            }
        }
    }
    let mut pieces = UnionFind::new(n + 2 * m);
    for faces in users.values() {
        let (mut in_node, mut out_node) = (None, None);
        let join = |slot: &mut Option<usize>, node: usize, pieces: &mut UnionFind| match *slot {
            Some(other) => pieces.union(other, node),
            None => *slot = Some(node),
        };
        for face in faces {
            if let Some(&i) = tool_index.get(face) {
                let slot = if inside(class(face)) {
                    &mut in_node
                } else {
                    &mut out_node
                };
                join(slot, i, &mut pieces);
            } else {
                let j = between_index[face];
                join(&mut in_node, inner(j), &mut pieces);
                join(&mut out_node, outer(j), &mut pieces);
            }
        }
    }
    // A tool face's piece is on its material's side; a target face's inner
    // side is in an inside piece, its outer side in an outside one.
    let piece_inside = |node: usize| match node {
        i if i < n => inside(class(&tool_faces[i])),
        i => i < n + m,
    };
    let mut kind: HashMap<usize, bool> = HashMap::new();
    for node in 0..n + 2 * m {
        kind.insert(pieces.find(node), piece_inside(node));
    }
    let mut neighbours: HashMap<usize, HashSet<usize>> = HashMap::new();
    for j in 0..m {
        let (a, b) = (pieces.find(inner(j)), pieces.find(outer(j)));
        neighbours.entry(a).or_default().insert(b);
        neighbours.entry(b).or_default().insert(a);
    }

    let origin_is = |face: &FaceId, name: &str| origins.get(face).is_some_and(|o| o == name);
    // The start may lie partly on either side of the target — a rib's
    // profile running on into the walls it stands between. Its pieces of
    // the kind wanted are the first ones it reaches; only where it has none
    // are the pieces next to it the first. Taking the pieces next to its
    // other fragments too would take a second piece, reached through the
    // target: the rest of the tool on the walls' far side.
    let start_pieces: HashSet<usize> = tool_faces
        .iter()
        .enumerate()
        .filter(|(_, face)| origin_is(face, start))
        .map(|(i, _)| pieces.find(i))
        .collect();
    let mut reached: HashSet<usize> = start_pieces
        .iter()
        .copied()
        .filter(|p| kind[p] == wanted_inside)
        .collect();
    if reached.is_empty() {
        for piece in &start_pieces {
            let next = neighbours.get(piece).into_iter().flatten();
            reached.extend(next.filter(|p| kind[p] == wanted_inside));
        }
    }
    if reached.is_empty() {
        let starts: Vec<String> = tool_faces
            .iter()
            .enumerate()
            .filter(|(_, face)| origin_is(face, start))
            .map(|(i, face)| {
                let piece = pieces.find(i);
                format!(
                    "{face} ({:?}, in an {} piece with {} neighbour(s))",
                    class(face),
                    if kind[&piece] { "inside" } else { "outside" },
                    neighbours.get(&piece).map_or(0, HashSet::len)
                )
            })
            .collect();
        let mut tool_origins: Vec<&str> = tool_faces
            .iter()
            .filter_map(|f| origins.get(f).map(String::as_str))
            .collect();
        tool_origins.sort_unstable();
        tool_origins.dedup();
        return Err(GeopError::new(format!(
            "up to next: from its start {start:?}, the profile meets nothing of the target to {} — does it go the right way? (the start's faces: [{}]; {} piece(s), {m} target face(s) between them; the tool's faces come from {tool_origins:?})",
            if wanted_inside {
                "cut into"
            } else {
                "grow up to"
            },
            starts.join(", "),
            kind.len(),
        )));
    }
    for (i, face) in tool_faces.iter().enumerate() {
        let in_reach = reached.contains(&pieces.find(i));
        if in_reach && origin_is(face, end) {
            return Err(GeopError::new(format!(
                "{NOTHING_STOPS}: it reaches the end {end:?} without meeting a face of the target all around"
            )));
        }
        if !in_reach {
            decisions.get_mut(face).expect("classified").1 = Keep::Drop;
        }
    }
    let mut bounding = Reached {
        tool: HashSet::new(),
        target: HashSet::new(),
    };
    for (i, face) in tool_faces.iter().enumerate() {
        if reached.contains(&pieces.find(i)) {
            bounding.tool.insert(*face);
        }
    }
    for (j, face) in between.iter().enumerate() {
        let facing = if wanted_inside { inner(j) } else { outer(j) };
        if reached.contains(&pieces.find(facing)) {
            bounding.target.insert(*face);
        } else {
            // Untouched by the piece taking part: the target's boundary
            // there stays — unless only what the tool reaches is kept.
            decisions.get_mut(face).expect("classified").1 = if keep_untouched {
                Keep::AsIs
            } else {
                Keep::Drop
            };
        }
    }
    Ok(bounding)
}

/// What [`reach_up_to_next`] found the tool's taken piece bounded by: the
/// tool's faces, and the target's faces between it and the rest.
struct Reached {
    tool: HashSet<FaceId>,
    target: HashSet<FaceId>,
}

/// Turns the decisions of a union of a target with the piece of a tool it
/// `reached` into those of the piece alone: its own faces as they are —
/// where it lies on the target too, its start drawn on a face of the target,
/// included — and the target's faces bounding it turned around to face out
/// of it. Nothing else of either.
fn keep_alone(reached: &Reached, decisions: &mut HashMap<FaceId, (FaceClassification, Keep)>) {
    use FaceClassification::*;
    for (face, (class, keep)) in decisions.iter_mut() {
        *keep = if reached.tool.contains(face) && matches!(class, Outside | OnOppositeNormal) {
            Keep::AsIs
        } else if reached.target.contains(face) {
            Keep::Reversed
        } else {
            Keep::Drop
        };
    }
}

/// Marks the error [`boolean_up_to_next`] and [`piece_up_to_next`] raise
/// when the tool is not stopped all round — when the target meets part of
/// the profile, or none, and the rest goes on past it. A caller can then go
/// only as far as the profile first meets the target instead. Matched on the
/// message, as `GeopError` carries no code.
pub const NOTHING_STOPS: &str = "up to next: nothing stops the profile";

/// Fails unless the kept faces close up: every edge they use must be used an
/// even number of times by them — twice where two faces meet, four times
/// where the result touches itself along the edge. An odd count is a hole in
/// the result, so a face was kept or dropped wrongly, or remesh left a face
/// straddling the other solid; the error lists every face along that edge
/// with its name and classification, which tells the two apart.
fn check_closed<S: Scalar>(
    part: &Part<S>,
    decisions: &HashMap<FaceId, (FaceClassification, Keep)>,
) -> GeopResult<()> {
    let model = part.topology();
    let mut uses: HashMap<EdgeId, usize> = HashMap::new();
    for (&face_id, &(_, decision)) in decisions {
        if decision == Keep::Drop {
            continue;
        }
        for coedge in model.iterate_face_coedges(face_id) {
            if let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry {
                *uses.entry(edge).or_default() += 1;
            }
        }
    }
    let Some((&edge, &count)) = uses
        .iter()
        .filter(|(_, n)| *n % 2 != 0)
        .min_by_key(|(e, _)| e.0)
    else {
        return Ok(());
    };
    let e = model.get_edge(edge)?;
    let faces: Vec<String> = model
        .coedges_of_edge(edge)
        .into_iter()
        .map(|coedge| {
            let face = model.get_coedge(coedge)?.face;
            let name = part.name_of(face).unwrap_or("?");
            let decision = match decisions.get(&face) {
                Some((class, decision)) => format!("{class:?} -> {decision:?}"),
                None => "not in either solid".to_string(),
            };
            Ok(format!(
                "{face} {name:?}: {decision}, bounded by {:?}",
                model.face_corners(face)?
            ))
        })
        .collect::<GeopResult<_>>()?;
    Err(GeopError::new(format!(
        "boolean: the kept faces use edge {edge} (from {:?} to {:?}) {count} time(s), leaving the result open there; faces along it: [{}]",
        model.get_vertex(e.start_vertex)?.point,
        model.get_vertex(e.end_vertex)?.point,
        faces.join(", ")
    )))
}

/// Where `face_id` sits relative to `other_solid`, decided at points strictly
/// inside the face's trimmed region.
///
/// One point would be enough *because remesh ran first*: every curve along
/// which the other solid's boundary crosses this face has been imprinted, and
/// the face split along it, so the face no longer straddles anything. Without
/// that guarantee this would be unsound, which is why it lives here rather
/// than as a general-purpose query. Every interior point offered (one or two
/// per boundary coedge, see `face_interior_point_where`) is classified anyway,
/// and points that disagree are an error: a missed intersection curve then
/// names the face it left straddling, instead of surfacing as a result that
/// is open somewhere else — or, by luck of which point came first, not at all.
///
/// A point that lands *on* the other solid's boundary doesn't decide it by
/// itself: the face may lie along that boundary over an area (coincident
/// patches, told apart by their normals), or merely touch it at a point or
/// along a curve — a cube face resting on a sphere's pole, say, where the
/// sphere has no normal at all. So such a point is set aside and the next
/// interior point tried (one per boundary coedge, see
/// `face_interior_point_where`); any point off the other solid's boundary
/// classifies the whole face. Only if every one lies on it are the patches
/// coincident, and their normals are compared.
pub fn classify_face<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    other_solid: SolidId,
    params: RemeshParams<S>,
) -> GeopResult<FaceClassification> {
    let face = model.get_face(face_id)?;
    let shells = model.get_solid(other_solid)?.shells.clone();
    let mut decided: Vec<(FaceClassification, Vector3<S>)> = Vec::new();
    let mut on_boundary = Vec::new();
    face_interior_point_where(
        model,
        face_id,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        SEED,
        |u, v| {
            let point = face.surface.evaluate(u, v)?;
            // Inside the solid is inside an odd number of its shells: inside
            // its outer shell, but not in a void.
            let mut inside = false;
            for &shell_id in &shells {
                match shell_contains(
                    model,
                    shell_id,
                    point,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    SEED,
                )? {
                    ShellPoint::Inside => inside = !inside,
                    ShellPoint::Outside => {}
                    ShellPoint::OnFace | ShellPoint::OnEdge | ShellPoint::OnVertex => {
                        on_boundary.push((u, v, point, shell_id));
                        return Ok(false);
                    }
                }
            }
            decided.push((
                if inside {
                    FaceClassification::Inside
                } else {
                    FaceClassification::Outside
                },
                point,
            ));
            Ok(false)
        },
    )?;
    if let Some(&(classification, point)) = decided.first() {
        if let Some((other, other_point)) = decided.iter().find(|(c, _)| *c != classification) {
            // The face's boundary: which curves were imprinted into it, and
            // so — between the two points — which one was not.
            let loops = model.face_corners(face_id)?;
            return Err(GeopError::new(format!(
                "classify_face: face {face_id} straddles solid {other_solid}'s boundary — {point:?} is {classification:?}, {other_point:?} is {other:?} — so remesh left an intersection curve unimprinted; the face is bounded by (corners, outer loop first) {loops:?}"
            )));
        }
        return Ok(classification);
    }

    // Every interior point lies on the other solid's boundary: the patches
    // coincide, and which way the shared patch faces is what separates "these
    // solids touch" from "these solids overlap along this patch". Any point
    // of the shared patch shows that equally well, so the first where both
    // normals exist is used — a revolve's cap collapses to its pole, where
    // it has none.
    let mut undefined = None;
    for &(u, v, point, shell_id) in &on_boundary {
        let normals = face
            .surface
            .normal(u, v)
            .and_then(|n| Ok((n, shell_normal_at(model, shell_id, &point, params)?)));
        let (this_normal, other_normal) = match normals {
            Ok(normals) => normals,
            Err(e) => {
                undefined = Some(e);
                continue;
            }
        };
        let alignment = this_normal.prod_dot(&other_normal);
        return if alignment.definitely_greater(S::ZERO) {
            Ok(FaceClassification::OnSameNormal)
        } else if alignment.definitely_less(S::ZERO) {
            Ok(FaceClassification::OnOppositeNormal)
        } else {
            Err(GeopError::new(format!(
                "boolean: face {face_id} lies on solid {other_solid}'s boundary at {point:?} (its interior point uv=({u:?}, {v:?})), but the two normals ({this_normal:?} and {other_normal:?}) are too close to perpendicular to tell which side is which"
            )))
        };
    }
    Err(match undefined {
        Some(e) => e.with_context(format!(
            "classify_face: face {face_id} lies on solid {other_solid}'s boundary at each of its {} interior points tried, and no normal comparison could be made at any of them",
            on_boundary.len()
        )),
        None => GeopError::new(format!(
            "classify_face: face {face_id} yielded interior points, yet none was classified or set aside"
        )),
    })
}

/// The normal of whichever face of `shell_id` contains `point`.
fn shell_normal_at<S: Scalar>(
    model: &Model<S>,
    shell_id: ShellId,
    point: &Vector3<S>,
    params: RemeshParams<S>,
) -> GeopResult<Vector3<S>> {
    for &face_id in &model.get_shell(shell_id)?.faces {
        let surface = &model.get_face(face_id)?.surface;
        let Some((u, v)) = geop_core_geometry::contains::surface::surface_could_contain(
            surface,
            point,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
        )?
        else {
            continue;
        };
        if !matches!(
            face_contains(
                model,
                face_id,
                u,
                v,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                SEED,
            )?,
            FacePoint::Outside
        ) {
            return surface.normal(u, v);
        }
    }
    Err(GeopError::new(format!(
        "boolean: no face of shell {shell_id} contains {point:?}, although the shell reported the point on its boundary"
    )))
}

#[cfg(test)]
mod tests {
    use super::{BooleanOp, FaceClassification, boolean, classify_face};
    use crate::{remesh::remesh::RemeshParams, scenes::all_scenes};
    use geop_core_math::{scalars::ScalInF64, scalars::Scalar, vector::Vector3};
    use geop_core_topology::{
        contains::rng::Rng,
        validation::{ValidationParameters, validate_fast},
    };
    use geop_ops::Namer;

    fn scene(name: &str) -> crate::scenes::TestScene<ScalInF64> {
        all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("scene {name} must exist"))
    }

    /// A fresh operation id, so that every solid and boolean a test builds
    /// gets names of its own.
    fn fresh_id() -> String {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        format!(
            "op{}",
            NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        )
    }

    fn namer() -> Namer {
        Namer::new("boolean", &fresh_id()).unwrap()
    }

    fn validation() -> ValidationParameters<ScalInF64> {
        let params = RemeshParams::<ScalInF64>::default();
        ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        }
    }

    /// Every face of an un-remeshed cube is outside a cylinder that only
    /// overlaps part of it — the classification itself, before any operator.
    #[test]
    fn classify_face_separates_inside_from_outside() {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        crate::remesh::remesh::remesh(&mut s.part, &namer(), s.solid_a, s.solid_b, params).unwrap();

        let mut inside = 0;
        let mut outside = 0;
        for face_id in s.part.topology().solid_faces(s.solid_a).unwrap() {
            match classify_face(s.part.topology(), face_id, s.solid_b, params).unwrap() {
                FaceClassification::Inside => inside += 1,
                FaceClassification::Outside => outside += 1,
                _ => {}
            }
        }
        assert!(
            inside > 0 && outside > 0,
            "a cylinder drilled through a cube must leave cube faces on both sides: {inside} inside, {outside} outside"
        );
    }

    /// Every face must yield an interior point, or classification is
    /// meaningless — this is the part of the algorithm with no fallback.
    #[test]
    fn every_remeshed_face_has_an_interior_point() {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        crate::remesh::remesh::remesh(&mut s.part, &namer(), s.solid_a, s.solid_b, params).unwrap();

        for solid in [s.solid_a, s.solid_b] {
            for face_id in s.part.topology().solid_faces(solid).unwrap() {
                let (u, v) = geop_core_topology::contains::face::face_interior_point(
                    s.part.topology(),
                    face_id,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    1234,
                )
                .unwrap_or_else(|e| panic!("face {face_id}: {e}"));
                let _ = s
                    .part
                    .topology()
                    .get_face(face_id)
                    .unwrap()
                    .surface
                    .evaluate(u, v)
                    .unwrap();
            }
        }
    }

    fn check_op(name: &str, op: BooleanOp) {
        let mut s = scene(name);
        let params = RemeshParams::<ScalInF64>::default();
        let result = boolean(&mut s.part, &namer(), s.solid_a, s.solid_b, op, params)
            .unwrap_or_else(|e| panic!("{name} {op:?}: {e}"))
            .unwrap_or_else(|| panic!("{name} {op:?}: result is empty"));

        assert!(
            !s.part.topology().solid_faces(result).unwrap().is_empty(),
            "{name} {op:?}: result has no faces"
        );
        if let Err(errors) = validate_fast(&validation(), s.part.topology()) {
            panic!(
                "{name} {op:?}: {} validate_fast error(s): {}",
                errors.len(),
                errors[0]
            );
        }
    }

    /// Where a probe point lands relative to the result solid.
    fn contains(
        model: &geop_core_topology::Model<ScalInF64>,
        solid: geop_core_topology::SolidId,
        p: (f64, f64, f64),
    ) -> bool {
        let params = RemeshParams::<ScalInF64>::default();
        let point = geop_core_math::vector::Vector3::from_array([
            ScalInF64::from_f64(p.0),
            ScalInF64::from_f64(p.1),
            ScalInF64::from_f64(p.2),
        ]);
        let shell = model.get_solid(solid).unwrap().shells[0];
        matches!(
            geop_core_topology::contains::shell::shell_contains(
                model,
                shell,
                point,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                0xA5A5_1234,
            )
            .unwrap(),
            geop_core_topology::contains::shell::PointClassification::Inside
        )
    }

    /// The cube is `[-0.5, 0.5]^3`; the cylinder has radius `0.2` on the z
    /// axis and spans `z in [-1, 1]`. So each probe below is unambiguously in
    /// one region, and the three operators must disagree about them in
    /// exactly the way their definitions say. Structural validity says
    /// nothing about *which* faces were kept — this is what does.
    const IN_CUBE_ONLY: (f64, f64, f64) = (0.4, 0.4, 0.0);
    const IN_CYLINDER_ONLY: (f64, f64, f64) = (0.0, 0.0, 0.8);
    const IN_BOTH: (f64, f64, f64) = (0.0, 0.0, 0.0);

    fn run(
        op: BooleanOp,
    ) -> (
        geop_core_topology::Model<ScalInF64>,
        geop_core_topology::SolidId,
    ) {
        let mut s = scene("box_cylinder_drilled_hole_through");
        let params = RemeshParams::<ScalInF64>::default();
        let result = boolean(&mut s.part, &namer(), s.solid_a, s.solid_b, op, params)
            .unwrap()
            .expect("this scene's operands overlap, so no operator is empty");
        (s.part.topology().clone(), result)
    }

    /// Regression: two cubes meeting along a shared edge, where the traced
    /// intersection curve *is* that edge — already part of both faces'
    /// boundaries. Splicing it carved a zero-area sliver instead of splitting
    /// anything. The direction check was passing because the corrector
    /// returned a sharpened `(u, v)` sitting 6e-17 off the trim boundary,
    /// which `face_contains` read as `Inside`; see
    /// `predictor_corrector_step`.
    #[test]
    fn box_grid_n1p00_n0p50_n1p00_difference_succeeds() {
        let mut s = scene("box_grid_n1p00_n0p50_n1p00");
        let params = RemeshParams::<ScalInF64>::default();
        boolean(
            &mut s.part,
            &namer(),
            s.solid_a,
            s.solid_b,
            BooleanOp::Difference,
            params,
        )
        .unwrap_or_else(|e| panic!("{e}"));
    }

    /// The defining property of a boolean, checked by sampling space rather
    /// than by inspecting topology: for a point `p`, membership in the result
    /// is a pure function of membership in the two operands.
    ///
    /// `union` keeps `p` iff either operand held it, `intersection` iff both
    /// did, `difference` iff `a` held it and `b` did not. Nothing about faces,
    /// coedges or orientation enters into it — which is exactly why it is
    /// worth checking. A result can be structurally perfect and still enclose
    /// the wrong region, if `BooleanOp::keeps` drops a face it should have
    /// kept or leaves one facing inward, and no structural check can see that.
    ///
    /// Points landing *on* a boundary are skipped rather than asserted about:
    /// membership there is genuinely ambiguous, and both the operands and the
    /// result may legitimately disagree about a surface they share.
    fn check_boolean_matches_point_membership(name: &str, op: BooleanOp, seed: u64) {
        let mut s = scene(name);
        let params = RemeshParams::<ScalInF64>::default();
        let (solid_a, solid_b) = (s.solid_a, s.solid_b);

        // Sampled against the operands *before* the boolean consumes them.
        let mut rng = Rng::new(seed);
        let mut samples = Vec::new();
        while samples.len() < SAMPLE_COUNT {
            let p = Vector3::from_array([
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
                ScalInF64::from_f64(rng.next_f64() * 4.0 - 2.0),
            ]);
            let (Some(in_a), Some(in_b)) = (
                strictly_inside(s.part.topology(), solid_a, p),
                strictly_inside(s.part.topology(), solid_b, p),
            ) else {
                continue;
            };
            samples.push((p, in_a, in_b));
        }

        let result = boolean(&mut s.part, &namer(), solid_a, solid_b, op, params)
            .unwrap_or_else(|e| panic!("{name} {op:?}: {e}"));

        for (p, in_a, in_b) in samples {
            let expected = match op {
                BooleanOp::Union => in_a || in_b,
                BooleanOp::Intersection => in_a && in_b,
                BooleanOp::Difference => in_a && !in_b,
            };
            let actual = match result {
                Some(solid) => match strictly_inside(s.part.topology(), solid, p) {
                    Some(inside) => inside,
                    // On the result's own boundary — ambiguous, no claim.
                    None => continue,
                },
                // An empty result contains nothing.
                None => false,
            };
            assert_eq!(
                actual,
                expected,
                "{name} {op:?}: point {p:?} is {} solid A and {} solid B, so the result should {} contain it",
                if in_a { "inside" } else { "outside" },
                if in_b { "inside" } else { "outside" },
                if expected { "" } else { "not" }
            );
        }
    }

    /// How many points each membership check samples.
    const SAMPLE_COUNT: usize = 20;

    /// `Some(true)`/`Some(false)` for a point strictly inside/outside every
    /// shell of `solid`; `None` if it lands on a boundary, where membership is
    /// not a yes/no question.
    fn strictly_inside(
        model: &geop_core_topology::Model<ScalInF64>,
        solid: geop_core_topology::SolidId,
        p: Vector3<ScalInF64>,
    ) -> Option<bool> {
        let params = RemeshParams::<ScalInF64>::default();
        let mut inside = false;
        for shell in model.get_solid(solid).ok()?.shells.clone() {
            match geop_core_topology::contains::shell::shell_contains(
                model,
                shell,
                p,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                0x5A3D_1234,
            ) {
                Ok(geop_core_topology::contains::shell::PointClassification::Inside) => {
                    inside = true
                }
                Ok(geop_core_topology::contains::shell::PointClassification::Outside) => {}
                _ => return None,
            }
        }
        Some(inside)
    }

    /// Scenes with genuinely overlapping operands, so every operator has
    /// something to do and the samples land on both sides of each boundary.
    const MEMBERSHIP_SCENES: &[&str] = &[
        "box_cylinder_drilled_hole_through",
        "box_cylinder_blind_hole",
        "figure8_cylinder_through_neck",
    ];

    #[test]
    fn union_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(name, BooleanOp::Union, 0xB001 + i as u64);
        }
    }

    #[test]
    fn intersection_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(
                name,
                BooleanOp::Intersection,
                0xB101 + i as u64,
            );
        }
    }

    #[test]
    fn difference_matches_point_membership() {
        for (i, name) in MEMBERSHIP_SCENES.iter().enumerate() {
            check_boolean_matches_point_membership(name, BooleanOp::Difference, 0xB201 + i as u64);
        }
    }

    #[test]
    fn union_contains_either_operand() {
        let (model, r) = run(BooleanOp::Union);
        assert!(contains(&model, r, IN_CUBE_ONLY), "cube-only point");
        assert!(contains(&model, r, IN_CYLINDER_ONLY), "cylinder-only point");
        assert!(contains(&model, r, IN_BOTH), "shared point");
    }

    #[test]
    fn intersection_contains_only_the_overlap() {
        let (model, r) = run(BooleanOp::Intersection);
        assert!(
            !contains(&model, r, IN_CUBE_ONLY),
            "cube-only point must be out"
        );
        assert!(
            !contains(&model, r, IN_CYLINDER_ONLY),
            "cylinder-only point must be out"
        );
        assert!(contains(&model, r, IN_BOTH), "shared point must be in");
    }

    #[test]
    fn difference_removes_the_second_operand() {
        let (model, r) = run(BooleanOp::Difference);
        assert!(
            contains(&model, r, IN_CUBE_ONLY),
            "cube-only point must remain"
        );
        assert!(
            !contains(&model, r, IN_CYLINDER_ONLY),
            "cylinder-only point must be out"
        );
        assert!(
            !contains(&model, r, IN_BOTH),
            "the drilled-out region must be gone"
        );
    }

    #[test]
    fn union_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Union);
    }

    #[test]
    fn intersection_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Intersection);
    }

    #[test]
    fn difference_of_box_and_cylinder_is_valid() {
        check_op("box_cylinder_drilled_hole_through", BooleanOp::Difference);
    }

    /// A chain of 3 differences (two axis-aligned slots cut from a block,
    /// then a sphere drilled out of the result) — captured from a browser
    /// session's timeline, exactly like `scenes`' hand-picked
    /// cases, built here directly from `geop_ops_extrude_revolve::shapes`
    /// and `boolean` rather than through `cad::Op`/wasm. Rendered to `outputs/` so the new
    /// curvature-adaptive rasterizer's output on a real chained-boolean
    /// result (flat cut faces plus the sphere's curved ones) can be
    /// inspected visually, the same way `scenes`' figure8/box
    /// cases are.
    // ── Chained booleans on the basic shapes, as the CAD front end builds them ──
    //
    // Each solid is created just before the boolean that uses it: every step
    // validates the *whole* model, and a solid not yet combined with anything
    // legitimately overlaps the others in space — which the validation would
    // report as edges crossing faces.

    type M = geop_ops::Part<ScalInF64>;

    fn v(x: f64, y: f64, z: f64) -> Vector3<ScalInF64> {
        Vector3::from_array([x, y, z].map(ScalInF64::from_f64))
    }

    /// `CreateCube(offset, dims)`: the axis-aligned box from `offset` to
    /// `offset + dims`.
    fn cube(part: &mut M, offset: [f64; 3], dims: [f64; 3]) -> geop_core_topology::SolidId {
        let [x, y, z] = offset;
        let [dx, dy, dz] = dims;
        let (min, max) = (v(x, y, z), v(x + dx, y + dy, z + dz));
        geop_ops_extrude_revolve::shapes::cube_solid(part, &fresh_id(), min, max).unwrap()
    }

    /// `CreateSphere(offset, r)`.
    fn sphere(part: &mut M, center: [f64; 3], r: f64) -> geop_core_topology::SolidId {
        let [x, y, z] = center;
        let r = ScalInF64::from_f64(r);
        geop_ops_extrude_revolve::shapes::sphere::sphere_solid(part, &fresh_id(), v(x, y, z), r)
            .unwrap()
    }

    /// `CreateCylinder(offset, r, h, axis)`: `offset` is the bottom cap's centre.
    fn cylinder(
        part: &mut M,
        base: [f64; 3],
        r: f64,
        h: f64,
        axis: geop_ops_extrude_revolve::shapes::cylinder::Axis,
    ) -> geop_core_topology::SolidId {
        let [x, y, z] = base;
        geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder_along_axis(
            part,
            &fresh_id(),
            v(x, y, z),
            ScalInF64::from_f64(r),
            ScalInF64::from_f64(h),
            axis,
        )
        .unwrap()
    }

    /// Run one boolean, requiring it to succeed with a non-empty solid that
    /// passes `validate_fast` and the full `validate`.
    fn op(
        part: &mut M,
        a: geop_core_topology::SolidId,
        b: geop_core_topology::SolidId,
        op: BooleanOp,
    ) -> geop_core_topology::SolidId {
        let result = boolean(part, &namer(), a, b, op, RemeshParams::default())
            .unwrap_or_else(|e| panic!("{op:?} failed: {e:?}"))
            .unwrap_or_else(|| panic!("{op:?} produced an empty solid"));
        part.check_names().unwrap();
        let model = part.topology();
        if let Err(errors) = validate_fast(&validation(), model) {
            panic!(
                "{op:?}: {} validate_fast error(s): {}",
                errors.len(),
                errors[0]
            );
        }
        // The full validation too: `validate_fast` checks each face on its
        // own, and passes an edge running through the middle of another face,
        // or two edges crossing away from any vertex. Not `validate_manifold`:
        // many of these results are genuinely non-manifold — solids touching
        // along a line leave edges shared by four faces.
        if let Err(errors) = geop_core_topology::validation::validate(&validation(), model) {
            panic!("{op:?}: {} validate error(s): {}", errors.len(), errors[0]);
        }
        // Closed, which neither check covers: every edge of the result is used
        // by an even number of coedges — two where two faces meet, four where
        // the solid touches itself along the edge. An odd count is a hole.
        for face in model.solid_faces(result).unwrap() {
            for coedge in model.iterate_face_coedges(face) {
                // A bare-vertex boundary has no edge to share.
                let Ok(edge) = model.get_coedge(coedge).unwrap().edge() else {
                    continue;
                };
                let uses = model.coedges_of_edge(edge).len();
                if uses % 2 != 0 {
                    let e = model.get_edge(edge).unwrap();
                    let at = |v| {
                        let p = model.get_vertex(v).unwrap().point;
                        [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
                    };
                    panic!(
                        "{op:?}: edge {edge} of face {face}, from {:?} to {:?}, is used by {uses} coedge(s) — the solid is open there",
                        at(e.start_vertex),
                        at(e.end_vertex)
                    );
                }
            }
        }
        result
    }

    /// Reported from the web front end: a cube with a bore through it, the
    /// sphere inscribed in the cube added back, then the `y > 0` half cut
    /// away. The last difference failed in `remesh_edges_x_edges` with "could
    /// not locate vertex on coedge's pcurve" at the cube corner
    /// `(-0.5, 0, -0.5)`, where the cutting cube's face meets an edge of the
    /// first result.
    #[test]
    fn bored_cube_plus_inscribed_sphere_minus_half() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, [-0.5, -0.5, -0.5], [1.0, 1.0, 1.0]);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let bored = op(&mut part, block, bore, BooleanOp::Difference);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let filled = op(&mut part, bored, ball, BooleanOp::Union);
        let half = cube(&mut part, [-0.5, 0.0, -0.5], [1.0, 1.0, 1.0]);
        op(&mut part, filled, half, BooleanOp::Difference);
    }

    /// Reported from the web front end: a cube with a flush inscribed bore
    /// (tangent to the four sides, caps coplanar with top and bottom), the
    /// inscribed sphere added back, then everything with `x < 0.05` cut away
    /// like a section view. The result looked open — two faces missing —
    /// with spurious triangles in the cutting plane. Not a manifold even when
    /// right: the bore touches the cube's sides along lines, and the one at
    /// `x = 0.5` survives the cut.
    #[test]
    fn flush_bored_cube_plus_inscribed_sphere_section() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        let bored = op(&mut part, block, bore, BooleanOp::Difference);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let filled = op(&mut part, bored, ball, BooleanOp::Union);
        let cutter = cube(&mut part, [-1.45, -1.05, -0.82], [1.5, 2.1, 1.65]);
        op(&mut part, filled, cutter, BooleanOp::Difference);
    }

    /// Like [`op`], for a boolean whose correct result is empty.
    fn op_empty(
        part: &mut M,
        a: geop_core_topology::SolidId,
        b: geop_core_topology::SolidId,
        op: BooleanOp,
    ) {
        let result = boolean(part, &namer(), a, b, op, RemeshParams::default())
            .unwrap_or_else(|e| panic!("{op:?} failed: {e:?}"));
        assert!(result.is_none(), "{op:?} should be empty");
    }

    const UNIT: [f64; 3] = [1.0, 1.0, 1.0];
    const CORNER: [f64; 3] = [-0.5, -0.5, -0.5];

    // Tangent contact. Where two surfaces touch without crossing, the
    // intersection is a curve or point along which a tiny error in one
    // surface moves the contact a long way (a perpendicular error `d` shifts a
    // tangency by `~sqrt(2 R d)`), which is what broke
    // `bored_cube_plus_inscribed_sphere_minus_half`.

    /// The cylinder touches all four side faces along vertical lines.
    #[test]
    fn cube_minus_inscribed_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    /// The cylinder is tangent to the four sides *and* its caps are flush
    /// with the top and bottom: each cap's rim lies in the cube's face and
    /// touches that face's four edges at their midpoints. The first step of
    /// `flush_bored_cube_plus_inscribed_sphere_section`.
    #[test]
    fn cube_minus_flush_inscribed_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    /// The sphere touches each face of the cube at its centre.
    #[test]
    fn cube_minus_inscribed_sphere() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Difference);
    }

    /// The sphere passes through all eight corners of the cube.
    #[test]
    fn cube_intersect_circumscribed_sphere() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 3f64.sqrt() / 2.0);
        op(&mut part, block, ball, BooleanOp::Intersection);
    }

    /// The cylinder's surface contains the cube's four vertical edges.
    #[test]
    fn cube_intersect_cylinder_through_its_edges() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5f64.sqrt(), 2.0, Axis::Z);
        op(&mut part, block, tube, BooleanOp::Intersection);
    }

    /// A coaxial cylinder of the sphere's radius touches it along the
    /// equator, a whole circle of tangency.
    #[test]
    fn sphere_union_tangent_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        op(&mut part, ball, tube, BooleanOp::Union);
    }

    /// A cube resting on the sphere's pole, where the sphere's patches all
    /// collapse to one point: tangent contact at a singular point.
    #[test]
    fn sphere_union_cube_touching_its_pole() {
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let block = cube(&mut part, [-0.5, -0.5, 0.5], UNIT);
        op(&mut part, ball, block, BooleanOp::Union);
    }

    /// A cylinder along x lying on the cube's top face: tangent along a line.
    #[test]
    fn cube_union_cylinder_lying_on_top() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let log = cylinder(&mut part, [-1.0, 0.0, 0.75], 0.25, 2.0, Axis::X);
        op(&mut part, block, log, BooleanOp::Union);
    }

    // Coincident features: shared faces, edges and corners, coplanar caps, and
    // identical solids.

    #[test]
    fn cubes_sharing_a_face_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, -0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn cubes_sharing_an_edge_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, 0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn cubes_sharing_a_corner_union() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.5, 0.5, 0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A half-overlapping cube: two of its faces are coplanar with the first
    /// cube's, over part of their area.
    #[test]
    fn cube_minus_offset_cube_with_coplanar_faces() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, [0.0, -0.5, -0.5], UNIT);
        op(&mut part, a, b, BooleanOp::Difference);
    }

    /// A hole whose caps are flush with the cube's top and bottom.
    #[test]
    fn cube_minus_flush_cylinder() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let bore = cylinder(&mut part, [0.0, 0.0, -0.5], 0.3, 1.0, Axis::Z);
        op(&mut part, block, bore, BooleanOp::Difference);
    }

    /// A plate 6 by 3 and a blind hole to drill into it from its top: a
    /// cylinder of radius 0.33 around `(3, 1.5)`, its start cap flush with
    /// the top.
    fn plate_and_blind_hole(
        part: &mut M,
    ) -> (geop_core_topology::SolidId, geop_core_topology::SolidId) {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let plate = cube(part, [0.0, 0.0, 0.0], [6.0, 3.0, 0.5]);
        let hole = cylinder(part, [3.0, 1.5, 0.2], 0.33, 0.3, Axis::Z);
        (plate, hole)
    }

    /// The plate and hole of [`plate_and_blind_hole`] remeshed, and the disc
    /// the remesh cuts from the plate's top, inside the hole's rim.
    fn plate_and_blind_hole_remeshed(
        part: &mut M,
    ) -> (geop_core_topology::SolidId, geop_core_topology::FaceId) {
        let (plate, hole) = plate_and_blind_hole(part);
        crate::remesh::remesh::remesh(part, &namer(), plate, hole, RemeshParams::default())
            .unwrap();
        let model = part.topology();
        if let Err(errors) = validate_fast(&validation(), model) {
            panic!("{} validate_fast error(s): {}", errors.len(), errors[0]);
        }
        let in_rim = |p: &[f64; 3]| (p[0] - 3.0).hypot(p[1] - 1.5) < 0.34 && p[2] > 0.49;
        let disc = model
            .solid_faces(plate)
            .unwrap()
            .into_iter()
            .find(|&face| {
                let corners = model.face_corners(face).unwrap();
                corners.len() == 1 && corners[0].iter().all(in_rim)
            })
            .expect("a face of the plate inside the hole's rim");
        (hole, disc)
    }

    /// Reported as the parametric plate made 6 wide with an M6 hole: the
    /// result was left open along the hole's rim, as the disc cut from the
    /// top was classified outside the hole rather than on its cap.
    #[test]
    fn plate_minus_blind_hole_from_its_top() {
        let mut part = M::new();
        let (plate, hole) = plate_and_blind_hole(&mut part);
        op(&mut part, plate, hole, BooleanOp::Difference);
    }

    /// The disc cut from the plate's top lies on the hole's start cap, so
    /// every point a boolean classifies it by is on the hole's boundary.
    #[test]
    fn blind_hole_cap_covers_the_disc_cut_from_the_top() {
        use geop_core_topology::contains::{
            face::face_interior_point_where,
            shell::{PointClassification, shell_contains},
        };
        let mut part = M::new();
        let (hole, disc) = plate_and_blind_hole_remeshed(&mut part);
        let model = part.topology();
        let params = RemeshParams::default();
        let shell = model.get_solid(hole).unwrap().shells[0];
        let mut checked = 0;
        face_interior_point_where(
            model,
            disc,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            super::SEED,
            |u, v| {
                let point = model.get_face(disc)?.surface.evaluate(u, v)?;
                let class = shell_contains(
                    model,
                    shell,
                    point,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    super::SEED,
                )?;
                assert!(
                    !matches!(
                        class,
                        PointClassification::Inside | PointClassification::Outside
                    ),
                    "{point:?} of the disc, at uv=({u:?}, {v:?}), is {class:?}"
                );
                checked += 1;
                Ok(false)
            },
        )
        .unwrap();
        assert!(checked > 0);
    }

    /// The point the disc cut from the plate's top was once classified by:
    /// beside it — 0.44 from the hole's axis, the rim 0.33 — and outside it,
    /// as a point and as the box around it `face_interior_point_where` asks
    /// about. With the boolean's seed, the first ray from the box grazes the
    /// rim; cast from the box itself, it counted that as one crossing.
    #[test]
    fn point_beside_the_disc_cut_from_the_top_is_outside_it() {
        use geop_core_math::scalars::Ring;
        use geop_core_topology::contains::face::{PointClassification, face_contains};
        let mut part = M::new();
        let (_, disc) = plate_and_blind_hole_remeshed(&mut part);
        let model = part.topology();
        let (u, v) = (
            ScalInF64::from_f64(0.42698015887332674),
            ScalInF64::from_f64(0.5220523370528238),
        );
        let point = model
            .get_face(disc)
            .unwrap()
            .surface
            .evaluate(u, v)
            .unwrap();
        assert!(
            (point[0].to_f64() - 3.0).hypot(point[1].to_f64() - 1.5) > 0.4,
            "{point:?}"
        );
        let params = RemeshParams::default();
        let epsilon = params.curve_curve_min_subdivision_size;
        let neighbourhood = |t: ScalInF64| t.sub(epsilon).union(t.add(epsilon));
        let mut wrong = Vec::new();
        for (u, v) in [(u, v), (neighbourhood(u), neighbourhood(v))] {
            for seed in (0..64).chain([super::SEED]) {
                let class =
                    face_contains(model, disc, u, v, params.max_nodes, epsilon, seed).unwrap();
                if class != PointClassification::Outside {
                    wrong.push(format!("{class:?} at ({u:?}, {v:?}) with seed {seed}"));
                }
            }
        }
        assert!(wrong.is_empty(), "{}", wrong.join("\n"));
    }

    #[test]
    fn identical_spheres_union() {
        let mut part = M::new();
        let a = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let b = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn identical_cubes_difference_is_empty() {
        let mut part = M::new();
        let a = cube(&mut part, CORNER, UNIT);
        let b = cube(&mut part, CORNER, UNIT);
        op_empty(&mut part, a, b, BooleanOp::Difference);
    }

    /// Every operator on two copies of one shape — same geometry, separate
    /// topology with ids of their own — so that each face of either copy
    /// coincides with a face of the other over its whole area, and every edge
    /// and vertex lies exactly on one of the other copy's. Union and
    /// intersection must give the shape back, difference must be empty.
    fn check_duplicated_shape(make: impl Fn(&mut M) -> geop_core_topology::SolidId) {
        let probes = [(0.0, 0.0, 0.0), (0.2, -0.1, 0.15), (0.9, 0.0, 0.0)];
        let inside_original = {
            let mut part = M::new();
            let shape = make(&mut part);
            probes.map(|p| contains(part.topology(), shape, p))
        };
        for operator in [BooleanOp::Union, BooleanOp::Intersection] {
            let mut part = M::new();
            let a = make(&mut part);
            let b = make(&mut part);
            let result = op(&mut part, a, b, operator);
            for (p, inside) in probes.iter().zip(inside_original) {
                assert_eq!(
                    contains(part.topology(), result, *p),
                    inside,
                    "{operator:?} of two copies must contain {p:?} exactly when one copy does"
                );
            }
        }
        let mut part = M::new();
        let a = make(&mut part);
        let b = make(&mut part);
        op_empty(&mut part, a, b, BooleanOp::Difference);
    }

    #[test]
    fn duplicated_cube_all_operators() {
        check_duplicated_shape(|part| cube(part, CORNER, UNIT));
    }

    #[test]
    fn duplicated_sphere_all_operators() {
        check_duplicated_shape(|part| sphere(part, [0.0, 0.0, 0.0], 0.5));
    }

    #[test]
    fn duplicated_cylinder_all_operators() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        check_duplicated_shape(|part| cylinder(part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z));
    }

    /// A sphere centred on the cube's corner: its three coordinate-plane
    /// seams lie exactly in three faces of the cube.
    #[test]
    fn cube_minus_sphere_on_its_corner() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.5, 0.5, 0.5], 0.5);
        op(&mut part, block, ball, BooleanOp::Difference);
    }

    // Curved against curved.

    /// Two equal cylinders crossing at right angles: their intersection
    /// curves (two ellipses) cross each other at two singular points.
    #[test]
    fn steinmetz_cylinders_union() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let b = cylinder(&mut part, [-1.0, 0.0, 0.0], 0.5, 2.0, Axis::X);
        op(&mut part, a, b, BooleanOp::Union);
    }

    #[test]
    fn steinmetz_cylinders_intersection() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let b = cylinder(&mut part, [-1.0, 0.0, 0.0], 0.5, 2.0, Axis::X);
        op(&mut part, a, b, BooleanOp::Intersection);
    }

    /// A bore along the sphere's axis, through both poles.
    #[test]
    fn sphere_minus_cylinder_through_its_poles() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let bore = cylinder(&mut part, [0.0, 0.0, -1.0], 0.2, 2.0, Axis::Z);
        op(&mut part, ball, bore, BooleanOp::Difference);
    }

    /// A sphere centred on a cylinder's cap: the cap cuts it along its
    /// equator, which is also where its patches meet.
    #[test]
    fn cylinder_union_sphere_on_its_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 1.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.3);
        op(&mut part, tube, ball, BooleanOp::Union);
    }

    /// The same with the sphere's radius equal to the cylinder's: the
    /// sphere's equator coincides with the cap's rim, *and* the sphere is
    /// tangent to the cylinder's side along that same circle.
    #[test]
    fn cylinder_union_equal_sphere_on_its_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 1.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, tube, ball, BooleanOp::Union);
    }

    /// External tangency at a point of the sphere's equator (not a pole).
    #[test]
    fn cube_union_sphere_touching_a_face() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [1.0, 0.0, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// A sphere half sunk into the cube through the centre of a face.
    #[test]
    fn cube_union_sphere_on_a_face() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let ball = sphere(&mut part, [0.5, 0.0, 0.0], 0.3);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// Two inscribed bores at right angles: the first two steps of
    /// `cube_minus_three_inscribed_bores`. The second bore's wall meets the
    /// first's in two Steinmetz ellipses, which end exactly where both bores
    /// touch the cube's faces — at the midpoints of its edges.
    #[test]
    fn cube_minus_two_inscribed_bores() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.5, 1.75, Axis::X);
        op(&mut part, a, x, BooleanOp::Difference);
    }

    /// Remesh `a` against `b`, then require an edge shared by faces of both
    /// between each pair of points in `arcs`, and the full validation to
    /// pass.
    ///
    /// The full validation cannot see a missing intersection arc that ends
    /// at vertices both solids share: faces sharing a vertex are not
    /// searched for crossings. A boolean only notices one as a face it cannot
    /// classify, or a result that is open, far from the cause. So the arcs a
    /// scene must have are named, and on failure the edges it does have are
    /// listed.
    fn check_remesh_imprints(
        part: &mut M,
        a: geop_core_topology::SolidId,
        b: geop_core_topology::SolidId,
        arcs: &[[[f64; 3]; 2]],
    ) {
        crate::remesh::remesh::remesh(part, &namer(), a, b, RemeshParams::default())
            .unwrap_or_else(|e| panic!("remesh failed: {e}"));
        let model = part.topology();
        let faces_a = model.solid_faces(a).unwrap();
        let faces_b = model.solid_faces(b).unwrap();
        let mut shared = Vec::new();
        for (&edge_id, edge) in &model.edges {
            let faces: Vec<_> = model
                .coedges_of_edge(edge_id)
                .iter()
                .map(|&c| model.get_coedge(c).unwrap().face)
                .collect();
            if faces.iter().any(|f| faces_a.contains(f))
                && faces.iter().any(|f| faces_b.contains(f))
            {
                shared.push([edge.start_vertex, edge.end_vertex].map(|v| {
                    let p = model.get_vertex(v).unwrap().point;
                    [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
                }));
            }
        }
        // Telling which known point a vertex is, not a kernel comparison:
        // the expected points are well apart, so any coarse tolerance names
        // them unambiguously.
        let near = |p: [f64; 3], q: [f64; 3]| (0..3).all(|i| (p[i] - q[i]).abs() < 1e-4);
        let missing: Vec<_> = arcs
            .iter()
            .filter(|[p, q]| {
                !shared
                    .iter()
                    .any(|[s, e]| (near(*s, *p) && near(*e, *q)) || (near(*s, *q) && near(*e, *p)))
            })
            .collect();
        assert!(
            missing.is_empty(),
            "no edge shared by both solids between {missing:?}; the shared edges are {shared:?}"
        );

        if let Err(errors) = geop_core_topology::validation::validate(&validation(), model) {
            let all: Vec<String> = errors.iter().map(|e| format!("{e}")).collect();
            panic!(
                "{} validate error(s):\n{}",
                errors.len(),
                all.join("\n---\n")
            );
        }
    }

    /// The remesh behind `cube_minus_two_inscribed_bores`' second step: each
    /// Steinmetz ellipse `x = ±z` of the two walls runs through midpoints
    /// `(±0.5, 0, ±0.5)` of the cube's edges and through `(0, ±0.5, 0)`,
    /// where the two ellipses cross with the walls tangent — so each of its
    /// eight arcs can only be traced from the midpoint end.
    #[test]
    fn two_inscribed_bores_remesh_imprints_every_arc() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.5, 1.75, Axis::X);
        let mut arcs = Vec::new();
        for (mx, mz) in [(0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5)] {
            for cy in [0.5, -0.5] {
                arcs.push([[mx, 0.0, mz], [0.0, cy, 0.0]]);
            }
        }
        check_remesh_imprints(&mut part, a, x, &arcs);
    }

    /// The remesh behind `cube_minus_three_inscribed_bores`' third step. In
    /// each octant the three walls meet at one point, `(±1, ±1, ±1) / sqrt 8`,
    /// where the arc the third wall cuts from the first meets the arc it cuts
    /// from the second: one runs from there to `(0, ±0.5, ±0.5)`, the other
    /// to `(±0.5, ±0.5, 0)`. That point is interior to the third wall, so both
    /// arcs leave it into the same face.
    #[test]
    fn three_inscribed_bores_remesh_imprints_every_arc() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.5, 1.75, Axis::X);
        let b = op(&mut part, a, x, BooleanOp::Difference);
        let y = cylinder(&mut part, [0.0, -0.875, 0.0], 0.5, 1.75, Axis::Y);
        let t = 0.125f64.sqrt();
        let mut arcs = Vec::new();
        for sx in [1.0, -1.0] {
            for sy in [1.0, -1.0] {
                for sz in [1.0, -1.0] {
                    let triple = [sx * t, sy * t, sz * t];
                    arcs.push([triple, [0.0, sy * 0.5, sz * 0.5]]);
                    arcs.push([triple, [sx * 0.5, sy * 0.5, 0.0]]);
                }
            }
        }
        check_remesh_imprints(&mut part, b, y, &arcs);
    }

    /// Inscribed bores along all three axes, one after another (a "jack").
    /// Each bore is tangent to four faces, and each later bore crosses the
    /// earlier ones at Steinmetz points.
    #[test]
    fn cube_minus_three_inscribed_bores() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.5, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.5, 1.75, Axis::X);
        let b = op(&mut part, a, x, BooleanOp::Difference);
        let y = cylinder(&mut part, [0.0, -0.875, 0.0], 0.5, 1.75, Axis::Y);
        op(&mut part, b, y, BooleanOp::Difference);
    }

    /// Thinner bores, so the cube keeps its faces: two crossing bores.
    #[test]
    fn cube_minus_two_crossing_bores() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let z = cylinder(&mut part, [0.0, 0.0, -0.875], 0.3, 1.75, Axis::Z);
        let a = op(&mut part, block, z, BooleanOp::Difference);
        let x = cylinder(&mut part, [-0.875, 0.0, 0.0], 0.3, 1.75, Axis::X);
        op(&mut part, a, x, BooleanOp::Difference);
    }

    // More tangencies, and a longer chain.

    /// Two spheres touching externally at a single point.
    #[test]
    fn touching_spheres_union() {
        let mut part = M::new();
        let a = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        let b = sphere(&mut part, [1.0, 0.0, 0.0], 0.5);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A sphere inside a cylinder of the same radius, touching it along the
    /// equator from inside.
    #[test]
    fn cylinder_minus_inscribed_sphere() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let tube = cylinder(&mut part, [0.0, 0.0, -1.0], 0.5, 2.0, Axis::Z);
        let ball = sphere(&mut part, [0.0, 0.0, 0.0], 0.5);
        op(&mut part, tube, ball, BooleanOp::Difference);
    }

    /// Two parallel cylinders touching along a line.
    #[test]
    fn touching_parallel_cylinders_union() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let a = cylinder(&mut part, [0.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        let b = cylinder(&mut part, [1.0, 0.0, -0.5], 0.5, 1.0, Axis::Z);
        op(&mut part, a, b, BooleanOp::Union);
    }

    /// A sphere touching one of the cube's edges from outside.
    #[test]
    fn cube_union_sphere_touching_an_edge() {
        let mut part = M::new();
        let block = cube(&mut part, CORNER, UNIT);
        let d = 0.5 + 0.5 / 2f64.sqrt();
        let ball = sphere(&mut part, [d, d, 0.0], 0.5);
        op(&mut part, block, ball, BooleanOp::Union);
    }

    /// A small part as a user might build it: a plate with a boss and a
    /// through hole, then a rounded cap on the boss.
    #[test]
    fn plate_with_boss_hole_and_cap() {
        use geop_ops_extrude_revolve::shapes::cylinder::Axis;
        let mut part = M::new();
        let plate = cube(&mut part, [-1.0, -1.0, -0.25], [2.0, 2.0, 0.5]);
        let boss = cylinder(&mut part, [0.0, 0.0, 0.25], 0.5, 0.5, Axis::Z);
        let a = op(&mut part, plate, boss, BooleanOp::Union);
        let cap = sphere(&mut part, [0.0, 0.0, 0.75], 0.5);
        let b = op(&mut part, a, cap, BooleanOp::Union);
        let hole = cylinder(&mut part, [0.0, 0.0, -0.5], 0.25, 1.5, Axis::Z);
        op(&mut part, b, hole, BooleanOp::Difference);
    }

    /// The second step of `chained_differences_block_with_two_slots_and_a_sphere`,
    /// stopped after remesh: a point of the second slot's side face, above the
    /// block, must classify as outside the slotted block. It came back
    /// `Inside` — a ray-parity miscount that the boolean only stopped hiding
    /// once `classify_face` checked every interior point it sampled.
    #[test]
    fn chained_slots_point_above_the_block_is_outside() {
        let f = ScalInF64::from_f64;
        let corner = |x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64| {
            (
                Vector3::from_array([f(x), f(y), f(z)]),
                Vector3::from_array([f(x + dx), f(y + dy), f(z + dz)]),
            )
        };
        let mut part = M::new();
        let (min_a, max_a) = corner(-0.50, -0.50, -0.50, 1.00, 1.00, 1.00);
        let a = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "a", min_a, max_a).unwrap();
        let (min_b, max_b) = corner(-1.13, -0.30, -0.33, 2.25, 0.60, 0.65);
        let b = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "b", min_b, max_b).unwrap();
        let c = op(&mut part, a, b, BooleanOp::Difference);
        let (min_d, max_d) = corner(-0.33, -0.28, -1.15, 0.65, 0.55, 2.30);
        let d = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "d", min_d, max_d).unwrap();
        crate::remesh::remesh::remesh(&mut part, &namer(), c, d, RemeshParams::default()).unwrap();

        let params = RemeshParams::<ScalInF64>::default();
        let shell = part.topology().get_solid(c).unwrap().shells[0];
        let classification = geop_core_topology::contains::shell::shell_contains(
            part.topology(),
            shell,
            v(-0.005, -0.28, 0.825),
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            super::SEED,
        )
        .unwrap();
        assert_eq!(
            classification,
            geop_core_topology::contains::shell::PointClassification::Outside
        );
    }

    #[test]
    fn chained_differences_block_with_two_slots_and_a_sphere() {
        let f = ScalInF64::from_f64;
        let corner = |x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64| {
            (
                Vector3::from_array([f(x), f(y), f(z)]),
                Vector3::from_array([f(x + dx), f(y + dy), f(z + dz)]),
            )
        };

        let mut part = M::new();
        let (min_a, max_a) = corner(-0.50, -0.50, -0.50, 1.00, 1.00, 1.00);
        let a = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "a", min_a, max_a).unwrap();
        let (min_b, max_b) = corner(-1.13, -0.30, -0.33, 2.25, 0.60, 0.65);
        let b = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "b", min_b, max_b).unwrap();
        let params = RemeshParams::<ScalInF64>::default();
        let c = boolean(&mut part, &namer(), a, b, BooleanOp::Difference, params)
            .unwrap()
            .expect("block minus the first slot must be non-empty");

        let (min_d, max_d) = corner(-0.33, -0.28, -1.15, 0.65, 0.55, 2.30);
        let d = geop_ops_extrude_revolve::shapes::cube_solid(&mut part, "d", min_d, max_d).unwrap();
        let e = boolean(&mut part, &namer(), c, d, BooleanOp::Difference, params)
            .unwrap()
            .expect("minus the second slot must be non-empty");

        let sphere = geop_ops_extrude_revolve::shapes::sphere::sphere_solid(
            &mut part,
            "s",
            Vector3::zero(),
            f(0.45),
        )
        .unwrap();
        let result = boolean(
            &mut part,
            &namer(),
            e,
            sphere,
            BooleanOp::Difference,
            params,
        )
        .unwrap()
        .expect("minus the sphere must be non-empty");
        part.check_names().unwrap();

        let model = part.topology();
        assert!(!model.solid_faces(result).unwrap().is_empty());
        if let Err(errors) = validate_fast(&validation(), model) {
            panic!("{} validate_fast error(s): {}", errors.len(), errors[0]);
        }

        let scene = geop_ops_rasterize::rasterize(model, 8)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue);
        std::fs::create_dir_all("outputs").unwrap();
        scene
            .save_to_file("outputs/chained_differences_block_with_two_slots_and_a_sphere.html")
            .unwrap();
    }

    /// Regression test, captured from a browser session: two cubes
    /// differenced, then a sphere differenced out of that, then a thin slab
    /// cube differenced out of *that*. The second cube and the sphere were
    /// anchored to corners of the first cube, picked by kernel id
    /// (`VertexId(13)` and `VertexId(9)`, its corners `p3,start` and
    /// `p2,start`); their coordinates as the session read them are given
    /// here, to the last bit.
    ///
    /// Used to fail inside the final boolean's `classify_face`:
    /// `shell_contains` reported a point on the slab's boundary that no face
    /// of that shell's `face_contains` agreed contained. Root cause:
    /// `shell_contains`'s face coincidence pre-check only asked
    /// `surface_could_contain` — proximity to a face's *untrimmed* surface —
    /// without checking the point fell within the face's trim. Proximity is
    /// not membership (see `AGENTS.md`).
    #[test]
    fn chained_differences_with_anchored_shapes_and_thin_slab_cutter_succeeds() {
        let mut part = M::new();
        let block = cube(&mut part, [-0.5, -0.5, -0.5], [1.0, 1.0, 1.0]);
        let [x, y, z] = [-0.5000000000000001, 0.5000000000000002, 0.5000000000000001];
        let second = cube(&mut part, [x - 0.5, y - 0.5, z - 0.5], [1.0, 1.0, 1.0]);
        let center = [0.5000000000000003, 0.5000000000000002, 0.5000000000000001];
        let ball = sphere(&mut part, center, 0.5);
        // Only success is asserted, as when this was captured: the full
        // `validate` that `op` runs finds an edge of the second result
        // crossing a face unsplit, which this regression never covered.
        let difference = |part: &mut M, a, b| {
            boolean(
                part,
                &namer(),
                a,
                b,
                BooleanOp::Difference,
                RemeshParams::default(),
            )
            .unwrap()
            .expect("the result is not empty")
        };
        let cut = difference(&mut part, block, second);
        let cut = difference(&mut part, cut, ball);
        let slab = cube(&mut part, [-1.13, -0.10, -0.15], [2.25, 0.20, 0.30]);
        difference(&mut part, cut, slab);
        part.check_names().unwrap();
    }

    /// Regression test: a cylinder drilled through a cube, its height
    /// chosen so *both* caps sit exactly flush with the cube's own
    /// opposite faces (`cylinder z in [-0.5, 0.5]` exactly matching the
    /// cube's), rather than the usual "drilled hole" test scenes (e.g.
    /// `box_cylinder_drilled_hole_through`) where the cylinder
    /// deliberately extends *past* both faces.
    ///
    /// That exact-coplanar-cap case used to (1) take ~30s — over 1000x the
    /// sub-second time every other boolean test in this file takes — and
    /// (2) produce a *wrong* result, cutting the hole through only one of
    /// the two coplanar-cap faces and leaving the other fully solid.
    ///
    /// Root cause: `revolve_at_oriented` builds any flat cap (a
    /// doubly-curved pole, like a sphere's, genuinely needs a fan of
    /// wedges meeting at a center vertex — a flat cap doesn't, but got the
    /// same treatment) as 4 wedges with radial "spoke" edges from rim to
    /// center. `find_coincident_pair` (`remesh_edges_x_faces.rs`) couldn't
    /// tell those spokes apart from the genuine circular rim boundary, so
    /// it tried imprinting all of them — the slowdown, and (via
    /// interleaved imprints across the two caps corrupting
    /// `Model::splice_edge_into_face`'s face-splitting bookkeeping) the
    /// wrong result. Fixed by `seam_plane`/`faces_are_coplanar`
    /// there: an edge whose two neighboring faces (within its own solid)
    /// are already coplanar with each other is never the only carrier of a
    /// genuine coincidence, so it's skipped — cutting straight to the rim
    /// arcs.
    #[test]
    fn cube_minus_z_cylinder_with_coplanar_cap_is_fast_and_correct() {
        let f = ScalInF64::from_f64;
        let mut part = M::new();
        let a = geop_ops_extrude_revolve::shapes::cube_solid(
            &mut part,
            "a",
            Vector3::from_array([f(-0.5), f(-0.5), f(-0.5)]),
            Vector3::from_array([f(0.5), f(0.5), f(0.5)]),
        )
        .unwrap();
        let b = geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder_along_axis(
            &mut part,
            "b",
            Vector3::from_array([f(0.0), f(0.0), f(-0.5)]),
            f(0.3),
            f(1.0),
            geop_ops_extrude_revolve::shapes::cylinder::Axis::Z,
        )
        .unwrap();
        let params = RemeshParams::<ScalInF64>::default();

        let t0 = std::time::Instant::now();
        let result = boolean(&mut part, &namer(), a, b, BooleanOp::Difference, params)
            .unwrap()
            .unwrap();
        let elapsed = t0.elapsed();
        let model = part.topology();
        // Ballpark-matches `difference_of_box_and_cylinder_is_valid`'s own
        // (non-coincident-cap) ~2s in isolation; running inside the full
        // suite under CPU contention from other parallel tests has been
        // observed up to ~12s. The threshold stays generous — the point is
        // catching a regression back toward the old ~32s (a *further*
        // 1000x-ish blowup on top of ordinary parallel-run noise), not
        // pinning down exact timing.
        assert!(
            elapsed.as_secs() < 20,
            "boolean took {elapsed:?}, expected well under 20s"
        );

        // Every point on both cap planes' hole boundary must be *outside*
        // the result (the cylinder's full bore is open at both ends) —
        // catches "hole only cut on one end" directly, unlike
        // `validate_fast`.
        let shell = model.get_solid(result).unwrap().shells[0];
        for z in [f(-0.45), f(0.45)] {
            let p = Vector3::from_array([f(0.0), f(0.0), z]);
            let outside = matches!(
                geop_core_topology::contains::shell::shell_contains(
                    model,
                    shell,
                    p,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    0xC7D1
                )
                .unwrap(),
                geop_core_topology::contains::shell::PointClassification::Outside
            );
            assert!(
                outside,
                "point {p:?} (near a cap's bore) should be outside the drilled result"
            );
        }

        let scene = geop_ops_rasterize::rasterize(model, 8)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue);
        std::fs::create_dir_all("outputs").unwrap();
        scene
            .save_to_file("outputs/cube_minus_coplanar_cap_cylinder.html")
            .unwrap();
    }

    /// A bore through a block must actually be empty in the rendered mesh:
    /// no triangle may cover the hole. End-to-end cover for the trimming
    /// that `geop_ops_rasterize::clip` does per grid cell — where a
    /// duplicated vertex in a clipped hole outline, or a concave fragment
    /// classified as a whole, used to leave a flap of surface hanging
    /// across an opening (isolated in that module's own tests).
    #[test]
    fn bore_renders_as_a_hole() {
        let mut part = M::new();
        let block = cube(&mut part, [-1.0, -1.0, 0.0], [2.0, 2.0, 0.5]);
        // Radius 0.5 of the 2x2 footprint puts the bore's outline exactly
        // through grid cell corners at this resolution, which is what it
        // takes to produce the duplicated vertex.
        let bore = cylinder(
            &mut part,
            [0.0, 0.0, -0.5],
            0.5,
            1.5,
            geop_ops_extrude_revolve::shapes::cylinder::Axis::Z,
        );
        op(&mut part, block, bore, BooleanOp::Difference);

        let rasterized = geop_ops_rasterize::rasterize(part.topology(), 24).unwrap();
        for triangle in rasterized.faces.values().flatten() {
            for p in [triangle.a, triangle.b, triangle.c] {
                let (x, y, z) = (p[0].to_f64(), p[1].to_f64(), p[2].to_f64());
                let r = x.hypot(y);
                // Strictly inside the bore, and within the block's height:
                // nothing may be drawn there. The margin keeps the bore
                // wall's own triangles (at r = 0.6, faceted slightly inwards)
                // out of the test.
                assert!(
                    r > 0.45 || !(0.01..0.49).contains(&z),
                    "a triangle corner sits inside the bore at ({x}, {y}, {z})"
                );
            }
        }
    }

    /// A box cut into the top of a cube, its corners known only to within
    /// 1e-12 — as a tool computed from a solid's own (interval) vertices
    /// is: a lip's groove offset from a shelled enclosure's rim. The same
    /// box with sharp corners was always cut cleanly.
    ///
    /// The march fed each corrector step the previous step's honest
    /// `(u, v)` as its first seed, unsharpened, so the width compounded
    /// step after step: from the corners' 1e-12 to a whole patch within a
    /// dozen steps, until the corrector no longer converged (see
    /// `predictor_corrector_step`).
    #[test]
    fn cube_minus_box_with_wide_corners() {
        let wide = |x: f64| ScalInF64::new(x - 1e-12, x + 1e-12);
        let corner = |p: [f64; 3]| Vector3::from_array(p.map(wide));
        let mut part = M::new();
        let block = cube(&mut part, [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
        let tool = geop_ops_extrude_revolve::shapes::cube_solid(
            &mut part,
            &fresh_id(),
            corner([1.72, 0.1, 0.88]),
            corner([1.9, 1.9, 1.08]),
        )
        .unwrap();
        op(&mut part, block, tool, BooleanOp::Difference);
    }
}
