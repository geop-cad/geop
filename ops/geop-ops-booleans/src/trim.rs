//! Trimming a sheet "up to next": of a sheet swept from a profile, keeping
//! only what its start reaches before it meets a solid's boundary.
//!
//! The solid is imprinted onto the sheet exactly as a boolean imprints two
//! bodies ([`remesh`]) — but a copy of it, so the solid itself is left as it
//! is: a sheet stopping at a solid does nothing to the solid. After that,
//! every face of the sheet lies inside the solid, outside it, or on its
//! boundary, and where the solid's boundary crosses the sheet there is an
//! edge between faces on either side of it. The piece kept is the connected
//! set of faces on the start's side, joined across edges the solid does not
//! cross.

use std::collections::{HashMap, HashSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
};
use geop_core_topology::{Body, CoedgeGeometry, EdgeId, FaceId, ShellId, SolidId};
use geop_ops::{Namer, Part};

use crate::{
    boolean::classify_face,
    remesh::remesh::{RemeshParams, remesh},
};

/// Trims the sheet `sheet` up to the next face of any of `targets`: keeps
/// the piece of it that its edges named `start` lie on — up to where it
/// meets their boundary — and drops the rest. `targets` are not changed.
///
/// What the trimming creates is named by `namer` as a boolean names what it
/// creates (see [`crate::naming`]). Fails if the piece also reaches an edge
/// named in `end`: nothing stopped the sheet.
pub fn trim_up_to_next<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    sheet: ShellId,
    targets: &[SolidId],
    (start, end): (&HashSet<String>, &HashSet<String>),
    params: RemeshParams<S>,
) -> GeopResult<ShellId> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "trim_up_to_next(name={}, sheet={sheet}, targets={targets:?})",
            namer.root()
        ))
    };
    // A copy of the target to imprint, so the target stays as it is.
    let mut target_faces = Vec::new();
    for &target in targets {
        target_faces.extend(part.topology().solid_faces(target).with_context(&ctx)?);
    }
    let copy = part
        .copy_faces(&target_faces, Some(namer.name(&["copy"])), |name| {
            namer.name(&["copy", name])
        })
        .with_context(&ctx)?;
    let copy = copy.solid.expect("built as a solid");

    let origins = remesh(part, namer, Body::Sheet(sheet), copy, params).with_context(&ctx)?;
    let model = part.topology();
    let faces = model.body_faces(Body::Sheet(sheet)).with_context(&ctx)?;
    let copy_faces: HashSet<FaceId> = model
        .solid_faces(copy)
        .with_context(&ctx)?
        .into_iter()
        .collect();
    let mut classes = Vec::with_capacity(faces.len());
    for &face in &faces {
        classes.push(
            classify_face(model, face, copy, params)
                .with_context(&ctx)
                .with_context(&|e: GeopError| e.with_context(format!("classifying face {face}")))?,
        );
    }

    // Faces on one side of the target, sharing an edge the target does not
    // cross, are one piece.
    let mut users: HashMap<EdgeId, Vec<usize>> = HashMap::new();
    let mut crossed: HashSet<EdgeId> = HashSet::new();
    for (k, &face) in faces.iter().enumerate() {
        for coedge in model.iterate_face_coedges(face) {
            if let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry {
                users.entry(edge).or_default().push(k);
            }
        }
    }
    for &face in &copy_faces {
        for coedge in model.iterate_face_coedges(face) {
            if let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry {
                crossed.insert(edge);
            }
        }
    }
    let mut pieces = UnionFind::new(faces.len());
    for (edge, users) in &users {
        if crossed.contains(edge) {
            continue;
        }
        for pair in users.windows(2) {
            if classes[pair[0]] == classes[pair[1]] {
                pieces.union(pair[0], pair[1]);
            }
        }
    }
    let on = |names: &HashSet<String>, edge: &EdgeId| {
        origins
            .edges
            .get(edge)
            .is_some_and(|origin| names.contains(origin))
    };
    let mut reached = HashSet::new();
    for (edge, users) in &users {
        if on(start, edge) {
            reached.extend(users.iter().map(|&k| pieces.find(k)));
        }
    }
    if reached.is_empty() {
        return Err(ctx(GeopError::new("trim: the sheet's start is gone")));
    }
    if users.iter().any(|(edge, users)| {
        on(end, edge) && users.iter().any(|&k| reached.contains(&pieces.find(k)))
    }) {
        return Err(ctx(GeopError::new(
            "up to next: nothing stops the face — it reaches its end without meeting the target all along",
        )));
    }
    let keep: Vec<FaceId> = (0..faces.len())
        .filter(|&k| reached.contains(&pieces.find(k)))
        .map(|k| faces[k])
        .collect();
    let kept = part
        .assemble_sheet(&[Body::Sheet(sheet), copy.into()], &keep)
        .with_context(&ctx)?;
    Ok(kept.expect("the start's piece is kept"))
}
