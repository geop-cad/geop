//! Splitting a solid into pieces with a sheet.
//!
//! The sheet is imprinted onto the solid exactly as a boolean imprints two
//! solids ([`remesh`]), after which every piece of the sheet lies wholly
//! inside the solid or wholly outside it. The pieces inside are where the
//! solid is cut. Each becomes a face of *two* pieces of the solid: as it is
//! on the side its normal points away from, turned around on the other.
//!
//! Which faces bound one piece is then purely topological. A cut face meets
//! the solid's boundary along the edges where the sheet crossed it, with one
//! face of the solid on either side; of the two, the one running along the
//! edge the other way than the cut face does is the one on the cut face's
//! side — two faces of a closed, oriented shell always run their shared edge
//! in opposite directions. Elsewhere, faces sharing an edge bound the same
//! piece. So the pieces are the connected sets of "face sides", and each is
//! built as a solid of its own: the cut sides it shares with its neighbours
//! are copies, so the pieces share nothing.

use std::collections::{HashMap, HashSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    union_find::UnionFind,
};
use geop_core_topology::{
    Body, CoedgeGeometry, EdgeId, FaceId, ShellId, SolidId,
    build::{BodySpec, BuiltBody},
};
use geop_ops::{BodyNames, Namer, Part, RefId};

use crate::{
    boolean::{Classified, FaceClassification, classify_face},
    remesh::remesh::{RemeshParams, remesh},
};

/// Splits `solid` along the sheet `sheet` into the pieces it cuts the solid
/// into, each a solid of its own. The solid is consumed; the sheet is left as
/// it is — a copy of it does the cutting.
///
/// The pieces are named `N(0)`, `N(1)`, ..., ordered by the smallest name of
/// a face they kept of the solid, so the order does not depend on how they
/// were found. Every face, edge and vertex keeps its name in the first piece
/// it is part of; a piece sharing it with an earlier one — the cut, on its
/// other side, and the edges and vertices around it — names its own copy
/// `N(name,k)` for piece `k`. What the cutting creates is named as a
/// boolean names what it creates (see [`crate::naming`]), the copy of the
/// sheet's face `F` being `N(F)`.
///
/// Fails, leaving the solid in pieces of no use, unless the sheet cuts the
/// solid clean through: a sheet ending inside the solid, or one along which
/// the solid stays connected, splits nothing.
pub fn split<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: SolidId,
    sheet: ShellId,
    params: RemeshParams<S>,
) -> GeopResult<Vec<SolidId>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "split(name={}, solid={solid}, sheet={sheet})",
            namer.root()
        ))
    };
    let name = |part: &Part<S>, id: RefId| -> GeopResult<String> {
        part.name_of(id)
            .map(str::to_string)
            .ok_or_else(|| GeopError::new(format!("split: {id} has no name")))
    };

    // The cutting copy of the sheet.
    let sheet_faces = part
        .topology()
        .body_faces(Body::Sheet(sheet))
        .with_context(&ctx)?;
    let cutter = part
        .copy_faces(&sheet_faces, None, |name| namer.name(&[name]))
        .with_context(&ctx)?;
    let cutter = Body::Sheet(cutter.shells[0]);

    remesh(part, namer, solid, cutter, params).with_context(&ctx)?;
    let model = part.topology();
    let solid_faces = model.solid_faces(solid).with_context(&ctx)?;
    let mut cuts = Vec::new();
    let mut classified = Classified::default();
    for face in model.body_faces(cutter).with_context(&ctx)? {
        let class = classify_face(model, face, solid, params, &mut classified)
            .with_context(&ctx)
            .with_context(&|e: GeopError| e.with_context(format!("classifying face {face}")))?;
        if class == FaceClassification::Inside {
            cuts.push(face);
        }
    }
    if cuts.is_empty() {
        return Err(ctx(GeopError::new(
            "split: the face does not cut through the solid",
        )));
    }

    // Face sides: each face of the solid, then each cut as it is ("back",
    // the side its normal points away from), then each turned around.
    let n = solid_faces.len();
    let k = cuts.len();
    let side: HashMap<FaceId, usize> = solid_faces
        .iter()
        .chain(&cuts)
        .enumerate()
        .map(|(i, &f)| (f, i))
        .collect();
    let turned = |i: usize| i + k;
    let mut pieces = UnionFind::new(n + 2 * k);
    let mut users: HashMap<EdgeId, Vec<(usize, geop_core_topology::Sense)>> = HashMap::new();
    for (&face, &i) in &side {
        for coedge in model.iterate_face_coedges(face) {
            let coedge = model.get_coedge(coedge)?;
            if let CoedgeGeometry::Edge(edge) = coedge.geometry {
                users.entry(edge).or_default().push((i, coedge.sense));
            }
        }
    }
    let mut edges: Vec<_> = users.into_iter().collect();
    edges.sort_by_key(|(e, _)| e.0);
    for (edge, uses) in edges {
        let (on_solid, on_cut): (Vec<_>, Vec<_>) = uses.into_iter().partition(|&(i, _)| i < n);
        match (&on_solid[..], &on_cut[..]) {
            ([(a, _), (b, _)], []) => pieces.union(*a, *b),
            // The sheet crossed the solid's boundary here.
            ([(a, sense_a), (b, _)], [(cut, sense)]) => {
                let (with, against) = if sense_a != sense { (*a, *b) } else { (*b, *a) };
                pieces.union(*cut, with);
                pieces.union(turned(*cut), against);
            }
            // Two cuts meeting inside the solid.
            ([], [(c, sense_c), (d, sense_d)]) => {
                if sense_c != sense_d {
                    pieces.union(*c, *d);
                    pieces.union(turned(*c), turned(*d));
                } else {
                    pieces.union(*c, turned(*d));
                    pieces.union(turned(*c), *d);
                }
            }
            ([], [_]) => {
                return Err(ctx(GeopError::new(format!(
                    "split: the face ends inside the solid, at edge {edge}; it must cut all the way through"
                ))));
            }
            _ => {
                return Err(ctx(GeopError::new(format!(
                    "split: {} face(s) of the solid and {} of the cut meet at edge {edge}, which does not divide the solid into pieces",
                    on_solid.len(),
                    on_cut.len()
                ))));
            }
        }
    }

    let groups = pieces.groups();
    let piece_of: HashMap<usize, usize> = groups
        .iter()
        .enumerate()
        .flat_map(|(g, members)| members.iter().map(move |&m| (m, g)))
        .collect();
    if let Some(&cut) = cuts
        .iter()
        .find(|c| piece_of[&side[c]] == piece_of[&turned(side[c])])
    {
        return Err(ctx(GeopError::new(format!(
            "split: the solid stays connected around the cut face {cut}: the face does not divide it into pieces"
        ))));
    }

    // Each piece, described: its faces of the solid as they are, its cuts as
    // they are or turned around.
    let sides: Vec<FaceId> = solid_faces
        .iter()
        .chain(&cuts)
        .chain(&cuts)
        .copied()
        .collect();
    let mut described: Vec<(String, BodySpec<S>, Vec<RefId>)> = Vec::new();
    for members in &groups {
        let faces: Vec<FaceId> = members.iter().map(|&m| sides[m]).collect();
        let (mut spec, sources) = model.body_spec(&faces, true).with_context(&ctx)?;
        for (face, &m) in spec.faces.iter_mut().zip(members) {
            if m >= n + k {
                *face = face.reversed();
            }
        }
        let first = members
            .iter()
            .filter(|&&m| m < n)
            .map(|&m| name(part, solid_faces[m].into()))
            .collect::<GeopResult<Vec<_>>>()?
            .into_iter()
            .min()
            .ok_or_else(|| ctx(GeopError::new("split: a piece has no face of the solid")))?;
        let entities = sources
            .vertices
            .iter()
            .map(|&v| RefId::from(v))
            .chain(sources.edges.iter().map(|&e| e.into()))
            .chain(sources.faces.iter().map(|&f| f.into()))
            .collect();
        described.push((first, spec, entities));
    }
    described.sort_by(|a, b| a.0.cmp(&b.0));

    // Names, before the originals and their names are gone.
    let mut taken: HashSet<RefId> = HashSet::new();
    let mut named: Vec<(BodySpec<S>, BodyNames)> = Vec::new();
    for (index, (_, spec, entities)) in described.into_iter().enumerate() {
        let mut all = Vec::with_capacity(entities.len());
        for id in &entities {
            let original = name(part, *id)?;
            all.push(if taken.insert(*id) {
                original
            } else {
                namer.name(&[&original, &index.to_string()])
            });
        }
        let (v, e) = (spec.vertices.len(), spec.edges.len());
        let names = BodyNames {
            vertices: all[..v].to_vec(),
            edges: all[v..v + e].to_vec(),
            faces: all[v + e..].to_vec(),
            solid: Some(namer.name(&[&index.to_string()])),
        };
        named.push((spec, names));
    }

    part.assemble_solid(&[solid.into(), cutter], &[], namer.root())
        .with_context(&ctx)?;
    named
        .into_iter()
        .map(|(spec, names)| {
            let BuiltBody { solid, .. } = part.build_body(spec, names).with_context(&ctx)?;
            Ok(solid.expect("built as a solid"))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::split;
    use crate::remesh::remesh::RemeshParams;
    use geop_core_math::{
        primitives::CoordinateSystem,
        scalars::{ScalInF64 as S, Scalar},
        vector::{Vector2, Vector3},
    };
    use geop_core_topology::{
        Body,
        contains::shell::{PointClassification, solid_contains},
        validation::{ValidationParameters, validate, validate_manifold},
    };
    use geop_ops::{Namer, Part};
    use geop_ops_extrude_revolve::{
        common::{Profile, polyline},
        extrude::extrude,
        shapes::cube_solid,
        sweep::SweepLoop,
    };

    fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A unit cube around the origin, and a sheet: the chain through
    /// `points` in the `z = -1` plane, swept up to `z = 1`.
    fn cube_and_sheet(points: &[[f64; 2]]) -> (Part<S>, geop_core_topology::SolidId, Body) {
        let mut part = Part::<S>::new();
        let cube = cube_solid(&mut part, "c", v3(-0.5, -0.5, -0.5), v3(0.5, 0.5, 0.5)).unwrap();
        let chain = polyline(
            &points
                .iter()
                .map(|p| Vector2::from_array([S::from_f64(p[0]), S::from_f64(p[1])]))
                .collect::<Vec<_>>(),
        )
        .unwrap();
        let plane = CoordinateSystem::world_at(v3(0.0, 0.0, -1.0));
        let namer = Namer::new("extrude", "cut").unwrap();
        let built = extrude(
            &mut part,
            &namer,
            None,
            &plane,
            S::ZERO,
            S::from_f64(2.0),
            &[SweepLoop::plain(Profile::open(chain))],
        )
        .unwrap();
        (part, cube, Body::Sheet(built.shells[0]))
    }

    fn assert_valid(part: &Part<S>) {
        let params = ValidationParameters::default();
        for result in [
            validate(&params, part.topology()),
            validate_manifold(&params, part.topology()),
        ] {
            if let Err(e) = result {
                panic!("{e:?}");
            }
        }
        part.check_names().unwrap();
    }

    /// Which of `pieces` holds `p`.
    fn holding(
        part: &Part<S>,
        pieces: &[geop_core_topology::SolidId],
        p: Vector3<S>,
    ) -> Vec<usize> {
        (0..pieces.len())
            .filter(|&k| {
                solid_contains(part.topology(), pieces[k], p, 2000, S::from_f64(1e-7), 7).unwrap()
                    == PointClassification::Inside
            })
            .collect()
    }

    /// A flat sheet across the cube cuts it in two halves, and stays as it
    /// was.
    #[test]
    fn a_flat_sheet_cuts_a_cube_in_two() {
        let (mut part, cube, sheet) = cube_and_sheet(&[[0.1, -1.0], [0.1, 1.0]]);
        let Body::Sheet(shell) = sheet else {
            unreachable!()
        };
        let namer = Namer::new("split", "s").unwrap();
        let pieces = split(&mut part, &namer, cube, shell, RemeshParams::default()).unwrap();
        assert_valid(&part);
        assert_eq!(pieces.len(), 2);
        assert_eq!(part.solid_names(), ["split(s,0)", "split(s,1)"]);
        let (left, right) = (
            holding(&part, &pieces, v3(-0.2, 0.0, 0.0)),
            holding(&part, &pieces, v3(0.3, 0.0, 0.0)),
        );
        assert_eq!(left.len(), 1);
        assert_eq!(right.len(), 1);
        assert_ne!(left, right);
        // Each half has the cube's six faces, cut down, the right one five
        // of them and its own side of the cut.
        let faces = |k: usize| part.topology().solid_faces(pieces[k]).unwrap().len();
        assert_eq!(faces(0) + faces(1), 6 + 4 + 2);
        assert_eq!(part.topology().body_faces(sheet).unwrap().len(), 1);
    }

    /// A sheet bent into a "U" whose bottom lies in the cube cuts the slot
    /// inside the "U" out of it; the rest stays one piece, around the
    /// bottom.
    #[test]
    fn a_bent_sheet_cuts_a_slot_out_of_a_cube() {
        let (mut part, cube, sheet) =
            cube_and_sheet(&[[-0.2, 1.0], [-0.2, -0.2], [0.2, -0.2], [0.2, 1.0]]);
        let Body::Sheet(shell) = sheet else {
            unreachable!()
        };
        let namer = Namer::new("split", "s").unwrap();
        let pieces = split(&mut part, &namer, cube, shell, RemeshParams::default()).unwrap();
        assert_valid(&part);
        assert_eq!(pieces.len(), 2);
        let slot = holding(&part, &pieces, v3(0.0, 0.2, 0.0));
        assert_eq!(slot.len(), 1);
        for p in [v3(-0.4, 0.0, 0.0), v3(0.4, 0.0, 0.0), v3(0.0, -0.4, 0.0)] {
            let rest = holding(&part, &pieces, p);
            assert!(rest.len() == 1 && rest != slot, "{p:?}");
        }
    }

    /// A sheet missing the cube cuts nothing.
    #[test]
    fn a_sheet_beside_the_solid_cuts_nothing() {
        let (mut part, cube, sheet) = cube_and_sheet(&[[2.0, -1.0], [2.0, 1.0]]);
        let Body::Sheet(shell) = sheet else {
            unreachable!()
        };
        let namer = Namer::new("split", "s").unwrap();
        let error = split(&mut part, &namer, cube, shell, RemeshParams::default()).unwrap_err();
        assert!(error.root_message().contains("does not cut"), "{error:?}");
    }
}
