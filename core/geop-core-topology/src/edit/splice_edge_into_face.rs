use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    polygon::polygon_signed_area,
    scalars::Scalar,
    vector::Vector2,
};

use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Curve2, EdgeId, Face, FaceId, Model, Sense, VertexId,
    boundary::{BoundaryIndex, BoundaryType},
    contains::face::{PointClassification, face_contains},
    loop_sampling::sample_loop_to_polygon,
};

impl<S: Scalar> Model<S> {
    /// Splice an **already-existing** `edge_id` into `face_id`'s boundary
    /// structure, as a forward/reversed coedge pair whose pcurves it fits onto
    /// the face's own surface.
    ///
    /// Unlike the `mer`/`mekr` Euler operators — which mint their own brand-new
    /// edge, and so can't be used for an edge that already exists because two
    /// faces are meant to *share* it (the whole point of imprinting an
    /// intersection curve) — this rewires loops around an edge it's handed.
    ///
    /// What it does depends on how much of the edge's own topology the face
    /// already knows about, which is the only thing that determines what a
    /// correct result even looks like:
    ///
    /// | edge's start/end vertex on this face's boundary | result |
    /// |---|---|
    /// | neither | a new self-contained ring floating inside the face — a **hole** |
    /// | exactly one | a spur (out along the edge and back) inserted into that vertex's own loop |
    /// | both, same hole | that hole is **divided into two holes** |
    /// | both, two different holes | those holes are **merged into one** |
    /// | both, a hole and the outer loop | the hole is **absorbed** into the outer loop |
    /// | both, the outer loop | the face is **split into two faces** |
    ///
    /// Splitting the face, and dividing a hole (whose material side becomes
    /// a face of its own), are the cases that create a face; its id is
    /// returned, `None` otherwise. The face split is the only one that has to
    /// reclassify the old face's holes — each now lies inside exactly one of
    /// the two halves (see [`split_face`]). The middle cases are the same
    /// restructurings `mer`/`mekr` perform; a spur changes no connectivity at
    /// all, being "wire" topology rather than a real trim boundary.
    pub fn splice_edge_into_face(
        &mut self,
        edge_id: EdgeId,
        face_id: FaceId,
        max_nodes: usize,
        min_subdivision_size: S,
    ) -> GeopResult<Option<FaceId>> {
        let model = self;
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::splice_edge_into_face(edge={edge_id}, face={face_id})"
            ))
        };

        let edge = model.get_edge(edge_id).with_context(&ctx)?;
        let (start_vertex, end_vertex) = (edge.start_vertex, edge.end_vertex);
        let curve = edge.curve.clone();

        // The coedge each of the edge's endpoints would attach *after*: one
        // already arriving at that vertex. Where the face already has one,
        // its pcurve end is the authoritative `(u, v)` there — the new pcurve
        // is pinned to it so the loop stays exactly continuous.
        let surface = model.get_face(face_id).with_context(&ctx)?.surface.clone();
        let fit = |at_start: Option<CoedgeId>, at_end: Option<CoedgeId>| {
            surface
                .fit_pcurve(
                    &curve,
                    pcurve_end_uv(model, at_start)?,
                    pcurve_end_uv(model, at_end)?,
                    max_nodes,
                    min_subdivision_size,
                )
                .with_context(&ctx)
        };
        let arriving_start = coedges_ending_at(model, face_id, start_vertex).with_context(&ctx)?;
        let arriving_end = coedges_ending_at(model, face_id, end_vertex).with_context(&ctx)?;
        let first = |arriving: &[CoedgeId]| arriving.first().copied();
        let mut pcurve_fwd = fit(first(&arriving_start), first(&arriving_end))?;
        // A vertex the loop passes more than once — the root of a spur, or a
        // point where the face touches itself — has as many corners there,
        // and the edge belongs in the one it leaves into. Its pcurve says
        // which, so it is chosen after a first fit and refitted if that
        // moved the pin.
        let at_start = choose_corner(
            model,
            &arriving_start,
            Leg {
                pcurve: &pcurve_fwd,
                from_end: false,
            },
        )
        .with_context(&ctx)?;
        let at_end = choose_corner(
            model,
            &arriving_end,
            Leg {
                pcurve: &pcurve_fwd,
                from_end: true,
            },
        )
        .with_context(&ctx)?;
        if at_start != first(&arriving_start) || at_end != first(&arriving_end) {
            pcurve_fwd = fit(at_start, at_end)?;
        }
        let pcurve_rev = pcurve_fwd.reverse();

        let fwd = model.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve: pcurve_fwd,
            next: CoedgeId(0),
            prev: CoedgeId(0),
            face: face_id,
        });
        let rev = model.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: pcurve_rev,
            next: CoedgeId(0),
            prev: CoedgeId(0),
            face: face_id,
        });

        let mut new_face = None;
        match (at_start, at_end) {
            // Neither endpoint is on this face yet: a self-contained
            // two-coedge ring, floating inside the face — that is a hole.
            (None, None) => {
                link(model, fwd, rev).with_context(&ctx)?;
                link(model, rev, fwd).with_context(&ctx)?;
                model
                    .get_face_mut(face_id)
                    .with_context(&ctx)?
                    .holes
                    .push(BoundaryType::Loop(fwd));
            }
            // Only one endpoint is on the face: splice a spur into that loop —
            // out along the edge and straight back, so the loop still closes.
            (Some(a), None) => {
                splice_spur(model, a, fwd, rev).with_context(&ctx)?;
            }
            (None, Some(a)) => {
                splice_spur(model, a, rev, fwd).with_context(&ctx)?;
            }
            // Both endpoints are already on the face, so the new edge joins
            // two points of its boundary structure. *Which* boundaries they
            // sit on decides what that means topologically, and the four
            // cases are genuinely different operations.
            (Some(a), Some(b)) => {
                let loop_a = model
                    .find_boundary_containing(face_id, a)
                    .with_context(&ctx)?;
                let loop_b = model
                    .find_boundary_containing(face_id, b)
                    .with_context(&ctx)?;

                // `fwd` runs start->end, so it leaves the coedge arriving at
                // `start_vertex` and lands on whatever leaves `end_vertex`;
                // `rev` closes the complementary path the other way round.
                let a_next = model.get_coedge(a).with_context(&ctx)?.next;
                let b_next = model.get_coedge(b).with_context(&ctx)?.next;

                link(model, a, fwd).with_context(&ctx)?;
                link(model, fwd, b_next).with_context(&ctx)?;
                link(model, b, rev).with_context(&ctx)?;
                link(model, rev, a_next).with_context(&ctx)?;

                match (loop_a, loop_b) {
                    // Two points of the *outer* loop: the edge cuts the face
                    // itself in two. Both rings bound material, so neither can
                    // be a hole of the other — this is the only case that
                    // creates a face.
                    (BoundaryIndex::Outer, BoundaryIndex::Outer) => {
                        // Both halves must enclose material. A half of zero
                        // area means the new edge runs along the stretch of
                        // boundary between its own endpoints — it duplicates
                        // a path the face already has, so "splitting" there
                        // carves off a sliver rather than two faces. Reported
                        // rather than built: a zero-area face is structurally
                        // perfect and every other check accepts it, so it
                        // would surface much later as a face nothing can be
                        // classified against.
                        for ring in [fwd, rev] {
                            let area = signed_area(model, ring).with_context(&ctx)?;
                            if !area.abs().definitely_greater(S::ZERO) {
                                return Err(ctx(GeopError::new(format!(
                                    "{DEGENERATE_SPLIT}: splicing edge {edge_id} into face {face_id} would split its outer loop into a ring of signed area {area:?} — the edge runs along the boundary it is being spliced into, so one side encloses nothing"
                                ))));
                            }
                        }
                        new_face = Some(
                            split_face(model, face_id, fwd, rev, max_nodes, min_subdivision_size)
                                .with_context(&ctx)?,
                        );
                    }
                    // Two points of the *same* hole. The edge runs through
                    // material, so together with one of the two arcs it just
                    // cut the hole into, it encloses a patch of material that
                    // is now bounded on its own — a new face — while the other
                    // arc still bounds a void and stays a hole.
                    //
                    // Which is which is exactly the winding: by this kernel's
                    // convention an outer loop runs counter-clockwise in
                    // `(u, v)` and a hole runs clockwise, so the ring with
                    // positive signed area bounds material and the one with
                    // negative area bounds a void. Nothing weaker will do —
                    // both rings pass through the same vertices and contain
                    // the same edge, so no purely topological test can tell
                    // them apart.
                    (BoundaryIndex::Hole(i), BoundaryIndex::Hole(j)) if i == j => {
                        let area = signed_area(model, fwd).with_context(&ctx)?;
                        let (material, void) = if area.definitely_greater(S::ZERO) {
                            (fwd, rev)
                        } else if area.definitely_less(S::ZERO) {
                            (rev, fwd)
                        } else {
                            return Err(ctx(GeopError::new(format!(
                                "splice_edge_into_face: splitting hole {i} of face {face_id} with edge {edge_id} produced a ring of signed area {area:?}, which is not decidably clockwise or counter-clockwise — the split would be degenerate"
                            ))));
                        };
                        model
                            .get_face_mut(face_id)
                            .with_context(&ctx)?
                            .set_boundary(BoundaryIndex::Hole(i), BoundaryType::Loop(void));
                        new_face = Some(
                            new_face_from_ring(
                                model,
                                face_id,
                                material,
                                max_nodes,
                                min_subdivision_size,
                            )
                            .with_context(&ctx)?,
                        );
                    }
                    // Two *different* holes: the bridge merges them into one
                    // hole.
                    (BoundaryIndex::Hole(i), BoundaryIndex::Hole(j)) => {
                        let face = model.get_face_mut(face_id).with_context(&ctx)?;
                        face.set_boundary(BoundaryIndex::Hole(i), BoundaryType::Loop(fwd));
                        face.holes.remove(j);
                    }
                    // A hole bridged to the outer loop: the hole stops being a
                    // separate boundary and its coedges become part of the one
                    // ring that now bounds the face.
                    (BoundaryIndex::Outer, BoundaryIndex::Hole(j))
                    | (BoundaryIndex::Hole(j), BoundaryIndex::Outer) => {
                        let face = model.get_face_mut(face_id).with_context(&ctx)?;
                        face.outer = BoundaryType::Loop(fwd);
                        face.holes.remove(j);
                    }
                }
            }
        }

        Ok(new_face)
    }
}

/// The `(u, v)` a coedge's pcurve ends at, if there is such a coedge.
fn pcurve_end_uv<S: Scalar>(
    model: &Model<S>,
    coedge: Option<CoedgeId>,
) -> GeopResult<Option<Vector2<S>>> {
    let Some(coedge) = coedge else {
        return Ok(None);
    };
    let pcurve = &model.get_coedge(coedge)?.pcurve;
    let (_, t1) = pcurve.domain();
    Ok(Some(pcurve.evaluate(t1)?))
}

/// Every coedge of `face_id` that *arrives* at `vertex`: one per time its
/// boundary passes through it. A new edge leaving `vertex` has to be spliced
/// in after one of them — see [`choose_corner`].
fn coedges_ending_at<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    vertex: VertexId,
) -> GeopResult<Vec<CoedgeId>> {
    let mut arriving = Vec::new();
    for coedge_id in model.iterate_face_coedges(face_id) {
        if model.coedge_end_vertex_id(coedge_id)? == vertex {
            arriving.push(coedge_id);
        }
    }
    Ok(arriving)
}

/// Of the coedges `arriving` at one vertex, the one whose corner a new edge
/// leaving the vertex along `leaving` runs into.
///
/// Each pass of the boundary through the vertex makes a corner: the face's
/// material fills the angle swept counter-clockwise from the way the boundary
/// leaves to the way it arrived from — material lies to the left of every
/// loop, outer and hole alike. One pass leaves no choice. Several do — a
/// spur's root, where the boundary passes once on each side of it — and
/// splicing into the wrong corner puts the edge on the wrong side of the
/// spur: the face split it makes then carries the spur into the half it does
/// not lie in. That happened on `three_inscribed_bores_remesh_imprints_every_arc`:
/// an arc spliced as a spur from a point of a bore's rim, and the rim arc
/// spliced at that same point a moment later, landed the spur in the outer
/// piece of the wall.
///
/// The curves are ordered around the vertex by where they first leave a
/// small circle around it, not by their tangents: see [`Leg`].
fn choose_corner<S: Scalar>(
    model: &Model<S>,
    arriving: &[CoedgeId],
    leaving: Leg<'_, S>,
) -> GeopResult<Option<CoedgeId>> {
    if arriving.len() <= 1 {
        return Ok(arriving.first().copied());
    }
    let mut corners = Vec::new();
    for &coedge_id in arriving {
        let coedge = model.get_coedge(coedge_id)?;
        let goes_to = Leg {
            pcurve: &model.get_coedge(coedge.next)?.pcurve,
            from_end: false,
        };
        let came_from = Leg {
            pcurve: &coedge.pcurve,
            from_end: true,
        };
        corners.push((coedge_id, goes_to, came_from));
    }

    // One radius for every leg, small enough that no leg meets another
    // vertex inside it (see `Leg`).
    let mut radius = leaving.reach()?;
    for (_, goes_to, came_from) in &corners {
        for leg in [goes_to, came_from] {
            let reach = leg.reach()?;
            if reach.definitely_less(radius) {
                radius = reach;
            }
        }
    }
    let radius = radius.div(S::TWO)?.sharpen();
    if !radius.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "splice_edge_into_face: a curve at the vertex returns to it at once, so no circle around the vertex orders the corners after coedges {arriving:?}"
        )));
    }

    let d = leaving.exit(radius)?;
    let mut chosen = Vec::new();
    for (coedge_id, goes_to, came_from) in &corners {
        let (from, to) = (goes_to.exit(radius)?, came_from.exit(radius)?);
        match within_corner(from, to, d) {
            Some(true) => chosen.push(*coedge_id),
            Some(false) => {}
            None => {
                return Err(GeopError::new(format!(
                    "splice_edge_into_face: cannot tell whether an edge leaving towards {d:?} runs into the corner after coedge {coedge_id}, from {from:?} counter-clockwise to {to:?} (at radius {radius:?})"
                )));
            }
        }
    }
    match chosen[..] {
        [coedge_id] => Ok(Some(coedge_id)),
        _ => Err(GeopError::new(format!(
            "splice_edge_into_face: an edge leaving towards {d:?} runs into {} of the corners after coedges {arriving:?}, where it must run into exactly one",
            chosen.len()
        ))),
    }
}

/// A pcurve run away from the vertex at one of its ends.
///
/// Several curves leaving one vertex are ordered around it by where each
/// first leaves a circle of some radius around the vertex. Within a radius
/// that no curve meets another vertex inside, their pieces in the disk are
/// paths from its centre to its rim that do not cross — curves of a face's
/// boundary, and an edge being spliced into it, meet only at vertices — and
/// such paths leave the disk in the same cyclic order they leave its centre.
/// So the order is exact at any such radius, and a larger one only separates
/// the curves further.
///
/// Tangents would answer the same question only where the curves leave in
/// different directions. Intersection curves of tangent configurations often
/// leave a vertex tangent to each other, and then only curvature separates
/// them — which, differentiated twice off a pcurve that encloses a traced
/// curve, comes out too wide to decide anything.
struct Leg<'a, S: Scalar> {
    pcurve: &'a Curve2<S>,
    /// Whether the vertex is at the pcurve's end (and it is run backwards).
    from_end: bool,
}

impl<S: Scalar> Leg<'_, S> {
    /// The parameters of the vertex end and of the pcurve's midpoint.
    fn ends(&self) -> GeopResult<(S, S)> {
        let (t0, t1) = self.pcurve.domain();
        let mid = t0.add(t1).div(S::TWO)?;
        Ok((if self.from_end { t1 } else { t0 }, mid))
    }

    /// A radius this leg reaches, before meeting any vertex other than the
    /// one it leaves: its distance to its midpoint, or to its far end where
    /// that is nearer. A curve returning to the vertex it leaves has no far
    /// end to meet.
    fn reach(&self) -> GeopResult<S> {
        let (t0, t1) = self.pcurve.domain();
        let (t_vertex, t_mid) = self.ends()?;
        let t_far = if self.from_end { t0 } else { t1 };
        let origin = self.pcurve.evaluate(t_vertex)?;
        let mid = self.pcurve.evaluate(t_mid)?.sub(&origin).norm();
        let far = self.pcurve.evaluate(t_far)?.sub(&origin).norm();
        Ok(
            if far.definitely_greater(S::ZERO) && far.definitely_less(mid) {
                far
            } else {
                mid
            },
        )
    }

    /// Where the leg leaves the circle of `radius` around its vertex,
    /// relative to the vertex. Located by bisection between the vertex and
    /// the midpoint, which lies outside a circle of at most half its reach;
    /// the parameters bisected are the search's own choice, so they are sharp.
    ///
    /// Bisection finds *a* crossing of the circle, which is the first exit
    /// the ordering argument needs as long as the leg does not leave the disk
    /// and come back before its midpoint — true of the short, smooth trim
    /// curves at half their reach, and an assumption for anything else.
    fn exit(&self, radius: S) -> GeopResult<Vector2<S>> {
        let (t_vertex, t_mid) = self.ends()?;
        let origin = self.pcurve.evaluate(t_vertex)?;
        let radius_sq = radius.mul(radius);
        let (mut inside, mut outside) = (t_vertex, t_mid);
        let mut point = self.pcurve.evaluate(outside)?.sub(&origin);
        for _ in 0..EXIT_BISECTIONS {
            let t = inside.add(outside).div(S::TWO)?.sharpen();
            point = self.pcurve.evaluate(t)?.sub(&origin);
            let excess = point.norm_sq().sub(radius_sq);
            if excess.definitely_less(S::ZERO) {
                inside = t;
            } else if excess.definitely_greater(S::ZERO) {
                outside = t;
            } else {
                break;
            }
        }
        Ok(point)
    }
}

/// Bisection steps for [`Leg::exit`]. Bounds effort only: each step halves
/// the stretch of curve the exit point is known to lie on, and the point
/// returned is on the curve either way.
const EXIT_BISECTIONS: usize = 60;

/// Whether `d` lies strictly inside the angle swept counter-clockwise from
/// `from` to `to`; `None` where the intervals cannot decide it. `from` and
/// `to` pointing the same way sweep the full turn — the tip of a spur, whose
/// corner is everything around it.
fn within_corner<S: Scalar>(from: Vector2<S>, to: Vector2<S>, d: Vector2<S>) -> Option<bool> {
    let sign = |x: S| {
        if x.definitely_greater(S::ZERO) {
            Some(true)
        } else if x.definitely_less(S::ZERO) {
            Some(false)
        } else {
            None
        }
    };
    let after_from = sign(from.prod_cross(&d));
    let before_to = sign(d.prod_cross(&to));
    let turn = from.prod_cross(&to);
    if turn.definitely_greater(S::ZERO) {
        // Less than half a turn: inside means past `from` and short of `to`.
        match (after_from, before_to) {
            (Some(false), _) | (_, Some(false)) => Some(false),
            (Some(true), Some(true)) => Some(true),
            _ => None,
        }
    } else if turn.definitely_less(S::ZERO) {
        // More than half a turn: everything but the complementary angle.
        match (after_from, before_to) {
            (Some(true), _) | (_, Some(true)) => Some(true),
            (Some(false), Some(false)) => Some(false),
            _ => None,
        }
    } else if from.prod_dot(&to).definitely_greater(S::ZERO) {
        // The full turn, unless `d` leaves along it.
        after_from.map(|_| true)
    } else {
        None
    }
}

/// Make `to` follow `from` in loop order, keeping `prev` consistent.
fn link<S: Scalar>(model: &mut Model<S>, from: CoedgeId, to: CoedgeId) -> GeopResult<()> {
    model.get_coedge_mut(from)?.next = to;
    model.get_coedge_mut(to)?.prev = from;
    Ok(())
}

/// Insert `out`/`back` (an out-and-return pair on the same edge) into the
/// loop right after `after`, leaving the rest of the loop untouched.
fn splice_spur<S: Scalar>(
    model: &mut Model<S>,
    after: CoedgeId,
    out: CoedgeId,
    back: CoedgeId,
) -> GeopResult<()> {
    let after_next = model.get_coedge(after)?.next;
    link(model, after, out)?;
    link(model, out, back)?;
    link(model, back, after_next)?;
    Ok(())
}

/// Marks the error raised when a splice would divide a face into a piece of
/// no area. Callers that are *imprinting* an edge they already know follows
/// this face's boundary can treat it as "already represented" rather than a
/// failure — see `booleans::remesh`'s tracing. Matched on the message because
/// `GeopError` carries no code; that is fragile enough to be worth the
/// constant rather than a literal at both ends.
pub const DEGENERATE_SPLIT: &str = "degenerate split";

/// Cut `face_id` in two along the newly inserted edge: `keep` and `moved`
/// anchor the two rings the old outer loop just split into.
///
/// `keep`'s ring stays with `face_id`; `moved`'s ring becomes the outer loop
/// of a brand-new face on the same surface and in the same shell. Both rings
/// bound material — that is what distinguishes this from every other case in
/// `splice_edge_into_face` — so neither can become a hole of the other.
///
/// The old face's holes then have to be re-sorted, since each now lies inside
/// exactly one of the two faces and the split has no idea which. Each hole is
/// classified by taking a point on it and asking `face_contains` — a hole that
/// the new face contains moves there, everything else stays. A hole with no
/// usable point (a bare `Vertex` boundary, or a loop whose `(u, v)` cannot be
/// read) stays put rather than being dropped: leaving it on the wrong face is
/// a recoverable error, losing it silently fills in a hole that should exist.
fn split_face<S: Scalar>(
    model: &mut Model<S>,
    face_id: FaceId,
    keep: CoedgeId,
    moved: CoedgeId,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<FaceId> {
    model
        .get_face_mut(face_id)?
        .set_boundary(BoundaryIndex::Outer, BoundaryType::Loop(keep));
    new_face_from_ring(model, face_id, moved, max_nodes, min_subdivision_size)
}

/// Split `ring` off `face_id` as the outer loop of a brand-new face on the
/// same surface and in the same shell.
///
/// `face_id`'s own holes are then re-sorted, since each now lies inside
/// exactly one of the two faces and the split has no idea which. Each is
/// classified by taking a point on it and asking `face_contains` — a hole the
/// new face contains moves there, everything else stays. A hole with no
/// usable point (a bare `Vertex` boundary, or a loop whose `(u, v)` cannot be
/// read) stays put rather than being dropped: leaving it on the wrong face is
/// a recoverable error, losing it silently fills in a hole that should exist.
fn new_face_from_ring<S: Scalar>(
    model: &mut Model<S>,
    face_id: FaceId,
    ring: CoedgeId,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<FaceId> {
    let old = model.get_face(face_id)?.clone();
    let new_face_id = model.insert_face(Face {
        surface: old.surface.clone(),
        outer: BoundaryType::Loop(ring),
        holes: Vec::new(),
        shell: old.shell,
    });
    model.get_shell_mut(old.shell)?.faces.push(new_face_id);

    // Every coedge of the moved ring now belongs to the new face. Walked with
    // a hard cap: a ring that never returns to its anchor would otherwise spin
    // here forever, and a corrupted `next` chain is exactly the kind of thing
    // a restructuring like this can introduce.
    for c in ring_coedges(model, ring)? {
        model.get_coedge_mut(c)?.face = new_face_id;
    }

    let mut stay = Vec::new();
    let mut move_over = Vec::new();
    for hole in old.holes {
        let Some(uv) = boundary_uv(model, hole)? else {
            stay.push(hole);
            continue;
        };
        let inside_new = matches!(
            face_contains(
                model,
                new_face_id,
                uv[0],
                uv[1],
                max_nodes,
                min_subdivision_size,
                HOLE_CLASSIFY_SEED,
            )?,
            PointClassification::Inside
        );
        if inside_new {
            move_over.push(hole);
        } else {
            stay.push(hole);
        }
    }
    for hole in &move_over {
        if let BoundaryType::Loop(anchor) = hole {
            for c in ring_coedges(model, *anchor)? {
                model.get_coedge_mut(c)?.face = new_face_id;
            }
        }
    }
    model.get_face_mut(face_id)?.holes = stay;
    model.get_face_mut(new_face_id)?.holes = move_over;
    Ok(new_face_id)
}

/// The signed area, in `(u, v)`, of the ring anchored at `anchor`: positive
/// counter-clockwise, negative clockwise.
///
/// Sampled through the same `sample_loop_to_polygon` the rasterizer uses, so
/// there is one notion of "what polygon does this loop trace" rather than two
/// that could drift apart.
fn signed_area<S: Scalar>(model: &Model<S>, anchor: CoedgeId) -> GeopResult<S> {
    let polygon = sample_loop_to_polygon(model, anchor, LOOP_AREA_SAMPLES)?;
    Ok(polygon_signed_area(&polygon))
}

/// Samples per coedge for [`signed_area`]. Only the *sign* is used, and a
/// curved trim loop needs a few samples per coedge for that sign to be right;
/// this bounds effort, not correctness — an undecidable sign is reported as
/// an error rather than guessed.
const LOOP_AREA_SAMPLES: usize = 8;

/// Every coedge of the ring anchored at `anchor`, erroring rather than
/// spinning if the `next` chain never returns to it.
fn ring_coedges<S: Scalar>(model: &Model<S>, anchor: CoedgeId) -> GeopResult<Vec<CoedgeId>> {
    let cap = model.coedges.len() + 1;
    let ring: Vec<CoedgeId> = model.iterate_loop_coedges(anchor).take(cap).collect();
    if ring.len() >= cap {
        return Err(GeopError::new(format!(
            "splice_edge_into_face: the ring anchored at coedge {anchor} never returns to its anchor"
        )));
    }
    Ok(ring)
}

/// Fixed seed for the ray casting behind hole classification — `face_contains`
/// retries until it finds a ray grazing nothing, so the answer is
/// seed-independent and a constant keeps a split reproducible run to run.
const HOLE_CLASSIFY_SEED: u64 = 0x1234_5678_9ABC_DEF0;

/// A `(u, v)` on `boundary`, for classifying which side of a split it falls
/// on. `None` for a bare `Vertex` boundary, which has no pcurve to read one
/// from.
fn boundary_uv<S: Scalar>(
    model: &Model<S>,
    boundary: BoundaryType,
) -> GeopResult<Option<geop_core_math::vector::Vector2<S>>> {
    let BoundaryType::Loop(anchor) = boundary else {
        return Ok(None);
    };
    let pcurve = &model.get_coedge(anchor)?.pcurve;
    let (t0, _) = pcurve.domain();
    Ok(Some(pcurve.evaluate(t0)?))
}
