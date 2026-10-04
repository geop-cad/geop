//! The sheet laid out flat — [`Layout`] — and cut.
//!
//! Every face of a sheet, flat or bent, is a region of one plane in the
//! flat pattern's coordinates: the first flat's sheet coordinates, every
//! other flat moved by its shift (see [`Sheet::flat_shifts`]), every bend
//! the strip between its two lines, as wide as its developed length. The
//! regions share the edges and vertices between them. A cut is subtracted
//! there ([`Layout::subtract`]), where a bend is no different from a flat
//! and every curve still a line or an arc, and then each face is folded
//! back into its own place (see [`Sheet::folded`]). So a hole across a bend
//! is cut where it lies unrolled, and the flat pattern of a cut body is
//! exact: unfolding moves its curves, never approximates them.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::{
    intersection::curve_curve_overlaps_and_crossings, nurb_curve::NurbCurve2D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::Sense;
use geop_ops::Namer;
use geop_ops_extrude_revolve::common::{arc2, embed_curve, end_point, line2, start_point};

use crate::{sheet::Sheet, thicken::translate2};

/// Bounds how hard a crossing search between curves that are no lines or
/// arcs tries: effort, not what an answer means.
const MAX_NODES: usize = 20_000;

/// Where that search hands over (see `AGENTS.md`).
fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-9)
}

/// Which face of the sheet an entity is folded with: a flat's, or a
/// bend's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Home {
    Flat(usize),
    Bend(usize),
}

/// A vertex of the layout.
#[derive(Clone, Debug)]
pub struct LayoutVertex<S: Scalar> {
    /// Where it is, in the flat pattern's coordinates.
    pub at: Vector2<S>,
    /// Its vertices are `N(a)`, `N(b)` — named after an edge starting
    /// there; `None` until a cut's vertex is.
    pub name: Option<Namer>,
    /// A vertex of the sheet as built: its flat, and where it is in that
    /// flat's own sheet coordinates.
    pub own: Option<(usize, Vector2<S>)>,
}

/// An edge of the layout, from vertex `start` to vertex `end`.
#[derive(Clone, Debug)]
pub struct LayoutEdge<S: Scalar> {
    /// In the flat pattern's coordinates.
    pub curve: NurbCurve2D<S>,
    pub start: usize,
    pub end: usize,
    /// Its edges `N(a)`, `N(b)`, its wall `N` (see [`crate::thicken`]).
    pub name: Namer,
    pub home: Home,
    /// The curve of the sheet as built, in its flat's own coordinates,
    /// while no cut has split it.
    pub own: Option<NurbCurve2D<S>>,
    /// The key of the sheet's edge it is, or is a piece of — a flat's edge,
    /// or the side `s0`/`s1` of a bend; `None` for an edge a cut made.
    pub origin: Option<String>,
}

/// A face of the layout: its loops, the outer one counter-clockwise and
/// holes clockwise, each a list of edges and the sense it runs them in.
#[derive(Clone, Debug)]
pub struct LayoutFace {
    pub home: Home,
    pub outer: Vec<(usize, Sense)>,
    pub holes: Vec<Vec<(usize, Sense)>>,
    /// Its faces are `N(a)` and `N(b)`.
    pub name: Namer,
}

impl LayoutFace {
    pub fn loops(&self) -> impl Iterator<Item = &Vec<(usize, Sense)>> {
        std::iter::once(&self.outer).chain(&self.holes)
    }
}

/// A sheet laid out flat (see the module docs). Edges and vertices a cut
/// took away stay in the lists, used by no face.
#[derive(Clone, Debug, Default)]
pub struct Layout<S: Scalar> {
    pub vertices: Vec<LayoutVertex<S>>,
    pub edges: Vec<LayoutEdge<S>>,
    pub faces: Vec<LayoutFace>,
    /// The keys of the sheet's edges a cut split or took away.
    pub changed: BTreeSet<String>,
    /// Per bend, the vertices at its corners as built — where its parent
    /// edge starts and ends, where its child edge ends and starts — which
    /// stay in the list when a cut takes them away.
    pub corners: Vec<[usize; 4]>,
}

/// A curve of a cut's loop, in the flat pattern's coordinates, and its
/// name.
#[derive(Clone, Debug)]
pub struct CutCurve<S: Scalar> {
    pub curve: NurbCurve2D<S>,
    pub name: Namer,
}

impl<S: Scalar> Sheet<S> {
    /// The sheet laid out flat, its cuts subtracted (see the module docs).
    pub fn layout(&self) -> GeopResult<Layout<S>> {
        let shifts = self.flat_shifts()?;
        let mut layout = Layout::default();
        let mut edge_of: BTreeMap<String, usize> = BTreeMap::new();
        for (f, flat) in self.flats.iter().enumerate() {
            let ctx = with_context!("flat {}", flat.name.root());
            let mut loops = Vec::new();
            for lp in std::iter::once(&flat.outer).chain(&flat.holes) {
                let first = layout.vertices.len();
                let n = lp.len();
                let mut coedges = Vec::new();
                for (k, e) in lp.iter().enumerate() {
                    let curve = translate2(&e.curve, &shifts[f]).with_context(ctx)?;
                    layout.vertices.push(LayoutVertex {
                        at: start_point(&curve)?,
                        name: Some(e.name.scoped("v")),
                        own: Some((f, start_point(&e.curve)?)),
                    });
                    layout.edges.push(LayoutEdge {
                        curve,
                        start: first + k,
                        end: first + (k + 1) % n,
                        name: e.name.clone(),
                        home: Home::Flat(f),
                        own: Some(e.curve.clone()),
                        origin: Some(e.key()),
                    });
                    let index = layout.edges.len() - 1;
                    if edge_of.insert(e.key(), index).is_some() {
                        return Err(GeopError::new(format!("two edges are named {}", e.key())))
                            .with_context(ctx);
                    }
                    coedges.push((index, Sense::Forward));
                }
                loops.push(coedges);
            }
            let outer = loops.remove(0);
            layout.faces.push(LayoutFace {
                home: Home::Flat(f),
                outer,
                holes: loops,
                name: flat.name.clone(),
            });
        }
        for (b, bend) in self.bends.iter().enumerate() {
            let edge = |key: &str| {
                edge_of.get(key).copied().ok_or_else(|| {
                    GeopError::new(format!("bend {} meets no edge {key}", bend.name.root()))
                })
            };
            let (pe, ce) = (edge(&bend.parent_edge)?, edge(&bend.child_edge)?);
            let (pa, pb) = (layout.edges[pe].start, layout.edges[pe].end);
            let (cb, ca) = (layout.edges[ce].start, layout.edges[ce].end);
            layout.corners.push([pa, pb, ca, cb]);
            for (side, start, end) in [("s0", pa, ca), ("s1", pb, cb)] {
                let name = bend.name.scoped(side);
                layout.edges.push(LayoutEdge {
                    curve: line2(layout.vertices[start].at, layout.vertices[end].at)?,
                    start,
                    end,
                    origin: Some(name.root()),
                    name,
                    home: Home::Bend(b),
                    own: None,
                });
            }
            let n = layout.edges.len();
            layout.faces.push(LayoutFace {
                home: Home::Bend(b),
                outer: vec![
                    (n - 2, Sense::Forward),
                    (ce, Sense::Reversed),
                    (n - 1, Sense::Reversed),
                    (pe, Sense::Reversed),
                ],
                holes: Vec::new(),
                name: bend.name.clone(),
            });
        }
        for cut in &self.cuts {
            let ctx = with_context!("cut {}", cut.name.root());
            let loops = cut
                .loops
                .iter()
                .map(|lp| {
                    lp.iter()
                        .map(|e| {
                            Ok(CutCurve {
                                curve: translate2(&e.curve, &shifts[cut.flat])?,
                                name: e.name.clone(),
                            })
                        })
                        .collect::<GeopResult<Vec<_>>>()
                })
                .collect::<GeopResult<Vec<_>>>()
                .with_context(ctx)?;
            layout.subtract(&loops).with_context(ctx)?;
        }
        Ok(layout)
    }
}

impl<S: Scalar> Layout<S> {
    /// The edges some face uses, in order.
    pub fn used_edges(&self) -> Vec<usize> {
        let used: BTreeSet<usize> = self
            .faces
            .iter()
            .flat_map(|f| f.loops().flatten().map(|&(e, _)| e))
            .collect();
        used.into_iter().collect()
    }

    /// Takes the area inside `cut` — closed loops, counter-clockwise, apart
    /// from each other — out of the faces: their edges inside it go, the
    /// cut's curves inside a face become its edges, every curve split where
    /// they cross. Refuses, naming the curves, a cut that crosses a curve
    /// at its end, touches one, runs along one, or crosses one that is no
    /// line or arc; and one that splits a face in two, takes one away
    /// entirely, or misses the sheet.
    pub fn subtract(&mut self, cut: &[Vec<CutCurve<S>>]) -> GeopResult<()> {
        let used = self.used_edges();
        let segs: BTreeMap<usize, Seg<S>> = used
            .iter()
            .map(|&e| Ok((e, Seg::of(&self.edges[e].curve)?)))
            .collect::<GeopResult<_>>()?;
        let cut_segs: Vec<Vec<Seg<S>>> = cut
            .iter()
            .map(|lp| lp.iter().map(|c| Seg::of(&c.curve)).collect())
            .collect::<GeopResult<_>>()?;
        // What the faces were before the cut, to place its curves in.
        let regions: Vec<Vec<Vec<Piece<'_, S>>>> = self
            .faces
            .iter()
            .map(|face| {
                face.loops()
                    .map(|lp| {
                        lp.iter()
                            .map(|&(e, _)| Piece {
                                seg: &segs[&e],
                                curve: &self.edges[e].curve,
                            })
                            .collect()
                    })
                    .collect()
            })
            .collect();
        let cut_region: Vec<Vec<Piece<'_, S>>> = cut
            .iter()
            .zip(&cut_segs)
            .map(|(lp, segs)| {
                lp.iter()
                    .zip(segs)
                    .map(|(c, seg)| Piece {
                        seg,
                        curve: &c.curve,
                    })
                    .collect()
            })
            .collect();

        // Where the cut's curves cross the edges: a vertex each, at its
        // place along both.
        let mut vertices = self.vertices.clone();
        let mut on_edge: BTreeMap<usize, Vec<(S, usize)>> = BTreeMap::new();
        let mut on_cut: Vec<Vec<Vec<(S, usize)>>> =
            cut.iter().map(|lp| vec![Vec::new(); lp.len()]).collect();
        for &e in &used {
            let edge = &self.edges[e];
            for (l, lp) in cut.iter().enumerate() {
                for (i, c) in lp.iter().enumerate() {
                    if !boxes_overlap(&edge.curve, &c.curve)? {
                        continue;
                    }
                    let found = crossings(&segs[&e], &cut_segs[l][i], &edge.curve, &c.curve)
                        .map_err(|why| {
                            GeopError::new(format!(
                                "the cut's curve {} {why} the edge {}",
                                c.name.root(),
                                edge.name.root()
                            ))
                        })?;
                    for (along_edge, along_cut, at) in found {
                        vertices.push(LayoutVertex {
                            at,
                            name: None,
                            own: None,
                        });
                        on_edge
                            .entry(e)
                            .or_default()
                            .push((along_edge, vertices.len() - 1));
                        on_cut[l][i].push((along_cut, vertices.len() - 1));
                    }
                }
            }
        }

        // The edges split where the cut crosses them.
        let mut edges = self.edges.clone();
        let mut pieces_of: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        let mut changed = self.changed.clone();
        for (&e, crossed) in &mut on_edge {
            let edge = &self.edges[e];
            let points = ordered(crossed, &edge.name)?;
            let mut stops = vec![edge.start];
            stops.extend(points);
            stops.push(edge.end);
            let ats: Vec<Vector2<S>> = stops.iter().map(|&v| vertices[v].at).collect();
            let curves = segs[&e].split(&ats)?;
            let mut pieces = Vec::new();
            for (k, curve) in curves.into_iter().enumerate() {
                edges.push(LayoutEdge {
                    curve,
                    start: stops[k],
                    end: stops[k + 1],
                    name: edge.name.scoped(&k.to_string()),
                    home: edge.home,
                    own: None,
                    origin: edge.origin.clone(),
                });
                pieces.push(edges.len() - 1);
            }
            changed.extend(edge.origin.clone());
            pieces_of.insert(e, pieces);
        }

        // The cut's curves split where they cross the edges, each piece
        // run backwards — the cut's area on its right, the face's left —
        // and placed in the face it lies in, if any.
        let mut added: Vec<Vec<usize>> = vec![Vec::new(); self.faces.len()];
        for (l, lp) in cut.iter().enumerate() {
            let joints: Vec<usize> = lp
                .iter()
                .map(|c| {
                    vertices.push(LayoutVertex {
                        at: start_point(&c.curve)?,
                        name: None,
                        own: None,
                    });
                    Ok(vertices.len() - 1)
                })
                .collect::<GeopResult<_>>()?;
            for (i, c) in lp.iter().enumerate() {
                let points = ordered(&mut on_cut[l][i], &c.name)?;
                let mut stops = vec![joints[i]];
                stops.extend(points);
                stops.push(joints[(i + 1) % lp.len()]);
                let ats: Vec<Vector2<S>> = stops.iter().map(|&v| vertices[v].at).collect();
                let curves = cut_segs[l][i].split(&ats)?;
                let split = curves.len() > 1;
                for (k, curve) in curves.into_iter().enumerate() {
                    let seg = Seg::of(&curve)?;
                    let middle = seg.middle(&curve)?;
                    let mut home = None;
                    for (f, region) in regions.iter().enumerate() {
                        if contains(region, &middle).map_err(|why| {
                            GeopError::new(format!(
                                "the cut's curve {} {why} face {}",
                                c.name.root(),
                                self.faces[f].name.root()
                            ))
                        })? {
                            home = Some(f);
                            break;
                        }
                    }
                    let Some(f) = home else {
                        continue;
                    };
                    edges.push(LayoutEdge {
                        curve: curve.reverse(),
                        start: stops[k + 1],
                        end: stops[k],
                        name: if split {
                            c.name.scoped(&k.to_string())
                        } else {
                            c.name.clone()
                        },
                        home: self.faces[f].home,
                        own: None,
                        origin: None,
                    });
                    added[f].push(edges.len() - 1);
                }
            }
        }

        // Each face's loops: what is left of its edges, and the cut's
        // pieces in it, joined end to start.
        let mut kept: BTreeMap<usize, bool> = BTreeMap::new();
        let mut faces = Vec::new();
        let mut any = false;
        for (f, face) in self.faces.iter().enumerate() {
            let mut pool: Vec<(usize, Sense)> = Vec::new();
            let mut touched = !added[f].is_empty();
            for &(e, sense) in face.loops().flatten() {
                let pieces = pieces_of.get(&e).cloned().unwrap_or_else(|| vec![e]);
                touched |= pieces.len() > 1;
                let ordered: Vec<usize> = match sense {
                    Sense::Forward => pieces,
                    Sense::Reversed => pieces.into_iter().rev().collect(),
                };
                for p in ordered {
                    let keep = match kept.get(&p) {
                        Some(&k) => k,
                        None => {
                            let seg = Seg::of(&edges[p].curve)?;
                            let middle = seg.middle(&edges[p].curve)?;
                            let inside = contains(&cut_region, &middle).map_err(|why| {
                                GeopError::new(format!(
                                    "the edge {} {why} the cut",
                                    edges[p].name.root()
                                ))
                            })?;
                            kept.insert(p, !inside);
                            !inside
                        }
                    };
                    if keep {
                        pool.push((p, sense));
                    } else {
                        touched = true;
                        changed.extend(edges[p].origin.clone());
                    }
                }
            }
            if !touched {
                faces.push(face.clone());
                continue;
            }
            any = true;
            pool.extend(added[f].iter().map(|&e| (e, Sense::Forward)));
            faces.extend(relink(face, &pool, &edges, &vertices)?);
        }
        if !any {
            return Err(GeopError::new(
                "the cut lies off the sheet: it takes nothing away",
            ));
        }

        // A cut's vertex is named after an edge starting there.
        for face in &faces {
            for &(e, sense) in face.loops().flatten() {
                let v = match sense {
                    Sense::Forward => edges[e].start,
                    Sense::Reversed => edges[e].end,
                };
                if vertices[v].name.is_none() {
                    vertices[v].name = Some(edges[e].name.scoped("v"));
                }
            }
        }
        self.vertices = vertices;
        self.edges = edges;
        self.faces = faces;
        self.changed = changed;
        Ok(())
    }
}

/// The face `face` bounded anew by the coedges `pool`: joined end to start
/// into loops, those turning counter-clockwise outer loops — of a bend,
/// one each for the pieces a cut right across leaves of it; of a flat,
/// only one. Refuses a flat the cut splits, and a face it takes away.
fn relink<S: Scalar>(
    face: &LayoutFace,
    pool: &[(usize, Sense)],
    edges: &[LayoutEdge<S>],
    vertices: &[LayoutVertex<S>],
) -> GeopResult<Vec<LayoutFace>> {
    let ends = |&(e, sense): &(usize, Sense)| match sense {
        Sense::Forward => (edges[e].start, edges[e].end),
        Sense::Reversed => (edges[e].end, edges[e].start),
    };
    let mut leaving: BTreeMap<usize, usize> = BTreeMap::new();
    for (k, coedge) in pool.iter().enumerate() {
        let (from, _) = ends(coedge);
        if leaving.insert(from, k).is_some() {
            return Err(GeopError::new(format!(
                "the cut leaves face {} touching itself at a point",
                face.name.root()
            )));
        }
    }
    let mut used = vec![false; pool.len()];
    let mut outer: Vec<Vec<(usize, Sense)>> = Vec::new();
    let mut holes = Vec::new();
    for first in 0..pool.len() {
        if used[first] {
            continue;
        }
        let mut lp = Vec::new();
        let mut k = first;
        while !used[k] {
            used[k] = true;
            lp.push(pool[k]);
            let (_, to) = ends(&pool[k]);
            k = *leaving.get(&to).ok_or_else(|| {
                GeopError::new(format!(
                    "the cut leaves face {} open at {:?}",
                    face.name.root(),
                    vertices[to].at
                ))
            })?;
        }
        if k != first {
            return Err(GeopError::new(format!(
                "the cut leaves face {}'s boundary tangled",
                face.name.root()
            )));
        }
        let area = loop_area(&lp, edges)?;
        if area.definitely_greater(S::ZERO) {
            outer.push(lp);
        } else if area.definitely_less(S::ZERO) {
            holes.push(lp);
        } else {
            return Err(GeopError::new(format!(
                "the cut leaves a sliver of face {}: a loop enclosing {area:?}",
                face.name.root()
            )));
        }
    }
    match (outer.len(), face.home) {
        (0, _) => Err(GeopError::new(format!(
            "the cut takes face {} away entirely: cut less, or cut before the flange it belongs to",
            face.name.root()
        ))),
        (1, _) => Ok(vec![LayoutFace {
            home: face.home,
            outer: outer.remove(0),
            holes,
            name: face.name.clone(),
        }]),
        // A bend cut right across is two bends, side by side, `N,0`,
        // `N,1`, ...: each takes the holes inside it.
        (n, Home::Bend(_)) => {
            let mut pieces: Vec<LayoutFace> = outer
                .into_iter()
                .enumerate()
                .map(|(k, lp)| LayoutFace {
                    home: face.home,
                    outer: lp,
                    holes: Vec::new(),
                    name: face.name.scoped(&k.to_string()),
                })
                .collect();
            for hole in holes {
                let (e, sense) = hole[0];
                let probe = vertices[match sense {
                    Sense::Forward => edges[e].start,
                    Sense::Reversed => edges[e].end,
                }]
                .at;
                let mut home = None;
                for (k, piece) in pieces.iter().enumerate() {
                    let segs = piece
                        .outer
                        .iter()
                        .map(|&(e, _)| Seg::of(&edges[e].curve))
                        .collect::<GeopResult<Vec<_>>>()?;
                    let boundary = vec![
                        piece
                            .outer
                            .iter()
                            .zip(&segs)
                            .map(|(&(e, _), seg)| Piece {
                                seg,
                                curve: &edges[e].curve,
                            })
                            .collect::<Vec<_>>(),
                    ];
                    if contains(&boundary, &probe).map_err(|why| {
                        GeopError::new(format!(
                            "a hole of the cut {why} a piece of face {}",
                            face.name.root()
                        ))
                    })? {
                        home = Some(k);
                        break;
                    }
                }
                let k = home.ok_or_else(|| {
                    GeopError::new(format!(
                        "a hole the cut leaves in face {} lies in none of its {n} pieces",
                        face.name.root()
                    ))
                })?;
                pieces[k].holes.push(hole);
            }
            Ok(pieces)
        }
        (n, Home::Flat(_)) => Err(GeopError::new(format!(
            "the cut splits face {} into {n} pieces: a flat stays in one piece — cut a slot short of its far side",
            face.name.root()
        ))),
    }
}

/// The vertices `crossed` along a curve, in order of their places along
/// it — which must be apart.
fn ordered<S: Scalar>(crossed: &mut [(S, usize)], name: &Namer) -> GeopResult<Vec<usize>> {
    crossed.sort_by(|a, b| a.0.to_f64().total_cmp(&b.0.to_f64()));
    for w in crossed.windows(2) {
        if !w[1].0.definitely_greater(w[0].0) {
            return Err(GeopError::new(format!(
                "the cut crosses {} twice too close together to tell apart",
                name.root()
            )));
        }
    }
    Ok(crossed.iter().map(|&(_, v)| v).collect())
}

/// The area a loop encloses: positive counter-clockwise.
fn loop_area<S: Scalar>(lp: &[(usize, Sense)], edges: &[LayoutEdge<S>]) -> GeopResult<S> {
    let mut area = S::ZERO;
    for &(e, sense) in lp {
        let curve = &edges[e].curve;
        let swept = Seg::of(curve)?.swept_area(curve)?;
        area = area.add(match sense {
            Sense::Forward => swept,
            Sense::Reversed => swept.neg(),
        });
    }
    Ok(area)
}

/// Whether the boxes around two curves' control points overlap.
fn boxes_overlap<S: Scalar>(a: &NurbCurve2D<S>, b: &NurbCurve2D<S>) -> GeopResult<bool> {
    let (a, b) = (bounds(a)?, bounds(b)?);
    Ok((0..2).all(|k| a[k].could_be_equal(b[k])))
}

/// The box around a curve's control points, which it lies in.
fn bounds<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<[S; 2]> {
    let mut points = curve
        .control_points
        .iter()
        .map(|cp| Ok([cp[0].div(cp[2])?, cp[1].div(cp[2])?]));
    let first: [S; 2] = points.next().expect("a curve has control points")?;
    points.try_fold(first, |acc, p: GeopResult<[S; 2]>| {
        let p = p?;
        Ok([acc[0].union(p[0]), acc[1].union(p[1])])
    })
}

/// A curve of the layout, as far as cutting is concerned: a line, an arc
/// of less than half a turn, or something else, which a cut may not cross.
#[derive(Clone, Debug)]
pub enum Seg<S: Scalar> {
    Line {
        a: Vector2<S>,
        b: Vector2<S>,
    },
    /// From `a` to `b` about `center`, counter-clockwise if `ccw`.
    Arc {
        center: Vector2<S>,
        radius: S,
        a: Vector2<S>,
        b: Vector2<S>,
        ccw: bool,
    },
    Other,
}

/// Where a point lies along a line or an arc it is known to lie on the
/// line or circle of.
enum Along<S: Scalar> {
    /// Strictly between its ends, this far along it.
    Inside(S),
    Outside,
    /// Possibly at an end.
    Undecided,
}

fn cross2<S: Scalar>(a: &Vector2<S>, b: &Vector2<S>) -> S {
    a.prod_cross(b)
}

impl<S: Scalar> Seg<S> {
    /// What `curve` is.
    pub fn of(curve: &NurbCurve2D<S>) -> GeopResult<Self> {
        let (a, b) = (start_point(curve)?, end_point(curve)?);
        if curve.degree == 1 && curve.control_points.len() == 2 {
            return Ok(Seg::Line { a, b });
        }
        let flat = embed_curve(
            curve,
            &Vector3::zero(),
            &Vector3::axis(0),
            &Vector3::axis(1),
        )?;
        let Some(arc) = flat.as_arc()? else {
            return Ok(Seg::Other);
        };
        let ccw = arc.circle.normal[2].definitely_greater(S::ZERO);
        let center = Vector2::from_array([arc.circle.center[0], arc.circle.center[1]]);
        let sigma = if ccw { S::ONE } else { S::ONE.neg() };
        // Less than half a turn: the center on the arc's left.
        let turn = cross2(&a.sub(&center), &b.sub(&center)).mul(sigma);
        if !turn.definitely_greater(S::ZERO) {
            return Ok(Seg::Other);
        }
        Ok(Seg::Arc {
            center,
            radius: arc.circle.radius,
            a,
            b,
            ccw,
        })
    }

    /// Where `p`, on its line or circle, lies along it.
    fn along(&self, p: &Vector2<S>) -> Along<S> {
        let (a, b, outside) = match self {
            Seg::Line { a, b } => (a, b, S::ZERO),
            Seg::Arc { a, b, ccw, .. } => {
                // The arc lies right of its chord, seen turning
                // counter-clockwise.
                let side = cross2(&b.sub(a), &p.sub(a));
                (a, b, if *ccw { side } else { side.neg() })
            }
            Seg::Other => return Along::Undecided,
        };
        if outside.definitely_greater(S::ZERO) {
            return Along::Outside;
        }
        let chord = b.sub(a);
        let key = p.sub(a).prod_dot(&chord);
        let length = chord.norm_sq();
        if key.definitely_less(S::ZERO) || key.definitely_greater(length) {
            Along::Outside
        } else if key.definitely_greater(S::ZERO)
            && key.definitely_less(length)
            && (matches!(self, Seg::Line { .. }) || outside.definitely_less(S::ZERO))
        {
            Along::Inside(key)
        } else {
            Along::Undecided
        }
    }

    /// A point of it, away from its ends: halfway, along a line or an arc.
    pub fn middle(&self, curve: &NurbCurve2D<S>) -> GeopResult<Vector2<S>> {
        match self {
            Seg::Line { a, b } => Ok(a.add(b).prod_scalar(S::ONE.div(S::TWO)?)),
            Seg::Arc {
                center,
                radius,
                a,
                b,
                ..
            } => {
                let bisector = a.sub(center).add(&b.sub(center)).normalize()?;
                Ok(center.add(&bisector.prod_scalar(*radius)))
            }
            Seg::Other => {
                let (t0, t1) = curve.domain();
                curve.evaluate(t0.add(t1).div(S::TWO)?.sharpen())
            }
        }
    }

    /// `∮ (x dy - y dx) / 2` along it: what it adds to a loop's area.
    fn swept_area(&self, curve: &NurbCurve2D<S>) -> GeopResult<S> {
        let half = S::ONE.div(S::TWO)?;
        Ok(match self {
            Seg::Line { a, b } => cross2(a, b).mul(half),
            Seg::Arc {
                center,
                radius,
                a,
                b,
                ccw,
            } => {
                let (u, v) = (a.sub(center), b.sub(center));
                let angle = cross2(&u, &v).abs().atan2(u.prod_dot(&v));
                // The segment between the chord and the arc, outside the
                // chord turning counter-clockwise.
                let segment = radius.mul(*radius).mul(angle.sub(angle.sin())).mul(half);
                let chord = cross2(a, b).mul(half);
                if *ccw {
                    chord.add(segment)
                } else {
                    chord.sub(segment)
                }
            }
            Seg::Other => {
                // A polygon through points of it: its area's sign is what
                // a loop's orientation is decided by, and that sign is far
                // from in doubt for anything but a sliver, which is refused.
                let (t0, t1) = curve.domain();
                let n = 64;
                let mut area = S::ZERO;
                let mut prev = curve.evaluate(t0)?;
                for k in 1..=n {
                    let t = if k == n {
                        t1
                    } else {
                        t0.add(t1.sub(t0).mul(S::from_ratio(k, n)?)).sharpen()
                    };
                    let p = curve.evaluate(t)?;
                    area = area.add(cross2(&prev, &p).mul(half));
                    prev = p;
                }
                area
            }
        })
    }

    /// The curve split at `stops` — its start, the points along it in
    /// order, its end — into pieces of the same line or circle.
    fn split(&self, stops: &[Vector2<S>]) -> GeopResult<Vec<NurbCurve2D<S>>> {
        stops
            .windows(2)
            .map(|w| match self {
                Seg::Line { .. } => line2(w[0], w[1]),
                Seg::Arc { center, radius, .. } => arc_between(center, *radius, &w[0], &w[1]),
                Seg::Other => Err(GeopError::new(
                    "a curve that is no line or arc cannot be split",
                )),
            })
            .collect()
    }
}

/// The arc about `center` of `radius` from `p` to `q`, less than half a
/// turn: a rational quadratic through the point where its end tangents
/// meet, `center + (u + v) r² / (r² + u·v)`, weighted `cos` of half its
/// angle.
fn arc_between<S: Scalar>(
    center: &Vector2<S>,
    radius: S,
    p: &Vector2<S>,
    q: &Vector2<S>,
) -> GeopResult<NurbCurve2D<S>> {
    let (u, v) = (p.sub(center), q.sub(center));
    let r2 = radius.mul(radius);
    let dot = u.prod_dot(&v);
    let apex = center.add(&u.add(&v).prod_scalar(r2.div(r2.add(dot))?));
    let weight = r2.add(dot).div(S::TWO.mul(r2))?.sqrt()?;
    arc2(*p, apex, *q, weight)
}

/// Where the line or arc `e` and the line or arc `c` cross: each crossing's
/// place along `e`, along `c`, and its point. A curve that is neither may
/// cross nothing. Why not, if they touch, overlap or cross at an end.
fn crossings<S: Scalar>(
    e: &Seg<S>,
    c: &Seg<S>,
    e_curve: &NurbCurve2D<S>,
    c_curve: &NurbCurve2D<S>,
) -> Result<Vec<(S, S, Vector2<S>)>, String> {
    if matches!(e, Seg::Other) || matches!(c, Seg::Other) {
        let (overlaps, crossings) =
            curve_curve_overlaps_and_crossings(e_curve, c_curve, MAX_NODES, min_subdivision_size())
                .map_err(|err| format!("could not be checked against ({err})"))?;
        if overlaps.is_empty() && crossings.is_empty() {
            return Ok(Vec::new());
        }
        return Err("meets, and a cut can only cross lines and arcs, not other curves, as".into());
    }
    let candidates: Vec<(Vector2<S>, bool)> = match (e, c) {
        (Seg::Line { a, b }, Seg::Line { a: p, b: q }) => {
            let (d1, d2) = (b.sub(a), q.sub(p));
            let den = cross2(&d1, &d2);
            if den.could_be_equal(S::ZERO) {
                // Parallel: apart, or along one line.
                if !cross2(&p.sub(a), &d1).could_be_equal(S::ZERO) {
                    return Ok(Vec::new());
                }
                let reach = |x: &Vector2<S>| x.sub(a).prod_dot(&d1);
                let (lo, hi) = (reach(p).min(reach(q)), reach(p).max(reach(q)));
                if hi.definitely_less(S::ZERO) || lo.definitely_greater(d1.norm_sq()) {
                    return Ok(Vec::new());
                }
                return Err("runs along".into());
            }
            let s = cross2(&p.sub(a), &d2).div(den).map_err(|e| e.to_string())?;
            vec![(a.add(&d1.prod_scalar(s)), false)]
        }
        (Seg::Line { a, b }, Seg::Arc { center, radius, .. })
        | (Seg::Arc { center, radius, .. }, Seg::Line { a, b }) => {
            line_circle(a, b, center, *radius).map_err(|e| e.to_string())?
        }
        (
            Seg::Arc {
                center: c1,
                radius: r1,
                ..
            },
            Seg::Arc {
                center: c2,
                radius: r2,
                ..
            },
        ) => circle_circle(c1, *r1, c2, *r2, e, c)?,
        _ => unreachable!("other curves are handled above"),
    };
    let mut found = Vec::new();
    for (p, tangent) in candidates {
        match (e.along(&p), c.along(&p)) {
            (Along::Outside, _) | (_, Along::Outside) => {}
            (Along::Inside(ke), Along::Inside(kc)) if !tangent => found.push((ke, kc, p)),
            (Along::Inside(_), Along::Inside(_)) => return Err("touches".into()),
            _ => return Err("crosses, at an end of one of them,".into()),
        }
    }
    Ok(found)
}

/// Where the line through `a` and `b` meets the circle: each point, and
/// whether it could be where the line only touches it.
fn line_circle<S: Scalar>(
    a: &Vector2<S>,
    b: &Vector2<S>,
    center: &Vector2<S>,
    radius: S,
) -> GeopResult<Vec<(Vector2<S>, bool)>> {
    let d = b.sub(a);
    let w = a.sub(center);
    let qa = d.norm_sq();
    let qb = d.prod_dot(&w);
    let qc = w.norm_sq().sub(radius.mul(radius));
    // `qa s² + 2 qb s + qc = 0`.
    let disc = qb.mul(qb).sub(qa.mul(qc));
    if disc.definitely_less(S::ZERO) {
        return Ok(Vec::new());
    }
    if disc.could_be_equal(S::ZERO) {
        let s = qb.neg().div(qa)?;
        return Ok(vec![(a.add(&d.prod_scalar(s)), true)]);
    }
    let root = disc.sqrt()?;
    [qb.neg().sub(root), qb.neg().add(root)]
        .into_iter()
        .map(|num| Ok((a.add(&d.prod_scalar(num.div(qa)?)), false)))
        .collect()
}

/// Where two circles meet: each point, and whether it could be where they
/// only touch. Why not, if they could be one circle along both arcs.
fn circle_circle<S: Scalar>(
    c1: &Vector2<S>,
    r1: S,
    c2: &Vector2<S>,
    r2: S,
    e: &Seg<S>,
    c: &Seg<S>,
) -> Result<Vec<(Vector2<S>, bool)>, String> {
    let d = c2.sub(c1);
    let d2 = d.norm_sq();
    if d2.could_be_equal(S::ZERO) {
        if !r1.could_be_equal(r2) {
            return Ok(Vec::new());
        }
        // One circle: apart only if neither arc reaches into the other.
        let ends = |s: &Seg<S>| match s {
            Seg::Arc { a, b, .. } => vec![*a, *b],
            _ => Vec::new(),
        };
        let apart = ends(c).iter().all(|p| matches!(e.along(p), Along::Outside))
            && ends(e).iter().all(|p| matches!(c.along(p), Along::Outside));
        return if apart {
            Ok(Vec::new())
        } else {
            Err("runs along".into())
        };
    }
    let err = |e: GeopError| e.to_string();
    // From `c1` along `d` by `k`, across it by `h`, both in units of `|d|`.
    let k = r1
        .mul(r1)
        .sub(r2.mul(r2))
        .add(d2)
        .div(S::TWO.mul(d2))
        .map_err(err)?;
    let h2 = r1.mul(r1).div(d2).map_err(err)?.sub(k.mul(k));
    if h2.definitely_less(S::ZERO) {
        return Ok(Vec::new());
    }
    let base = c1.add(&d.prod_scalar(k));
    let across = Vector2::from_array([d[1].neg(), d[0]]);
    if h2.could_be_equal(S::ZERO) {
        return Ok(vec![(base, true)]);
    }
    let h = h2.sqrt().map_err(err)?;
    Ok(vec![
        (base.add(&across.prod_scalar(h)), false),
        (base.sub(&across.prod_scalar(h)), false),
    ])
}

/// A curve of a region's loop, with what it is.
struct Piece<'a, S: Scalar> {
    seg: &'a Seg<S>,
    curve: &'a NurbCurve2D<S>,
}

/// Directions to cast a ray in, until one gives a clear answer: spread by
/// the golden angle, none along an axis — which is where the edges of a
/// sheet most often run.
fn ray_direction<S: Scalar>(k: usize) -> Vector2<S> {
    let angle = 0.7 + 2.399_963_229_728_653 * k as f64;
    Vector2::from_array([S::from_f64(angle.cos()), S::from_f64(angle.sin())])
}

/// How many directions a ray cast tries before giving up.
const RAY_ATTEMPTS: usize = 16;

/// Whether `p` lies inside the region bounded by `loops`, by the parity
/// of a ray's crossings. Why not, if every ray met the boundary at a
/// point it could not tell inside from outside — `p` on it.
fn contains<S: Scalar>(loops: &[Vec<Piece<'_, S>>], p: &Vector2<S>) -> Result<bool, String> {
    let reach = loops
        .iter()
        .flatten()
        .map(|piece| bounds(piece.curve))
        .collect::<GeopResult<Vec<_>>>()
        .map_err(|e| e.to_string())?
        .into_iter()
        .map(|b| {
            [0, 1]
                .map(|k| b[k].sub(p[k]).abs().upper().to_f64())
                .iter()
                .sum::<f64>()
        })
        .fold(1.0, f64::max);
    'direction: for k in 0..RAY_ATTEMPTS {
        let d = ray_direction::<S>(k);
        let mut count = 0usize;
        for piece in loops.iter().flatten() {
            match ray_crossings(piece, p, &d, reach) {
                Some(n) => count += n,
                None => continue 'direction,
            }
        }
        return Ok(count % 2 == 1);
    }
    Err("cannot be told inside or outside, lying on the boundary of,".into())
}

/// How many times the ray from `p` along `d` crosses the curve; `None` if
/// it could be through an end of it, along it or touching it.
fn ray_crossings<S: Scalar>(
    piece: &Piece<'_, S>,
    p: &Vector2<S>,
    d: &Vector2<S>,
    reach: f64,
) -> Option<usize> {
    let ahead = |q: &Vector2<S>| -> Option<bool> {
        let t = q.sub(p).prod_dot(d);
        if t.definitely_greater(S::ZERO) {
            Some(true)
        } else if t.definitely_less(S::ZERO) {
            Some(false)
        } else {
            None
        }
    };
    let far = p.add(&d.prod_scalar(S::from_f64(2.0 * reach)));
    let points: Vec<(Vector2<S>, bool)> = match piece.seg {
        Seg::Line { a, b } => {
            let e = b.sub(a);
            let den = cross2(d, &e);
            if den.could_be_equal(S::ZERO) {
                return if cross2(&a.sub(p), d).could_be_equal(S::ZERO) {
                    None
                } else {
                    Some(0)
                };
            }
            let s = cross2(&p.sub(a), d).div(den).ok()?.neg();
            vec![(a.add(&e.prod_scalar(s)), false)]
        }
        Seg::Arc { center, radius, .. } => line_circle(p, &far, center, *radius).ok()?,
        Seg::Other => {
            let ray = line2(*p, far).ok()?;
            let (overlaps, crossings) = curve_curve_overlaps_and_crossings(
                &ray,
                piece.curve,
                MAX_NODES,
                min_subdivision_size(),
            )
            .ok()?;
            if !overlaps.is_empty() {
                return None;
            }
            let (t0, t1) = piece.curve.domain();
            let mut n = 0;
            for (_, t) in crossings {
                if t.could_be_equal(t0) || t.could_be_equal(t1) {
                    return None;
                }
                n += 1;
            }
            return Some(n);
        }
    };
    let mut n = 0;
    for (q, tangent) in points {
        match (piece.seg.along(&q), ahead(&q)) {
            (Along::Outside, _) | (_, Some(false)) => {}
            (Along::Inside(_), Some(true)) if !tangent => n += 1,
            _ => return None,
        }
    }
    Some(n)
}
