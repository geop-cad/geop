//! A body skinned over sections: the topology a sweep, a loft and a blend's
//! tool share, built into a [`BodySpec`] with a name for everything (see
//! [`skin`]).
//!
//! A skinned body is made of *strips*, each a section loop — joints joined
//! by curves — carried through the stations of a path: every joint sits at
//! every station (a vertex), every curve lies at every station (an edge),
//! every joint travels along every span between two stations (a *lateral*
//! edge), and every curve sweeps a *wall* face through every span. Where
//! the path is open, the strips may be closed off by flat caps at either
//! end. Where a strip's geometry comes from — sections placed by frames,
//! or control rows given outright — is its own ([`StripGeometry`]); the
//! grid is the same.
//!
//! Some of the grid collapses. A *pole* is a joint that stays put — on a
//! revolve's axis — and is one vertex at every station. A joint's two ends
//! of a span being one vertex, it has no lateral edge there, and the walls
//! beside it close over it with a coedge sitting at the vertex (see
//! [`CoedgeOn::Vertex`]). A curve *on the axis* runs between two poles,
//! sweeps no wall, and is one edge at every station — or none, where no cap
//! bounds it. And a strip may be joined to what was built before it: a
//! joint at a station to a vertex already in the spec, or a whole station
//! to a station of an earlier strip, with its vertices and its curves'
//! edges ([`Glue`]).
//!
//! Each wall is parametrized `u` along its span and `v` along its curve
//! (see [`StripGeometry::wall`]); its sides in those parameters are the
//! curve at the span's first station, run backwards along `u = 0`, the path
//! of the curve's start along `v = 0`, the curve at the span's last station
//! along `u = 1` and the path of its end, backwards along `v = 1`.

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};

use crate::{BodyNames, Namer};

/// Where a strip's entities are: its joints, curves, laterals and walls,
/// by station `s`, span `j`, joint or curve `i`.
pub trait StripGeometry<S: Scalar> {
    /// Joint `i` at station `s`.
    fn joint(&self, s: usize, i: usize) -> GeopResult<Vector3<S>>;
    /// Curve `i` at station `s`, from joint `i` to the next.
    fn curve(&self, s: usize, i: usize) -> GeopResult<NurbCurve3D<S>>;
    /// The path of joint `i` through span `j`, from its first station to
    /// its last.
    fn lateral(&self, j: usize, i: usize) -> GeopResult<NurbCurve3D<S>>;
    /// The wall curve `i` sweeps through span `j`: `u` along the span, `v`
    /// along the curve, both over `[0, 1]`.
    fn wall(&self, j: usize, i: usize) -> GeopResult<NurbSurface3D<S>>;
}

/// What a strip is joined to that was built before it.
#[derive(Clone, Debug)]
pub enum Glue {
    /// Joint `joint` at station `station` is the spec's vertex `vertex`.
    Vertex {
        station: usize,
        joint: usize,
        vertex: usize,
    },
    /// Station `station` is station `at` of the earlier strip `strip`: its
    /// joint `i` that one's `joints[i]`, and each of its curves the curve
    /// there between the same vertices, either way round.
    Section {
        station: usize,
        strip: usize,
        at: usize,
        joints: Vec<usize>,
    },
}

/// One strip of a skinned body (see the module docs): a loop of joints and
/// the curves between them — curve `i` from joint `i` to the next, around
/// to the first for a closed loop, which has as many curves as joints; an
/// open chain has one fewer — through stations and the spans between consecutive ones,
/// around to the first again on a `closed` path, which has as many spans as
/// stations; an open one has one fewer.
///
/// Everything is named by `namer`, after the joints, curves, stations and
/// spans:
///
/// | entity | name |
/// |---|---|
/// | vertex of joint `P` at station `s` | `N(P,s)`; `N(P)` for a pole |
/// | curve `X` at station `s` | `N(X,s)`; `N(X)` for a curve on the axis |
/// | lateral edge of `P` through span `q` | `N(P,q)`, or `N(P)` for an unnamed span |
/// | wall swept by `X` through span `q` | `N(X,q)`, or `N(X)` |
pub struct Strip<'a, S: Scalar> {
    pub namer: &'a Namer,
    pub joint_names: &'a [String],
    pub curve_names: &'a [String],
    pub station_names: &'a [String],
    pub span_names: &'a [Option<String>],
    pub closed: bool,
    /// Per joint: whether it is a pole.
    pub poles: &'a [bool],
    /// Per curve: whether it lies on the axis, between two poles.
    pub on_axis: &'a [bool],
    pub glued: Vec<Glue>,
    pub geometry: &'a dyn StripGeometry<S>,
}

impl<S: Scalar> Strip<'_, S> {
    fn joints(&self) -> usize {
        self.joint_names.len()
    }

    /// The joint curve `i` ends at.
    fn next(&self, i: usize) -> usize {
        (i + 1) % self.joints()
    }

    /// The station after station `j`, around a closed path.
    fn station_after(&self, j: usize) -> usize {
        (j + 1) % self.station_names.len()
    }

    /// `name` qualified by the span `j`'s name, if it has one.
    fn qualified(&self, name: &str, j: usize) -> String {
        match &self.span_names[j] {
            Some(q) => self.namer.name(&[name, q]),
            None => self.namer.name(&[name]),
        }
    }

    fn check(&self) -> GeopResult<()> {
        let (stations, spans) = (self.station_names.len(), self.span_names.len());
        let expected = if self.closed {
            stations
        } else {
            stations.saturating_sub(1)
        };
        if spans == 0 || spans != expected {
            return Err(GeopError::new(format!(
                "skin: a {} path of {stations} stations cannot have {spans} spans",
                if self.closed { "closed" } else { "open" }
            )));
        }
        let curves = self.curve_names.len();
        if self.poles.len() != self.joints()
            || self.on_axis.len() != curves
            || !(curves == self.joints() || curves + 1 == self.joints())
        {
            return Err(GeopError::new(format!(
                "skin: a loop of {} joints, {} curves, {} pole flags and {} axis flags",
                self.joints(),
                self.curve_names.len(),
                self.poles.len(),
                self.on_axis.len()
            )));
        }
        Ok(())
    }
}

/// A flat cap closing off strips at one station: on `surface`, its loops
/// one per strip, the outer first, each the strip's curves there by their
/// pcurves — each running the way its curve does. At the start of the
/// strips (`forward`), it runs every loop as the curves do; at the end,
/// backwards: the walls run them the other way.
pub struct StripCap<S: Scalar> {
    pub name: String,
    pub station: usize,
    pub forward: bool,
    pub surface: NurbSurface3D<S>,
    pub loops: Vec<(usize, Vec<NurbCurve2D<S>>)>,
}

/// What [`skin`] built, by strip, station and joint or curve: the vertices,
/// and the curves' edges, each with whether it runs against its curve —
/// none for a curve on the axis that no cap bounds.
pub struct Skinned {
    pub vertices: Vec<Vec<Vec<usize>>>,
    pub edges: Vec<Vec<Vec<Option<(usize, bool)>>>>,
}

/// A wall's sides in its own parameters (see the module docs).
fn wall_pcurves<S: Scalar>() -> GeopResult<[NurbCurve2D<S>; 4]> {
    let p = |u: f64, v: f64| Vector2::from_array([S::from_f64(u), S::from_f64(v)]);
    Ok([
        NurbCurve2D::line(p(0.0, 1.0), p(0.0, 0.0))?,
        NurbCurve2D::line(p(0.0, 0.0), p(1.0, 0.0))?,
        NurbCurve2D::line(p(1.0, 0.0), p(1.0, 1.0))?,
        NurbCurve2D::line(p(1.0, 1.0), p(0.0, 1.0))?,
    ])
}

/// `sense`, the other way where an edge runs `against` its curve.
fn along(sense: Sense, against: bool) -> Sense {
    match (sense, against) {
        (Sense::Forward, true) => Sense::Reversed,
        (Sense::Reversed, true) => Sense::Forward,
        (sense, false) => sense,
    }
}

/// Adds `strips` and their `caps` to `spec`, named in `names` (see the
/// module docs): first each strip's vertices, station curves and lateral
/// edges, then every strip's walls, then the caps.
pub fn skin<S: Scalar>(
    spec: &mut BodySpec<S>,
    names: &mut BodyNames,
    strips: &[Strip<'_, S>],
    caps: &[StripCap<S>],
) -> GeopResult<Skinned> {
    let mut built = Skinned {
        vertices: Vec::new(),
        edges: Vec::new(),
    };
    // Per strip, joint and span: its lateral edge, or the vertex it
    // collapses to.
    let mut laterals: Vec<Vec<Vec<Result<usize, usize>>>> = Vec::new();
    for (l, strip) in strips.iter().enumerate() {
        strip.check()?;
        let stations = strip.station_names.len();
        let mut glued_vertex = vec![vec![None; strip.joints()]; stations];
        let mut glued_section = vec![None; stations];
        for glue in &strip.glued {
            match glue {
                Glue::Vertex {
                    station,
                    joint,
                    vertex,
                } => glued_vertex[*station][*joint] = Some(*vertex),
                Glue::Section {
                    station,
                    strip: m,
                    at,
                    joints,
                } => {
                    let earlier = built.vertices.get(*m).filter(|_| *m < l).ok_or_else(|| {
                        GeopError::new(format!(
                            "skin: strip {l} is joined to strip {m}, which is not built before it"
                        ))
                    })?;
                    for (i, &k) in joints.iter().enumerate() {
                        glued_vertex[*station][i] = Some(earlier[*at][k]);
                    }
                    glued_section[*station] = Some((*m, *at));
                }
            }
        }

        let mut vertex = vec![vec![0; strip.joints()]; stations];
        for i in 0..strip.joints() {
            let name = &strip.joint_names[i];
            if strip.poles[i] {
                spec.vertices.push(strip.geometry.joint(0, i)?);
                names.vertices.push(strip.namer.name(&[name]));
                for row in vertex.iter_mut() {
                    row[i] = spec.vertices.len() - 1;
                }
                continue;
            }
            for s in 0..stations {
                vertex[s][i] = match glued_vertex[s][i] {
                    Some(v) => v,
                    None => {
                        spec.vertices.push(strip.geometry.joint(s, i)?);
                        names
                            .vertices
                            .push(strip.namer.name(&[name, &strip.station_names[s]]));
                        spec.vertices.len() - 1
                    }
                };
            }
        }

        let capped = caps.iter().any(|c| c.loops.iter().any(|(m, _)| *m == l));
        let mut edges = vec![vec![None; strip.curve_names.len()]; stations];
        for (i, name) in strip.curve_names.iter().enumerate() {
            let next = strip.next(i);
            if strip.on_axis[i] {
                // One edge for every station — or none, without caps to
                // bound: no wall uses it.
                if capped {
                    spec.edges.push(EdgeSpec {
                        curve: strip.geometry.curve(0, i)?,
                        start: vertex[0][i],
                        end: vertex[0][next],
                    });
                    names.edges.push(strip.namer.name(&[name]));
                    for row in edges.iter_mut() {
                        row[i] = Some((spec.edges.len() - 1, false));
                    }
                }
                continue;
            }
            for s in 0..stations {
                let (from, to) = (vertex[s][i], vertex[s][next]);
                edges[s][i] = Some(match glued_section[s] {
                    Some((m, at)) => built.edges[m][at]
                        .iter()
                        .flatten()
                        .find_map(|&(e, _)| {
                            let edge = &spec.edges[e];
                            if (edge.start, edge.end) == (from, to) {
                                Some((e, false))
                            } else if (edge.start, edge.end) == (to, from) {
                                Some((e, true))
                            } else {
                                None
                            }
                        })
                        .ok_or_else(|| {
                            GeopError::new(format!(
                                "skin: strip {l} at station {s} is joined to a section with no curve {name} of its own"
                            ))
                        })?,
                    None => {
                        spec.edges.push(EdgeSpec {
                            curve: strip.geometry.curve(s, i)?,
                            start: from,
                            end: to,
                        });
                        names
                            .edges
                            .push(strip.namer.name(&[name, &strip.station_names[s]]));
                        (spec.edges.len() - 1, false)
                    }
                });
            }
        }

        let spans = strip.span_names.len();
        let mut paths = Vec::with_capacity(strip.joints());
        for i in 0..strip.joints() {
            let mut path = Vec::with_capacity(spans);
            for j in 0..spans {
                let (from, to) = (vertex[j][i], vertex[strip.station_after(j)][i]);
                if from == to {
                    path.push(Err(from));
                    continue;
                }
                spec.edges.push(EdgeSpec {
                    curve: strip.geometry.lateral(j, i)?,
                    start: from,
                    end: to,
                });
                names.edges.push(strip.qualified(&strip.joint_names[i], j));
                path.push(Ok(spec.edges.len() - 1));
            }
            paths.push(path);
        }
        built.vertices.push(vertex);
        built.edges.push(edges);
        laterals.push(paths);
    }

    let [first_back, start_side, last_forward, end_side] = wall_pcurves::<S>()?;
    for (l, strip) in strips.iter().enumerate() {
        for (i, name) in strip.curve_names.iter().enumerate() {
            if strip.on_axis[i] {
                continue;
            }
            let next = strip.next(i);
            for j in 0..strip.span_names.len() {
                // Along a joint's path, or at its vertex where it has none.
                let side = |joint: usize, sense: Sense, pcurve: &NurbCurve2D<S>| CoedgeSpec {
                    on: match laterals[l][joint][j] {
                        Ok(edge) => CoedgeOn::Edge(edge, sense),
                        Err(vertex) => CoedgeOn::Vertex(vertex),
                    },
                    pcurve: pcurve.clone(),
                };
                // Along the curve at a station, the other way where its
                // edge runs against it.
                let at = |s: usize, sense: Sense, pcurve: &NurbCurve2D<S>| {
                    let (edge, against) =
                        built.edges[l][s][i].expect("a curve off the axis has an edge");
                    CoedgeSpec {
                        on: CoedgeOn::Edge(edge, along(sense, against)),
                        pcurve: pcurve.clone(),
                    }
                };
                spec.faces.push(FaceSpec {
                    surface: strip.geometry.wall(j, i)?,
                    outer: vec![
                        at(j, Sense::Reversed, &first_back),
                        side(i, Sense::Forward, &start_side),
                        at(strip.station_after(j), Sense::Forward, &last_forward),
                        side(next, Sense::Reversed, &end_side),
                    ],
                    holes: Vec::new(),
                });
                names.faces.push(strip.qualified(name, j));
            }
        }
    }

    for cap in caps {
        let mut boundaries = Vec::with_capacity(cap.loops.len());
        for (l, pcurves) in &cap.loops {
            let mut coedges = pcurves
                .iter()
                .enumerate()
                .map(|(i, pcurve)| {
                    let (edge, against) = built.edges[*l][cap.station][i]
                        .ok_or_else(|| GeopError::new("skin: a cap's curves have edges"))?;
                    Ok(if cap.forward {
                        CoedgeSpec {
                            on: CoedgeOn::Edge(edge, along(Sense::Forward, against)),
                            pcurve: pcurve.clone(),
                        }
                    } else {
                        CoedgeSpec {
                            on: CoedgeOn::Edge(edge, along(Sense::Reversed, against)),
                            pcurve: pcurve.reverse(),
                        }
                    })
                })
                .collect::<GeopResult<Vec<_>>>()?;
            if !cap.forward {
                coedges.reverse();
            }
            boundaries.push(coedges);
        }
        if boundaries.is_empty() {
            return Err(GeopError::new(format!(
                "skin: the cap {} has no loop",
                cap.name
            )));
        }
        let outer = boundaries.remove(0);
        spec.faces.push(FaceSpec {
            surface: cap.surface.clone(),
            outer,
            holes: boundaries,
        });
        names.faces.push(cap.name.clone());
    }
    Ok(built)
}
