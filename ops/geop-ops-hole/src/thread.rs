//! Threads on cylindrical walls: where one runs ([`Wall`],
//! [`ThreadPlacement`]), recorded as a [`CosmeticThread`], or modelled — the
//! ISO metric profile swept along a helix ([`thread_tool`]) and cut away.
//!
//! The ISO metric basic profile (ISO 68-1) is a 60° triangle of height `H =
//! sqrt(3) / 2 P`, cut flat at the crests and roots. A bolt's thread is cut
//! by the groove between its teeth: `P / 4` wide at the minor diameter `d1`,
//! widening at 30° per flank out past the shaft. A nut's thread is cut by a
//! bolt's tooth: `P / 8` wide at the major diameter `D`, widening inwards
//! past the hole's wall. Either runs a sixteenth of `H` past the wall, so
//! that it cuts through it rather than along it, and stays narrower than a
//! pitch, so that its turns never touch.

use geop_core_geometry::{
    nurb_curve::{Handedness, cos_sin},
    shape::{Axis, Cylinder},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{Body, CoedgeGeometry, FaceId, SolidId};
use geop_ops::{CosmeticThread, Namer, Part, operation::frame_along};
use geop_ops_extrude_revolve::{
    common::{Profile, polygon},
    revolve::screw,
    sweep::SweepLoop,
};

use crate::iso::{MetricSize, TRIANGLE_HEIGHT};

/// A cylindrical wall of a solid: the faces on one cylinder, joined across
/// their edges — a hole's wall or a shaft's, whatever quarters and pieces
/// it is made of — and how far it reaches along its axis.
#[derive(Clone, Debug)]
pub struct Wall<S: Scalar> {
    pub cylinder: Cylinder<S>,
    pub faces: Vec<FaceId>,
    /// From where to where along `cylinder.axis.direction`, measured from
    /// `cylinder.axis.point`: the enclosures of its nearest and its
    /// farthest corner.
    pub from: S,
    pub to: S,
    /// The solid lies outside the cylinder: a hole, not a shaft.
    pub internal: bool,
    /// Whether a mating part could be screwed on at its end at `from`, and
    /// at its end at `to` (see [`Wall::of`]).
    pub open: [bool; 2],
}

/// Whether `a` and `b` could be the same cylinder.
fn same_cylinder<S: Scalar>(a: &Cylinder<S>, b: &Cylinder<S>) -> bool {
    a.radius.could_be_equal(b.radius)
        && a.axis.could_be_parallel(&b.axis)
        && a.axis.could_contain(&b.axis.point)
}

/// The faces across an edge from `face`.
fn neighbours<S: Scalar>(part: &Part<S>, face: FaceId) -> Vec<FaceId> {
    let model = part.topology();
    let mut found = Vec::new();
    for coedge in model.iterate_face_coedges(face) {
        let Ok(c) = model.get_coedge(coedge) else {
            continue;
        };
        if let CoedgeGeometry::Edge(edge) = c.geometry {
            for other in model.coedges_of_edge(edge) {
                if let Ok(o) = model.get_coedge(other)
                    && o.face != face
                {
                    found.push(o.face);
                }
            }
        }
    }
    found
}

impl<S: Scalar> Wall<S> {
    /// The wall `face` is part of. Fails, naming it, for a face that is not
    /// cylindrical.
    pub fn of(part: &Part<S>, face: FaceId) -> GeopResult<Self> {
        let name = part.name_of(face).unwrap_or_default().to_string();
        let ctx = with_context!("the cylindrical wall of face {name:?}");
        let model = part.topology();
        let surface = &model.get_face(face).with_context(ctx)?.surface;
        let Some(cylinder) = surface.as_cylinder().with_context(ctx)? else {
            return Err(GeopError::new(format!(
                "face {name:?} is not cylindrical: a thread runs on a cylindrical face"
            )));
        };
        let mut faces = vec![face];
        let mut todo = vec![face];
        while let Some(f) = todo.pop() {
            for n in neighbours(part, f) {
                if faces.contains(&n) {
                    continue;
                }
                let on = model.get_face(n)?.surface.as_cylinder()?;
                if on.is_some_and(|c| same_cylinder(&cylinder, &c)) {
                    faces.push(n);
                    todo.push(n);
                }
            }
        }
        faces.sort_by_key(|f| f.0);
        let mut extent: Option<(S, S)> = None;
        for &f in &faces {
            for coedge in model.iterate_face_coedges(f) {
                let c = model.get_coedge(coedge)?;
                let CoedgeGeometry::Edge(edge) = c.geometry else {
                    continue;
                };
                let edge = model.get_edge(edge)?;
                for v in [edge.start_vertex, edge.end_vertex] {
                    let p = model.get_vertex(v)?.point;
                    let along = p
                        .sub(&cylinder.axis.point)
                        .prod_dot(&cylinder.axis.direction);
                    extent = Some(match extent {
                        None => (along, along),
                        Some((from, to)) => (
                            if along.to_f64() < from.to_f64() {
                                along
                            } else {
                                from
                            },
                            if along.to_f64() > to.to_f64() {
                                along
                            } else {
                                to
                            },
                        ),
                    });
                }
            }
        }
        let Some((from, to)) = extent.filter(|(from, to)| to.definitely_greater(*from)) else {
            return Err(GeopError::new(format!(
                "face {name:?} has no length along its axis"
            )))
            .with_context(ctx);
        };
        // Which side the solid is on: the face's normal points out of it.
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        let (u, v) = (u0.add(u1).div(S::TWO)?, v0.add(v1).div(S::TWO)?);
        let p = surface.evaluate(u, v)?;
        let outwards = p.sub(&cylinder.axis.project(&p));
        let facing = surface.normal(u, v)?.prod_dot(&outwards);
        let internal = if facing.definitely_less(S::ZERO) {
            true
        } else if facing.definitely_greater(S::ZERO) {
            false
        } else {
            return Err(GeopError::new(format!(
                "cannot tell which side of face {name:?} its solid is on"
            )));
        };
        let mut wall = Wall {
            cylinder,
            faces,
            from,
            to,
            internal,
            open: [false; 2],
        };
        wall.open = [wall.is_open(part, from)?, wall.is_open(part, to)?];
        Ok(wall)
    }

    /// Whether its end at `end` along its axis is open, as the faces it
    /// meets there say: a shaft's where they reach no farther out than the
    /// shaft — its end face, or a chamfer — and not where one steps out to
    /// a shoulder; a hole's where one reaches out beyond the hole — the
    /// face it was drilled into — and not at its bottom.
    fn is_open(&self, part: &Part<S>, end: S) -> GeopResult<bool> {
        let model = part.topology();
        let axis = &self.cylinder.axis;
        let along = |p: &Vector3<S>| p.sub(&axis.point).prod_dot(&axis.direction);
        let mut beyond = false;
        for &f in &self.faces {
            for coedge in model.iterate_face_coedges(f) {
                let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry else {
                    continue;
                };
                let e = model.get_edge(edge)?;
                let at_end = [e.start_vertex, e.end_vertex].into_iter().all(|v| {
                    model
                        .get_vertex(v)
                        .is_ok_and(|v| along(&v.point).could_be_equal(end))
                });
                if !at_end {
                    continue;
                }
                for other in model.coedges_of_edge(edge) {
                    let face = model.get_coedge(other)?.face;
                    if self.faces.contains(&face) {
                        continue;
                    }
                    for c in model.iterate_face_coedges(face) {
                        let CoedgeGeometry::Edge(e) = model.get_coedge(c)?.geometry else {
                            continue;
                        };
                        let e = model.get_edge(e)?;
                        for v in [e.start_vertex, e.end_vertex] {
                            let p = model.get_vertex(v)?.point;
                            let r = p.sub(&axis.project(&p)).norm();
                            beyond |= r.definitely_greater(self.cylinder.radius);
                        }
                    }
                }
            }
        }
        Ok(beyond == self.internal)
    }

    /// The end a thread on it starts from — `(where along its axis, the way
    /// into it)` — unless `reversed`: the one open end, if only one is;
    /// else its end at `to`.
    pub fn start(&self, reversed: bool) -> (S, Vector3<S>) {
        let at_from = self.open == [true, false];
        let direction = self.cylinder.axis.direction;
        if at_from != reversed {
            (self.from, direction)
        } else {
            (self.to, direction.neg())
        }
    }

    /// The wall of the solid `solid` on `cylinder` that reaches `at` along
    /// its axis — a hole's wall at the face it was drilled into — if any.
    pub fn at(
        part: &Part<S>,
        solid: SolidId,
        cylinder: &Cylinder<S>,
        at: &Vector3<S>,
    ) -> GeopResult<Option<Self>> {
        let model = part.topology();
        for face in model.body_faces(Body::Solid(solid))? {
            let on = model.get_face(face)?.surface.as_cylinder()?;
            if !on.is_some_and(|c| same_cylinder(cylinder, &c)) {
                continue;
            }
            let wall = Wall::of(part, face)?;
            let along = at
                .sub(&wall.cylinder.axis.point)
                .prod_dot(&wall.cylinder.axis.direction);
            if !along.definitely_less(wall.from) && !along.definitely_greater(wall.to) {
                return Ok(Some(wall));
            }
        }
        Ok(None)
    }

    /// The point of its axis `along` its direction.
    pub fn axis_point(&self, along: S) -> Vector3<S> {
        self.cylinder
            .axis
            .point
            .add(&self.cylinder.axis.direction.prod_scalar(along))
    }

    /// How long it is along its axis.
    pub fn length(&self) -> S {
        self.to.sub(self.from)
    }

    /// The name of its first face: what a thread on it records it was put
    /// on.
    pub fn face_name(&self, part: &Part<S>) -> String {
        part.name_of(self.faces[0]).unwrap_or_default().to_string()
    }
}

/// Where a thread runs: from `start`, on the axis of a wall of `radius`,
/// along the unit vector `direction`, `length` far; in a hole if
/// `internal`.
#[derive(Clone, Debug)]
pub struct ThreadPlacement<S: Scalar> {
    pub start: Vector3<S>,
    pub direction: Vector3<S>,
    pub radius: S,
    pub length: f64,
    pub internal: bool,
}

impl<S: Scalar> ThreadPlacement<S> {
    /// Checks that a thread of `size` fits a wall of its radius — the face
    /// `face`: a shaft between the minor and the nominal diameter, a hole
    /// from the minor diameter up to short of the nominal one.
    pub fn check_fits(&self, size: &MetricSize, face: &str) -> GeopResult<()> {
        let d = self.radius.add(self.radius);
        let (minor, major) = (size.minor_diameter(), size.diameter);
        let (lo, hi) = (S::from_f64(minor), S::from_f64(major));
        let fits = if self.internal {
            !d.definitely_less(lo) && d.definitely_less(hi)
        } else {
            d.definitely_greater(lo) && !d.definitely_greater(hi)
        };
        if !fits {
            return Err(GeopError::new(format!(
                "face {face:?} is Ø{:.3}: an {} thread {} needs {} Ø{minor:.3} to Ø{major}",
                d.to_f64(),
                if self.internal {
                    "internal"
                } else {
                    "external"
                },
                size.designation(),
                if self.internal {
                    "a hole of"
                } else {
                    "a shaft of"
                },
            )));
        }
        if !(self.length.is_finite() && self.length > 0.0) {
            return Err(GeopError::new(format!(
                "the thread's length {} must be more than 0",
                self.length
            )));
        }
        Ok(())
    }

    /// Where a modelled thread of `pitch` is swept from, and how far, on
    /// `wall`: where it is to run, but with each of its ends that is an end
    /// of the wall moved a pitch off it — out beyond an open end, where it
    /// runs out of the solid, and back from a closed one, a shoulder or a
    /// hole's bottom, which a thread stops short of.
    ///
    /// Swept from exactly the end of the wall, its end face would cut the
    /// face there through the middle of its profile, right where the wall's
    /// rim crosses it — three faces through one point that the booleans did
    /// not splice (the trace along the end face and the shank's end ran into
    /// two corners at the rim). A pitch off, the profile, less than a pitch
    /// wide, is clear of that face altogether.
    pub fn swept(&self, wall: &Wall<S>, pitch: f64) -> GeopResult<(Vector3<S>, f64)> {
        let axis = &wall.cylinder.axis;
        let along = |p: &Vector3<S>| p.sub(&axis.point).prod_dot(&axis.direction);
        let ends = [(wall.from, wall.open[0]), (wall.to, wall.open[1])];
        // How far out past the wall's end at `at`, if it is one.
        let past = |at: S| {
            ends.iter()
                .find(|(end, _)| at.could_be_equal(*end))
                .map_or(0.0, |&(_, open)| if open { pitch } else { -pitch })
        };
        let start = along(&self.start);
        let end = start.add(
            axis.direction
                .prod_dot(&self.direction)
                .mul(S::from_f64(self.length)),
        );
        let (before, after) = (past(start), past(end));
        let length = self.length + before + after;
        if length <= 0.0 {
            return Err(GeopError::new(format!(
                "a thread {} long is too short to model: it stops a pitch short of the wall's closed ends",
                self.length
            )));
        }
        let start = self
            .start
            .sub(&self.direction.prod_scalar(S::from_f64(before)));
        Ok((start, length))
    }

    /// The thread of `size`, as recorded on `face`.
    pub fn cosmetic(&self, size: &MetricSize, face: String) -> GeopResult<CosmeticThread<S>> {
        Ok(CosmeticThread {
            designation: size.designation(),
            face,
            axis: Axis::try_new(self.start, self.direction)?,
            radius: self.radius,
            major_diameter: size.diameter,
            minor_diameter: size.minor_diameter(),
            pitch: size.pitch,
            length: self.length,
            internal: self.internal,
            handedness: Handedness::Right,
        })
    }
}

/// The angle, in degrees from `frame.u()` towards `frame.v()`, a thread
/// turning `turns` times in `frame` starts at, so that none of its
/// stations — its ends, and where each span of a quarter turn at most
/// meets the next, every one a half-plane through the axis with edges in it
/// — lies in a half-plane through a corner of `wall`. Where on its circle a
/// thread starts is a free choice; one that puts a station through a
/// corner the wall already has asks the boolean to cross that corner with
/// the station's edges, a degenerate configuration (it failed to splice a
/// tooth into a tapped hole whose seams lay in the stations' half-planes).
/// So of every half degree, take the one farthest from all of them.
fn clear_start<S: Scalar>(
    part: &Part<S>,
    wall: &Wall<S>,
    frame: &CoordinateSystem<S>,
    turns: f64,
) -> GeopResult<f64> {
    let model = part.topology();
    let mut corners = Vec::new();
    for &f in &wall.faces {
        for coedge in model.iterate_face_coedges(f) {
            let CoedgeGeometry::Edge(edge) = model.get_coedge(coedge)?.geometry else {
                continue;
            };
            let e = model.get_edge(edge)?;
            for v in [e.start_vertex, e.end_vertex] {
                let q = frame.to_uvw(&model.get_vertex(v)?.point);
                corners.push(q[1].to_f64().atan2(q[0].to_f64()).to_degrees());
            }
        }
    }
    let spans = (turns * 4.0).ceil() as usize;
    let step = 360.0 * turns / spans as f64;
    let apart = |a: f64, b: f64| {
        let d = (a - b).rem_euclid(360.0);
        d.min(360.0 - d)
    };
    let clearance = |start: f64| {
        (0..=spans)
            .flat_map(|k| {
                corners
                    .iter()
                    .map(move |&c| apart(start + k as f64 * step, c))
            })
            .fold(f64::INFINITY, f64::min)
    };
    Ok((0..720)
        .map(|k| k as f64 / 2.0)
        .max_by(|&a, &b| clearance(a).total_cmp(&clearance(b)))
        .expect("there are candidates"))
}

/// The solid that cuts the thread of `size` at `placement` on `wall` — a
/// bolt's groove, or in a hole a bolt's tooth (see the module docs) — swept
/// along its helix from where it starts as far as it runs, starting at an
/// angle clear of the wall's corners ([`clear_start`]): a solid named
/// `solid`, its faces `N(c0,q)` .. `N(c3,q)` for each quarter turn `q` and
/// its ends `N(start)` and `N(end)` by `namer`.
pub fn thread_tool<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: &str,
    size: &MetricSize,
    placement: &ThreadPlacement<S>,
    wall: &Wall<S>,
) -> GeopResult<SolidId> {
    let pitch = size.pitch;
    let h = TRIANGLE_HEIGHT * pitch;
    let tan30 = 1.0 / 3f64.sqrt();
    let radius = placement.radius.to_f64();
    // `(r, half width)` at its inner and outer ends.
    let (inner, outer) = if placement.internal {
        let half = |r: f64| pitch / 16.0 + (size.diameter / 2.0 - r) * tan30;
        let r_in = radius - h / 16.0;
        let r_out = size.diameter / 2.0;
        ((r_in, half(r_in)), (r_out, half(r_out)))
    } else {
        let minor = size.minor_diameter() / 2.0;
        let half = |r: f64| pitch / 8.0 + (r - minor) * tan30;
        let r_out = radius + h / 16.0;
        ((minor, half(minor)), (r_out, half(r_out)))
    };
    let p = |r: f64, z: f64| Vector2::from_array([S::from_f64(r), S::from_f64(z)]);
    // Counter-clockwise in `(r, z)`.
    let corners = [
        p(inner.0, -inner.1),
        p(outer.0, -outer.1),
        p(outer.0, outer.1),
        p(inner.0, inner.1),
    ];
    let profile = Profile::closed(polygon(&corners)?);
    // Where it starts and ends, moved off the wall's ends (see
    // `ThreadPlacement::swept`).
    let (start, length) = placement.swept(wall, pitch)?;
    let turns = length / pitch;
    let frame = frame_along(start, &placement.direction)?;
    let (c, s) = cos_sin(clear_start(part, wall, &frame, turns)?);
    let u = frame
        .u()
        .prod_scalar(S::from_f64(c))
        .add(&frame.v().prod_scalar(S::from_f64(s)));
    let w = *frame.w();
    let axes = CoordinateSystem::try_new(start, u, w.prod_cross(&u), w)?;
    screw(
        part,
        namer,
        solid,
        &axes,
        S::from_f64(pitch),
        turns,
        Handedness::Right,
        &[SweepLoop::plain(profile)],
    )
}
