//! Threads on round walls, straight or tapered: where one runs ([`Wall`],
//! [`ThreadPlacement`]), recorded as a [`CosmeticThread`], or modelled — the
//! thread's profile swept along a helix ([`thread_tool`]) and cut away.
//!
//! A thread is an ISO metric one or an NPT pipe thread ([`ThreadSize`]):
//! both a 60° triangle of height `H = sqrt(3) / 2 P`, cut flat at the crests
//! and roots, the pipe thread on a cone. A bolt's thread is cut by the
//! groove between its teeth, at the minor diameter as wide as the sharp V's
//! truncation there leaves it, widening at 30° per flank out past the
//! shaft. A nut's thread is cut by a bolt's tooth, as wide at the major
//! diameter as the truncation at the crest leaves it, widening inwards past
//! the hole's wall. Either runs a little past the wall, so that it cuts
//! through it rather than along it, and stays narrower than a pitch, so
//! that its turns never touch. A tapered thread's profile moves out from
//! the axis as it rises, by the taper ([`Helix::taper`]).

use geop_core_geometry::{
    nurb_curve::{Handedness, Helix, cos_sin},
    shape::{Axis, RoundWall},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    solvers::walk::{Take, flood},
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{Body, FaceId, SolidId};
use geop_lib_sweep::{SweepLoop, polygon, screw};
use geop_ops_model::Profile;
use geop_ops_model::{CosmeticThread, NamedModel, Namer, frame_along};

use crate::{
    iso::{self, MetricSize, TRIANGLE_HEIGHT},
    pipe::{self, PipeSize},
};

/// The size of a thread: an ISO metric one, or an NPT pipe thread.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ThreadSize {
    Metric(&'static MetricSize),
    Pipe(&'static PipeSize),
}

impl ThreadSize {
    /// Every size, metric then pipe, smallest first.
    pub fn all() -> impl Iterator<Item = ThreadSize> {
        (iso::SIZES.iter().map(ThreadSize::Metric)).chain(pipe::SIZES.iter().map(ThreadSize::Pipe))
    }

    /// The size called `name`: `M6`, or `1/2 NPT`.
    pub fn named(name: &str) -> GeopResult<Self> {
        match name.strip_suffix(" NPT") {
            Some(pipe) => pipe::npt(pipe).map(ThreadSize::Pipe),
            None => iso::metric(name).map(ThreadSize::Metric),
        }
    }

    /// What it is called: `M6`, or `1/2 NPT`.
    pub fn name(&self) -> String {
        match self {
            ThreadSize::Metric(s) => s.name.to_string(),
            ThreadSize::Pipe(s) => format!("{} NPT", s.name),
        }
    }

    /// How the thread is called: `M6x1`, or `1/2-14 NPT`.
    pub fn designation(&self) -> String {
        match self {
            ThreadSize::Metric(s) => s.designation(),
            ThreadSize::Pipe(s) => s.designation(),
        }
    }

    pub fn pitch(&self) -> f64 {
        match self {
            ThreadSize::Metric(s) => s.pitch,
            ThreadSize::Pipe(s) => s.pitch(),
        }
    }

    /// How much its radius grows per unit of length towards its large end:
    /// none for a straight thread.
    pub fn taper(&self) -> f64 {
        match self {
            ThreadSize::Metric(_) => 0.0,
            ThreadSize::Pipe(_) => pipe::TAPER,
        }
    }

    /// How a wall's radius changes along a thread of this size running
    /// from where it starts: into a hole from its large end it shrinks,
    /// along a shaft from its small end it grows.
    pub fn taper_along(&self, internal: bool) -> f64 {
        if internal {
            -self.taper()
        } else {
            self.taper()
        }
    }

    /// Its major and minor diameters where it starts: internal, at the
    /// face a hole is tapped from; external, at the shaft's end.
    pub fn diameters(&self, internal: bool) -> (f64, f64) {
        match self {
            ThreadSize::Metric(s) => (s.diameter, s.minor_diameter()),
            ThreadSize::Pipe(s) => s.diameters(internal),
        }
    }

    /// How far the sharp V's crest lies beyond the major diameter, in
    /// radius: `H / 8` for ISO metric.
    pub fn crest(&self) -> f64 {
        match self {
            ThreadSize::Metric(s) => TRIANGLE_HEIGHT * s.pitch / 8.0,
            ThreadSize::Pipe(s) => s.crest(),
        }
    }

    /// How far the sharp V's root lies inside the minor diameter, in
    /// radius: what of `H` neither the thread's height nor its crest takes
    /// — `H / 4` for ISO metric.
    pub fn root(&self) -> f64 {
        let (major, minor) = self.diameters(false);
        TRIANGLE_HEIGHT * self.pitch() - (major - minor) / 2.0 - self.crest()
    }

    /// Half the width of the flat at its crest and at its root: `P / 16`
    /// and `P / 8` for ISO metric, which the truncations leave at 30° per
    /// flank.
    pub fn half_flats(&self) -> (f64, f64) {
        match self {
            ThreadSize::Metric(s) => (s.pitch / 16.0, s.pitch / 8.0),
            ThreadSize::Pipe(_) => {
                let tan30 = 1.0 / 3f64.sqrt();
                (self.crest() * tan30, self.root() * tan30)
            }
        }
    }
}

/// A round wall of a solid, straight or tapered: the faces on one
/// [`RoundWall`], joined across their edges — a hole's wall or a shaft's,
/// whatever quarters and pieces it is made of — and how far it reaches
/// along its axis.
#[derive(Clone, Debug)]
pub struct Wall {
    pub round: RoundWall,
    pub faces: Vec<FaceId>,
    /// From where to where along `round.axis.direction`, measured from
    /// `round.axis.point`: the enclosures of its nearest and its farthest
    /// corner.
    pub from: Scalar,
    pub to: Scalar,
    /// The solid lies outside the wall: a hole, not a shaft.
    pub internal: bool,
    /// Whether a mating part could be screwed on at its end at `from`, and
    /// at its end at `to` (see [`Wall::of`]).
    pub open: [bool; 2],
}

/// The faces across an edge from `face`.
fn neighbours(model: &NamedModel, face: FaceId) -> Vec<FaceId> {
    let topology = model.topology();
    let mut found = Vec::new();
    for edge in topology.face_edges(face) {
        for other in topology.coedges_of_edge(edge) {
            if let Ok(o) = topology.get_coedge(other)
                && o.face != face
            {
                found.push(o.face);
            }
        }
    }
    found
}

impl Wall {
    /// The wall `face` is part of. Fails, naming it, for a face that is not
    /// round.
    pub fn of(model: &NamedModel, face: FaceId) -> GeopResult<Self> {
        let name = model.name_of(face).unwrap_or_default().to_string();
        let ctx = with_context!("the round wall of face {name:?}");
        let topology = model.topology();
        let surface = &topology.get_face(face).with_context(ctx)?.surface;
        let Some(round) = surface.as_round_wall().with_context(ctx)? else {
            return Err(GeopError::new(format!(
                "face {name:?} is not a round wall: a thread runs on a face turned round an axis from a straight line"
            )));
        };
        // Out from `face` across its edges to every face of the same wall.
        let mut faces = vec![face];
        flood([face], Take::Newest, |&f| {
            let mut found = Vec::new();
            for n in neighbours(model, f) {
                if faces.contains(&n) {
                    continue;
                }
                let on = topology.get_face(n)?.surface.as_round_wall()?;
                if on.is_some_and(|w| round.could_be_equal(&w)) {
                    faces.push(n);
                    found.push(n);
                }
            }
            Ok::<_, GeopError>(found)
        })?;
        faces.sort_by_key(|f| f.0);
        let mut extent: Option<(Scalar, Scalar)> = None;
        for &f in &faces {
            for edge in topology.face_edges(f) {
                let edge = topology.get_edge(edge)?;
                for v in [edge.start_vertex, edge.end_vertex] {
                    let along = round.along(&topology.get_vertex(v)?.point);
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
        let (u, v) = (u0.add(u1).div(Scalar::TWO)?, v0.add(v1).div(Scalar::TWO)?);
        let p = surface.evaluate(u, v)?;
        let outwards = p.sub(&round.axis.project(&p));
        let facing = surface.normal(u, v)?.prod_dot(&outwards);
        let internal = if facing.definitely_less(Scalar::ZERO) {
            true
        } else if facing.definitely_greater(Scalar::ZERO) {
            false
        } else {
            return Err(GeopError::new(format!(
                "cannot tell which side of face {name:?} its solid is on"
            )));
        };
        let mut wall = Wall {
            round,
            faces,
            from,
            to,
            internal,
            open: [false; 2],
        };
        wall.open = [wall.is_open(model, from)?, wall.is_open(model, to)?];
        Ok(wall)
    }

    /// Whether its end at `end` along its axis is open, as the faces it
    /// meets there say: a shaft's where they reach no farther out than the
    /// shaft — its end face, or a chamfer — and not where one steps out to
    /// a shoulder; a hole's where one reaches out beyond the hole — the
    /// face it was drilled into — and not at its bottom.
    fn is_open(&self, model: &NamedModel, end: Scalar) -> GeopResult<bool> {
        let topology = model.topology();
        let axis = &self.round.axis;
        let mut beyond = false;
        for &f in &self.faces {
            for edge in topology.face_edges(f) {
                let e = topology.get_edge(edge)?;
                let at_end = [e.start_vertex, e.end_vertex].into_iter().all(|v| {
                    topology
                        .get_vertex(v)
                        .is_ok_and(|v| self.round.along(&v.point).could_be_equal(end))
                });
                if !at_end {
                    continue;
                }
                for other in topology.coedges_of_edge(edge) {
                    let face = topology.get_coedge(other)?.face;
                    if self.faces.contains(&face) {
                        continue;
                    }
                    for e in topology.face_edges(face) {
                        let e = topology.get_edge(e)?;
                        for v in [e.start_vertex, e.end_vertex] {
                            let p = topology.get_vertex(v)?.point;
                            let r = p.sub(&axis.project(&p)).norm();
                            beyond |= r.definitely_greater(self.round.radius_at(&p));
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
    pub fn start(&self, reversed: bool) -> (Scalar, Vector3) {
        let at_from = self.open == [true, false];
        let direction = self.round.axis.direction;
        if at_from != reversed {
            (self.from, direction)
        } else {
            (self.to, direction.neg())
        }
    }

    /// The wall of the solid `solid` on `round` that reaches `at` along
    /// its axis — a hole's wall at the face it was drilled into — if any.
    pub fn at(
        model: &NamedModel,
        solid: SolidId,
        round: &RoundWall,
        at: &Vector3,
    ) -> GeopResult<Option<Self>> {
        let topology = model.topology();
        for face in topology.body_faces(Body::Solid(solid))? {
            let on = topology.get_face(face)?.surface.as_round_wall()?;
            if !on.is_some_and(|w| round.could_be_equal(&w)) {
                continue;
            }
            let wall = Wall::of(model, face)?;
            let along = wall.round.along(at);
            if !along.definitely_less(wall.from) && !along.definitely_greater(wall.to) {
                return Ok(Some(wall));
            }
        }
        Ok(None)
    }

    /// The point of its axis `along` its direction.
    pub fn axis_point(&self, along: Scalar) -> Vector3 {
        let axis = &self.round.axis;
        axis.point.add(&axis.direction.prod_scalar(along))
    }

    /// Where a thread on it runs: from the point `along` its axis, the way
    /// `direction` goes — along the axis or against it — `length` far or,
    /// if none, as far as the wall does. Refused if it would run past the
    /// wall's end.
    pub fn placement(
        &self,
        (along, direction): (Scalar, Vector3),
        length: Option<f64>,
    ) -> GeopResult<ThreadPlacement> {
        let ahead = direction.prod_dot(&self.round.axis.direction);
        let runs = if ahead.definitely_greater(Scalar::ZERO) {
            self.to.sub(along)
        } else {
            along.sub(self.from)
        };
        let length = length.unwrap_or_else(|| runs.to_f64());
        if Scalar::from_f64(length).definitely_greater(runs) {
            return Err(GeopError::new(format!(
                "the thread runs {length} far, but its wall only {}",
                runs.to_f64()
            )));
        }
        let start = self.axis_point(along);
        Ok(ThreadPlacement {
            start,
            direction,
            radius: self.round.radius_at(&start),
            taper: self.round.taper.mul(ahead),
            length,
            internal: self.internal,
        })
    }

    /// The name of its first face: what a thread on it records it was put
    /// on.
    pub fn face_name(&self, model: &NamedModel) -> String {
        model.name_of(self.faces[0]).unwrap_or_default().to_string()
    }
}

/// Where a thread runs: from `start`, on the axis of a wall of `radius`
/// there, along the unit vector `direction`, `length` far, the wall's
/// radius changing by `taper` per unit along it; in a hole if `internal`.
#[derive(Clone, Debug)]
pub struct ThreadPlacement {
    pub start: Vector3,
    pub direction: Vector3,
    pub radius: Scalar,
    pub taper: Scalar,
    pub length: f64,
    pub internal: bool,
}

impl ThreadPlacement {
    /// Checks that a thread of `size` fits a wall of its radius and taper
    /// — the face `face`: tapered as the thread is, from its start; there,
    /// a shaft between the minor and the major diameter, a hole from the
    /// minor diameter up to short of the major one.
    pub fn check_fits(&self, size: &ThreadSize, face: &str) -> GeopResult<()> {
        let taper = size.taper_along(self.internal);
        if !self.taper.could_be_equal(Scalar::from_f64(taper)) {
            let wall = match self.taper.to_f64() {
                t if t.abs() < 1e-12 => "is straight".to_string(),
                t => format!("tapers 1:{:.1} on the diameter", 1.0 / (2.0 * t.abs())),
            };
            let thread = match (taper, self.internal) {
                (0.0, _) => "runs on a straight wall".to_string(),
                (_, true) => format!(
                    "tapers 1:{} on the diameter, narrowing into the hole from its start",
                    1.0 / (2.0 * size.taper())
                ),
                (_, false) => format!(
                    "tapers 1:{} on the diameter, widening along the shaft from its end",
                    1.0 / (2.0 * size.taper())
                ),
            };
            return Err(GeopError::new(format!(
                "face {face:?} {wall}, but a {} thread {thread}",
                size.designation()
            )));
        }
        let d = self.radius.add(self.radius);
        let (major, minor) = size.diameters(self.internal);
        let (lo, hi) = (Scalar::from_f64(minor), Scalar::from_f64(major));
        let fits = if self.internal {
            !d.definitely_less(lo) && d.definitely_less(hi)
        } else {
            d.definitely_greater(lo) && !d.definitely_greater(hi)
        };
        if !fits {
            return Err(GeopError::new(format!(
                "face {face:?} is Ø{:.3}: an {} thread {} needs {} Ø{minor:.3} to Ø{major:.3}",
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
    /// hole's bottom, which a thread stops short of — and how far before
    /// its start that is.
    ///
    /// Swept from exactly the end of the wall, its end face would cut the
    /// face there through the middle of its profile, right where the wall's
    /// rim crosses it — three faces through one point that the booleans did
    /// not splice (the trace along the end face and the shank's end ran into
    /// two corners at the rim). A pitch off, the profile, less than a pitch
    /// wide, is clear of that face altogether.
    pub fn swept(&self, wall: &Wall, pitch: f64) -> GeopResult<(Vector3, f64, f64)> {
        let ends = [(wall.from, wall.open[0]), (wall.to, wall.open[1])];
        // How far out past the wall's end at `at`, if it is one.
        let past = |at: Scalar| {
            ends.iter()
                .find(|(end, _)| at.could_be_equal(*end))
                .map_or(0.0, |&(_, open)| if open { pitch } else { -pitch })
        };
        let start = wall.round.along(&self.start);
        let end = start.add(
            (wall.round.axis.direction)
                .prod_dot(&self.direction)
                .mul(Scalar::from_f64(self.length)),
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
            .sub(&self.direction.prod_scalar(Scalar::from_f64(before)));
        Ok((start, length, before))
    }

    /// The thread of `size`, as recorded on `face`: tapered as the standard
    /// says, which [`ThreadPlacement::check_fits`] finds the wall to be.
    pub fn cosmetic(&self, size: &ThreadSize, face: String) -> GeopResult<CosmeticThread> {
        let (major_diameter, minor_diameter) = size.diameters(self.internal);
        Ok(CosmeticThread {
            designation: size.designation(),
            face,
            axis: Axis::try_new(self.start, self.direction)?,
            radius: self.radius,
            taper: Scalar::from_f64(size.taper_along(self.internal)),
            major_diameter,
            minor_diameter,
            pitch: size.pitch(),
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
fn clear_start(
    model: &NamedModel,
    wall: &Wall,
    frame: &CoordinateSystem,
    turns: f64,
) -> GeopResult<f64> {
    let topology = model.topology();
    let mut corners = Vec::new();
    for &f in &wall.faces {
        for edge in topology.face_edges(f) {
            let e = topology.get_edge(edge)?;
            for v in [e.start_vertex, e.end_vertex] {
                let q = frame.to_uvw(&topology.get_vertex(v)?.point);
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
/// along its helix, moving out by the placement's taper as it rises, from
/// where it starts as far as it runs, starting at an angle clear of the
/// wall's corners ([`clear_start`]): a solid named `solid`, its faces
/// `N(c0,q)` .. `N(c3,q)` for each quarter turn `q` and its ends `N(start)`
/// and `N(end)` by `namer`.
///
/// It runs a sixteenth of `H` past the wall, or, where the truncation on
/// that side is less than `H / 8` (a pipe thread's), half of that: at the
/// sharp V's base a tooth is a pitch wide, and the turns would touch.
pub fn thread_tool(
    model: &mut NamedModel,
    namer: &Namer,
    solid: &str,
    size: &ThreadSize,
    placement: &ThreadPlacement,
    wall: &Wall,
) -> GeopResult<SolidId> {
    let pitch = size.pitch();
    let h = TRIANGLE_HEIGHT * pitch;
    let tan30 = 1.0 / 3f64.sqrt();
    // Where it starts and ends, moved off the wall's ends (see
    // `ThreadPlacement::swept`), and its radii there: the standard's taper,
    // which `ThreadPlacement::check_fits` found the wall's to be.
    let (start, length, before) = placement.swept(wall, pitch)?;
    let taper = size.taper_along(placement.internal);
    let moved = -taper * before;
    let radius = placement.radius.to_f64() + moved;
    let (major, minor) = size.diameters(placement.internal);
    let (major, minor) = (major / 2.0 + moved, minor / 2.0 + moved);
    let (crest, root) = size.half_flats();
    // `(r, half width)` at its inner and outer ends.
    let (inner, outer) = if placement.internal {
        let half = |r: f64| crest + (major - r) * tan30;
        let r_in = radius - (h / 16.0).min(size.root() / 2.0);
        ((r_in, half(r_in)), (major, half(major)))
    } else {
        let half = |r: f64| root + (r - minor) * tan30;
        let r_out = radius + (h / 16.0).min(size.crest() / 2.0);
        ((minor, half(minor)), (r_out, half(r_out)))
    };
    // On a tapered wall the section's radii are measured where it starts,
    // and a corner `z` along the axis from there sits where the wall has
    // moved by the taper times `z`: else a corner half a pitch along lies
    // that much nearer the wall, and a pipe thread's corner, run past the
    // wall by less than that, came to within `2e-3` of it.
    let p = |r: f64, z: f64| {
        let r = if taper == 0.0 { r } else { r + taper * z };
        Vector2::from_array([Scalar::from_f64(r), Scalar::from_f64(z)])
    };
    // Counter-clockwise in `(r, z)`.
    let corners = [
        p(inner.0, -inner.1),
        p(outer.0, -outer.1),
        p(outer.0, outer.1),
        p(inner.0, inner.1),
    ];
    let profile = Profile::closed(polygon(&corners)?);
    let turns = length / pitch;
    let frame = frame_along(start, &placement.direction)?;
    let (c, s) = cos_sin(clear_start(model, wall, &frame, turns)?);
    let u = frame
        .u()
        .prod_scalar(Scalar::from_f64(c))
        .add(&frame.v().prod_scalar(Scalar::from_f64(s)));
    let w = *frame.w();
    let axes = CoordinateSystem::try_from_xz(start, u, w)?;
    let helix = Helix::new(Scalar::from_f64(pitch), turns, Handedness::Right)
        .tapered(Scalar::from_f64(taper));
    screw(
        model,
        namer,
        solid,
        &axes,
        &helix,
        &[SweepLoop::plain(profile)],
    )
}
