//! What a drawing draws: the faces of a part and of every part placed in
//! it, however deep, each where it is placed ([`Scene`]) — and seeing them
//! all from one direction at once ([`Scene::project`]).
//!
//! An assembly places the same part many times — a hundred screws of one
//! size — so its hidden lines are found in two stages, and the costly one
//! is done once per part however often it is placed:
//!
//! 1. **Each part on its own.** A part's lines, and which of them its own
//!    faces hide, are those [`project_view`] finds looking along the view's
//!    direction turned into the part's own frame. Placed parts of one
//!    component turned alike look at it from the same direction: they share
//!    that work, and draw the same lines moved to where each is.
//! 2. **The parts together.** Along a line one part shows, what another
//!    part hides of it changes only where the line passes behind that
//!    part's outline on the paper, which the lines that part shows hold.
//!    So each line a part shows is cut where it crosses or runs along a
//!    line another part shows, and each piece is decided by a ray from its
//!    middle towards the eye, against the faces of every other part that
//!    could be in front of it. A line its own part hides stays hidden.
//!
//! A line is not cut where it enters another part, only where it passes
//! behind its outline: parts that pierce each other are a known limit. The
//! parts of an assembly touch, they do not overlap.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Motion, Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{FaceId, Model};
use geop_ops::{Part, operation::INSTANCE_SEPARATOR};

use crate::{
    drawing::drawn_faces,
    hidden_lines::{
        Occluder, ProjectedView, ViewLine, ViewOptions, bounds, boxes_meet, cut_points,
        drop_drawn, meetings, project_view, ray_length, seen,
    },
    view::{ViewAxis, ViewFrame},
};

/// The faces of one part, where it is placed.
pub struct Body<'p, S: Scalar> {
    /// Where it is placed, as the entities of a placed part are named: `""`
    /// for the drawn part's own faces, `screw` for the part placed as
    /// `screw`, `arm/screw` for one placed in that.
    pub path: String,
    pub part: &'p Part<S>,
    pub faces: Vec<FaceId>,
    /// Where its part is placed; `None` for the drawn part's own faces.
    pub pose: Option<Pose<S>>,
}

impl<'p, S: Scalar> Body<'p, S> {
    pub fn model(&self) -> &'p Model<S> {
        self.part.topology()
    }

    /// How it is turned, if it is turned at all.
    fn turn(&self) -> Option<Quaternion<S>> {
        self.pose
            .as_ref()
            .map(|p| p.rotation())
            .filter(|r| !r.is_identity())
    }
}

/// A body as one view sees it: its box on the paper, `[x_min, y_min,
/// x_max, y_max]`, and how near the eye it comes.
#[derive(Clone, Copy)]
pub(crate) struct Reach {
    paper: [f64; 4],
    near: f64,
}

/// The bodies a drawing draws, and what seeing them needs.
pub struct Scene<'p, S: Scalar> {
    pub bodies: Vec<Body<'p, S>>,
    /// Per body, moving its part's geometry to where it is placed, and
    /// back; `None` for the drawn part's own.
    there: Vec<Option<Motion<S>>>,
    back: Vec<Option<Motion<S>>>,
    /// The faces that can hide something, once per model; body `k`'s are
    /// `occluders[occluders_of[k]]`, in its part's own frame.
    occluders: Vec<Vec<Occluder<S>>>,
    occluders_of: Vec<usize>,
    /// The corners of each body's box, where it is placed.
    corners: Vec<[Vector3<S>; 8]>,
    /// The ray length that clears every body from anywhere on them.
    length: S,
}

impl<'p, S: Scalar> Scene<'p, S> {
    /// The faces of `part` and of every part placed in it, however deep,
    /// each where it is placed: the part's own first, then those placed,
    /// in the order they were placed, each followed by those placed in it.
    pub fn of(part: &'p Part<S>) -> GeopResult<Self> {
        let mut bodies = Vec::new();
        collect(part, "", None, &mut bodies)?;
        Self::new(bodies)
    }

    /// The scene of `bodies`.
    pub fn new(bodies: Vec<Body<'p, S>>) -> GeopResult<Self> {
        let mut occluders: Vec<Vec<Occluder<S>>> = Vec::new();
        let mut models: Vec<&Model<S>> = Vec::new();
        let mut occluders_of = Vec::new();
        let mut corners = Vec::new();
        let (mut there, mut back) = (Vec::new(), Vec::new());
        for body in &bodies {
            let model = body.model();
            let index = match models.iter().position(|m| std::ptr::eq(*m, model)) {
                Some(index) => index,
                None => {
                    occluders.push(
                        body.faces
                            .iter()
                            .map(|&f| Occluder::of(model, f))
                            .collect::<GeopResult<_>>()?,
                    );
                    models.push(model);
                    models.len() - 1
                }
            };
            occluders_of.push(index);
            let motion = body.pose.map(|p| p.motion());
            let mut b = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
            for o in &occluders[index] {
                for k in 0..3 {
                    b[k][0] = b[k][0].min(o.bounds[k][0]);
                    b[k][1] = b[k][1].max(o.bounds[k][1]);
                }
            }
            corners.push(std::array::from_fn(|i| {
                let c = Vector3::from_array(std::array::from_fn(|k| {
                    S::from_f64(b[k][(i >> k) & 1])
                }));
                match &motion {
                    Some(m) => m.apply(&c),
                    None => c,
                }
            }));
            there.push(motion);
            back.push(body.pose.map(|p| p.inverse().motion()));
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for c in corners.iter().flatten() {
            for k in 0..3 {
                lo[k] = lo[k].min(c[k].lower().to_f64());
                hi[k] = hi[k].max(c[k].upper().to_f64());
            }
        }
        let diagonal = (0..3)
            .map(|k| (hi[k] - lo[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        let length = match bodies.len() {
            // One body: as a view of its model alone has it.
            1 => ray_length(&occluders[0]),
            _ => S::from_f64(2.0 * diagonal + 1.0),
        };
        Ok(Scene {
            bodies,
            there,
            back,
            occluders,
            occluders_of,
            corners,
            length,
        })
    }

    /// Which side of the plane through `origin` normal to `normal` body `k`
    /// lies on: wholly on the side `normal` points to (`Some(true)`),
    /// wholly on the other (`Some(false)`), or on both, or touching it
    /// (`None`).
    pub fn side_of(&self, k: usize, origin: &Vector3<S>, normal: &Vector3<S>) -> Option<bool> {
        let heights: Vec<S> = self.corners[k]
            .iter()
            .map(|c| c.sub(origin).prod_dot(normal))
            .collect();
        if heights.iter().all(|h| h.definitely_greater(S::ZERO)) {
            Some(true)
        } else if heights.iter().all(|h| h.definitely_less(S::ZERO)) {
            Some(false)
        } else {
            None
        }
    }

    /// The bodies that share the work of the first stage (see the module
    /// docs): the same model, turned alike. Each group in the order of its
    /// first body.
    pub fn groups(&self) -> Vec<Vec<usize>> {
        let mut groups: Vec<Vec<usize>> = Vec::new();
        for (k, body) in self.bodies.iter().enumerate() {
            let alike = |g: &&mut Vec<usize>| {
                let first = &self.bodies[g[0]];
                std::ptr::eq(first.model(), body.model()) && same_turn(first.turn(), body.turn())
            };
            match groups.iter_mut().find(alike) {
                Some(group) => group.push(k),
                None => groups.push(vec![k]),
            }
        }
        groups
    }

    /// `frame` as the part of body `k` sees it, in its own frame.
    fn local_frame(&self, k: usize, frame: &ViewFrame<S>) -> ViewFrame<S> {
        match (&self.back[k], self.bodies[k].turn()) {
            (Some(back), Some(_)) => {
                let turn = |axis: &ViewAxis<S>| ViewAxis::General(back.rotate(&axis.vector()));
                ViewFrame {
                    direction: turn(&frame.direction),
                    right: turn(&frame.right),
                    up: turn(&frame.up),
                }
            }
            _ => *frame,
        }
    }

    /// The line `line` of body `k`'s part, in its own frame, where the body
    /// is, seen through `frame`.
    fn placed_line(
        &self,
        k: usize,
        line: &ViewLine<S>,
        frame: &ViewFrame<S>,
    ) -> GeopResult<ViewLine<S>> {
        let (curve3, curve) = match &self.there[k] {
            Some(motion) => {
                let curve3 = line.curve3.transform(motion);
                let curve = frame.project_curve(&curve3)?;
                (curve3, curve)
            }
            None => (line.curve3.clone(), line.curve.clone()),
        };
        Ok(ViewLine {
            curve,
            curve3,
            kind: line.kind,
            visible: line.visible,
            edge: line.edge,
            body: k,
            source: line.source,
            range: line.range,
        })
    }

    /// Every body seen through `frame`, each line visible or hidden: see
    /// the module docs.
    pub fn project(
        &self,
        frame: &ViewFrame<S>,
        options: &ViewOptions,
    ) -> GeopResult<ProjectedView<S>> {
        let mut lines = Vec::new();
        for group in self.groups() {
            let first = &self.bodies[group[0]];
            let view = project_view(
                first.model(),
                &first.faces,
                &self.local_frame(group[0], frame),
                options,
            )
            .map_err(|e| e.with_context(format!("the part placed as {:?}", first.path)))?;
            for &k in &group {
                for line in &view.lines {
                    lines.push(self.placed_line(k, line, frame)?);
                }
            }
        }
        if self.bodies.len() > 1 {
            lines = self.together(lines, frame, options)?;
        }
        lines.sort_by_key(|l| l.body);
        Ok(ProjectedView {
            frame: *frame,
            lines,
        })
    }

    /// How `frame` sees each body.
    fn reach(&self, frame: &ViewFrame<S>) -> Vec<Reach> {
        let toward_eye = frame.toward_eye();
        self.corners
            .iter()
            .map(|corners| {
                let mut paper = [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ];
                let mut near = f64::NEG_INFINITY;
                for c in corners {
                    let p = frame.project_point(c);
                    for k in 0..2 {
                        paper[k] = paper[k].min(p[k].lower().to_f64());
                        paper[k + 2] = paper[k + 2].max(p[k].upper().to_f64());
                    }
                    near = near.max(toward_eye.prod_dot(c).upper().to_f64());
                }
                Reach { paper, near }
            })
            .collect()
    }

    /// Whether any of the bodies `bodies` hides `point` from the eye
    /// through `frame` (see [`seen`]), each asked in its own part's frame.
    /// A body wholly behind the point, or beside it on the paper, cannot.
    fn hidden_by(
        &self,
        bodies: impl IntoIterator<Item = usize>,
        reach: &[Reach],
        frame: &ViewFrame<S>,
        point: Vector3<S>,
    ) -> GeopResult<bool> {
        let toward_eye = frame.toward_eye();
        let depth = toward_eye.prod_dot(&point).lower().to_f64();
        let on_paper = frame.project_point(&point);
        let (x, y) = (on_paper[0], on_paper[1]);
        for b in bodies {
            let r = &reach[b];
            if r.near < depth
                || x.upper().to_f64() < r.paper[0]
                || r.paper[2] < x.lower().to_f64()
                || y.upper().to_f64() < r.paper[1]
                || r.paper[3] < y.lower().to_f64()
            {
                continue;
            }
            let (p, d) = match &self.back[b] {
                Some(back) => (back.apply(&point), back.rotate(&toward_eye)),
                None => (point, toward_eye),
            };
            let model = self.bodies[b].model();
            let occluders = &self.occluders[self.occluders_of[b]];
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "whether the part placed as {:?} hides {point:?}",
                    self.bodies[b].path
                ))
            };
            if !seen(model, occluders, p, &d, self.length).with_context(&ctx)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Whether `point` is seen through `frame`, every body in front of it
    /// counted: what decides a line drawn that is no edge or silhouette of
    /// a body, such as a cosmetic thread.
    pub fn point_seen(&self, frame: &ViewFrame<S>, point: Vector3<S>) -> GeopResult<bool> {
        let reach = self.reach(frame);
        Ok(!self.hidden_by(0..self.bodies.len(), &reach, frame, point)?)
    }

    /// The second stage (see the module docs): `lines`, each body's as its
    /// own faces hide them, with what the other bodies hide of them.
    fn together(
        &self,
        lines: Vec<ViewLine<S>>,
        frame: &ViewFrame<S>,
        options: &ViewOptions,
    ) -> GeopResult<Vec<ViewLine<S>>> {
        let n = self.bodies.len();
        let reach = self.reach(frame);
        let boxes = lines
            .iter()
            .map(|l| bounds(&l.curve))
            .collect::<GeopResult<Vec<_>>>()?;
        let mut shown: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (i, line) in lines.iter().enumerate() {
            if line.visible {
                shown[line.body].push(i);
            }
        }
        // Where each line shown has to be cut: where it meets a line
        // another body shows.
        let mut cuts: Vec<Vec<S>> = vec![Vec::new(); lines.len()];
        for a in 0..n {
            for b in a + 1..n {
                if !boxes_meet(&reach[a].paper, &reach[b].paper) {
                    continue;
                }
                for &i in &shown[a] {
                    if !boxes_meet(&boxes[i], &reach[b].paper) {
                        continue;
                    }
                    for &j in &shown[b] {
                        if !boxes_meet(&boxes[i], &boxes[j]) {
                            continue;
                        }
                        let (li, lj) = (&lines[i], &lines[j]);
                        let (on_i, on_j) = meetings((&li.curve, &li.curve3), (&lj.curve, &lj.curve3))
                            .map_err(|e| {
                                e.with_context(format!(
                                    "where the {:?} of edge {:?} of the part placed as {:?} and \
                                     the {:?} of edge {:?} of the part placed as {:?} cross on \
                                     the paper",
                                    li.kind,
                                    li.edge,
                                    self.bodies[a].path,
                                    lj.kind,
                                    lj.edge,
                                    self.bodies[b].path
                                ))
                            })?;
                        cuts[i].extend(on_i);
                        cuts[j].extend(on_j);
                    }
                }
            }
        }

        let mut out = Vec::new();
        for (i, line) in lines.into_iter().enumerate() {
            let others: Vec<usize> = (0..n)
                .filter(|&b| b != line.body && boxes_meet(&boxes[i], &reach[b].paper))
                .collect();
            if !line.visible || others.is_empty() {
                out.push(line);
                continue;
            }
            let domain = line.curve.domain();
            let mut ends = vec![domain.0];
            ends.extend(cut_points(std::mem::take(&mut cuts[i]), domain));
            ends.push(domain.1);
            // Each piece decided by its middle — or, should a ray from
            // there graze a face too closely for its search to settle, by
            // a point a quarter along either way.
            let mut pieces: Vec<(S, S, GeopResult<bool>)> = Vec::new();
            for pair in ends.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                let mut outcome = Err(GeopError::new("no point tried"));
                for f in [0.5, 0.25, 0.75] {
                    let decide = || -> GeopResult<bool> {
                        let t = a.add(b.sub(a).mul(S::from_f64(f))).sharpen();
                        let point = line.curve3.evaluate(t)?;
                        Ok(!self.hidden_by(others.iter().copied(), &reach, frame, point)?)
                    };
                    outcome = decide().map_err(|e| {
                        e.with_context(format!(
                            "the piece [{a:?}, {b:?}] of the {:?} of edge {:?} of the part \
                             placed as {:?}",
                            line.kind, line.edge, self.bodies[line.body].path
                        ))
                    });
                    if outcome.is_ok() {
                        break;
                    }
                }
                pieces.push((a, b, outcome));
            }
            // A piece none of whose points could be decided takes the
            // visibility of its nearest neighbour along the line.
            let known: Vec<Option<bool>> = pieces
                .iter()
                .map(|p| p.2.as_ref().ok().copied())
                .collect();
            let mut decided: Vec<(S, S, bool)> = Vec::new();
            for (k, (a, b, outcome)) in pieces.into_iter().enumerate() {
                let visible = match outcome {
                    Ok(v) => v,
                    Err(e) => (1..known.len())
                        .find_map(|step| {
                            let before = k.checked_sub(step).and_then(|i| known[i]);
                            before.or_else(|| known.get(k + step).copied().flatten())
                        })
                        .ok_or(e)?,
                };
                match decided.last_mut() {
                    Some(last) if last.2 == visible => last.1 = b,
                    _ => decided.push((a, b, visible)),
                }
            }
            if let [(_, _, visible)] = decided.as_slice() {
                out.push(ViewLine {
                    visible: *visible,
                    ..line
                });
                continue;
            }
            for (a, b, visible) in decided {
                out.push(ViewLine {
                    curve: line.curve.sub_curve(a, b)?,
                    curve3: line.curve3.sub_curve(a, b)?,
                    visible,
                    range: (a, b),
                    ..line.clone()
                });
            }
        }
        if !options.hidden_lines {
            out.retain(|l| l.visible);
        }
        // Each body's own lines were cleared of those lying on another in
        // its own view.
        drop_drawn(out, |a, b| a.body != b.body)
    }
}

/// Whether two turns are the same: no turn at all, or rotations enclosed
/// alike, bound for bound. Sharing a view between rotations that are merely
/// close would draw one with the other's lines.
fn same_turn<S: Scalar>(a: Option<Quaternion<S>>, b: Option<Quaternion<S>>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => a.components().iter().zip(b.components()).all(|(x, y)| {
            x.lower().to_f64() == y.lower().to_f64() && x.upper().to_f64() == y.upper().to_f64()
        }),
        _ => false,
    }
}

/// Adds the bodies of `part`, placed at `pose`, as `path`, and of every
/// part placed in it, to `bodies`.
fn collect<'p, S: Scalar>(
    part: &'p Part<S>,
    path: &str,
    pose: Option<Pose<S>>,
    bodies: &mut Vec<Body<'p, S>>,
) -> GeopResult<()> {
    let faces = drawn_faces(part.topology());
    if !faces.is_empty() {
        bodies.push(Body {
            path: path.to_string(),
            part,
            faces,
            pose,
        });
    }
    for (id, instance) in part.instances() {
        let name = part
            .name_of(id)
            .ok_or_else(|| GeopError::new(format!("placed part {id} has no name")))?;
        let inner = match &pose {
            Some(outer) => outer.compose(&instance.pose),
            None => instance.pose,
        };
        let path = match path {
            "" => name.to_string(),
            _ => format!("{path}{INSTANCE_SEPARATOR}{name}"),
        };
        collect(instance.part(), &path, Some(inner), bodies)?;
    }
    Ok(())
}
