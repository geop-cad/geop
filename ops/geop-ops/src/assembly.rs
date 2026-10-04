//! Mates: what holds the parts placed in a part together — two entities,
//! each of a placed part or of the part itself, and what they must be to
//! each other (see [`MateKind`]) — and solving them, by moving the placed
//! parts ([`Part::solve_mates`]).
//!
//! The solving is `geop_core_solve::mates`'s. What is added here is how an
//! entity, named like any other ([`EntityRef`]), becomes a point, a line or
//! a plane attached to a rigid body: an entity of a placed part — named
//! behind the instance's name — moves with that instance; an entity of the
//! part itself is ground, and never moves.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_solve::mates::{Assembly, Body, Constraint, Feature, Geometry, Pull as BodyPull};
use serde::{Deserialize, Serialize};

/// What a mate asks of its two entities, as a program holds it.
pub type MateKind = geop_core_solve::mates::Kind<Design>;

use crate::{
    Design, Instance, Part,
    operation::{Aspects, EntityRef, INSTANCE_SEPARATOR},
    part::{ParamValue, State},
};

/// A mate: what two entities must be to each other. Serializes flat:
/// `{"type": "distance", "value": 1.0, "entities": [...]}`.
///
/// One being edited may hold fewer than two entities yet; only a complete
/// one ([`Mate::pair`]) holds anything.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mate {
    #[serde(flatten)]
    pub kind: MateKind,
    #[serde(default)]
    pub entities: Vec<EntityRef>,
}

impl Mate {
    /// Its two entities, if it has exactly two.
    pub fn pair(&self) -> Option<[&EntityRef; 2]> {
        match self.entities.as_slice() {
            [a, b] => Some([a, b]),
            _ => None,
        }
    }

    /// What an entity has to be for this kind of mate, in words.
    pub fn needs(&self) -> &'static str {
        match self.kind {
            MateKind::Coincident | MateKind::Distance { .. } => "a point, a line or a plane",
            MateKind::Concentric => "something round, or a line",
            MateKind::Parallel | MateKind::Perpendicular | MateKind::Angle { .. } => {
                "a line or a plane"
            }
        }
    }

    /// The geometry `aspects` contributes to this kind of mate: for one that
    /// touches or keeps a distance, a point before a line before a plane —
    /// the center of a circle is a point; for a concentric one, an axis; for
    /// one that turns, a direction.
    fn geometry<S: Scalar>(&self, aspects: &Aspects<S>) -> Option<Geometry<S>> {
        let point = || {
            aspects
                .point
                .as_ref()
                .or(aspects.arc.as_ref().map(|a| &a.circle.center))
                .map(|p| Geometry::Point { at: *p })
        };
        let line = |axes: [&Option<geop_core_geometry::shape::Axis<S>>; 2]| {
            axes.into_iter().flatten().next().map(|a| Geometry::Line {
                point: a.point,
                direction: a.direction,
            })
        };
        let plane = || {
            aspects.plane.as_ref().map(|p| Geometry::Plane {
                point: *p.origin(),
                normal: *p.w(),
            })
        };
        match self.kind {
            MateKind::Coincident | MateKind::Distance { .. } => point()
                .or_else(|| line([&aspects.line, &aspects.round]))
                .or_else(plane),
            MateKind::Concentric => line([&aspects.round, &aspects.line]),
            MateKind::Parallel | MateKind::Perpendicular | MateKind::Angle { .. } => {
                line([&aspects.line, &aspects.round]).or_else(plane)
            }
        }
    }
}

/// A drag of a placed part: its point `local` — in its own frame — pulled
/// towards the point `target`, before the mates are solved alone (see
/// `geop_core_solve::mates`). The part is named by the parameter its pose
/// is.
#[derive(Clone, Debug, PartialEq)]
pub struct Drag<S: Scalar> {
    pub parameter: String,
    pub local: Vector3<S>,
    pub target: Vector3<S>,
}

/// How a solve went.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MateReport {
    pub converged: bool,
    /// The mates that do not hold, by name.
    pub failed: Vec<String>,
    /// How many steps the solve took, and each of its minimizations — none
    /// for a check: why it stopped, after how many steps, and the largest
    /// residual it left.
    pub iterations: usize,
    pub phases: Vec<(geop_core_math::least_squares::Stop, usize, f64)>,
}

/// The box around every vertex of `part` and of the parts placed in it, as
/// placed — what a body turns about and how large a solve is are free
/// choices made from it, so its corners are sharp.
pub(crate) fn bounds<S: Scalar>(part: &Part<S>) -> Option<[Vector3<S>; 2]> {
    let own = part.topology().vertices.values().map(|v| v.point.sharpen());
    let placed = part.instances().flat_map(|(_, instance)| {
        let corners = instance.component.bounds().map(|[lo, hi]| {
            (0..8).map(move |i| {
                let corner = Vector3::from_array(
                    [0, 1, 2].map(|k| if i >> k & 1 == 0 { lo[k] } else { hi[k] }),
                );
                instance.pose.apply(&corner).sharpen()
            })
        });
        corners.into_iter().flatten().collect::<Vec<_>>()
    });
    own.chain(placed).fold(None, |acc, p| {
        let [lo, hi] = acc.unwrap_or([p, p]);
        Some([
            Vector3::from_array([0, 1, 2].map(|k| lo[k].min(p[k]))),
            Vector3::from_array([0, 1, 2].map(|k| hi[k].max(p[k]))),
        ])
    })
}

/// A rigid body of a solve: a part placed in the part — or, inside one
/// placed flexibly, a part placed in that, however deep.
struct Placed<'p, S: Scalar> {
    /// Its name in the part: `hinge/pin`.
    name: String,
    /// The parameter of the part its pose is — relative to `parent`'s
    /// frame: `hinge/pin.pose`.
    parameter: String,
    instance: &'p Instance<S>,
    /// The body it is placed in, if it is placed in a flexible one.
    parent: Option<usize>,
    /// Where it is in the part.
    world: Pose<S>,
}

/// The rigid bodies of a part and the constraints between them, the
/// bodies as placed, and the names of the mates the constraints are.
type Solvable<'p, S> = (Assembly<S>, Vec<Placed<'p, S>>, Vec<String>);

/// The bodies of a solve in `part`: its instances, and inside every
/// flexible one, the instances of the part it places — named, and their
/// parameters named, behind `prefix`, and placed in the body `parent` at
/// `frame`.
fn placed<'p, S: Scalar>(
    part: &'p Part<S>,
    prefix: &str,
    parent: Option<usize>,
    frame: &Pose<S>,
    out: &mut Vec<Placed<'p, S>>,
) {
    for (id, instance) in part.instances() {
        let name = format!("{prefix}{}", part.name_of(id).unwrap_or_default());
        let world = frame.compose(&instance.pose);
        out.push(Placed {
            name: name.clone(),
            parameter: format!("{prefix}{}", instance.parameter),
            instance,
            parent,
            world,
        });
        if instance.flexible {
            let body = Some(out.len() - 1);
            placed(
                instance.part(),
                &format!("{name}{INSTANCE_SEPARATOR}"),
                body,
                &world,
                out,
            );
        }
    }
}

impl<S: Scalar> Part<S> {
    /// The mates as constraints between rigid bodies — its instances, and
    /// the instances of every part placed flexibly in it, however deep (see
    /// [`placed`]), each free unless fixed or, with `only`, other than the
    /// one whose pose is that parameter — those bodies, and the names of
    /// the mates the constraints are, in order. The mates of a part placed
    /// flexibly hold its parts as they held them in it, named behind its
    /// name. Mates still missing an entity hold nothing, and are left out,
    /// and so are those `named` does not pick.
    fn assembly(
        &self,
        only: Option<&str>,
        named: &dyn Fn(&str) -> bool,
    ) -> GeopResult<Solvable<'_, S>> {
        let mut bodies_of = Vec::new();
        placed(self, "", None, &Pose::identity(), &mut bodies_of);
        let mut scale = S::ONE;
        let mut bodies = Vec::new();
        for body in &bodies_of {
            let center = match body.instance.component.bounds() {
                Some([lo, hi]) => {
                    let diagonal = hi.sub(&lo).norm().sharpen();
                    if diagonal.definitely_greater(scale) {
                        scale = diagonal;
                    }
                    lo.add(&hi).prod_scalar(S::ONE.div(S::TWO)?).sharpen()
                }
                None => Vector3::zero(),
            };
            bodies.push(Body {
                pose: body.world,
                free: !body.instance.fixed && only.is_none_or(|o| o == body.parameter),
                center,
            });
        }
        // The mates of the part, and of every part placed flexibly in it,
        // as the part names them.
        let mut mates: Vec<(String, Mate)> = self
            .mates()
            .map(|(name, mate)| (name.to_string(), mate.clone()))
            .collect();
        for body in bodies_of.iter().filter(|b| b.instance.flexible) {
            for (name, mate) in body.instance.part().mates() {
                let entities = mate
                    .entities
                    .iter()
                    .map(|e| e.in_instance(&body.name))
                    .collect();
                mates.push((
                    format!("{}{INSTANCE_SEPARATOR}{name}", body.name),
                    Mate {
                        kind: mate.kind,
                        entities,
                    },
                ));
            }
        }
        let body_named: std::collections::HashMap<&str, usize> = bodies_of
            .iter()
            .enumerate()
            .map(|(i, b)| (b.name.as_str(), i))
            .collect();
        let mut constraints = Vec::new();
        let mut names = Vec::new();
        for (name, mate) in &mates {
            let Some([a, b]) = mate.pair().filter(|_| named(name)) else {
                continue;
            };
            let ctx = with_context!("mate {name:?}");
            // The body an entity moves with: the innermost one it is of.
            let feature = |entity: &EntityRef| -> GeopResult<Feature<S>> {
                let mut body: Option<usize> = None;
                let mut rest = entity.clone();
                while let Some((instance, inner)) = rest.split_instance() {
                    let name = match body {
                        Some(b) => format!("{}{INSTANCE_SEPARATOR}{instance}", bodies_of[b].name),
                        None => instance,
                    };
                    let Some(&found) = body_named.get(name.as_str()) else {
                        break;
                    };
                    body = Some(found);
                    rest = inner;
                }
                let aspects = match body {
                    Some(b) => Aspects::of(&rest, bodies_of[b].instance.part())?,
                    None => Aspects::of(entity, self)?,
                };
                let geometry = mate
                    .geometry(&aspects)
                    .ok_or_else(|| GeopError::new(format!("{entity} is not {}", mate.needs())))?;
                Ok(Feature { body, geometry })
            };
            let constraint = Constraint {
                kind: mate.kind.cast(),
                a: feature(a).with_context(ctx)?,
                b: feature(b).with_context(ctx)?,
            };
            constraint.validate().with_context(ctx)?;
            constraints.push(constraint);
            names.push(name.clone());
        }
        let assembly = Assembly {
            bodies,
            constraints,
            scale,
        };
        Ok((assembly, bodies_of, names))
    }

    /// Where the instances — and the parts placed in those placed flexibly
    /// — would have to be for every mate to hold, as little away from where
    /// they are as the mates allow, after `drags` pulled them — only the one
    /// whose pose is the parameter `only`, if given, and never a fixed one:
    /// the new values of the parameters their poses are, each relative to
    /// the part it is placed in, and which mates hold there. Fails for a
    /// mate that cannot hold between its entities at all.
    pub fn solve_mates(
        &self,
        only: Option<&str>,
        drags: &[Drag<S>],
    ) -> GeopResult<(State, MateReport)> {
        let (mut assembly, bodies_of, names) = self.assembly(only, &|_| true)?;
        let pulls = drags
            .iter()
            .map(|drag| {
                let body = bodies_of
                    .iter()
                    .position(|b| b.parameter == drag.parameter)
                    .ok_or_else(|| {
                        GeopError::new(format!("no placed part's pose is {:?}", drag.parameter))
                    })?;
                Ok(BodyPull::Point {
                    body,
                    local: drag.local,
                    target: drag.target,
                })
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let report = assembly.solve(&pulls)?;
        let mut moved = State::new();
        for (placed, body) in bodies_of.iter().zip(&assembly.bodies) {
            if body.free {
                let pose = match placed.parent {
                    Some(parent) => assembly.bodies[parent].pose.inverse().compose(&body.pose),
                    None => body.pose,
                };
                moved.insert(placed.parameter.clone(), ParamValue::Pose(pose.cast()));
            }
        }
        Ok((
            moved,
            MateReport {
                converged: report.converged,
                failed: report.failed.iter().map(|&i| names[i].clone()).collect(),
                iterations: report.iterations,
                phases: report
                    .phases
                    .iter()
                    .map(|p| (p.stop, p.iterations, p.max_residual.to_f64()))
                    .collect(),
            },
        ))
    }

    /// Which of the mates `named` picks — by name — hold where the
    /// instances are now: `|_| true` for all of them. Checking only some
    /// costs only theirs.
    pub fn check_mates(&self, named: impl Fn(&str) -> bool) -> GeopResult<MateReport> {
        let (assembly, _, names) = self.assembly(None, &named)?;
        let report = assembly.report()?;
        Ok(MateReport {
            converged: report.converged,
            failed: report.failed.iter().map(|&i| names[i].clone()).collect(),
            iterations: 0,
            phases: Vec::new(),
        })
    }
}
