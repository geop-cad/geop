//! [`DeformBody`]: bodies tapered, twisted or bent about an axis.

use geop_core_editor::{
    edit_form::{EditForm, FormState, choices, chosen},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::deform::AxialMap;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_core_topology::{Body, build::BodySpec};
use geop_lib_booleans::{RemeshParams, imprint_on_face};
use geop_ops_model::{
    BodyNames, ModelOperation, NamedModel, Namer, build_model, read_model, resolve_bodies,
};
use serde::{Deserialize, Serialize};

use crate::common::{BodyRef, bodies_field, body};

/// How a [`DeformBody`] deforms: each about the axis, over the zone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Deform {
    /// Scaled across the axis, by one at the zone's start and by `amount`
    /// at its end, linearly in between: exact.
    #[default]
    Taper,
    /// Turned about the axis, by nothing at the zone's start and by
    /// `amount` degrees at its end, uniformly in between.
    Twist,
    /// The zone wound onto an arc of `amount` degrees, curving towards
    /// `toward`, its length kept along the axis.
    Bend,
}

const DEFORMS: [(Deform, &str, &str); 3] = [
    (Deform::Taper, "taper", "Taper"),
    (Deform::Twist, "twist", "Twist"),
    (Deform::Bend, "bend", "Bend"),
];

/// Tapers, twists or bends the bodies `bodies` — solids, and sheets by one
/// of their faces — about the axis through `origin` along `axis`, over the
/// zone from `start` to `end` along it (measured from `origin`): in place,
/// every entity keeping its name. Before the zone a body stays where it
/// is; past it, it is carried on as the zone's end leaves it — turned
/// rigidly by a twist or a bend, scaled across the axis by a taper.
///
/// Where a body crosses an end of the zone, its faces are first divided
/// there (as a curve projected onto them divides them), so that each piece
/// lies on one side. Every curve and surface is then composed with the
/// deformation on its control points (see
/// [`geop_core_geometry::deform`]): exactly for a taper, to within a proven
/// bound that widens the result for a twist or a bend. Each surface keeps
/// its parametrization, so every pcurve stays as it is.
///
/// Refused: a zone that does not run forward, a taper to a factor that is
/// not positive, a bend of a whole turn or more or one that would wind
/// material past its centre, and a zone end through a corner of a body
/// that crosses it (move the zone).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeformBody;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeformBodyArgs {
    /// The bodies to deform: solids, or a face of each sheet.
    pub bodies: Vec<BodyRef>,
    /// Taper, twist or bend.
    #[serde(default)]
    pub deform: Deform,
    /// A point of the axis, from which the zone is measured.
    #[serde(default)]
    pub origin: [f64; 3],
    /// The axis's direction.
    pub axis: [f64; 3],
    /// Where a bend curves towards: a direction across the axis.
    #[serde(default)]
    pub toward: [f64; 3],
    /// Where the zone starts and ends along the axis.
    pub start: f64,
    pub end: f64,
    /// The taper's factor at the zone's end, or the twist's or the bend's
    /// angle in degrees.
    pub amount: f64,
}

impl DeformBodyArgs {
    /// The map these arguments ask for.
    fn map(&self) -> GeopResult<AxialMap> {
        let numbers = (self.origin.iter())
            .chain(&self.axis)
            .chain(&self.toward)
            .chain([&self.start, &self.end, &self.amount]);
        if let Some(bad) = numbers.clone().find(|v| !v.is_finite()) {
            return Err(GeopError::new(format!("{bad} is not a number")));
        }
        let point = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
        let (origin, axis) = (point(self.origin), point(self.axis));
        let zone = (Scalar::from_f64(self.start), Scalar::from_f64(self.end));
        let amount = Scalar::from_f64(self.amount);
        match self.deform {
            Deform::Taper => AxialMap::taper(origin, &axis, zone, amount),
            Deform::Twist => AxialMap::twist(origin, &axis, zone, amount.degrees_to_radians()?),
            Deform::Bend => AxialMap::bend(
                origin,
                &axis,
                &point(self.toward),
                zone,
                amount.degrees_to_radians()?,
            ),
        }
    }
}

part_operation!(DeformBody(DeformBodyArgs));

impl PartOperation for DeformBody {
    const KIND: &'static str = "deform_body";
    const INFO: OperationInfo = OperationInfo::new(
        "Deform body",
        "Taper, twist or bend bodies about an axis, over a zone along it.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::DEFORM_BODY);
    type Args = DeformBodyArgs;
    type Session = ();

    /// The newest solid, tapered along `z` over its whole height to half
    /// its width.
    fn new_args(context: &Context) -> DeformBodyArgs {
        let model = read_model(&context.before).ok();
        let solid = model.as_ref().and_then(|m| m.newest_solid());
        let (start, end) = (model.as_ref().zip(solid.as_ref()))
            .and_then(|(m, s)| height(m, s))
            .unwrap_or((0.0, 1.0));
        DeformBodyArgs {
            bodies: solid.map(body).into_iter().collect(),
            deform: Deform::Taper,
            origin: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            toward: [1.0, 0.0, 0.0],
            start,
            end,
            amount: 0.5,
        }
    }

    fn run(ctx: &mut StepContext, args: &DeformBodyArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The bodies, picked; the deformation, the axis, the zone and the
    /// amount — the toward direction for a bend.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &DeformBodyArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, DeformBodyArgs> {
        let mut f = EditForm::<DeformBodyArgs>::default();
        bodies_field(&mut f, &context.before, &args.bodies, |edit, picked| {
            edit.args.bodies = picked
        });
        let (value, options) = choices(&DEFORMS, &args.deform);
        f.select("deform", "deform", value, options, false, |edit, value| {
            if let Some(deform) = chosen(&DEFORMS, value) {
                edit.args.deform = deform;
            }
        });
        let amount = match args.deform {
            Deform::Taper => Number::new("factor", args.amount, Unit::Fraction),
            Deform::Twist | Deform::Bend => Number::new("angle", args.amount, Unit::Angle),
        };
        f.number("amount", amount, |edit, v| edit.args.amount = v);
        for (key, label, value) in [
            ("start", "zone start", args.start),
            ("end", "zone end", args.end),
        ] {
            let number = Number::new(label, value, Unit::Length);
            f.number(key, number, move |edit, v| match key {
                "start" => edit.args.start = v,
                _ => edit.args.end = v,
            });
        }
        // A field per coordinate of a vector of the arguments, keyed
        // `keys`, setting what `field` reaches.
        type Field = fn(&mut DeformBodyArgs) -> &mut [f64; 3];
        let mut vector =
            |keys: [&'static str; 3], label: &str, value: [f64; 3], unit, field: Field| {
                for (k, (key, axis)) in keys.into_iter().zip(["x", "y", "z"]).enumerate() {
                    let number = Number::new(format!("{label} {axis}"), value[k], unit);
                    f.number(key, number, move |edit, v| field(edit.args)[k] = v);
                }
            };
        let origin = ["origin_x", "origin_y", "origin_z"];
        vector(origin, "origin", args.origin, Unit::Length, |a| {
            &mut a.origin
        });
        let axis = ["axis_x", "axis_y", "axis_z"];
        vector(axis, "axis", args.axis, Unit::Fraction, |a| &mut a.axis);
        if args.deform == Deform::Bend {
            let toward = ["toward_x", "toward_y", "toward_z"];
            vector(toward, "toward", args.toward, Unit::Fraction, |a| {
                &mut a.toward
            });
        }
        f
    }
}

/// The lowest and the highest `z` of the control points of the solid
/// `solid`'s faces: the zone a new deformation along `z` spans.
fn height(model: &NamedModel, solid: &str) -> Option<(f64, f64)> {
    let id = model.solid_id(solid).ok()?;
    let faces = model.topology().body_faces(id).ok()?;
    let z = (faces.iter())
        .map(|f| model.topology().faces[f].surface.aabb()[2])
        .reduce(Scalar::union)?;
    (z.lo.is_finite() && z.hi.is_finite() && z.hi > z.lo).then_some((z.lo, z.hi))
}

impl ModelOperation for DeformBody {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &DeformBodyArgs,
    ) -> GeopResult<()> {
        let map = args.map()?;
        let namer = Namer::new(DeformBody::KIND, step)?;
        for body in resolve_bodies(reader, &args.bodies)? {
            let (start, end) = map.zone();
            for (side, t) in [("start", start), ("end", end)] {
                divide_across(model, &namer.scoped(side), body, &map, t)?;
            }
            model.transform_body(body, &map)?;
        }
        Ok(())
    }
}

/// Divides the faces of `body` that cross the plane across `map`'s axis at
/// `t`, each along where the plane crosses it (see [`imprint_on_face`]), so
/// that every face lies on one side. Nothing where the body does not reach
/// definitely across the plane. Refused where a corner of a body that does
/// could lie on the plane: the cut would land on it.
fn divide_across(
    model: &mut NamedModel,
    namer: &Namer,
    body: Body,
    map: &AxialMap,
    t: Scalar,
) -> GeopResult<()> {
    let topology = model.topology();
    let mut vertices: Vec<_> = topology.iter_body_vertices(body)?.collect();
    vertices.sort_by_key(|v| v.0);
    let along = |v| map.along(&topology.vertices[&v].point);
    let before = vertices.iter().any(|&v| along(v).definitely_less(t));
    let after = vertices.iter().any(|&v| along(v).definitely_greater(t));
    if !(before && after) {
        return Ok(());
    }
    if let Some(&on) = vertices.iter().find(|&&v| along(v).could_be_equal(t)) {
        return Err(GeopError::new(format!(
            "the zone's end at {t:?} along the axis passes through the corner {} of the body; move the zone off it",
            model.name_of(on).unwrap_or("?")
        )));
    }
    // Every face with corners on both sides, divided by a sheet of its own
    // reaching past the whole body.
    let mut faces = topology.body_faces(body)?;
    faces.sort_by_key(|f| f.0);
    let crossing: Vec<_> = (faces.into_iter())
        .filter(|&f| {
            let corners: Vec<_> = (topology.face_edges(f))
                .flat_map(|e| {
                    [
                        topology.edges[&e].start_vertex,
                        topology.edges[&e].end_vertex,
                    ]
                })
                .map(along)
                .collect();
            corners.iter().any(|c| c.definitely_less(t))
                && corners.iter().any(|c| c.definitely_greater(t))
        })
        .collect();
    let reach = (topology.body_faces(body)?.iter())
        .map(|f| topology.faces[f].surface.size())
        .try_fold(Scalar::ONE, |sum, s| Ok::<_, GeopError>(sum.add(s?)))?;
    let reach = reach.add(
        (vertices.iter())
            .map(|&v| topology.vertices[&v].point.norm())
            .fold(Scalar::ZERO, Scalar::max),
    );
    for face in crossing {
        let name = model.name_of(face).unwrap_or("?").to_string();
        let sheet_namer = namer.scoped(&name);
        let spec = BodySpec::sheet_on(map.plane_across(t, reach.upper())?)?;
        let names = BodyNames {
            vertices: (0..4)
                .map(|k| sheet_namer.name(&[&format!("v{k}")]))
                .collect(),
            edges: (0..4)
                .map(|k| sheet_namer.name(&[&format!("e{k}")]))
                .collect(),
            faces: vec![sheet_namer.name(&["plane"])],
            solid: None,
        };
        let built = model.build_body(spec, names)?;
        let sheet = *built
            .shells
            .first()
            .ok_or_else(|| GeopError::new("a sheet came out as no shell"))?;
        imprint_on_face(model, &sheet_namer, face, &[sheet], RemeshParams::default())
            .map_err(|e| e.with_context(format!("dividing face {name} at the zone's end {t:?}")))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_lib_sweep::shapes::{block_part, cylinder};
    use geop_ops_model::testing::{apply_to_model, assert_valid};

    fn solid_volume(part: &NamedModel) -> f64 {
        let solid = part.solid_id(&part.solid_names()[0]).unwrap();
        (part.topology().mass_properties(solid).unwrap().volume).to_f64()
    }

    fn deformed(part: NamedModel, args: DeformBodyArgs) -> GeopResult<NamedModel> {
        apply_to_model::<DeformBody>(part, "d", &args)
    }

    fn args(part: &NamedModel, deform: Deform, zone: (f64, f64), amount: f64) -> DeformBodyArgs {
        DeformBodyArgs {
            bodies: vec![body(part.solid_names()[0].clone())],
            deform,
            origin: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            toward: [1.0, 0.0, 0.0],
            start: zone.0,
            end: zone.1,
            amount,
        }
    }

    /// A box tapered along its height to half its width: a frustum of a
    /// pyramid, exactly — its volume `h (A + sqrt(A a) + a) / 3`.
    #[test]
    fn a_box_tapered_to_half_is_a_frustum() {
        let part = block_part("b", [-1.0, -1.0, 0.0], [1.0, 1.0, 3.0]);
        let part = deformed(part.clone(), args(&part, Deform::Taper, (0.0, 3.0), 0.5)).unwrap();
        assert_valid(&part);
        let volume = 3.0 * (4.0 + 2.0 + 1.0) / 3.0;
        assert!(
            (solid_volume(&part) - volume).abs() < 1e-9,
            "{}",
            solid_volume(&part)
        );
    }

    /// A box twisted a quarter turn about its own axis keeps its volume:
    /// each slice across the axis only turns.
    #[test]
    fn a_twisted_box_keeps_its_volume() {
        let part = block_part("b", [-1.0, -0.5, 0.0], [1.0, 0.5, 4.0]);
        let part = deformed(part.clone(), args(&part, Deform::Twist, (0.0, 4.0), 90.0)).unwrap();
        assert_valid(&part);
        assert!(
            (solid_volume(&part) - 8.0).abs() < 1e-6,
            "{}",
            solid_volume(&part)
        );
    }

    /// A bar bent a quarter turn over the middle of its length: the faces
    /// crossing the zone's ends are divided there, the bar before the zone
    /// stays where it was and the part past it is carried round the circle
    /// of radius `4 / π` about `x = 4 / π`, `z = 2`: its far corner `(0.5,
    /// 0.5, 6)` lands at `(4 / π + 2, 0.5, 1.5 + 4 / π)`.
    #[test]
    fn a_bar_bent_over_part_of_its_length() {
        let part = block_part("b", [-0.5, -0.5, 0.0], [0.5, 0.5, 6.0]);
        let a = args(&part, Deform::Bend, (2.0, 4.0), 90.0);
        let part = deformed(part, a).unwrap();
        assert_valid(&part);
        let radius = 4.0 / std::f64::consts::PI;
        let corner = [radius + 2.0, 0.5, 1.5 + radius];
        let found = (part.topology().vertices.values())
            .any(|v| (0..3).all(|k| (v.point[k].to_f64() - corner[k]).abs() < 1e-9));
        assert!(found, "no corner at {corner:?}");
        // The four sides divided at both ends of the zone.
        assert_eq!(part.topology().faces.len(), 14);
    }

    /// A cylinder beside the axis, twisted round it and then tapered: its
    /// rational patches composed, still a valid solid.
    #[test]
    fn a_cylinder_beside_the_axis_twisted_and_tapered() {
        let mut part = NamedModel::new();
        cylinder(&mut part, "c", [1.5, 0.0, 0.0], 0.5, 3.0);
        let part = deformed(part.clone(), args(&part, Deform::Twist, (0.0, 3.0), 60.0)).unwrap();
        assert_valid(&part);
        let part = deformed(part.clone(), args(&part, Deform::Taper, (0.0, 3.0), 0.6)).unwrap();
        assert_valid(&part);
    }

    /// A cylinder twisted about its own axis is the same cylinder.
    #[test]
    fn a_cylinder_twisted_about_its_axis() {
        let mut part = NamedModel::new();
        cylinder(&mut part, "c", [0.0; 3], 1.0, 2.0);
        let part = deformed(part.clone(), args(&part, Deform::Twist, (0.0, 2.0), 45.0)).unwrap();
        assert_valid(&part);
        let volume = 2.0 * std::f64::consts::PI;
        assert!(
            (solid_volume(&part) - volume).abs() < 1e-6,
            "{}",
            solid_volume(&part)
        );
    }

    /// A taper to nothing is refused.
    #[test]
    fn refusals_say_why() {
        let part = block_part("b", [0.0; 3], [1.0, 1.0, 2.0]);
        let error =
            deformed(part.clone(), args(&part, Deform::Taper, (0.0, 2.0), 0.0)).unwrap_err();
        assert!(format!("{error}").contains("positive"), "{error}");
    }
}
