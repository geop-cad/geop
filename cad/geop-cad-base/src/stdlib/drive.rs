//! The drive train: involute spur gears and their rack, GT2 timing
//! pulleys, shaft collars and flange couplings. Each turns around the `z`
//! axis — the rack runs along it — its datum `axis`.
//!
//! A gear's teeth, a rack's and a pulley's are what is left of a blank
//! when one gap, patterned round it or along it, is cut away: a boolean
//! per tooth, the price of a tooth count that is a parameter like any
//! other. Every gap reaches past the blank on all sides it leaves by, so
//! that no face of it lies on one of the blank's.
//!
//! # Meshing
//!
//! Gears of one module mesh with their pitch circles touching: centres
//! `m · (z1 + z2) / 2` apart. Each gear's datum `axis` is a joint's
//! connector; a revolute joint on each, and a [`CouplingKind::Gear`] of
//! the two joints with `ratio = z2 / z1` — the first joint's gear turns
//! that many times per turn of the second's — and `reverse`, since external
//! gears turn opposite ways, keeps them in mesh. A rack slides by the arc
//! its pinion's pitch circle rolls off: a [`CouplingKind::RackPinion`] of
//! `radius = m · z / 2`, the pinion's pitch radius, which its sketch
//! `pitch` draws. Two GT2 pulleys on one belt turn the same way, at the
//! ratio of their tooth counts: a gear coupling without `reverse`.
//!
//! A gear has a tooth centred on `+x`. Two gears whose centres lie along
//! `x` mesh when the second has a gap facing the first: turned half a
//! tooth, `180° / z2`, if `z2` is even, and not at all if it is odd.
//!
//! [`CouplingKind::Gear`]: geop_ops::assembly::CouplingKind::Gear
//! [`CouplingKind::RackPinion`]: geop_ops::assembly::CouplingKind::RackPinion

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
};
use geop_ops::{
    EntityRef, ORIGIN,
    parameters::{Material, Parameter, ParameterKind, Parameters, Row},
};
use geop_ops_booleans::{BooleanArgs, Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::{Extent, Extents};
use geop_ops_pattern::{CircularPatternArgs, Direction, LinearPatternArgs, Spacing};

use super::{
    StandardPart,
    drawing::Drawing,
    involute::{self, DEGREE, FEWEST_TEETH},
    steps::{
        Around, axis_datum, col, extrude, offset_datum, outline_plane, plane_datum, profile_plane,
        revolve, size, z_axis,
    },
    tables,
};
use crate::Program;

/// The table parameter a gear's tooth count is chosen by.
pub const TEETH: &str = "teeth";

/// The most teeth a gear of the library has.
pub const MOST_TEETH: usize = 120;

/// The columns of the [`TEETH`] table: `z`, then the lower side of the gap
/// between two teeth, of module 1 (see [`involute::Gap`]): its foot `fx`,
/// `fy`, and the control points of its flank `c0x`, `c0y`, ...
const GAP_COLUMNS: [&str; 3 + 2 * (DEGREE + 1)] = [
    "z", "fx", "fy", "c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y", "c4x", "c4y", "c5x",
    "c5y", "c6x", "c6y", "c7x", "c7y", "c8x", "c8y", "c9x", "c9y",
];

/// Every tooth count from [`FEWEST_TEETH`] to [`MOST_TEETH`], as the
/// table parameter [`TEETH`]: row `z20` the gap of a gear of 20 teeth.
fn teeth() -> GeopResult<Parameter> {
    let rows = (FEWEST_TEETH..=MOST_TEETH)
        .map(|z| {
            let gap = involute::gap(z)?;
            let mut values = vec![z as f64];
            values.extend(gap.foot);
            values.extend(gap.flank.iter().flatten());
            Ok(Row {
                name: format!("z{z}"),
                values,
            })
        })
        .collect::<GeopResult<_>>()?;
    Ok(Parameter {
        name: TEETH.into(),
        kind: ParameterKind::Table {
            columns: GAP_COLUMNS.iter().map(|c| c.to_string()).collect(),
            rows,
            selected: "z20".into(),
        },
    })
}

/// A number parameter, `expression` by default — a formula of those
/// before it, or a value.
fn number(name: &str, expression: &str, min: f64, max: f64) -> Parameter {
    Parameter {
        name: name.into(),
        kind: ParameterKind::Number {
            expression: expression.into(),
            min: Some(min),
            max: Some(max),
        },
    }
}

fn steel() -> Option<Material> {
    Some(Material {
        name: "Steel".into(),
        density: 7850.0,
    })
}

/// An involute spur gear, 20° pressure angle, of module `size.m` and
/// [`TEETH`]`.z` teeth, `width` wide up `z` from its side on the `xy`
/// plane, bored `bore` through: a disc of the tip diameter, the gap of
/// [`involute::gap`] patterned round it and cut away. Its datums: `axis`,
/// `side`, and the sketch `pitch`, its pitch circle on the side.
pub fn spur_gear() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#9aa0a8".into()),
        values: vec![
            size(tables::gear_modules()),
            teeth()?,
            number("width", &format!("8 * {}", col("m")), 1.0, 100.0),
            number(
                "bore",
                &format!("max(2, round({} * {TEETH}.z / 4))", col("m")),
                1.0,
                100.0,
            ),
        ],
    };
    let m = col("m");
    let z = format!("{TEETH}.z");
    let mut blank = Drawing::new(&program.parameters)?;
    let centre = blank.origin();
    blank.circle(centre, &format!("{m} * ({z} + 2)"))?;
    blank.circle(centre, "bore")?;
    extrude(
        &mut program,
        "blank",
        "blank_outline",
        (blank, outline_plane()),
        Extents::blind("width"),
        Combine::NewBody,
    )?;

    // One gap, round from its foot on the root circle: up its lower side,
    // over the arc past the tip circle, down its upper side — the lower
    // one's mirror image — and back along the root circle. Extruded both
    // ways from the gear's middle, past both its sides.
    let mut gap = Drawing::new(&program.parameters)?;
    let at = |column: &str, sign: &str| format!("{sign}{m} * {TEETH}.{column}");
    let lower =
        |d: &mut Drawing, k: &str| d.point(at(&format!("{k}x"), ""), at(&format!("{k}y"), ""));
    let upper =
        |d: &mut Drawing, k: &str| d.point(at(&format!("{k}x"), ""), at(&format!("{k}y"), "-"));
    let foot = lower(&mut gap, "f")?;
    let flank = (0..=DEGREE)
        .map(|k| lower(&mut gap, &format!("c{k}")))
        .collect::<GeopResult<Vec<_>>>()?;
    let upper_flank = (0..=DEGREE)
        .rev()
        .map(|k| upper(&mut gap, &format!("c{k}")))
        .collect::<GeopResult<Vec<_>>>()?;
    let upper_foot = upper(&mut gap, "f")?;
    gap.line(foot, flank[0]);
    gap.bezier(flank.clone());
    let past = format!("{m} * ({z} / 2 + 1.5)");
    gap.arc(flank[DEGREE], upper_flank[0], &past, true)?;
    gap.bezier(upper_flank.clone());
    gap.line(upper_flank[DEGREE], upper_foot);
    gap.arc(upper_foot, foot, &format!("{m} * ({z} / 2 - 1.25)"), false)?;
    plane_datum(&mut program, "middle", "width / 2");
    extrude(
        &mut program,
        "gap",
        "gap_outline",
        (gap, EntityRef::datum("middle")),
        Extents {
            symmetric: true,
            ..Extents::blind(format!("width + 2 * {m}"))
        },
        Combine::NewBody,
    )?;
    program.push(
        "teeth",
        CircularPatternArgs {
            bodies: vec![EntityRef::Solid {
                name: "extrude(gap)".into(),
            }],
            axis: Some(z_axis()),
            reversed: false,
            count: z.as_str().into(),
            angle: Spacing::extent(360.0),
            combine: Combine::Difference {
                target: "extrude(blank)".into(),
            },
        },
    );

    let mut pitch = Drawing::new(&program.parameters)?;
    let centre = pitch.origin();
    pitch.circle(centre, &format!("{m} * {z}"))?;
    program.push("pitch", pitch.on(outline_plane())?);
    axis_datum(&mut program);
    plane_datum(&mut program, "side", "0");
    Ok(StandardPart {
        file: "std:spur_gear.geop",
        title: "Involute spur gear, 20° pressure angle",
        designation: "Spur gear",
        base: "side",
        program,
        threaded: Vec::new(),
    })
}

/// A gear rack of module `size.m`, `size.b` wide, its pitch line `size.h`
/// above its back, running `length` along `z` from the `xy` plane: its
/// teeth face `+y`, its pitch plane is the `xz` plane, and it is centred
/// on it across `x`. A bar up to the tip line, with a straight-sided gap
/// — 20° to the teeth's centre lines, `π m / 2` wide on the pitch line —
/// patterned every `π m` along it from `z = 0` and cut away, every gap
/// reaching past both sides. Its datums: `axis`, the `z` axis on the pitch
/// plane — a slider joint's connector — `pitch`, and `back`.
pub fn rack() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#9aa0a8".into()),
        values: vec![size(tables::racks()), number("length", "100", 10.0, 1000.0)],
    };
    let (m, b, h) = (col("m"), col("b"), col("h"));
    // In the profile plane: `x` up the teeth, along `y`; `y` along `z`.
    let mut bar = Drawing::new(&program.parameters)?;
    bar.polygon(&[
        [format!("-{h}"), "0".into()],
        [m.clone(), "0".into()],
        [m.clone(), "length".into()],
        [format!("-{h}"), "length".into()],
    ])?;
    let across = |length: String| Extents {
        symmetric: true,
        ..Extents::blind(length)
    };
    extrude(
        &mut program,
        "bar",
        "bar_outline",
        (bar, profile_plane()),
        across(b.clone()),
        Combine::NewBody,
    )?;
    let half = |height: &str| format!("pi * {m} / 4 + {height} * tan(20)");
    let (root, past) = (format!("-1.25 * {m}"), format!("1.5 * {m}"));
    let mut gap = Drawing::new(&program.parameters)?;
    gap.polygon(&[
        [root.clone(), format!("-({})", half(&root))],
        [past.clone(), format!("-({})", half(&past))],
        [past.clone(), half(&past)],
        [root.clone(), half(&root)],
    ])?;
    extrude(
        &mut program,
        "gap",
        "gap_outline",
        (gap, profile_plane()),
        across(format!("{b} + 2 * {m}")),
        Combine::NewBody,
    )?;
    program.push(
        "teeth",
        LinearPatternArgs {
            bodies: vec![EntityRef::Solid {
                name: "extrude(gap)".into(),
            }],
            first: Direction {
                along: Some(z_axis()),
                reversed: false,
                count: format!("floor(length / (pi * {m})) + 1").into(),
                spacing: Spacing::step(format!("pi * {m}")),
            },
            second: None,
            combine: Combine::Difference {
                target: "extrude(bar)".into(),
            },
        },
    );
    axis_datum(&mut program);
    let xz = || EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y));
    offset_datum(&mut program, "pitch", xz(), "0");
    offset_datum(&mut program, "back", xz(), &format!("-{h}"));
    plane_datum(&mut program, "end", "0");
    Ok(StandardPart {
        file: "std:gear_rack.geop",
        title: "Gear rack, 20° pressure angle",
        designation: "Gear rack",
        base: "back",
        program,
        threaded: Vec::new(),
    })
}

fn aluminium() -> Option<Material> {
    Some(Material {
        name: "Aluminium 6061".into(),
        density: 2700.0,
    })
}

/// A GT2 timing pulley for a 6 mm belt, of `size.z` teeth: a hub `dh`
/// across and `lh` long from its end on the `xy` plane, then the teeth `w`
/// wide between two flanges `df` across and `t` thick, bored `d` through.
///
/// The teeth are simplified: a ring of the outside diameter — the pitch
/// diameter `2z / π` less twice the belt's pitch line offset, 0.254 —
/// with a round groove 0.555 in radius and 0.76 deep for each tooth of
/// the belt, cut by a pin patterned round it; the ring is then joined to
/// the hub and flanges, turned in one profile. Its datums: `axis`, `end`,
/// and the plane `belt` through the middle of the belt, on which the
/// sketch `pitch` draws its pitch circle.
pub fn gt2_pulley() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: aluminium(),
        color: Some("#c8ccd2".into()),
        values: vec![size(tables::gt2_pulleys())],
    };
    let (z, d, df, dh, lh, t, w) = (
        col("z"),
        col("d"),
        col("df"),
        col("dh"),
        col("lh"),
        col("t"),
        col("w"),
    );
    let outside = format!("({z} * 2 / pi - 0.508) / 2");
    let core = format!("{outside} - 1");
    let (bore, hub, flange) = (format!("{d} / 2"), format!("{dh} / 2"), format!("{df} / 2"));
    let (inner_face, outer_face, end) = (
        format!("{lh} + {t}"),
        format!("{lh} + {t} + {w}"),
        format!("{lh} + 2 * {t} + {w}"),
    );
    let zero = || "0".to_string();
    let mut profile = Drawing::new(&program.parameters)?;
    profile.polygon(&[
        [bore.clone(), zero()],
        [hub.clone(), zero()],
        [hub, lh.clone()],
        [flange.clone(), lh.clone()],
        [flange.clone(), inner_face.clone()],
        [core.clone(), inner_face],
        [core, outer_face.clone()],
        [flange.clone(), outer_face],
        [flange, end.clone()],
        [bore, end],
    ])?;
    revolve(
        &mut program,
        "body",
        "profile",
        profile,
        Around::ZAxis,
        Combine::NewBody,
    )?;

    // The ring runs from the middle of one flange to the middle of the
    // other, so that none of its faces lies on one of theirs.
    plane_datum(&mut program, "ring_floor", &format!("{lh} + {t} / 2"));
    let mut ring = Drawing::new(&program.parameters)?;
    let centre = ring.origin();
    ring.circle(centre, &format!("2 * ({outside})"))?;
    ring.circle(centre, &format!("2 * ({outside}) - 3"))?;
    extrude(
        &mut program,
        "ring",
        "ring_outline",
        (ring, EntityRef::datum("ring_floor")),
        Extents::blind(format!("{w} + {t}")),
        Combine::NewBody,
    )?;
    // A groove reaches past both ends of the ring.
    plane_datum(&mut program, "belt", &format!("{lh} + {t} + {w} / 2"));
    let mut pin = Drawing::new(&program.parameters)?;
    let centre = pin.point(format!("{outside} - 0.205"), "0")?;
    pin.circle(centre, "1.11")?;
    extrude(
        &mut program,
        "groove",
        "groove_outline",
        (pin, EntityRef::datum("belt")),
        Extents {
            symmetric: true,
            ..Extents::blind(format!("{w} + 2 * {t}"))
        },
        Combine::NewBody,
    )?;
    program.push(
        "grooves",
        CircularPatternArgs {
            bodies: vec![EntityRef::Solid {
                name: "extrude(groove)".into(),
            }],
            axis: Some(z_axis()),
            reversed: false,
            count: z.as_str().into(),
            angle: Spacing::extent(360.0),
            combine: Combine::Difference {
                target: "extrude(ring)".into(),
            },
        },
    );
    program.push(
        "pulley",
        BooleanArgs {
            a: "revolve(body)".into(),
            b: "circular_pattern(grooves)".into(),
            op: BooleanOp::Union,
        },
    );

    axis_datum(&mut program);
    plane_datum(&mut program, "end", "0");
    let mut pitch = Drawing::new(&program.parameters)?;
    let centre = pitch.origin();
    pitch.circle(centre, &format!("{z} * 2 / pi"))?;
    program.push("pitch", pitch.on(EntityRef::datum("belt"))?);
    Ok(StandardPart {
        file: "std:gt2_pulley.geop",
        title: "GT2 timing pulley, 6 mm belt",
        designation: "GT2 pulley",
        base: "end",
        program,
        threaded: Vec::new(),
    })
}

/// A shaft collar after DIN 705 A: a ring bored `d`, `D` across and `b`
/// wide, standing on the `xy` plane, with a set screw hole `ds` across
/// through its wall along `+x`, half way up — where its thread goes.
pub fn shaft_collar() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#8e949c".into()),
        values: vec![size(tables::shaft_collars())],
    };
    let (d, big, b) = (col("d"), col("D"), col("b"));
    let mut profile = Drawing::new(&program.parameters)?;
    profile.polygon(&[
        [format!("{d} / 2"), "0".into()],
        [format!("{big} / 2"), "0".into()],
        [format!("{big} / 2"), b.clone()],
        [format!("{d} / 2"), b.clone()],
    ])?;
    revolve(
        &mut program,
        "ring",
        "profile",
        profile,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    // Drawn in the profile plane, through the axis, and extruded out along
    // `+x` from inside the bore.
    let mut hole = Drawing::new(&program.parameters)?;
    let centre = hole.point("0", format!("{b} / 2"))?;
    let circle = hole.circle(centre, &col("ds"))?;
    extrude(
        &mut program,
        "set_screw",
        "set_screw_hole",
        (hole, profile_plane()),
        Extents {
            side1: Extent::ThroughAll,
            symmetric: false,
            side2: None,
            reversed: false,
        },
        Combine::Difference {
            target: "revolve(ring)".into(),
        },
    )?;
    axis_datum(&mut program);
    plane_datum(&mut program, "base", "0");
    let threaded = ["", "#1", "#2", "#3"]
        .map(|piece| format!("extrude(set_screw,set_screw_hole,{circle}{piece})"))
        .to_vec();
    Ok(StandardPart {
        file: "std:shaft_collar.geop",
        title: "Shaft collar with set screw, DIN 705 A",
        designation: "DIN 705 A",
        base: "base",
        program,
        threaded,
    })
}

/// A rigid flange coupling as an envelope, joining a shaft `d1` across to
/// one `d2` across: two hubs `D` across, `L` long in all from its `d1` end
/// on the `xy` plane, and between them, in the middle, the bolted flanges
/// `F` across and `f` thick together. Its datums: `axis`, `end` — the `d1`
/// end — and `far_end`.
pub fn flange_coupling() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: aluminium(),
        color: Some("#c8ccd2".into()),
        values: vec![size(tables::flange_couplings())],
    };
    let (d1, d2, big, flange, l, f) =
        (col("d1"), col("d2"), col("D"), col("F"), col("L"), col("f"));
    let (hub, rim) = (format!("{big} / 2"), format!("{flange} / 2"));
    let (low, mid, high) = (
        format!("({l} - {f}) / 2"),
        format!("{l} / 2"),
        format!("({l} + {f}) / 2"),
    );
    let mut profile = Drawing::new(&program.parameters)?;
    profile.polygon(&[
        [format!("{d1} / 2"), "0".into()],
        [hub.clone(), "0".into()],
        [hub.clone(), low.clone()],
        [rim.clone(), low],
        [rim, high.clone()],
        [hub.clone(), high],
        [hub, l.clone()],
        [format!("{d2} / 2"), l.clone()],
        [format!("{d2} / 2"), mid.clone()],
        [format!("{d1} / 2"), mid],
    ])?;
    revolve(
        &mut program,
        "coupling",
        "profile",
        profile,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    plane_datum(&mut program, "end", "0");
    plane_datum(&mut program, "far_end", &l);
    Ok(StandardPart {
        file: "std:flange_coupling.geop",
        title: "Rigid flange shaft coupling",
        designation: "Flange coupling",
        base: "end",
        program,
        threaded: Vec::new(),
    })
}
