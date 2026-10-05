//! The drive train: involute spur gears and their rack, GT2 timing
//! pulleys, shaft collars and flange couplings. Each turns around the `z`
//! axis — the rack runs along it — its datum `axis`.
//!
//! A gear's teeth are what is left of a disc when one gap, patterned round
//! it, is cut away: a boolean per tooth, the price of a tooth count that is
//! a parameter like any other — building a gear takes seconds per tooth.
//! Every gap reaches past the disc on all sides it leaves by, so that no
//! face of it lies on one of the disc's. A rack and a pulley, whose teeth
//! would be cut along one long edge or past two flanges, are each drawn in
//! one profile instead: a rack of at most [`RACK_TEETH`] teeth cut to
//! length, and a pulley family per tooth count.
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
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extent, Extents};
use geop_ops_pattern::{CircularPatternArgs, Spacing};

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

/// The most teeth a gear of the library has: with 120, each arc of the rim
/// is cut by ten gaps, and grows too wide on the way (see [`RIM_ARCS`]).
pub const MOST_TEETH: usize = 80;

/// How many arcs a gear's rim is drawn in (see [`spur_gear`]).
const RIM_ARCS: usize = 12;

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
    // The disc's rim in twelve arcs, one every 30°: every gap cut splits
    // the arc it crosses, and an edge split again and again grows wider
    // with each split — a quarter of a circle crossed by the gaps of more
    // than about 35 teeth came out too wide to cut the next one. Their
    // ends, on multiples of 30°, are tooth centres or at least 0.015 of a
    // module from where any gap of up to 120 teeth crosses the rim.
    let mut blank = Drawing::new(&program.parameters)?;
    let tip = format!("{m} * ({z} / 2 + 1)");
    let rim = (0..RIM_ARCS)
        .map(|j| {
            let angle = 360 * j / RIM_ARCS;
            blank.point(
                format!("{tip} * cos({angle})"),
                format!("{tip} * sin({angle})"),
            )
        })
        .collect::<GeopResult<Vec<_>>>()?;
    for j in 0..RIM_ARCS {
        blank.arc(rim[j], rim[(j + 1) % RIM_ARCS], &tip, true)?;
    }
    let centre = blank.origin();
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
    // over a point on `x` past the tip circle, down its upper side — the
    // lower one's mirror image — and back across the root. Extruded both
    // ways from the gear's middle, past both its sides.
    //
    // Only lines and the flanks, every point placed by a formula: a sketch
    // the solver places in one step for any size. With arcs, whose sweeps
    // it has to find, a gear far from the one drawn sent it off to
    // infinity. The root is so a chord of the root circle, at most
    // `r (1 - cos(π / 2z))` below it — 0.04 of a module for 12 teeth.
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
    let past = gap.point(format!("{m} * ({z} / 2 + 2.5)"), "0")?;
    gap.line(flank[DEGREE], past);
    gap.line(past, upper_flank[0]);
    gap.bezier(upper_flank.clone());
    gap.line(upper_flank[DEGREE], upper_foot);
    gap.line(upper_foot, foot);
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

/// The most teeth a rack of the library has.
pub const RACK_TEETH: usize = 30;

/// A gear rack of module `size.m`, `size.b` wide, its pitch line `size.h`
/// above its back, with `teeth` teeth — at most [`RACK_TEETH`] — running
/// `teeth · π m` along `z` from the `xy` plane: its teeth face `+y`, its
/// pitch plane is the `xz` plane, and it is centred on it across `x`. The
/// teeth are straight-sided, 20° to their centre lines, `π m / 2` thick on
/// the pitch line, the first centred half a pitch up from `z = 0`.
///
/// One profile of all [`RACK_TEETH`] teeth and half a gap at either end,
/// extruded across, then cut to length — through the middle of a gap — by
/// a box: a single boolean, where cutting a gap per tooth would split the
/// rack's long edges dozens of times over. Its datums: `axis`, the `z` axis
/// on the pitch plane — a slider joint's connector — `pitch`, `back`, and
/// `end`.
pub fn rack() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#9aa0a8".into()),
        values: vec![
            size(tables::racks()),
            number("teeth", "20", 1.0, RACK_TEETH as f64),
        ],
    };
    let (m, b, h) = (col("m"), col("b"), col("h"));
    // In the profile plane: `x` up the teeth, along `y`; `y` along `z`.
    let half = |height: &str| format!("pi * {m} / 4 - {height} * tan(20)");
    let (root, tip) = (format!("-1.25 * {m}"), m.clone());
    let along = |k: usize, sign: &str, height: &str| {
        format!("{}.5 * pi * {m} {sign} ({})", k, half(height))
    };
    let mut corners = vec![
        [format!("-{h}"), "0".to_string()],
        [root.clone(), "0".into()],
    ];
    for k in 0..=RACK_TEETH {
        corners.push([root.clone(), along(k, "-", &root)]);
        corners.push([tip.clone(), along(k, "-", &tip)]);
        corners.push([tip.clone(), along(k, "+", &tip)]);
        corners.push([root.clone(), along(k, "+", &root)]);
    }
    let end = format!("{} * pi * {m}", RACK_TEETH + 1);
    corners.push([root.clone(), end.clone()]);
    corners.push([format!("-{h}"), end]);
    let mut profile = Drawing::new(&program.parameters)?;
    profile.polygon(&corners)?;
    let across = |length: String| Extents {
        symmetric: true,
        ..Extents::blind(length)
    };
    extrude(
        &mut program,
        "teeth",
        "profile",
        (profile, profile_plane()),
        across(b.clone()),
        Combine::NewBody,
    )?;
    let mut length = Drawing::new(&program.parameters)?;
    length.polygon(&[
        [format!("-{h} - {m}"), format!("-{m}")],
        [format!("2 * {m}"), format!("-{m}")],
        [format!("2 * {m}"), format!("teeth * pi * {m}")],
        [format!("-{h} - {m}"), format!("teeth * pi * {m}")],
    ])?;
    extrude(
        &mut program,
        "rack",
        "length",
        (length, profile_plane()),
        across(format!("{b} + 2 * {m}")),
        Combine::Intersection {
            target: "extrude(teeth)".into(),
        },
    )?;
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

/// The radius of a GT2 pulley's round groove, one per tooth of the belt.
const GT2_GROOVE: f64 = 0.555;

/// How deep a GT2 pulley's groove is, from its outside diameter.
const GT2_DEPTH: f64 = 0.76;

/// A GT2 timing pulley of `z` teeth for a 6 mm belt: a hub `dh` across and
/// `lh` long from its end on the `xy` plane, then the teeth `w` wide
/// between two flanges `df` across and `t` thick, bored `d` through — `d`,
/// `dh` and `lh` from its table, by bore.
///
/// The teeth are simplified: the outside diameter is the pitch diameter
/// `2z / π` less twice the belt's pitch line offset, 0.254, and a round
/// groove [`GT2_GROOVE`] in radius and [`GT2_DEPTH`] deep is cut for each
/// tooth of the belt. Their count fixes the outline, so each count is a
/// family: a ring of that outline, drawn in one sketch, joined to the hub
/// and flanges turned in one profile — one boolean. Its datums: `axis`,
/// `end`, and the plane `belt` through the middle of the belt, on which
/// the sketch `pitch` draws its pitch circle.
fn gt2_pulley(z: usize, file: &'static str, bores: tables::Table) -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: aluminium(),
        color: Some("#c8ccd2".into()),
        values: vec![size(bores)],
    };
    let (d, dh, lh) = (col("d"), col("dh"), col("lh"));
    let (t, w) = (1.0, 7.0);
    let pitch = 2.0 * z as f64 / std::f64::consts::PI;
    let outside = (pitch - 0.508) / 2.0;
    let flange = format!("{}", outside + 2.0);
    let core = format!("{}", outside - 1.0);
    let (bore, hub) = (format!("{d} / 2"), format!("{dh} / 2"));
    let (inner_face, outer_face, end) = (
        format!("{lh} + {t}"),
        format!("{lh} + {}", t + w),
        format!("{lh} + {}", 2.0 * t + w),
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
    // other, so that none of its faces lies on one of theirs: round, per
    // tooth, a land on the outside circle and a groove, drawn as two arcs
    // meeting at its bottom so that each is less than half a circle.
    plane_datum(&mut program, "ring_floor", &format!("{lh} + {}", t / 2.0));
    let centre = outside - GT2_DEPTH + GT2_GROOVE;
    let half =
        ((outside.powi(2) + centre.powi(2) - GT2_GROOVE.powi(2)) / (2.0 * outside * centre)).acos();
    let mut ring = Drawing::new(&program.parameters)?;
    let at = |r: f64, a: f64| [format!("{}", r * a.cos()), format!("{}", r * a.sin())];
    let mut groove_ends = Vec::new();
    for k in 0..z {
        // Half a tooth round from `x`: no groove meets the seams of the
        // turned flanks, which lie on the axes.
        let a = 2.0 * std::f64::consts::PI * (k as f64 + 0.5) / z as f64;
        let [x0, y0] = at(outside, a - half);
        let [xb, yb] = at(outside - GT2_DEPTH, a);
        let [x1, y1] = at(outside, a + half);
        groove_ends.push([
            ring.point(x0, y0)?,
            ring.point(xb, yb)?,
            ring.point(x1, y1)?,
        ]);
    }
    let (groove, land) = (format!("{GT2_GROOVE}"), format!("{outside}"));
    for k in 0..z {
        let [start, bottom, end] = groove_ends[k];
        ring.arc(start, bottom, &groove, false)?;
        ring.arc(bottom, end, &groove, false)?;
        ring.arc(end, groove_ends[(k + 1) % z][0], &land, true)?;
    }
    let centre_point = ring.origin();
    ring.circle(centre_point, &format!("{}", 2.0 * (outside - 1.5)))?;
    extrude(
        &mut program,
        "ring",
        "ring_outline",
        (ring, EntityRef::datum("ring_floor")),
        Extents::blind(t + w),
        Combine::Union {
            target: "revolve(body)".into(),
        },
    )?;

    axis_datum(&mut program);
    plane_datum(&mut program, "end", "0");
    plane_datum(&mut program, "belt", &format!("{lh} + {}", t + w / 2.0));
    let mut circle = Drawing::new(&program.parameters)?;
    let centre_point = circle.origin();
    circle.circle(centre_point, &format!("{pitch}"))?;
    program.push("pitch", circle.on(EntityRef::datum("belt"))?);
    Ok(StandardPart {
        file,
        title: "GT2 timing pulley, 6 mm belt",
        designation: "GT2 pulley",
        base: "end",
        program,
        threaded: Vec::new(),
    })
}

pub fn gt2_pulley_16() -> GeopResult<StandardPart> {
    gt2_pulley(16, "std:gt2_pulley_16t.geop", tables::gt2_pulleys(16))
}

pub fn gt2_pulley_20() -> GeopResult<StandardPart> {
    gt2_pulley(20, "std:gt2_pulley_20t.geop", tables::gt2_pulleys(20))
}

pub fn gt2_pulley_36() -> GeopResult<StandardPart> {
    gt2_pulley(36, "std:gt2_pulley_36t.geop", tables::gt2_pulleys(36))
}

/// A shaft collar after DIN 705 A: a ring bored `d`, `D` across and `b`
/// wide, standing on the `xy` plane, with a set screw hole `ds` across
/// through its wall along `+x`, half way up: the thread's nominal
/// diameter. The boolean cutting it names the hole's faces after what it
/// crossed, so they are not listed as threaded.
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
    hole.circle(centre, &col("ds"))?;
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
    Ok(StandardPart {
        file: "std:shaft_collar.geop",
        title: "Shaft collar with set screw, DIN 705 A",
        designation: "DIN 705 A",
        base: "base",
        program,
        threaded: Vec::new(),
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
