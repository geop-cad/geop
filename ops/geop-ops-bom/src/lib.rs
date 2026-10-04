//! The bill of materials of an assembly: the table a part is ordered and
//! made from ([`bom`]).
//!
//! **Lines.** Placed parts are grouped into one line when they are the same
//! part: placed from the same file, built with the same parameter values —
//! every `std:iso4762_socket_head_cap_screw.geop` at size `M4x12` is one
//! line, however many there are and wherever they are placed. Where a part
//! is placed, and where the parts placed in it are, is no part of what it
//! is; its colour neither, since it is not ordered by it.
//!
//! **Structure.** [`Structure::Flat`] lists what is made or bought: every
//! part with bodies of its own, however deep it is placed, its quantity
//! the count in the whole assembly. [`Structure::Indented`] lists the tree:
//! the parts placed in the assembly, then under each sub-assembly the parts
//! placed in it, its quantity the count per one of the sub-assembly — and a
//! sub-assembly's mass that of all of it.
//!
//! **Columns.** Per part its name — a standard part's title, else its
//! file's — the file, the parameter values it is built with, a standard
//! part's designation (`ISO 4762 M4x12`, see [`Standard`]), its material,
//! the thickness of its sheet-metal bodies, and its mass: per piece and
//! for the quantity, from the exact geometry of its bodies and its own
//! material (see [`geop_ops_inspect::mass`]). A part of no given material
//! is weighed as water, and its line says so.
//!
//! **Wires.** A routed harness (see [`geop_ops::part::Cable`]) is ordered as its
//! wires: each is a line of its own under the part that routes it, with its
//! gauge, colour and the length to cut it to. Its swept bundle is how it is
//! drawn, not something ordered, so it is neither a part's body nor
//! weighed.
//!
//! [`Bom::to_csv`] writes the table for a spreadsheet.

use std::collections::{BTreeMap, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_ops::{
    Component, Part,
    parameters::ParameterKind,
    part::{ParamValue, State},
};
use geop_ops_inspect::{Bounded, bodies::PlacedSolid, mass::material_of};
use geop_ops_sheetmetal::{FlatPatternData, Sheet};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod tests;

/// How the lines of a bill of materials are laid out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Structure {
    /// Every part that is made or bought, once, with its count in the whole
    /// assembly.
    #[default]
    Flat,
    /// The tree of sub-assemblies, each part counted per one of the
    /// assembly it is placed in.
    Indented,
}

/// What a standard part is, as ordered: the family's title (`ISO 4762
/// socket head cap screw`) and the designation of the size built
/// (`ISO 4762 M4x12`).
#[derive(Clone, Debug, PartialEq)]
pub struct Standard {
    pub title: String,
    pub designation: String,
}

/// What a line of a bill of materials is.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum LineKind {
    /// A part, or a sub-assembly.
    Part {
        /// Its program file, as the library names it.
        file: String,
        /// The values of the parameters it defines, as built: `size=M4x12`.
        parameters: String,
        material: String,
        /// Whether no material was given, and it is weighed as water.
        assumed: bool,
        /// The thickness of each sheet-metal body of its own (mm), each
        /// thickness once.
        thickness: Vec<f64>,
        /// The mass of one (kg): its own bodies' — and, in an indented
        /// bill, a sub-assembly's of all of it. `None` if it could not be
        /// computed: `error` says why.
        unit_mass: Option<Bounded>,
        /// `unit_mass` times the quantity (kg).
        total_mass: Option<Bounded>,
        error: Option<String>,
    },
    /// A wire of a routed harness, cut to length.
    Wire {
        /// The cable it is routed in: the name of its swept bundle.
        cable: String,
        /// Its American wire gauge, if it was given by one.
        gauge: Option<i32>,
        /// Its outer diameter (mm), insulation included.
        diameter: f64,
        /// `#rrggbb`.
        colour: String,
        /// The length to cut each piece to (mm).
        cut_length: Bounded,
        /// `cut_length` times the quantity (mm).
        total_length: Bounded,
    },
}

/// One line of a bill of materials.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Line {
    /// Its item number: `3` in a flat bill, `2.1` — the first under item
    /// 2 — in an indented one.
    pub item: String,
    /// How deep it is placed: 0 in a flat bill, 1 for what the assembly
    /// places itself in an indented one.
    pub level: usize,
    pub quantity: u64,
    /// A standard part's title, else its file's name; a wire's name.
    pub name: String,
    /// A standard part's designation; a wire's gauge or diameter.
    pub designation: Option<String>,
    #[serde(flatten)]
    pub kind: LineKind,
}

/// A bill of materials.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Bom {
    pub structure: Structure,
    pub lines: Vec<Line>,
    /// The mass of everything listed (kg) — if every line's could be
    /// computed: a total missing a part would mislead.
    pub total_mass: Option<Bounded>,
}

/// What a part is, by what tells it apart from another: its file and the
/// values of the parameters it defines.
fn key<S: Scalar>(file: &str, part: &Part<S>) -> String {
    format!("{file}\n{}", parameters(part))
}

/// The values of the parameters `part` defines, as built: `size=M4x12,
/// length=40`.
fn parameters<S: Scalar>(part: &Part<S>) -> String {
    let inputs = part.inputs();
    part.parameters()
        .values
        .iter()
        .filter_map(|p| {
            let value = match inputs.get(&p.name)? {
                ParamValue::Text(text) => text.clone(),
                ParamValue::Number(n) => format!("{}", n.to_f64()),
                ParamValue::Pose(_) => return None,
            };
            Some(format!("{}={value}", p.name))
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The values of the parameters `part` defines, by name, as built — what
/// a standard part's designation is made of.
fn defined<S: Scalar>(part: &Part<S>) -> State {
    let inputs = part.inputs();
    part.parameters()
        .values
        .iter()
        .filter(|p| {
            matches!(
                p.kind,
                ParameterKind::Number { .. } | ParameterKind::Table { .. }
            )
        })
        .filter_map(|p| Some((p.name.clone(), inputs.get(&p.name)?.clone())))
        .collect()
}

/// What a bill needs of one part, worked out once however often it is
/// placed.
struct Info<S: Scalar> {
    name: String,
    file: String,
    parameters: String,
    designation: Option<String>,
    material: String,
    assumed: bool,
    thickness: Vec<f64>,
    /// Whether it has bodies of its own — what makes a part something to
    /// make or buy, rather than only an assembly of others.
    own_bodies: bool,
    /// The mass of its own bodies, or why it could not be computed.
    own_mass: Result<S, String>,
}

/// The parts placed in a part, grouped: one per kind of part, in the order
/// each was first placed, with how many there are, and the component the
/// first is (which keeps its mass properties once integrated).
struct Group<'p, S: Scalar> {
    key: String,
    file: &'p str,
    part: &'p Part<S>,
    component: &'p Component<S>,
    count: u64,
}

fn groups<S: Scalar>(part: &Part<S>) -> Vec<Group<'_, S>> {
    let mut out: Vec<Group<'_, S>> = Vec::new();
    for (_, instance) in part.instances() {
        let file = instance.component.file.as_str();
        let key = key(file, instance.part());
        match out.iter_mut().find(|g| g.key == key) {
            Some(group) => group.count += 1,
            None => out.push(Group {
                key,
                file,
                part: instance.part(),
                component: &instance.component,
                count: 1,
            }),
        }
    }
    out
}

/// Builds a bill of materials, remembering what it worked out of each part.
struct Builder<'f, S: Scalar> {
    /// The standard part placed from a file, built with these values.
    standard: &'f dyn Fn(&str, &State) -> Option<Standard>,
    infos: HashMap<String, Info<S>>,
    /// The mass of all of a part — its own bodies and those of the parts
    /// placed in it — by key.
    whole: HashMap<String, Result<S, String>>,
}

impl<S: Scalar> Builder<'_, S> {
    /// What the bill needs of the part `key`, built from `file` — placed
    /// as `component`, unless it is the part the bill is of.
    fn info(
        &mut self,
        key: &str,
        file: &str,
        part: &Part<S>,
        component: Option<&Component<S>>,
    ) -> GeopResult<&Info<S>> {
        if !self.infos.contains_key(key) {
            let info = self
                .work_out(file, part, component)
                .with_context(&|e: GeopError| {
                    e.with_context(format!("the bill of materials line of {file:?}"))
                })?;
            self.infos.insert(key.to_string(), info);
        }
        Ok(&self.infos[key])
    }

    fn work_out(
        &self,
        file: &str,
        part: &Part<S>,
        component: Option<&Component<S>>,
    ) -> GeopResult<Info<S>> {
        let stem = file
            .rsplit(['/', ':'])
            .next()
            .unwrap_or(file)
            .trim_end_matches(".geop");
        let standard = (self.standard)(file, &defined(part));
        let (material, _, assumed) = material_of(part);
        // Its own bodies: every solid but a harness' bundle.
        let bodies: Vec<String> = part
            .solid_names()
            .into_iter()
            .filter(|name| !part.cables().contains_key(name))
            .collect();
        let mut thickness: Vec<f64> = bodies
            .iter()
            .filter_map(|name| {
                part.body_data::<Sheet<S>>(name)
                    .map(|sheet| sheet.rules.thickness)
                    .or_else(|| {
                        part.body_data::<FlatPatternData>(name)
                            .map(|flat| flat.thickness)
                    })
            })
            .collect();
        thickness.sort_by(f64::total_cmp);
        thickness.dedup();
        let mut own_mass = Ok(S::ZERO);
        for name in &bodies {
            let solid = PlacedSolid {
                name: name.clone(),
                part,
                component,
                solid: part.solid_id(name)?,
                pose: None,
            };
            own_mass = own_mass.and_then(|sum: S| match solid.mass_properties() {
                Ok(p) => Ok(sum.add(p.mass)),
                Err(e) => Err(format!("the mass of {name}: {e}")),
            });
        }
        Ok(Info {
            name: standard
                .as_ref()
                .map_or_else(|| stem.to_string(), |s| s.title.clone()),
            file: file.to_string(),
            parameters: parameters(part),
            designation: standard.map(|s| s.designation),
            material,
            assumed,
            thickness,
            own_bodies: !bodies.is_empty(),
            own_mass,
        })
    }

    /// The mass of all of `part`: its own bodies, and every part placed in
    /// it.
    fn whole_mass(
        &mut self,
        key: &str,
        file: &str,
        part: &Part<S>,
        component: Option<&Component<S>>,
    ) -> GeopResult<Result<S, String>> {
        if let Some(mass) = self.whole.get(key) {
            return Ok(mass.clone());
        }
        let mut mass = self.info(key, file, part, component)?.own_mass.clone();
        for group in groups(part) {
            let placed =
                self.whole_mass(&group.key, group.file, group.part, Some(group.component))?;
            mass = mass.and_then(|m| Ok(m.add(times(group.count, placed?))));
        }
        self.whole.insert(key.to_string(), mass.clone());
        Ok(mass)
    }

    /// The line of the part `key`, `quantity` of it, weighing `unit_mass`
    /// each.
    fn part_line(
        &self,
        key: &str,
        level: usize,
        quantity: u64,
        unit_mass: &Result<S, String>,
    ) -> Line {
        let info = &self.infos[key];
        let (unit, total, error) = match unit_mass {
            Ok(m) => (
                Some(Bounded::of(*m)),
                Some(Bounded::of(times(quantity, *m))),
                None,
            ),
            Err(e) => (None, None, Some(e.clone())),
        };
        Line {
            item: String::new(),
            level,
            quantity,
            name: info.name.clone(),
            designation: info.designation.clone(),
            kind: LineKind::Part {
                file: info.file.clone(),
                parameters: info.parameters.clone(),
                material: info.material.clone(),
                assumed: info.assumed,
                thickness: info.thickness.clone(),
                unit_mass: unit,
                total_mass: total,
                error,
            },
        }
    }

    /// Adds the parts of `part` — placed `multiplier` times in all — to
    /// the flat bill `lines`, by key: itself if it has bodies of its own,
    /// or is no assembly; the parts placed in it, however deep; its wires.
    fn flat(
        &mut self,
        key: &str,
        file: &str,
        part: &Part<S>,
        component: Option<&Component<S>>,
        multiplier: u64,
        lines: &mut Vec<(String, Line)>,
    ) -> GeopResult<()> {
        let info = self.info(key, file, part, component)?;
        if info.own_bodies || part.instances().next().is_none() {
            let mass = info.own_mass.clone();
            match lines.iter_mut().find(|(k, _)| k == key) {
                Some((_, line)) => add_quantity(line, multiplier),
                None => {
                    let line = self.part_line(key, 0, multiplier, &mass);
                    lines.push((key.to_string(), line));
                }
            }
        }
        for group in groups(part) {
            self.flat(
                group.key.as_str(),
                group.file,
                group.part,
                Some(group.component),
                multiplier * group.count,
                lines,
            )?;
        }
        for (cable, wire) in wire_lines(part, 0, multiplier) {
            let wire_key = format!("{key}\n{cable}\n{}", wire.name);
            match lines.iter_mut().find(|(k, _)| *k == wire_key) {
                Some((_, line)) => add_quantity(line, multiplier),
                None => lines.push((wire_key, wire)),
            }
        }
        Ok(())
    }

    /// Adds what is placed in `part` — its groups, each followed by what is
    /// placed in it, then its wires — to the indented bill `lines`, at
    /// `level`, numbered after `item` from `after + 1` on.
    fn indented(
        &mut self,
        part: &Part<S>,
        item: &str,
        after: usize,
        level: usize,
        lines: &mut Vec<Line>,
    ) -> GeopResult<()> {
        let mut number = after;
        let mut next = |line: &mut Line| {
            number += 1;
            line.item = match item {
                "" => number.to_string(),
                _ => format!("{item}.{number}"),
            };
            line.item.clone()
        };
        for group in groups(part) {
            let mass =
                self.whole_mass(&group.key, group.file, group.part, Some(group.component))?;
            let mut line = self.part_line(&group.key, level, group.count, &mass);
            let item = next(&mut line);
            lines.push(line);
            self.indented(group.part, &item, 0, level + 1, lines)?;
        }
        for (_, mut wire) in wire_lines(part, level, 1) {
            next(&mut wire);
            lines.push(wire);
        }
        Ok(())
    }
}

/// `n` times `m`.
fn times<S: Scalar>(n: u64, m: S) -> S {
    S::from_i64(n as i64).mul(m)
}

/// Adds `more` to a line's quantity, and its total with it.
fn add_quantity(line: &mut Line, more: u64) {
    line.quantity += more;
    match &mut line.kind {
        LineKind::Part {
            unit_mass,
            total_mass,
            ..
        } => *total_mass = unit_mass.map(|unit| scale_unit(unit, line.quantity)),
        LineKind::Wire {
            cut_length,
            total_length,
            ..
        } => *total_length = scale_unit(*cut_length, line.quantity),
    }
}

/// `unit` times `quantity`, as shown.
fn scale_unit(unit: Bounded, quantity: u64) -> Bounded {
    Bounded {
        value: unit.value * quantity as f64,
        error: unit.error * quantity as f64,
    }
}

/// The wires of every cable of `part` itself, each a line at `level` of
/// `quantity` pieces, with the cable's name.
fn wire_lines<S: Scalar>(part: &Part<S>, level: usize, quantity: u64) -> Vec<(String, Line)> {
    let mut out = Vec::new();
    for (cable_name, cable) in part.cables() {
        for wire in &cable.wires {
            let cut_length = Bounded::of(wire.cut_length);
            let designation = match wire.gauge {
                Some(gauge) => format!("AWG {gauge}"),
                None => format!("Ø{} mm", wire.diameter),
            };
            out.push((
                cable_name.clone(),
                Line {
                    item: String::new(),
                    level,
                    quantity,
                    name: wire.name.clone(),
                    designation: Some(designation),
                    kind: LineKind::Wire {
                        cable: cable_name.clone(),
                        gauge: wire.gauge,
                        diameter: wire.diameter,
                        colour: wire.colour.clone(),
                        cut_length,
                        total_length: scale_unit(cut_length, quantity),
                    },
                },
            ));
        }
    }
    out
}

/// The bill of materials of `part`, the program `file` builds, laid out as
/// `structure` says. `standard` says which files are standard parts, and
/// how one built with the given parameter values is designated.
///
/// The part's own bodies — those it is modelled with besides what it
/// places — are a line of their own, named after `file`: for a part that
/// places nothing, that line is the whole bill.
pub fn bom<S: Scalar>(
    part: &Part<S>,
    file: &str,
    structure: Structure,
    standard: &dyn Fn(&str, &State) -> Option<Standard>,
) -> GeopResult<Bom> {
    let mut builder = Builder {
        standard,
        infos: HashMap::new(),
        whole: HashMap::new(),
    };
    let root = key(file, part);
    let lines = match structure {
        Structure::Flat => {
            let mut keyed = Vec::new();
            builder.flat(&root, file, part, None, 1, &mut keyed)?;
            let mut lines: Vec<Line> = keyed.into_iter().map(|(_, line)| line).collect();
            for (k, line) in lines.iter_mut().enumerate() {
                line.item = (k + 1).to_string();
            }
            lines
        }
        Structure::Indented => {
            let mut lines = Vec::new();
            let info = builder.info(&root, file, part, None)?;
            if info.own_bodies || part.instances().next().is_none() {
                let mass = info.own_mass.clone();
                let mut line = builder.part_line(&root, 1, 1, &mass);
                line.item = "1".into();
                lines.push(line);
            }
            let after = lines.len();
            builder.indented(part, "", after, 1, &mut lines)?;
            lines
        }
    };
    let total_mass = builder
        .whole_mass(&root, file, part, None)?
        .ok()
        .map(Bounded::of);
    Ok(Bom {
        structure,
        lines,
        total_mass,
    })
}

impl Bom {
    /// The table as CSV (RFC 4180): a header, a row per line, and the
    /// total mass. Masses in kilograms, lengths in millimetres.
    pub fn to_csv(&self) -> String {
        let mut rows: Vec<Vec<String>> = vec![
            [
                "Item",
                "Level",
                "Quantity",
                "Name",
                "Designation",
                "File",
                "Parameters",
                "Material",
                "Thickness (mm)",
                "Unit mass (kg)",
                "Total mass (kg)",
                "Cut length (mm)",
                "Total length (mm)",
            ]
            .map(String::from)
            .to_vec(),
        ];
        let mass = |b: &Option<Bounded>| b.map(|b| format!("{:.6}", b.value)).unwrap_or_default();
        for line in &self.lines {
            let mut row = vec![
                line.item.clone(),
                line.level.to_string(),
                line.quantity.to_string(),
                line.name.clone(),
                line.designation.clone().unwrap_or_default(),
            ];
            match &line.kind {
                LineKind::Part {
                    file,
                    parameters,
                    material,
                    assumed,
                    thickness,
                    unit_mass,
                    total_mass,
                    error,
                    ..
                } => row.extend([
                    file.clone(),
                    parameters.clone(),
                    match assumed {
                        true => format!("{material} (assumed)"),
                        false => material.clone(),
                    },
                    thickness
                        .iter()
                        .map(|t| t.to_string())
                        .collect::<Vec<_>>()
                        .join("; "),
                    match error {
                        Some(e) => format!("not computed: {e}"),
                        None => mass(unit_mass),
                    },
                    mass(total_mass),
                    String::new(),
                    String::new(),
                ]),
                LineKind::Wire {
                    cable,
                    colour,
                    cut_length,
                    total_length,
                    ..
                } => row.extend([
                    String::new(),
                    format!("cable={cable}, colour={colour}"),
                    String::new(),
                    String::new(),
                    String::new(),
                    String::new(),
                    format!("{:.1}", cut_length.value),
                    format!("{:.1}", total_length.value),
                ]),
            }
            rows.push(row);
        }
        let mut total = vec![String::new(); 13];
        total[3] = "Total".into();
        total[10] = mass(&self.total_mass);
        rows.push(total);
        let mut out = String::new();
        for row in rows {
            let cells: Vec<String> = row.iter().map(|c| csv_cell(c)).collect();
            out.push_str(&cells.join(","));
            out.push_str("\r\n");
        }
        out
    }
}

/// `text` as a CSV cell: quoted, its quotes doubled, if it holds a comma,
/// a quote or a line break.
fn csv_cell(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

impl Bom {
    /// The line numbered `item`.
    pub fn line(&self, item: &str) -> Option<&Line> {
        self.lines.iter().find(|l| l.item == item)
    }

    /// How many of each thing there are, by name: the quantities of its
    /// lines summed.
    pub fn quantities(&self) -> BTreeMap<String, u64> {
        let mut out = BTreeMap::new();
        for line in &self.lines {
            *out.entry(line.name.clone()).or_default() += line.quantity;
        }
        out
    }
}
