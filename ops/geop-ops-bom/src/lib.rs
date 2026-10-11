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
//! part's designation (`ISO 4762 M4x12`, see [`designation`]), its
//! material, the thickness of its sheet-metal bodies, and its mass: per
//! piece and for the quantity, from the exact geometry of its bodies and
//! its own material (see [`geop_ops::solids`]). A part of no given
//! material is weighed as water, and its line says so.
//!
//! **Wires.** The wires noted on a solid (see
//! [`geop_ops::solids::SolidNote::wires`]) — a routed harness' — are
//! ordered: each is a line of its own under the part that routes it, with
//! its gauge, colour and the length to cut it to. Its swept bundle is how
//! it is drawn, not something ordered: a solid only drawn
//! ([`geop_ops::solids::SolidNote::drawn_only`]) is neither a part's body
//! nor weighed.
//!
//! [`Bom::to_csv`] writes the table for a spreadsheet. The bill is offered
//! as an inspector ([`BillOfMaterials`]) and as a CSV file
//! ([`BomCsv`]).

use std::collections::HashMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::{Ring, Scalar},
};
use geop_ops::{
    Design, EntityRef, ParamValue,
    operation::{BuildView, instance_path},
    output::{
        Answer, AnswerAction, Block, Bounded, Content, Does, Export, Exporter, ExporterInfo,
        Inspector, InspectorInfo, Output, Row,
    },
    placed::{Instance, about, declared_values, designation, instance_name},
    solids::{Shapes, notes_of, own_solids},
    ui::{Tone, Visual},
};
use serde::{Deserialize, Serialize};

mod icons;
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
    /// Where the parts it counts are placed, as the entities of placed
    /// parts are named: in a flat bill from the assembly, `arm/screw` for
    /// the part placed as `screw` in the one placed as `arm`, `""` for the
    /// assembly's own bodies; in an indented one from the sub-assembly it
    /// is listed under. Empty for a wire. What a drawing balloons.
    #[serde(skip)]
    pub placements: Vec<String>,
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
fn key(view: BuildView) -> String {
    format!("{}\n{}", about(view).file, parameters(view))
}

/// The values of the parameters the part `view` shows defines, as built:
/// `size=M4x12, length=40`.
fn parameters(view: BuildView) -> String {
    let values = declared_values(view);
    (about(view).parameters.values.iter())
        .filter_map(|p| {
            let value = match values.get(&p.name)? {
                ParamValue::Text(text) => text.clone(),
                ParamValue::Number(n) => format!("{}", n.to_f64()),
                ParamValue::Pose(_) => return None,
            };
            Some(format!("{}={value}", p.name))
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// What a bill needs of one part, worked out once however often it is
/// placed.
struct Info {
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
    own_mass: Result<Design, String>,
}

/// The parts placed in a part, grouped: one per kind of part, in the order
/// each was first placed, with the names they are placed as.
struct Group<'a> {
    key: String,
    view: BuildView<'a>,
    names: Vec<&'a str>,
}

impl Group<'_> {
    /// How many there are.
    fn count(&self) -> u64 {
        self.names.len() as u64
    }
}

/// The parts placed in the part `view` shows itself, grouped.
fn groups(view: BuildView<'_>) -> Vec<Group<'_>> {
    let mut out: Vec<Group<'_>> = Vec::new();
    for (target, instance) in view.targets_of::<Instance>() {
        let Some(name) = instance_name(target) else {
            continue;
        };
        let placed = instance.component.view();
        let key = key(placed);
        match out.iter_mut().find(|g| g.key == key) {
            Some(group) => group.names.push(name),
            None => out.push(Group {
                key,
                view: placed,
                names: vec![name],
            }),
        }
    }
    out
}

/// Whether the part `view` shows places no part.
fn places_nothing(view: BuildView) -> bool {
    view.targets_of::<Instance>().is_empty()
}

/// The paths of the parts placed as `name` in each of the parts placed at
/// `paths` (see [`Line::placements`]).
fn placed_in(paths: &[String], names: &[&str]) -> Vec<String> {
    paths
        .iter()
        .flat_map(|path| {
            names.iter().map(move |name| match path.as_str() {
                "" => name.to_string(),
                _ => instance_path(path, name),
            })
        })
        .collect()
}

/// Builds a bill of materials, remembering what it worked out of each part.
#[derive(Default)]
struct Builder {
    infos: HashMap<String, Info>,
    /// The mass of all of a part — its own bodies and those of the parts
    /// placed in it — by key.
    whole: HashMap<String, Result<Design, String>>,
    shapes: Shapes,
}

impl Builder {
    /// What the bill needs of the part `key`, which `view` shows.
    fn info(&mut self, key: &str, view: BuildView) -> GeopResult<&Info> {
        if !self.infos.contains_key(key) {
            let info = self.work_out(view).with_context(&|e: GeopError| {
                e.with_context(format!(
                    "the bill of materials line of {:?}",
                    about(view).file
                ))
            })?;
            self.infos.insert(key.to_string(), info);
        }
        Ok(&self.infos[key])
    }

    fn work_out(&mut self, view: BuildView) -> GeopResult<Info> {
        let about = about(view);
        let material = about.material();
        let bodies = own_solids(view)?;
        let mut thickness: Vec<f64> = bodies.iter().filter_map(|s| s.thickness()).collect();
        thickness.sort_by(f64::total_cmp);
        thickness.dedup();
        let mut own_mass = Ok(Design::ZERO);
        for solid in &bodies {
            own_mass =
                own_mass.and_then(
                    |sum: Design| match solid.mass_properties(&mut self.shapes) {
                        Ok(p) => Ok(sum.add(p.mass)),
                        Err(e) => Err(format!("the mass of {}: {e}", solid.name)),
                    },
                );
        }
        Ok(Info {
            name: about.name().to_string(),
            file: about.file.clone(),
            parameters: parameters(view),
            designation: designation(view),
            material: material.name,
            assumed: material.assumed,
            thickness,
            own_bodies: !bodies.is_empty(),
            own_mass,
        })
    }

    /// The mass of all of the part `key`, which `view` shows: its own
    /// bodies, and every part placed in it.
    fn whole_mass(&mut self, key: &str, view: BuildView) -> GeopResult<Result<Design, String>> {
        if let Some(mass) = self.whole.get(key) {
            return Ok(mass.clone());
        }
        let mut mass = self.info(key, view)?.own_mass.clone();
        for group in groups(view) {
            let placed = self.whole_mass(&group.key, group.view)?;
            mass = mass.and_then(|m| Ok(m.add(times(group.count(), placed?))));
        }
        self.whole.insert(key.to_string(), mass.clone());
        Ok(mass)
    }

    /// The line of the part `key`, placed at `placements`, weighing
    /// `unit_mass` each.
    fn part_line(
        &self,
        key: &str,
        level: usize,
        placements: Vec<String>,
        unit_mass: &Result<Design, String>,
    ) -> Line {
        let quantity = placements.len() as u64;
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
            placements,
        }
    }

    /// Adds the parts of the part `key`, which `view` shows — placed at
    /// `paths` (see [`Line::placements`]) — to the flat bill `lines`, by
    /// key: itself if it has bodies of its own, or is no assembly; the
    /// parts placed in it, however deep; its wires.
    fn flat(
        &mut self,
        key: &str,
        view: BuildView,
        paths: &[String],
        lines: &mut Vec<(String, Line)>,
    ) -> GeopResult<()> {
        let multiplier = paths.len() as u64;
        let info = self.info(key, view)?;
        if info.own_bodies || places_nothing(view) {
            let mass = info.own_mass.clone();
            match lines.iter_mut().find(|(k, _)| k == key) {
                Some((_, line)) => {
                    add_quantity(line, multiplier);
                    line.placements.extend_from_slice(paths);
                }
                None => {
                    let line = self.part_line(key, 0, paths.to_vec(), &mass);
                    lines.push((key.to_string(), line));
                }
            }
        }
        for group in groups(view) {
            let paths = placed_in(paths, &group.names);
            self.flat(&group.key, group.view, &paths, lines)?;
        }
        for (cable, wire) in wire_lines(view, 0, multiplier) {
            let wire_key = format!("{key}\n{cable}\n{}", wire.name);
            match lines.iter_mut().find(|(k, _)| *k == wire_key) {
                Some((_, line)) => add_quantity(line, multiplier),
                None => lines.push((wire_key, wire)),
            }
        }
        Ok(())
    }

    /// Adds what is placed in the part `view` shows — its groups, each
    /// followed by what is placed in it, then its wires — to the indented
    /// bill `lines`, at `level`, numbered after `item` from `after + 1` on.
    fn indented(
        &mut self,
        view: BuildView,
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
        for group in groups(view) {
            let mass = self.whole_mass(&group.key, group.view)?;
            let placements = group.names.iter().map(|n| n.to_string()).collect();
            let mut line = self.part_line(&group.key, level, placements, &mass);
            let item = next(&mut line);
            lines.push(line);
            self.indented(group.view, &item, 0, level + 1, lines)?;
        }
        for (_, mut wire) in wire_lines(view, level, 1) {
            next(&mut wire);
            lines.push(wire);
        }
        Ok(())
    }
}

/// `n` times `m`.
fn times(n: u64, m: Design) -> Design {
    Design::from_i64(n as i64).mul(m)
}

/// Adds `more` to a line's quantity, and its total with it.
fn add_quantity(line: &mut Line, more: u64) {
    line.quantity += more;
    match &mut line.kind {
        LineKind::Part {
            unit_mass,
            total_mass,
            ..
        } => *total_mass = unit_mass.map(|unit| unit.times(line.quantity as f64)),
        LineKind::Wire {
            cut_length,
            total_length,
            ..
        } => *total_length = cut_length.times(line.quantity as f64),
    }
}

/// The wires noted on the solids of the part `view` shows itself, each a
/// line at `level` of `quantity` pieces, with the name of the solid — the
/// cable — it is noted on.
fn wire_lines(view: BuildView, level: usize, quantity: u64) -> Vec<(String, Line)> {
    let mut out = Vec::new();
    let Some(notes) = notes_of(view) else {
        return out;
    };
    for (cable_name, note) in notes.iter() {
        for wire in &note.wires {
            let cut_length = Bounded::of(wire.cut_length);
            let designation = match wire.gauge {
                Some(gauge) => format!("AWG {gauge}"),
                None => format!("Ø{} mm", wire.diameter),
            };
            out.push((
                cable_name.to_string(),
                Line {
                    item: String::new(),
                    level,
                    quantity,
                    name: wire.name.clone(),
                    designation: Some(designation),
                    kind: LineKind::Wire {
                        cable: cable_name.to_string(),
                        gauge: wire.gauge,
                        diameter: wire.diameter,
                        colour: wire.colour.clone(),
                        cut_length,
                        total_length: cut_length.times(quantity as f64),
                    },
                    placements: Vec::new(),
                },
            ));
        }
    }
    out
}

/// The bill of materials of the part `view` shows, laid out as
/// `structure` says. A part placed is listed by its own title and
/// designation, if it has them (see [`designation`]).
///
/// The part's own bodies — those it is modelled with besides what it
/// places — are a line of their own, named after its file: for a part
/// that places nothing, that line is the whole bill.
pub fn bom(view: BuildView, structure: Structure) -> GeopResult<Bom> {
    let mut builder = Builder::default();
    let root = key(view);
    let lines = match structure {
        Structure::Flat => {
            let mut keyed = Vec::new();
            builder.flat(&root, view, &[String::new()], &mut keyed)?;
            let mut lines: Vec<Line> = keyed.into_iter().map(|(_, line)| line).collect();
            for (k, line) in lines.iter_mut().enumerate() {
                line.item = (k + 1).to_string();
            }
            lines
        }
        Structure::Indented => {
            let mut lines = Vec::new();
            let info = builder.info(&root, view)?;
            if info.own_bodies || places_nothing(view) {
                let mass = info.own_mass.clone();
                let mut line = builder.part_line(&root, 1, vec![String::new()], &mass);
                line.item = "1".into();
                lines.push(line);
            }
            let after = lines.len();
            builder.indented(view, "", after, 1, &mut lines)?;
            lines
        }
    };
    let total_mass = builder.whole_mass(&root, view)?.ok().map(Bounded::of);
    Ok(Bom {
        structure,
        lines,
        total_mass,
    })
}

/// How a bill of materials is asked for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct BomOptions {
    pub structure: Structure,
}

/// The bill of materials of the part, as the inspect panel shows it.
pub struct BillOfMaterials;

impl Inspector for BillOfMaterials {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "bom",
        label: "Bill of materials",
        doc: "Every part with its quantity, designation, material and mass, and every wire cut to length.",
        icon: Some(crate::icons::BOM),
        picks: None,
    };
    type Options = BomOptions;
    type Report = Bom;

    fn inspect(
        output: &mut Output,
        _: &[EntityRef],
        options: BomOptions,
    ) -> GeopResult<(Bom, Vec<Visual<Design>>)> {
        Ok((bom(output.part(), options.structure)?, Vec::new()))
    }

    /// The structures to ask for, the file to save, and the table: per
    /// line its item, quantity, name and designation, and its total mass
    /// or length — in full while pointed at.
    fn show(bom: &Bom) -> Answer {
        let options = |structure: Structure| serde_json::json!({ "structure": structure });
        let ask = |structure, label: &str, doc: &str| AnswerAction {
            label: label.into(),
            doc: doc.into(),
            chosen: bom.structure == structure,
            does: Does::Inspect {
                options: options(structure),
            },
        };
        let mut blocks = vec![Block::Actions {
            actions: vec![
                ask(
                    Structure::Flat,
                    "Flat",
                    "Every part once, counted in the whole assembly",
                ),
                ask(
                    Structure::Indented,
                    "Indented",
                    "By sub-assembly, counted per assembly",
                ),
                AnswerAction {
                    label: "Save CSV".into(),
                    doc: "Save the bill of materials as a CSV file".into(),
                    chosen: false,
                    does: Does::Export {
                        exporter: BomCsv::INFO.kind,
                        options: options(bom.structure),
                    },
                },
            ],
        }];
        if bom.lines.is_empty() {
            blocks.push(Block::Text {
                text: "Nothing to list.".into(),
                tone: Tone::Hint,
            });
            return Answer {
                blocks,
                plane: None,
            };
        }
        let rows = (bom.lines.iter()).map(Line::shown).collect();
        let foot = (bom.total_mass.iter())
            .map(|mass| {
                Row::labelled(
                    "Total",
                    vec![String::new(), String::new(), mass.show_mass()],
                )
            })
            .collect();
        blocks.push(Block::Table {
            head: ["#", "Qty", "Part", "Total"].map(String::from).to_vec(),
            rows,
            foot,
        });
        Answer {
            blocks,
            plane: None,
        }
    }
}

impl Line {
    /// The line as the table shows it: its item, quantity, name — indented
    /// by its level, its designation under it — and its total mass or
    /// length; all it says in full while pointed at.
    fn shown(&self) -> Row {
        let indent = "\u{2003}".repeat(self.level.saturating_sub(1));
        let mut name = vec![format!("{indent}{}", self.name)];
        name.extend(self.designation.iter().map(|d| format!("{indent}{d}")));
        let (total, detail, tone) = match &self.kind {
            LineKind::Wire {
                cable,
                diameter,
                colour,
                cut_length,
                total_length,
                ..
            } => (
                format!("{:.0} mm", total_length.value),
                [
                    format!("cable {cable}"),
                    colour.clone(),
                    format!("Ø{diameter} mm"),
                    format!("cut to {:.1} mm", cut_length.value),
                ]
                .join("\n"),
                Tone::Normal,
            ),
            LineKind::Part {
                file,
                parameters,
                material,
                assumed,
                thickness,
                unit_mass,
                total_mass,
                error,
            } => {
                if *assumed {
                    name.push(format!("{indent}no material"));
                }
                let thickness = (!thickness.is_empty()).then(|| {
                    let each: Vec<String> = thickness.iter().map(f64::to_string).collect();
                    format!("sheet {} mm", each.join(", "))
                });
                let detail = [
                    Some(file.clone()),
                    Some(parameters.clone()),
                    Some(match assumed {
                        true => "No material given: weighed as water".into(),
                        false => material.clone(),
                    }),
                    thickness,
                    unit_mass.map(|m| format!("{} each", m.show_mass())),
                    error.clone(),
                ]
                .into_iter()
                .flatten()
                .filter(|s| !s.is_empty())
                .collect::<Vec<_>>()
                .join("\n");
                let total = total_mass.map_or("?".into(), |m| m.show_mass());
                let tone = match error {
                    Some(_) => Tone::Error,
                    None => Tone::Normal,
                };
                (total, detail, tone)
            }
        };
        Row {
            label: None,
            cells: vec![
                self.item.clone(),
                self.quantity.to_string(),
                name.join("\n"),
                total,
            ],
            detail: Some(detail),
            light: Vec::new(),
            tone,
        }
    }
}

/// The bill of materials of the part as a CSV file, for a spreadsheet.
pub struct BomCsv;

impl Exporter for BomCsv {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "bom",
        label: "Bill of materials",
        doc: "parts and wires, for a spreadsheet",
        formats: &["csv"],
    };
    type Options = BomOptions;

    fn export(output: &mut Output, _: &str, options: BomOptions) -> GeopResult<Export> {
        let bom = bom(output.part(), options.structure)?;
        Ok(Export {
            name: format!("{}_bom.csv", output.stem()),
            media_type: "text/csv".into(),
            content: Content::Text(bom.to_csv()),
        })
    }
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
}
