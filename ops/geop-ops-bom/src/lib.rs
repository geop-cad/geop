//! The bill of materials of an assembly (see
//! [`geop_ops_model::placed::bom`]), offered as an inspector
//! ([`BillOfMaterials`]) and as a CSV file ([`BomCsv`]).

use geop_core_editor::{
    editing::{Tone, Visual},
    output::{
        Answer, AnswerAction, Block, Content, Does, Export, Exporter, ExporterInfo, Inspector,
        InspectorInfo, Output, Row,
    },
    registry::Registry,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{element::PickedElement, values::Design};
use geop_ops_model::{
    parameters::stem,
    placed::bom::{Bom, Line, LineKind, Structure, bom},
};
use serde::Deserialize;

mod icons;
#[cfg(test)]
mod tests;

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
        _: &[PickedElement],
        options: BomOptions,
    ) -> GeopResult<(Bom, Vec<Visual<Design>>)> {
        Ok((
            bom(&output.part(), output.file_name(), options.structure)?,
            Vec::new(),
        ))
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
        let rows = (bom.lines.iter()).map(shown).collect();
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

/// The line `line` as the table shows it: its item, quantity, name —
/// indented by its level, its designation under it — and its total mass or
/// length; all it says in full while pointed at.
fn shown(line: &Line) -> Row {
    let indent = "\u{2003}".repeat(line.level.saturating_sub(1));
    let mut name = vec![format!("{indent}{}", line.name)];
    name.extend(line.designation.iter().map(|d| format!("{indent}{d}")));
    let (total, detail, tone) = match &line.kind {
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
            line.item.clone(),
            line.quantity.to_string(),
            name.join("\n"),
            total,
        ],
        detail: Some(detail),
        light: Vec::new(),
        tone,
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
        let bom = bom(&output.part(), output.file_name(), options.structure)?;
        Ok(Export {
            name: format!("{}_bom.csv", stem(output.file_name())),
            media_type: "text/csv".into(),
            content: Content::Text(bom.to_csv()),
        })
    }
}

/// Adds what this crate offers an editor to `r`: the bill of materials:
/// asked of a part, and written as CSV.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.exporter(BomCsv)?.inspector(BillOfMaterials)
}
