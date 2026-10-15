//! [`DrawingExporter`]: a drawing of the part written as SVG or DXF — one a
//! [`crate::Drawing`] step describes ([`drawing`]), or one of the whole part
//! by default — and the drawings a part's steps describe.

use geop_core_math::geop_error::{GeopError, GeopResult, WithContext};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    reader::Reader,
    target::{Target, TargetReference},
};
use geop_ops_output::{Content, Export, Exporter, ExporterInfo, Output, sheet::Format};
use serde::Deserialize;

use crate::{DrawingArgs, Projection, SheetSize, ViewKind, compose, parts_list};

/// The drawing the step `step` describes: its arguments, checked against
/// the part before it.
pub fn drawing(step: &str) -> TargetReference<DrawingArgs> {
    TargetReference::new(format!("{PREFIX}{step}"))
}

/// What the target of a drawing is named, before its step's id.
const PREFIX: &str = "drawing:";

/// The generator of [`drawing`]: the arguments it is given.
pub(crate) fn described(_: &Reader, args: &DrawingArgs) -> GeopResult<DrawingArgs> {
    Ok(args.clone())
}

/// Each drawing step's own.
impl Target for DrawingArgs {}

/// Read whole.
impl Kind for DrawingArgs {
    const NAME: &'static str = "drawing";
}

impl HasElements<DrawingArgs> for DrawingArgs {
    type Element<'a> = &'a DrawingArgs;

    fn element(&self, name: &str) -> GeopResult<&DrawingArgs> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

/// Every drawing the steps `reader` reads describe, by the id of its step,
/// in the order of the steps.
pub fn drawing_steps<'a>(reader: &Reader<'a>) -> GeopResult<Vec<(String, &'a DrawingArgs)>> {
    let mut out = Vec::new();
    for target in reader.targets_of::<DrawingArgs>() {
        let Some(step) = target.name().strip_prefix(PREFIX) else {
            continue;
        };
        let step = step.to_string();
        out.push((
            step,
            reader.retrieve_element(&ElementReference::whole(target))?,
        ));
    }
    Ok(out)
}

/// Writes a drawing of the part as SVG or DXF.
pub struct DrawingExporter;

/// Which drawing is written, and what of it is changed from what its step
/// describes — as the command line asks.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct DrawingOptions {
    /// The drawing step to write, by id. Else the step being edited, if it
    /// is a drawing; else the program's last drawing; else a drawing of
    /// the whole part, as a new step describes it.
    pub step: Option<String>,
    /// The views, by name: `front`, `top`, `right`, `iso`, `left`,
    /// `bottom`, `back`.
    pub views: Vec<String>,
    /// First-angle projection (ISO) rather than third angle (ASME).
    pub first_angle: bool,
    /// The scale as written, `1:2` or `5:1`.
    pub scale: Option<String>,
    /// The paper: `a4` to `a0`.
    pub sheet: Option<String>,
    /// The part's name for the title block.
    pub name: Option<String>,
    /// What it is made of, for the title block.
    pub material: Option<String>,
    /// List the parts placed above the title block.
    pub bom: bool,
}

/// A scale as written, `1:2` or `5:1`, as paper length per model length.
fn parse_scale(text: &str) -> GeopResult<f64> {
    let bad = || GeopError::new(format!("the scale {text:?} is not like 1:2 or 5:1"));
    let (a, b) = text.split_once(':').ok_or_else(bad)?;
    let (a, b): (f64, f64) = (
        a.trim().parse().map_err(|_| bad())?,
        b.trim().parse().map_err(|_| bad())?,
    );
    match a > 0.0 && b > 0.0 {
        true => Ok(a / b),
        false => Err(bad()),
    }
}

impl DrawingOptions {
    /// `args` changed as these options say.
    fn apply(&self, mut args: DrawingArgs) -> GeopResult<DrawingArgs> {
        if !self.views.is_empty() {
            args.views = (self.views.iter())
                .map(|v| {
                    ViewKind::from_name(v.trim()).ok_or_else(|| {
                        let known: Vec<&str> = ViewKind::ALL.iter().map(|k| k.name()).collect();
                        GeopError::new(format!(
                            "no view is named {v:?}; the views are {}",
                            known.join(", ")
                        ))
                    })
                })
                .collect::<GeopResult<_>>()?;
        }
        if self.first_angle {
            args.projection = Projection::FirstAngle;
        }
        if let Some(scale) = &self.scale {
            args.scale = Some(parse_scale(scale)?);
        }
        if let Some(sheet) = &self.sheet {
            args.sheet = (SheetSize::ALL.into_iter())
                .find(|s| s.name() == sheet.to_ascii_lowercase())
                .ok_or_else(|| GeopError::new(format!("no paper is named {sheet:?}: a4 to a0")))?;
        }
        if let Some(name) = &self.name {
            args.name = name.clone();
        }
        if let Some(material) = &self.material {
            args.material = material.clone();
        }
        args.bom |= self.bom;
        Ok(args)
    }
}

impl Exporter for DrawingExporter {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "drawing",
        label: "Drawing",
        doc: "2-D drawing: views, dimensions, title block",
        formats: &["svg", "dxf"],
    };
    type Options = DrawingOptions;

    /// The drawing of the part as the steps before its step built it —
    /// the title block's name and material, where the drawing gives none,
    /// the program's (see [`geop_ops_parameters::Properties`]).
    fn export(output: &mut Output, format: &str, options: DrawingOptions) -> GeopResult<Export> {
        let format = Format::of_extension(format)
            .ok_or_else(|| GeopError::new(format!("a drawing is no .{format} file")))?;
        let current = output.part();
        let steps = drawing_steps(&current)?;
        let found = match &options.step {
            Some(step) => Some(
                (steps.iter().find(|(s, _)| s == step))
                    .ok_or_else(|| GeopError::new(format!("the step {step:?} is no drawing")))?,
            ),
            None => (output.editing)
                .and_then(|editing| steps.iter().find(|(s, _)| s == editing))
                .or(steps.last()),
        };
        let (step, args) = match found {
            Some((step, args)) => (Some(step.to_string()), (*args).clone()),
            None => (None, DrawingArgs::default()),
        };
        let mut args = options.apply(args)?;
        let properties = output.properties()?;
        if args.name.is_empty() {
            let stem = output.stem()?;
            args.name = properties.name(&stem).to_string();
        }
        let material = properties.material();
        if args.material.is_empty() && !material.assumed {
            args.material = material.name;
        }
        let reader = match &step {
            Some(step) => output.part_before(step)?,
            None => output.part(),
        };
        let ctx = |e: GeopError| match &step {
            Some(step) => e.with_context(format!("the drawing {step:?}")),
            None => e.with_context("the drawing of the whole part"),
        };
        let parts = parts_list(&reader, output.file_name(), &args).with_context(&ctx)?;
        let sheet = compose(&reader, &args, output.date, &parts).with_context(&ctx)?;
        Ok(Export {
            name: format!("{}.{}", output.stem()?, format.extension()),
            media_type: format.media_type().into(),
            content: Content::Text(sheet.write(format)),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scales_are_read_as_written() {
        assert_eq!(parse_scale("1:5").unwrap(), 0.2);
        assert_eq!(parse_scale("2:1").unwrap(), 2.0);
        assert!(parse_scale("0:1").is_err());
        assert!(parse_scale("half").is_err());
    }
}
