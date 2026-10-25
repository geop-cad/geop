//! [`Stl`]: the solids of the part as an STL mesh — the mesh the editor
//! draws (see [`geop_core_rasterize::stl`]), so a file looks exactly like
//! the part on screen, the parts it places included, each where it is.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_rasterize::stl::StlFormat;
use serde::Deserialize;

use geop_core_editor::output::{Content, Export, Exporter, ExporterInfo, Output};
use geop_ops_model::{STL_QUALITY, parameters::stem, placed::solid_meshes, stl_bytes};

/// Writes the solids of the part as an STL mesh.
pub struct Stl;

/// How an STL file is written.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(default)]
pub struct StlOptions {
    /// Only the solids of these names (`/pin:extrude(pin)`), if any are
    /// named; a name no solid has fails, listing the names there are.
    pub solids: Vec<String>,
    /// Plain text rather than binary.
    pub ascii: bool,
    /// How finely curved faces are meshed: higher is smoother, and bigger
    /// (see [`solid_meshes`]). Flat faces are meshed exactly whatever this
    /// is.
    pub quality: usize,
}

impl Default for StlOptions {
    fn default() -> Self {
        Self {
            solids: Vec::new(),
            ascii: false,
            quality: STL_QUALITY,
        }
    }
}

impl Exporter for Stl {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "stl",
        label: "STL",
        doc: "mesh, for 3-D printing",
        formats: &["stl"],
    };
    type Options = StlOptions;

    fn export(output: &mut Output, _: &str, options: StlOptions) -> GeopResult<Export> {
        let all = solid_meshes(&output.part(), options.quality)?;
        let chosen: Vec<_> = match options.solids.is_empty() {
            true => all.iter().collect(),
            false => (options.solids.iter())
                .map(|solid| {
                    all.iter().find(|(n, _)| n == solid).ok_or_else(|| {
                        let known: Vec<&str> = all.iter().map(|(n, _)| n.as_str()).collect();
                        GeopError::new(format!("no solid is named {solid:?}"))
                            .with_context(format!("the part's solids are: {}", known.join(", ")))
                    })
                })
                .collect::<GeopResult<_>>()?,
        };
        let format = match options.ascii {
            true => StlFormat::Ascii,
            false => StlFormat::Binary,
        };
        let name = stem(output.file_name());
        let triangles = chosen.iter().flat_map(|(_, triangles)| triangles);
        let bytes = stl_bytes(triangles, name, format)?;
        Ok(Export {
            name: format!("{name}.stl"),
            media_type: "model/stl".into(),
            content: Content::Bytes(bytes),
        })
    }
}
