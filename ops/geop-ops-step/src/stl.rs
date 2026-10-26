//! [`ImportStl`]: a mesh of triangles, from an STL file, added to the part
//! as a B-rep — a solid where it closes, a sheet where it does not.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_core_topology::build::BodySpec;
use geop_ops_model::{
    BodyNames, ModelOperation, NamedModel, Namer, build_model, parameters::file_choices,
};
use serde::{Deserialize, Serialize};

/// Adds the mesh of the STL file `file` to the part: its triangles as
/// faces, joined where they share corners, and the triangles that lie in
/// one plane made one face (see [`BodySpec::of_triangles`] and
/// [`BodySpec::merged`]). A solid named `import_stl(I)` for the operation
/// `I` where every edge is shared by two triangles, a sheet otherwise;
/// each face, edge and vertex named `import_stl(I,fK)`, `eK`, `vK` in the
/// order the merged mesh has them. Corners at the very same coordinates are
/// one; the file's text form (ASCII STL) is read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImportStl;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImportStlArgs {
    /// The STL file, relative to the program.
    pub file: String,
}

/// The extension of an STL file.
pub const STL_EXTENSIONS: [&str; 1] = ["stl"];

/// Whether `path` names an STL file.
pub fn is_stl_file(path: &str) -> bool {
    path.to_ascii_lowercase().ends_with(".stl")
}

part_operation!(ImportStl(ImportStlArgs));

impl PartOperation for ImportStl {
    const KIND: &'static str = "import_stl";
    const INFO: geop_core_editor::editable::OperationInfo =
        geop_core_editor::editable::OperationInfo::new(
            "Import STL",
            "Add the mesh of an STL file next to the program, as a solid or a sheet of its triangles, those in one plane one face.",
            geop_core_editor::editable::OperationGroup::Output,
            OperationTier::Small,
        )
        .reads(&STL_EXTENSIONS)
        .icon(crate::icons::IMPORT_STL);
    type Args = ImportStlArgs;
    type Session = ();

    /// No file yet.
    fn new_args(_: &Context) -> ImportStlArgs {
        ImportStlArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &ImportStlArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The file: one of the STL files next to the program, or one chosen
    /// from elsewhere.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ImportStlArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ImportStlArgs> {
        let mut f = EditForm::<ImportStlArgs>::default();
        let files = file_choices(context.runner, is_stl_file, &args.file);
        f.file(
            "file",
            "file",
            args.file.clone(),
            files,
            &STL_EXTENSIONS,
            |Edit { args, .. }, file| args.file = file.to_string(),
        );
        f
    }
}

impl ModelOperation for ImportStl {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &ImportStlArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("import_stl({step}, file={:?})", args.file);
        if args.file.is_empty() {
            return Err(GeopError::new("choose an STL file to import")).with_context(ctx);
        }
        let (_, text) = reader.read_file(&args.file).with_context(ctx)?;
        let namer = Namer::new("import_stl", step)?;
        add_mesh(model, &namer, &text).with_context(ctx)
    }
}

/// Adds the mesh of the ASCII STL text `text` to `model`, named by
/// `namer` (see [`ImportStl`]).
pub fn add_mesh(model: &mut NamedModel, namer: &Namer, text: &str) -> GeopResult<()> {
    let (points, triangles) = read_stl(text)?;
    let spec = BodySpec::of_triangles(&points, &triangles)?;
    let (spec, _) = spec.merged(|_, _| true)?;
    let named = |prefix: &str, n: usize| -> Vec<String> {
        (0..n)
            .map(|k| namer.name(&[&format!("{prefix}{k}")]))
            .collect()
    };
    let names = BodyNames {
        vertices: named("v", spec.vertices.len()),
        edges: named("e", spec.edges.len()),
        faces: named("f", spec.faces.len()),
        solid: spec.solid.then(|| namer.root()),
    };
    model.build_body(spec, names)?;
    Ok(())
}

/// The corners and triangles of the ASCII STL text `text`: each `facet`'s
/// three `vertex` lines, corners at the very same coordinates one. The
/// facets' normals are not read — the order of the corners says which way
/// a triangle faces.
pub fn read_stl(text: &str) -> GeopResult<(Vec<Vector3>, Vec<[usize; 3]>)> {
    let mut points: Vec<Vector3> = Vec::new();
    let mut index: std::collections::HashMap<[u64; 3], usize> = Default::default();
    let mut triangles = Vec::new();
    let mut corners: Vec<usize> = Vec::with_capacity(3);
    if !text.trim_start().starts_with("solid") {
        return Err(GeopError::new(
            "the file is no ASCII STL (it does not start with `solid`): binary STL is not read",
        ));
    }
    for (number, line) in text.lines().enumerate() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("vertex") => {
                let mut xyz = [0.0f64; 3];
                for c in &mut xyz {
                    *c = words
                        .next()
                        .and_then(|w| w.parse::<f64>().ok())
                        .filter(|v| v.is_finite())
                        .ok_or_else(|| {
                            GeopError::new(format!(
                                "line {}: a vertex is three numbers: {line:?}",
                                number + 1
                            ))
                        })?;
                }
                let key = xyz.map(|c| (c + 0.0).to_bits());
                let p = *index.entry(key).or_insert_with(|| {
                    points.push(Vector3::from_array(xyz.map(Scalar::from_f64)));
                    points.len() - 1
                });
                corners.push(p);
            }
            Some("endloop") => {
                let [a, b, c] = corners[..] else {
                    return Err(GeopError::new(format!(
                        "line {}: a facet has {} corners, not three",
                        number + 1,
                        corners.len()
                    )));
                };
                triangles.push([a, b, c]);
                corners.clear();
            }
            _ => {}
        }
    }
    Ok((points, triangles))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_ops_model::testing::assert_valid;

    /// The unit cube as an ASCII STL, two triangles a side.
    fn cube_stl() -> String {
        let quads = [
            [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]],
            [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
            [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
            [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
            [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]],
            [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]],
        ];
        let mut text = String::from("solid cube\n");
        for [a, b, c, d] in quads {
            for tri in [[a, b, c], [a, c, d]] {
                text.push_str("facet normal 0 0 0\nouter loop\n");
                for p in tri {
                    text.push_str(&format!("vertex {} {} {}\n", p[0], p[1], p[2]));
                }
                text.push_str("endloop\nendfacet\n");
            }
        }
        text.push_str("endsolid cube\n");
        text
    }

    /// A cube's twelve triangles: one valid box of six faces.
    #[test]
    fn a_cube_mesh_is_a_box() {
        let mut model = NamedModel::new();
        let namer = Namer::new("import_stl", "i").unwrap();
        add_mesh(&mut model, &namer, &cube_stl()).unwrap();
        assert_valid(&model);
        let solid = model.solid_id("import_stl(i)").unwrap();
        assert_eq!(model.topology().solid_faces(solid).unwrap().len(), 6);
        assert_eq!(model.topology().edges.len(), 12);
    }

    /// Binary STL is refused, saying so.
    #[test]
    fn binary_stl_is_refused() {
        let error = read_stl("\u{0}\u{1}binary").unwrap_err();
        assert!(format!("{error}").contains("binary"), "{error}");
    }
}
