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
/// one; the file is read in either form, text (ASCII STL) or binary.
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
        let (_, bytes) = reader.read_bytes(&args.file).with_context(ctx)?;
        let namer = Namer::new("import_stl", step)?;
        add_mesh(model, &namer, &bytes).with_context(ctx)
    }
}

/// Adds the mesh of the STL file `file` — its bytes, in either form — to
/// `model`, named by `namer` (see [`ImportStl`]).
pub fn add_mesh(model: &mut NamedModel, namer: &Namer, file: &[u8]) -> GeopResult<()> {
    let (points, triangles) = read_stl(file)?;
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

/// The corners and triangles of the STL file `file`, corners at the very
/// same coordinates one. Binary where its length is that of a binary STL
/// of as many triangles as its header says — 84 bytes and 50 a triangle,
/// which a text file starting `solid` (as some binary headers do too) all
/// but never is — and text (ASCII STL) otherwise. The facets' normals are
/// not read: the order of the corners says which way a triangle faces.
pub fn read_stl(file: &[u8]) -> GeopResult<(Vec<Vector3>, Vec<[usize; 3]>)> {
    let mut mesh = Mesh::default();
    let count = file
        .get(80..84)
        .map(|n| u32::from_le_bytes([n[0], n[1], n[2], n[3]]) as usize);
    if let Some(count) = count
        && file.len() == 84 + 50 * count
    {
        for (k, facet) in file[84..].chunks_exact(50).enumerate() {
            let corner = |c: usize| {
                let at = 12 + 12 * c;
                [0, 1, 2].map(|i| {
                    let b = &facet[at + 4 * i..at + 4 * i + 4];
                    f64::from(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                })
            };
            let corners = [corner(0), corner(1), corner(2)];
            let mut triangle = [0; 3];
            for (t, xyz) in triangle.iter_mut().zip(corners) {
                *t = mesh.corner(xyz).ok_or_else(|| {
                    GeopError::new(format!("triangle {k}: a corner is no number: {xyz:?}"))
                })?;
            }
            mesh.triangles.push(triangle);
        }
        return Ok((mesh.points, mesh.triangles));
    }
    let Ok(text) = std::str::from_utf8(file) else {
        return Err(GeopError::new(format!(
            "the file is no STL: not text, and {} bytes are no binary STL of {} triangles",
            file.len(),
            count.map_or("its".to_string(), |n| n.to_string())
        )));
    };
    if !text.trim_start().starts_with("solid") {
        return Err(GeopError::new(
            "the file is no STL: its text does not start with `solid`, and its length is no binary STL's",
        ));
    }
    let mut corners: Vec<usize> = Vec::with_capacity(3);
    for (number, line) in text.lines().enumerate() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("vertex") => {
                let mut xyz = [f64::NAN; 3];
                for c in &mut xyz {
                    if let Some(v) = words.next().and_then(|w| w.parse::<f64>().ok()) {
                        *c = v;
                    }
                }
                let p = mesh.corner(xyz).ok_or_else(|| {
                    GeopError::new(format!(
                        "line {}: a vertex is three numbers: {line:?}",
                        number + 1
                    ))
                })?;
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
                mesh.triangles.push([a, b, c]);
                corners.clear();
            }
            _ => {}
        }
    }
    Ok((mesh.points, mesh.triangles))
}

/// A mesh being read: its corners, each once, and its triangles.
#[derive(Default)]
struct Mesh {
    points: Vec<Vector3>,
    index: std::collections::HashMap<[u64; 3], usize>,
    triangles: Vec<[usize; 3]>,
}

impl Mesh {
    /// The corner at `xyz`, the one there already if there is: none if a
    /// coordinate is no finite number.
    fn corner(&mut self, xyz: [f64; 3]) -> Option<usize> {
        if !xyz.iter().all(|c| c.is_finite()) {
            return None;
        }
        let key = xyz.map(|c| (c + 0.0).to_bits());
        let points = &mut self.points;
        Some(*self.index.entry(key).or_insert_with(|| {
            points.push(Vector3::from_array(xyz.map(Scalar::from_f64)));
            points.len() - 1
        }))
    }
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
        add_mesh(&mut model, &namer, cube_stl().as_bytes()).unwrap();
        assert_valid(&model);
        let solid = model.solid_id("import_stl(i)").unwrap();
        assert_eq!(model.topology().solid_faces(solid).unwrap().len(), 6);
        assert_eq!(model.topology().edges.len(), 12);
    }

    /// The cube's triangles as a binary STL: an 80-byte header — starting
    /// `solid`, as some exporters write it — the count, and 50 bytes a
    /// triangle.
    fn cube_binary() -> Vec<u8> {
        let (points, triangles) = read_stl(cube_stl().as_bytes()).unwrap();
        let mut file = b"solid cube, binary".to_vec();
        file.resize(80, b' ');
        file.extend((triangles.len() as u32).to_le_bytes());
        for triangle in &triangles {
            file.extend([0u8; 12]);
            for &corner in triangle {
                for c in points[corner].to_array() {
                    file.extend((c.to_f64() as f32).to_le_bytes());
                }
            }
            file.extend([0u8; 2]);
        }
        file
    }

    /// The same cube read from a binary STL: the same box.
    #[test]
    fn a_binary_cube_mesh_is_the_same_box() {
        let binary = cube_binary();
        assert_eq!(binary.len(), 84 + 50 * 12);
        assert_eq!(
            read_stl(&binary).unwrap(),
            read_stl(cube_stl().as_bytes()).unwrap()
        );
        let mut model = NamedModel::new();
        let namer = Namer::new("import_stl", "i").unwrap();
        add_mesh(&mut model, &namer, &binary).unwrap();
        assert_valid(&model);
        let solid = model.solid_id("import_stl(i)").unwrap();
        assert_eq!(model.topology().solid_faces(solid).unwrap().len(), 6);
    }

    /// A binary STL chosen in the dialog's file field, stored next to the
    /// program as the front end stores a file of bytes — a `data:` URL of
    /// them — is offered, read and imported: a solid named after the step.
    #[test]
    fn a_binary_stl_chosen_in_the_dialog_is_imported() {
        use geop_core_editor::{
            editing::{Control, Value},
            editor::{Command, Editor},
            registry::Registry,
            testing::{dialog, handled},
        };
        let registry = geop_ops_model::register(Registry::default())
            .and_then(crate::register)
            .unwrap();
        let mut editor = Editor::new(registry, std::collections::BTreeMap::new());
        let stored = geop_core_program::workspace::content(cube_binary());
        assert!(stored.starts_with(geop_core_program::workspace::BYTES_PREFIX));
        let files = std::collections::BTreeMap::from([("cube.stl".to_string(), Some(stored))]);
        assert!(editor.handle(Command::Files { files }).error.is_none());
        let update = editor.handle(Command::Load {
            program: geop_core_program::program::Program::new(),
            path: Some("part.geop".into()),
        });
        assert!(update.error.is_none(), "{:?}", update.error);
        let update = editor.handle(Command::New {
            kind: "import_stl".into(),
        });
        let step = update.step.expect("a step is edited");
        let Some(Control::File { options, .. }) = step.presentation.dialog.get("file") else {
            panic!("the file is a file field");
        };
        assert!(options.iter().any(|o| o.value == "cube.stl"), "{options:?}");
        let update = editor.handle(dialog("file", Value::Choice("cube.stl".into())));
        let step = update.step.expect("a step is edited");
        assert!(step.error.is_none(), "{:?}", step.error);
        handled(&mut editor, Command::Commit);
        let scene = editor.handle(Command::Show).scene.expect("the scene");
        assert_eq!(scene.part.solids, ["import_stl(import_stl1)"]);
    }

    /// What is neither form is refused, saying so: bytes of no binary
    /// STL's length, text not starting `solid`, a vertex short of a
    /// number.
    #[test]
    fn what_is_no_stl_is_refused() {
        let mut short = cube_binary();
        short.pop();
        let error = read_stl(&short).unwrap_err();
        assert!(format!("{error}").contains("no binary STL"), "{error}");
        let error = read_stl(b"facet").unwrap_err();
        assert!(format!("{error}").contains("`solid`"), "{error}");
        let error = read_stl(b"solid\nvertex 1 2\n").unwrap_err();
        assert!(format!("{error}").contains("three numbers"), "{error}");
    }
}
