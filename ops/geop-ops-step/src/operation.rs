//! [`ImportStep`]: add the bodies of a STEP file to the part.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::{
    Model,
    validation::{ValidationParameters, validate_fast},
};
use geop_ops::{
    BodyNames, Context, Library, Namer, Part,
    operation::Operation,
    ui::Form,
};
use serde::{Deserialize, Serialize};

use crate::import::{ImportedBody, read_step};

/// Adds every body of the STEP file `file` — its solids, and its sheets of
/// faces — to the part, where the file has them: the parts of an assembly
/// each where it is placed, in millimetres.
///
/// Named `import(I,sK)` for the `K`-th body of the file and the operation
/// `I`, what it is made of `import(I,sK,X)` after its entity `X` in the
/// file: `vN`, `eN`, `fN` for the file's `N`-th vertex, edge and face,
/// counted as the body has them. A face going all the way round an axis
/// is cut into sectors `fN,qM`, along meridians `fN,mM` — one going round
/// a torus' tube likewise into pieces `fN,qM` along parallels `fN,mM`, and a
/// whole torus first into bands `fN,bK` along parallels `fN,mK` starting at
/// `fN,mK,v`, and a strip turning more than once into pieces `fN,qM` along
/// meridians `fN,mM`; an edge split where a cut crosses it, or because it
/// closes on itself, into pieces `eN,pM` at the vertices `eN,cM`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImportStep;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImportStepArgs {
    /// The STEP file, relative to the program.
    pub file: String,
}

/// The extensions of a STEP file.
pub const STEP_EXTENSIONS: [&str; 2] = ["step", "stp"];

/// Whether `path` names a STEP file.
pub fn is_step_file(path: &str) -> bool {
    let lower = path.to_ascii_lowercase();
    STEP_EXTENSIONS
        .iter()
        .any(|e| lower.ends_with(&format!(".{e}")))
}

/// Adds the bodies `bodies` to `part`, named by `namer` as [`ImportStep`]
/// names them, after checking they are well formed on their own.
pub fn add_bodies<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    bodies: Vec<ImportedBody<S>>,
) -> GeopResult<()> {
    // Checked on their own first: a body the file describes badly is
    // refused by name, before it is mixed into the part.
    let mut alone = Model::new();
    // What the file calls each entity built, by its id in `alone`: for
    // naming what a problem found mentions.
    let mut file_names: Vec<(String, String)> = Vec::new();
    for (k, body) in bodies.iter().enumerate() {
        let built = alone
            .build_body(body.spec.clone())
            .map_err(|e| e.with_context(format!("building the body {:?}", body.label)))?;
        let mut name = |id: String, name: &[String]| {
            file_names.push((id, format!("s{k},{}", name.join(","))));
        };
        for (id, n) in built.vertices.iter().zip(&body.vertex_names) {
            name(id.to_string(), n);
        }
        for (id, n) in built.edges.iter().zip(&body.edge_names) {
            name(id.to_string(), n);
        }
        for (id, n) in built.faces.iter().zip(&body.face_names) {
            name(id.to_string(), n);
        }
    }
    if let Err(errors) = validate_fast(&ValidationParameters::default(), &alone) {
        let shown: Vec<String> = errors.iter().take(5).map(|e| e.to_string()).collect();
        let shown = shown.join("; ");
        let mentioned: Vec<String> = file_names
            .iter()
            .filter(|(id, _)| shown.contains(id.as_str()))
            .map(|(id, name)| format!("{id} = {name}"))
            .collect();
        return Err(GeopError::new(format!(
            "the file's bodies are not valid in geop ({} problem{}): {shown} — where {}",
            errors.len(),
            if errors.len() == 1 { "" } else { "s" },
            mentioned.join(", ")
        )));
    }
    for (k, body) in bodies.into_iter().enumerate() {
        let solid = format!("s{k}");
        let named = |args: &[String]| {
            let mut all: Vec<&str> = vec![&solid];
            all.extend(args.iter().map(String::as_str));
            namer.name(&all)
        };
        let names = BodyNames {
            vertices: body.vertex_names.iter().map(|n| named(n)).collect(),
            edges: body.edge_names.iter().map(|n| named(n)).collect(),
            faces: body.face_names.iter().map(|n| named(n)).collect(),
            solid: body.spec.solid.then(|| namer.name(&[&solid])),
        };
        part.build_body(body.spec, names)?;
    }
    Ok(())
}

impl Operation for ImportStep {
    type Args = ImportStepArgs;
    type Session = ();

    fn new_args<S: Scalar>(&self, _: &Part<S>) -> ImportStepArgs {
        ImportStepArgs::default()
    }

    /// The file: one of the STEP files next to the program, or one the
    /// user chooses from elsewhere, which the front end stores next to it.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &ImportStepArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ImportStepArgs> {
        let mut f = Form::<S, ImportStepArgs>::new();
        let mut files: Vec<String> = context
            .library
            .files()
            .into_iter()
            .filter(|f| is_step_file(f))
            .collect();
        files.sort();
        if !args.file.is_empty() && !files.contains(&args.file) {
            files.push(args.file.clone());
        }
        f.file(
            "file",
            "file",
            args.file.clone(),
            files,
            &STEP_EXTENSIONS,
            |args, file| args.file = file.to_string(),
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ImportStepArgs,
        library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("import_step({operation_id}, {args:?})");
        if args.file.is_empty() {
            return Err(GeopError::new("choose a STEP file to import")).with_context(ctx);
        }
        let namer = Namer::new("import", operation_id).with_context(ctx)?;
        let (_, text) = library.read(&args.file).with_context(ctx)?;
        let bodies = read_step::<S>(&text).with_context(ctx)?;
        if bodies.is_empty() {
            return Err(GeopError::new(format!(
                "{} has no solids or sheets",
                args.file
            )))
            .with_context(ctx);
        }
        add_bodies(&mut part, &namer, bodies).with_context(ctx)?;
        Ok(part)
    }
}
