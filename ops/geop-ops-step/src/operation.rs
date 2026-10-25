//! [`ImportStep`]: add the bodies of a STEP file to the part.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::Tone,
    part_operation,
    part_operation::PartOperation,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::StepContext,
    reader::Reader,
    target::{Target, TargetReference},
    values::Design,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::validation::{ValidationParameters, validate_fast};
use geop_ops_model::{BodyNames, ModelOperation, NamedModel, Namer, build_model};
use geop_ops_model::parameters::file_choices;
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
///
/// The file's bodies, as read, are the step's target [`imported`]: read
/// once, and again only when the file changes. Where the file disagrees
/// with itself further than the kernel can carry as one point or curve,
/// what is rebuilt from its surfaces — and how far it moved — is kept with
/// its body there ([`ImportedBody::healed`]) and shown in the dialog.
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

/// Adds the bodies `bodies` to `model`, named by `namer` as [`ImportStep`]
/// names them, after checking they are well formed on their own.
pub fn add_bodies<S: Scalar>(
    model: &mut NamedModel<S>,
    namer: &Namer,
    bodies: &[ImportedBody<S>],
) -> GeopResult<()> {
    // Checked on their own first, named as the file names them: a body the
    // file describes badly is refused by name, before it is mixed into the
    // part.
    let mut alone = NamedModel::new();
    for (k, body) in bodies.iter().enumerate() {
        let named = |names: &[Vec<String>]| -> Vec<String> {
            (names.iter())
                .map(|n| format!("s{k},{}", n.join(",")))
                .collect()
        };
        let names = BodyNames {
            vertices: named(&body.vertex_names),
            edges: named(&body.edge_names),
            faces: named(&body.face_names),
            solid: body.spec.solid.then(|| format!("s{k}")),
        };
        (alone.build_body(body.spec.clone(), names))
            .map_err(|e| e.with_context(format!("building the body {:?}", body.label)))?;
    }
    if let Err(errors) = validate_fast(&ValidationParameters::default(), alone.topology()) {
        let shown: Vec<String> = errors.iter().take(5).map(|e| e.to_string()).collect();
        let shown = shown.join("; ");
        let mentioned = alone.names_mentioned(&shown);
        return Err(GeopError::new(format!(
            "the file's bodies are not valid in geop ({} problem{}): {shown} — where {}",
            errors.len(),
            if errors.len() == 1 { "" } else { "s" },
            mentioned.join(", ")
        )));
    }
    for (k, body) in bodies.iter().enumerate() {
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
        model.build_body(body.spec.clone(), names)?;
    }
    Ok(())
}

/// The bodies of a STEP file, as read: the value of an import's target
/// [`imported`].
#[derive(Clone, Debug, PartialEq)]
pub struct Imported(pub Vec<ImportedBody<Design>>);

/// Each import step's own.
impl Target for Imported {}

/// Read whole.
impl Kind for Imported {
    const NAME: &'static str = "imported";
}

impl HasElements<Imported> for Imported {
    type Element<'a> = &'a Imported;
    const SINGLETON: Option<&'static str> = None;

    fn element(&self, name: &str) -> GeopResult<&Imported> {
        whole(self, name)
    }
}

/// The bodies the import step `step` read.
pub fn imported(step: &str) -> TargetReference<Imported> {
    TargetReference::new(format!("import:{step}"))
}

/// The bodies the import step `step` of the part `reader` reads read.
pub fn read_imported<'a>(reader: &Reader<'a>, step: &str) -> GeopResult<&'a Imported> {
    reader.retrieve_element(&ElementReference::whole(imported(step)))
}

/// The bodies of the file `file` the step `step` reads, the read recorded:
/// read again once the file changes.
fn read(reader: &Reader, (step, file): &(String, String)) -> GeopResult<Imported> {
    let ctx = with_context!("import_step({step}, file={file:?})");
    let (_, text) = reader.read_file(file).with_context(ctx)?;
    let bodies = read_step::<Design>(&text).with_context(ctx)?;
    if bodies.is_empty() {
        return Err(GeopError::new(format!("{file} has no solids or sheets"))).with_context(ctx);
    }
    Ok(Imported(bodies))
}

part_operation!(ImportStep(ImportStepArgs));

impl PartOperation for ImportStep {
    const KIND: &'static str = "import_step";
    const INFO: OperationInfo = OperationInfo::new(
        "Import STEP",
        "Add the solids and sheets of a STEP file next to the program.",
        OperationGroup::Output,
        OperationTier::Small,
    )
    .reads(&STEP_EXTENSIONS)
    .icon(crate::icons::IMPORT_STEP);
    type Args = ImportStepArgs;
    type Session = ();

    /// No file yet.
    fn new_args(_: &Context) -> ImportStepArgs {
        ImportStepArgs::default()
    }

    /// Reads the file, then adds its bodies.
    fn run(ctx: &mut StepContext, args: &ImportStepArgs) -> GeopResult<()> {
        let step = ctx.step();
        let context = with_context!("import_step({step}, {args:?})");
        if args.file.is_empty() {
            return Err(GeopError::new("choose a STEP file to import")).with_context(context);
        }
        let read_from = (step.to_string(), args.file.clone());
        ctx.produce(&imported(step), &read_from, read)?;
        build_model::<Self>(ctx, args)
    }

    /// The file: one of the STEP files next to the program, or one the user
    /// chooses from elsewhere, which the front end stores next to it — and
    /// what the import rebuilt, body by body, as the step last read them.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ImportStepArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ImportStepArgs> {
        let mut f = EditForm::<Design, ImportStepArgs>::default();
        let files = file_choices(context.runner, is_step_file, &args.file);
        f.file(
            "file",
            "file",
            args.file.clone(),
            files,
            &STEP_EXTENSIONS,
            |Edit { args, .. }, file| args.file = file.to_string(),
        );
        let read =
            (context.built.as_ref()).and_then(|built| read_imported(built, context.step).ok());
        if let (Some(Imported(bodies)), Ok(namer)) = (read, Namer::new("import", context.step)) {
            for (k, body) in bodies.iter().enumerate() {
                if body.healed.is_empty() {
                    continue;
                }
                let solid = namer.name(&[&format!("s{k}")]);
                f.form.dialog.text(
                    &format!("healed_{k}"),
                    format!(
                        "Healed {} ({}): {}.",
                        body.label,
                        solid,
                        body.healed.summary()
                    ),
                    Tone::Hint,
                );
            }
        }
        f
    }
}

impl ModelOperation for ImportStep {
    /// Adds the bodies the step read (see [`imported`]).
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        _: &ImportStepArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("import", step)?;
        let Imported(bodies) = read_imported(reader, step)?;
        add_bodies(model, &namer, bodies)
    }
}
