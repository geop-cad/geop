//! `geop`: the kernel on the command line.
//!
//! `geop compile part.geop` builds a program — the JSON the web
//! editor saves (see `geop_cad_base::Program`) — and writes the part it
//! makes as an STL mesh. The mesh is the one the editor draws (see
//! `geop_ops_rasterize::stl`), so a compiled file looks exactly like the
//! part on screen — the parts it places included, each where it is placed,
//! read from the program files next to it. With an output ending in
//! `.step` or `.stp`, it writes the part's B-rep as a STEP file instead
//! (see `geop_ops_step::export`): exact geometry other CAD systems open.
//!
//! `geop serve` is the other way in: the same editor the web app runs, as a
//! process a front end spawns and talks to over stdin/stdout (see
//! [`serve`]).

use std::{
    collections::HashMap,
    fs::File,
    io::{BufRead, BufWriter, Write},
    path::{Path, PathBuf},
    process::ExitCode,
    sync::Arc,
};

use clap::{Parser, Subcommand};
use geop_cad_base::{Editor, Program, Workspace, stdlib::WithStandardParts};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Pose, TriangleFace},
    scalars::scal_in_f64::ScalInF64,
};
use geop_ops::{Component, Files, Part, operation::INSTANCE_SEPARATOR};
use geop_ops_rasterize::{
    rasterize,
    stl::{StlFormat, StlTriangle, outward, write_stl},
};

type S = ScalInF64;

/// How finely curved faces are meshed unless asked otherwise — what the
/// web editor draws with.
const DEFAULT_QUALITY: u16 = 24;

#[derive(Parser)]
#[command(
    name = "geop",
    version,
    about = "The geop CAD kernel on the command line."
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Build a program (the JSON the web editor saves) and write its part as an STL mesh.
    Compile(CompileArgs),
    /// Write every built-in example (see `geop_cad_base::examples`) as a program and an STL mesh.
    Examples(ExamplesArgs),
    /// Run the editor (see `geop_cad_base::editor`) as a host process for a
    /// front end: one JSON command per line on stdin, one JSON update per
    /// line on stdout. This is how the VS Code extension drives the kernel.
    Serve,
}

#[derive(clap::Args)]
struct CompileArgs {
    /// The program to build, e.g. `part.geop`.
    program: PathBuf,
    /// Where to write the part: an STL mesh, or — ending in `.step` or
    /// `.stp` — a STEP file. Defaults to the program's path with `.geop`
    /// replaced by `.stl`.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Only this solid, by name (e.g. `extrude(hole)`); repeat for several.
    /// Every solid of the part, if not given.
    #[arg(short, long = "solid", value_name = "NAME")]
    solids: Vec<String>,
    /// Write ASCII STL instead of binary.
    #[arg(long)]
    ascii: bool,
    /// How finely curved faces are meshed: higher is smoother, and bigger.
    /// Flat faces are meshed exactly whatever this is.
    #[arg(short, long, default_value_t = DEFAULT_QUALITY, value_parser = clap::value_parser!(u16).range(2..))]
    quality: u16,
}

#[derive(clap::Args)]
struct ExamplesArgs {
    /// Where to write `<name>.geop` and `<name>.stl` for each example.
    #[arg(short, long, default_value = "examples")]
    out_dir: PathBuf,
    /// Only this example, by name (e.g. `handle_with_hole`); repeat for
    /// several. Every built-in example, if not given.
    #[arg(long = "only", value_name = "NAME")]
    only: Vec<String>,
    /// Write ASCII STL instead of binary.
    #[arg(long)]
    ascii: bool,
    /// How finely curved faces are meshed: higher is smoother, and bigger.
    #[arg(short, long, default_value_t = DEFAULT_QUALITY, value_parser = clap::value_parser!(u16).range(2..))]
    quality: u16,
}

/// What a compile wrote, for the report.
#[derive(Debug)]
struct Compiled {
    output: PathBuf,
    steps: usize,
    solids: usize,
    triangles: usize,
    /// The mates the program's state did not hold, which it was
    /// solved for before it was written; none if they all held.
    solved_for: Vec<String>,
}

/// `program`'s path with its extension replaced by `.stl`: `part.geop`
/// becomes `part.stl`.
fn default_output(program: &Path) -> PathBuf {
    program.with_extension("stl")
}

/// The program files on disk, by their paths.
struct Disk;

impl Files for Disk {
    /// A file that is not UTF-8 throughout — a STEP file with a Latin-1
    /// name in it — is read with what is not replaced.
    fn read(&self, path: &str) -> GeopResult<String> {
        std::fs::read(path)
            .map(|bytes| String::from_utf8_lossy(&bytes).into_owned())
            .map_err(|e| GeopError::new(format!("reading {path}: {e}")))
    }

    /// Nothing: a compile only reads what the program places.
    fn list(&self) -> Vec<String> {
        Vec::new()
    }
}

/// The solids of a part, each by name with its triangles.
type Meshed = Vec<(String, Vec<TriangleFace<S>>)>;

/// Every solid of `part` itself — not of the parts placed in it — by name,
/// as triangles in its own frame, meshed `quality` fine.
fn own_solids(part: &Part<S>, quality: usize) -> GeopResult<Meshed> {
    let model = part.topology();
    let raster = rasterize(model, quality)?;
    let mut out = Vec::new();
    for &solid in model.solids.keys() {
        let mut faces = model.solid_faces(solid)?;
        faces.sort_by_key(|f| f.0);
        let triangles = faces
            .iter()
            .filter_map(|f| raster.faces.get(f))
            .flatten()
            .cloned()
            .collect();
        let name = part.name_of(solid).unwrap_or_default();
        out.push((name.to_string(), triangles));
    }
    Ok(out)
}

/// Every solid of a part — whose own are `own` (see [`own_solids`]) — and
/// of the parts placed in it, by name — a placed part's behind its
/// instance's name — as triangles, moved by `pose` and meshed `quality`
/// fine. A component placed many times is meshed once (`meshed`, by the
/// component).
fn solids(
    part: &Part<S>,
    own: &Meshed,
    pose: &Pose<S>,
    prefix: &str,
    quality: usize,
    meshed: &mut HashMap<*const Component<S>, Arc<Meshed>>,
    out: &mut Meshed,
) -> GeopResult<()> {
    let motion = pose.motion();
    let place = |t: &TriangleFace<S>| TriangleFace {
        a: motion.apply(&t.a),
        b: motion.apply(&t.b),
        c: motion.apply(&t.c),
        normal: motion.rotate(&t.normal),
        vertex_normals: t.vertex_normals.map(|ns| ns.map(|n| motion.rotate(&n))),
    };
    for (name, triangles) in own {
        out.push((
            format!("{prefix}{name}"),
            triangles.iter().map(place).collect(),
        ));
    }
    for (id, instance) in part.instances() {
        let name = part.name_of(id).unwrap_or_default();
        let prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
        let key = Arc::as_ptr(&instance.component);
        if !meshed.contains_key(&key) {
            meshed.insert(key, Arc::new(own_solids(instance.part(), quality)?));
        }
        let inner = meshed[&key].clone();
        solids(
            instance.part(),
            &inner,
            &pose.compose(&instance.pose),
            &prefix,
            quality,
            meshed,
            out,
        )?;
    }
    Ok(())
}

fn compile(args: &CompileArgs) -> GeopResult<Compiled> {
    let io_err = |what: &str, path: &Path| {
        let what = what.to_string();
        let path = path.display().to_string();
        move |e: std::io::Error| GeopError::new(format!("{what} {path}: {e}"))
    };
    let path = args.program.to_string_lossy();
    // A standard part — `std:iso4032_hex_nut.geop` — compiles too.
    let workspace = Workspace::<S, Disk>::new(WithStandardParts(Disk));
    let mut program = Program::from_json(&workspace.files().read(&path)?)?;
    let library = workspace.scope(&path);
    let mut part = program.build(&library)?;
    // State stale — a file it places changed since it was saved — are
    // solved for here, not in the file: that is the editor's to write.
    let report = part.check_mates(|_| true)?;
    if !report.converged {
        let (moved, _) = part.solve_mates(None, &[], &[])?;
        program.state.extend(moved);
        part = program.build(&library)?;
    }

    let output = args
        .output
        .clone()
        .unwrap_or_else(|| default_output(&args.program));
    let name = output
        .file_stem()
        .and_then(|n| n.to_str())
        .unwrap_or("part");
    if geop_ops_step::is_step_file(&output.to_string_lossy()) {
        if !args.solids.is_empty() {
            return Err(GeopError::new(
                "choosing solids (--solid) is only for an STL mesh: a STEP file holds the whole part",
            ));
        }
        let text = geop_ops_step::write_step(&part, name)?;
        std::fs::write(&output, text).map_err(io_err("writing", &output))?;
        let solids = part.topology().solids.len();
        return Ok(Compiled {
            output,
            steps: program.steps.len(),
            solids,
            triangles: 0,
            solved_for: report.failed,
        });
    }

    // The solids to write, in name order so the file does not depend on
    // how the part stores them.
    let mut all = Vec::new();
    let quality = usize::from(args.quality);
    solids(
        &part,
        &own_solids(&part, quality)?,
        &Pose::identity(),
        "",
        quality,
        &mut HashMap::new(),
        &mut all,
    )?;
    all.sort_by(|a, b| a.0.cmp(&b.0));
    let chosen: Vec<&(String, Vec<TriangleFace<S>>)> = if args.solids.is_empty() {
        all.iter().collect()
    } else {
        args.solids
            .iter()
            .map(|name| {
                all.iter().find(|(n, _)| n == name).ok_or_else(|| {
                    let known: Vec<&str> = all.iter().map(|(n, _)| n.as_str()).collect();
                    GeopError::new(format!("no solid is named {name:?}"))
                        .with_context(format!("the part's solids are: {}", known.join(", ")))
                })
            })
            .collect::<GeopResult<_>>()?
    };
    let triangles: Vec<StlTriangle> = chosen
        .iter()
        .flat_map(|(_, triangles)| triangles.iter().map(outward))
        .collect();

    let format = if args.ascii {
        StlFormat::Ascii
    } else {
        StlFormat::Binary
    };
    let mut out = BufWriter::new(File::create(&output).map_err(io_err("creating", &output))?);
    write_stl(&triangles, name, format, &mut out)
        .and_then(|()| std::io::Write::flush(&mut out))
        .map_err(io_err("writing", &output))?;

    Ok(Compiled {
        output,
        steps: program.steps.len(),
        solids: chosen.len(),
        triangles: triangles.len(),
        solved_for: report.failed,
    })
}

/// Write every built-in example as `<out_dir>/<name>.geop` and
/// `<out_dir>/<name>.stl` — and every example of several files as those
/// files in `<out_dir>/<name>/`, its first one compiled — via [`compile`],
/// so an example's mesh is generated exactly the way any other program's
/// would be.
fn export_examples(args: &ExamplesArgs) -> GeopResult<Vec<Compiled>> {
    let dir_err = |dir: &Path| {
        let dir = dir.display().to_string();
        move |e: std::io::Error| GeopError::new(format!("creating {dir}: {e}"))
    };
    std::fs::create_dir_all(&args.out_dir).map_err(dir_err(&args.out_dir))?;
    let singles = geop_cad_base::examples::all();
    let workspaces = geop_cad_base::examples::workspaces();
    let known = || -> Vec<&str> {
        singles
            .iter()
            .map(|(n, _)| *n)
            .chain(workspaces.iter().map(|(n, _)| *n))
            .collect()
    };
    if let Some(unknown) = args.only.iter().find(|n| !known().contains(&n.as_str())) {
        return Err(GeopError::new(format!(
            "no example named {unknown:?}; the built-in examples are: {}",
            known().join(", ")
        )));
    }
    let chosen = |name: &str| args.only.is_empty() || args.only.iter().any(|n| n == name);
    let write = |path: &Path, program: &geop_cad_base::Program| {
        std::fs::write(path, program.to_json()?)
            .map_err(|e| GeopError::new(format!("writing {}: {e}", path.display())))
    };
    let compile_to = |program: PathBuf, output: PathBuf| {
        compile(&CompileArgs {
            program,
            output: Some(output),
            solids: Vec::new(),
            ascii: args.ascii,
            quality: args.quality,
        })
    };
    let mut compiled = Vec::new();
    for (name, program) in singles.iter().filter(|(n, _)| chosen(n)) {
        let path = args.out_dir.join(format!("{name}.geop"));
        write(&path, program)?;
        let output = args.out_dir.join(format!("{name}.stl"));
        compiled.push(
            compile_to(path, output)
                .map_err(|e| e.with_context(format!("export_examples(name={name})")))?,
        );
    }
    for (name, files) in workspaces.iter().filter(|(n, _)| chosen(n)) {
        let dir = args.out_dir.join(name);
        std::fs::create_dir_all(&dir).map_err(dir_err(&dir))?;
        for (file, program) in files {
            write(&dir.join(file), program)?;
        }
        let (main, _) = files.first().expect("an example has files");
        let output = args.out_dir.join(format!("{name}.stl"));
        compiled.push(
            compile_to(dir.join(main), output)
                .map_err(|e| e.with_context(format!("export_examples(name={name})")))?,
        );
    }
    Ok(compiled)
}

/// Run one [`Editor`] until stdin closes: every line is a command, and the
/// answer is one line — the update as JSON, or `{"fatal": "..."}` if the
/// command could not be read at all. A panic inside the kernel is reported
/// the same way (the editor is then in an unknown state, so the process
/// ends and the front end restarts it) rather than leaving the front end
/// waiting for an answer that never comes.
fn serve() -> GeopResult<()> {
    let io_err = |e: std::io::Error| GeopError::new(format!("serving: {e}"));
    let mut editor = Editor::<S>::new();
    let stdin = std::io::stdin().lock();
    let mut stdout = std::io::stdout().lock();
    for line in stdin.lines() {
        let line = line.map_err(io_err)?;
        if line.trim().is_empty() {
            continue;
        }
        let answer =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| editor.handle_json(&line)));
        let (reply, alive) = match answer {
            Ok(Ok(update)) => (update, true),
            Ok(Err(message)) => (serde_json::json!({ "fatal": message }).to_string(), true),
            Err(_) => (
                serde_json::json!({ "fatal": "the kernel panicked" }).to_string(),
                false,
            ),
        };
        writeln!(stdout, "{reply}")
            .and_then(|()| stdout.flush())
            .map_err(io_err)?;
        if !alive {
            return Err(GeopError::new("the kernel panicked"));
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Serve => serve(),
        Command::Compile(args) => compile(&args).map(|c| {
            if !c.solved_for.is_empty() {
                eprintln!(
                    "warning: the program's parts are not where its mates hold them ({}); compiled where they do",
                    c.solved_for.join(", ")
                );
            }
            let mesh = if c.triangles > 0 {
                format!(", {} triangles", c.triangles)
            } else {
                String::new()
            };
            eprintln!(
                "{} steps, {} solid{}{mesh} -> {}",
                c.steps,
                c.solids,
                if c.solids == 1 { "" } else { "s" },
                c.output.display()
            );
        }),
        Command::Examples(args) => export_examples(&args).map(|compiled| {
            for c in &compiled {
                eprintln!("{} triangles -> {}", c.triangles, c.output.display());
            }
            eprintln!(
                "{} example(s) -> {}",
                compiled.len(),
                args.out_dir.display()
            );
        }),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use geop_cad_base::examples;

    use geop_core_math::scalars::Scalar;

    use super::*;

    /// A fresh directory for one test's files.
    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("geop-cli-{test}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn args(program: PathBuf) -> CompileArgs {
        CompileArgs {
            program,
            output: None,
            solids: Vec::new(),
            ascii: false,
            quality: DEFAULT_QUALITY,
        }
    }

    #[test]
    fn default_output_replaces_the_program_extension() {
        assert_eq!(
            default_output(Path::new("a/part.geop")),
            Path::new("a/part.stl")
        );
        assert_eq!(
            default_output(Path::new("part.json")),
            Path::new("part.stl")
        );
        assert_eq!(default_output(Path::new("part")), Path::new("part.stl"));
    }

    /// A command that cannot be read is answered, not fatal: the front end
    /// is waiting for exactly one line per line it sent.
    #[test]
    fn serve_answers_every_command_with_one_line() {
        let mut editor = Editor::<S>::new();
        let shown = editor.handle_json(r#"{"command": "show"}"#).unwrap();
        let shown: serde_json::Value = serde_json::from_str(&shown).unwrap();
        assert!(shown["error"].is_null(), "{shown}");
        assert!(
            shown["program"]["examples"]
                .as_array()
                .is_some_and(|e| !e.is_empty())
        );
        assert!(editor.handle_json("not json").is_err());
    }

    /// An output ending in `.step` is a STEP file of the part, which a
    /// program next to it imports back to as many solids.
    #[test]
    fn compiles_to_step_and_imports_it_back() {
        let dir = scratch("step");
        let program = examples::all()
            .into_iter()
            .find(|(name, _)| *name == "box_with_drill_hole")
            .unwrap()
            .1;
        let path = dir.join("box.geop");
        std::fs::write(&path, program.to_json().unwrap()).unwrap();
        let compiled = compile(&CompileArgs {
            output: Some(dir.join("box.step")),
            ..args(path)
        })
        .unwrap();
        let text = std::fs::read_to_string(&compiled.output).unwrap();
        assert!(text.starts_with("ISO-10303-21;"), "{}", &text[..40]);
        assert!(text.contains("MANIFOLD_SOLID_BREP"));

        let mut import = geop_cad_base::Program::default();
        import.push(
            "imp",
            geop_cad_base::PartOperation::ImportStep(geop_ops_step::ImportStepArgs {
                file: "box.step".into(),
            }),
        );
        let path = dir.join("import.geop");
        std::fs::write(&path, import.to_json().unwrap()).unwrap();
        let back = compile(&args(path)).unwrap();
        assert_eq!(back.solids, compiled.solids);
        assert!(back.triangles > 0);
    }

    #[test]
    fn compiles_every_example() {
        let dir = scratch("examples");
        for (name, program) in examples::all() {
            let path = dir.join(format!("{name}.geop"));
            std::fs::write(&path, program.to_json().unwrap()).unwrap();
            let compiled = compile(&args(path)).unwrap();
            assert_eq!(compiled.output, dir.join(format!("{name}.stl")));
            assert!(compiled.triangles > 0, "{name}: no triangles");
            let bytes = std::fs::read(&compiled.output).unwrap();
            assert_eq!(bytes.len(), 84 + 50 * compiled.triangles, "{name}");
        }
    }

    #[test]
    fn export_examples_writes_every_example_as_json_and_stl() {
        let dir = scratch("export-examples");
        let compiled = export_examples(&ExamplesArgs {
            out_dir: dir.clone(),
            only: Vec::new(),
            ascii: false,
            quality: DEFAULT_QUALITY,
        })
        .unwrap();
        assert_eq!(
            compiled.len(),
            examples::all().len() + examples::workspaces().len()
        );
        for (name, _) in examples::all() {
            assert!(dir.join(format!("{name}.geop")).is_file(), "{name}");
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
        for (name, files) in examples::workspaces() {
            for (file, _) in files {
                assert!(dir.join(name).join(file).is_file(), "{name}/{file}");
            }
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
    }

    /// An assembly compiles with the parts it places, each where it is
    /// placed, read from the files beside it; one solid of a placed part is
    /// chosen by its name behind the instance's.
    #[test]
    fn compiles_an_assembly_from_its_files() {
        let dir = scratch("assembly");
        let mut sizes = std::collections::BTreeMap::new();
        let (_, files) = examples::workspaces()
            .into_iter()
            .find(|(name, _)| *name == "pin_in_plate")
            .unwrap();
        {
            for (file, program) in &files {
                std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
            }
            for (file, _) in files {
                sizes.insert(file, compile(&args(dir.join(file))).unwrap().triangles);
            }
        }
        assert_eq!(
            sizes["assembly.geop"],
            sizes["plate.geop"] + sizes["pin.geop"]
        );
        let pin = compile(&CompileArgs {
            solids: vec!["pin/extrude(pin)".into()],
            ..args(dir.join("assembly.geop"))
        })
        .unwrap();
        assert_eq!(pin.triangles, sizes["pin.geop"]);

        // Its state stale, it is solved for before it is written.
        let mut stale = examples::pin_in_plate_assembly();
        stale.state.insert(
            geop_ops::part::pose_parameter("pin"),
            geop_ops::part::ParamValue::Pose(
                Pose::from_euler(
                    geop_core_math::vector::Vector3::from_array([3.5, 1.0, 0.0].map(S::from_f64)),
                    [S::ZERO; 3],
                )
                .unwrap(),
            ),
        );
        let path = dir.join("stale.geop");
        std::fs::write(&path, stale.to_json().unwrap()).unwrap();
        let compiled = compile(&args(path)).unwrap();
        assert_eq!(
            compiled.solved_for,
            ["add_part(pin,m1)", "add_part(pin,m2)"]
        );
        assert_eq!(compiled.triangles, sizes["assembly.geop"]);
    }

    #[test]
    fn export_examples_only_writes_the_named_ones() {
        let dir = scratch("export-examples-only");
        let compiled = export_examples(&ExamplesArgs {
            out_dir: dir.clone(),
            only: vec!["cross_drilled_shaft".into(), "luggage_tag".into()],
            ascii: false,
            quality: DEFAULT_QUALITY,
        })
        .unwrap();
        assert_eq!(compiled.len(), 2);
        assert!(dir.join("cross_drilled_shaft.stl").is_file());
        assert!(dir.join("luggage_tag.stl").is_file());
        assert!(!dir.join("box_with_drill_hole.stl").exists());
    }

    #[test]
    fn export_examples_only_names_the_known_ones_when_unknown() {
        let dir = scratch("export-examples-unknown");
        let err = export_examples(&ExamplesArgs {
            out_dir: dir,
            only: vec!["not_a_real_example".into()],
            ascii: false,
            quality: DEFAULT_QUALITY,
        })
        .unwrap_err();
        assert!(err.to_string().contains("luggage_tag"), "{err}");
    }

    #[test]
    fn an_unknown_solid_names_the_known_ones() {
        let dir = scratch("unknown-solid");
        let path = dir.join("box.geop");
        std::fs::write(&path, examples::box_with_drill_hole().to_json().unwrap()).unwrap();
        let err = compile(&CompileArgs {
            solids: vec!["nothing".into()],
            ..args(path)
        })
        .unwrap_err();
        assert!(err.to_string().contains("extrude(hole)"), "{err}");
    }
}
