//! `geop`: the kernel on the command line.
//!
//! `geop compile part.geop` builds a program — the JSON the web
//! editor saves (see `Program`) — and writes the part it
//! makes with the exporter the editor offers for the output's extension
//! (see `geop_cad_base::outputs`), told the options given as `-O key=value`
//! — by default in the first format the editor offers, its STL mesh — the
//! parts it places included, each where it is placed, read from the
//! program files next to it.
//!
//! `geop serve` is the other way in: the same editor the web app runs, as a
//! process a front end spawns and talks to over stdin/stdout (see
//! [`serve`]).

use clap::{Parser, Subcommand};
use geop_cad_base::{
    Editor,
    operations::{self, BuiltPart, outputs, workspace},
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    program::Program,
    values::{Settings, StateValues},
    workspace::FileStore,
};
use geop_ops_assembly::solving::{Rerun, check as check_mates, solve};
use geop_ops_output::Output;
use std::{
    io::{BufRead, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

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
    /// Build a program (the JSON the web editor saves) and write its part, in the format the output's extension names.
    Compile(CompileArgs),
    /// Write every built-in example (see `geop_cad_base::examples`) as a program and its part, in the default format.
    Examples(ExamplesArgs),
    /// Run the editor (see `geop_cad_base::editor`) as a host process for a
    /// front end: one JSON command per line on stdin, one JSON update per
    /// line on stdout. This is how the VS Code extension drives the kernel.
    Serve(ServeArgs),
}

#[derive(clap::Args)]
struct ServeArgs {}

#[derive(clap::Args)]
struct CompileArgs {
    /// The program to build, e.g. `part.geop`.
    program: PathBuf,
    /// Where to write the part, in the format its extension names — one an
    /// exporter writes. Defaults to the program's path with the extension
    /// of the default format (see [`default_output`]).
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// An option of the exporter, `key=value`, the value JSON or else
    /// text: `-O quality=8`, `-O 'solids=["extrude(hole)"]'`. Repeat for
    /// several; those the exporter does not take, it ignores.
    #[arg(short = 'O', long = "option", value_name = "KEY=VALUE")]
    options: Vec<String>,
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
    /// An option of the exporter, as for `compile`.
    #[arg(short = 'O', long = "option", value_name = "KEY=VALUE")]
    options: Vec<String>,
}

/// The exporter options `pairs` give, `key=value` each, as JSON: each value
/// read as JSON, or else as text.
fn options(pairs: &[String]) -> GeopResult<serde_json::Value> {
    let options = (pairs.iter())
        .map(|pair| {
            let (key, value) = pair
                .split_once('=')
                .ok_or_else(|| GeopError::new(format!("the option {pair:?} is not `key=value`")))?;
            let value = serde_json::from_str(value)
                .unwrap_or_else(|_| serde_json::Value::String(value.to_string()));
            Ok((key.to_string(), value))
        })
        .collect::<GeopResult<serde_json::Map<_, _>>>()?;
    Ok(serde_json::Value::Object(options))
}

/// Today's date, `YYYY-MM-DD` (UTC): what an exported file is dated.
fn today() -> String {
    let days = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| (d.as_secs() / 86_400) as i64)
        .unwrap_or(0);
    // Days since 1970-01-01 to a civil date (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}")
}

/// What a compile wrote, for the report.
#[derive(Debug)]
struct Compiled {
    output: PathBuf,
    steps: usize,
    /// How big the file written is.
    bytes: usize,
    /// The mates the program's state did not hold, which it was
    /// solved for before it was written; none if they all held.
    solved_for: Vec<String>,
}

/// `program`'s path with its extension replaced by the first format the
/// first exporter offered writes: `part.geop` becomes `part.stl`.
fn default_output(program: &Path) -> GeopResult<PathBuf> {
    let outputs = outputs()?;
    let format = (outputs.exporters().first())
        .and_then(|exporter| exporter.formats.first())
        .ok_or_else(|| GeopError::new("the editor offers no exporter"))?;
    Ok(program.with_extension(format))
}

/// The program files on disk, by their paths.
struct Disk;

impl FileStore for Disk {
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

/// The program at `path`, built, its parts where its mates hold them, and
/// the mates the program's state did not hold. A stale state — a file it
/// places changed since it was saved — is solved for here, not in the
/// file: that is the editor's to write.
fn build(path: &Path) -> GeopResult<(Program, BuiltPart, Vec<String>)> {
    let path = path.to_string_lossy();
    // A standard part — `std:iso4032_hex_nut.geop` — builds too.
    let workspace = workspace(Disk);
    let mut program = Program::from_json(&workspace.files().read(&path)?)?;
    let mut built = operations::build(&program, &path, workspace.clone())?;
    let report = check_mates(&built.part.current())?;
    if !report.converged {
        let inputs = Settings::new();
        let mut rerun = Rerun {
            runner: &built.runner,
            part: &mut built.part,
            program: &program,
            inputs: &inputs,
        };
        let (moved, _) = solve(&mut rerun, &program.state, &[], None)?;
        program.state.extend(moved);
        program.run(
            &built.runner,
            &mut built.part,
            &Settings::new(),
            &StateValues::new(),
        )?;
    }
    Ok((program, built, report.failed))
}

fn compile(args: &CompileArgs) -> GeopResult<Compiled> {
    let io_err = |what: &str, path: &Path| {
        let what = what.to_string();
        let path = path.display().to_string();
        move |e: std::io::Error| GeopError::new(format!("{what} {path}: {e}"))
    };
    let (program, mut built_part, solved_for) = build(&args.program)?;

    let output = match &args.output {
        Some(output) => output.clone(),
        None => default_output(&args.program)?,
    };
    // The exporter of the output's format, told the options of the command
    // line: those it does not take, it ignores.
    let format = (output.extension())
        .and_then(|e| e.to_str())
        .ok_or_else(|| GeopError::new(format!("{} names no format", output.display())))?;
    let outputs = outputs()?;
    let exporter = outputs.exporter_for(format)?;
    let options = options(&args.options)?;
    let today = today();
    let mut built = Output {
        runner: &built_part.runner,
        part: &mut built_part.part,
        file: &program,
        editing: None,
        date: &today,
    };
    let export = exporter.export(&mut built, Some(format), &options)?;
    std::fs::write(&output, export.bytes()).map_err(io_err("writing", &output))?;

    Ok(Compiled {
        output,
        steps: program.steps.len(),
        bytes: export.bytes().len(),
        solved_for,
    })
}

/// Write every built-in example as `<out_dir>/<name>.geop` and its part
/// `<out_dir>/<name>.stl`, in the default format (see [`default_output`])
/// — and every example of several files as those files in
/// `<out_dir>/<name>/`, its first one compiled — via [`compile`], so an
/// example's part is written exactly the way any other program's would be.
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
    let write = |path: &Path, program: &Program| {
        std::fs::write(path, program.to_json()?)
            .map_err(|e| GeopError::new(format!("writing {}: {e}", path.display())))
    };
    let compile_to = |program: PathBuf, output: PathBuf| {
        let output = default_output(&output)?;
        compile(&CompileArgs {
            program,
            output: Some(output),
            options: args.options.clone(),
        })
    };
    let mut compiled = Vec::new();
    for (name, program) in singles.iter().filter(|(n, _)| chosen(n)) {
        let path = args.out_dir.join(format!("{name}.geop"));
        write(&path, &program())?;
        let output = args.out_dir.join(name);
        compiled.push(
            compile_to(path, output)
                .map_err(|e| e.with_context(format!("export_examples(name={name})")))?,
        );
    }
    for (name, files) in workspaces.iter().filter(|(n, _)| chosen(n)) {
        let dir = args.out_dir.join(name);
        std::fs::create_dir_all(&dir).map_err(dir_err(&dir))?;
        let files = files();
        for (file, program) in &files {
            write(&dir.join(file), program)?;
        }
        let (main, _) = files.first().expect("an example has files");
        let output = args.out_dir.join(name);
        compiled.push(
            compile_to(dir.join(main), output)
                .map_err(|e| e.with_context(format!("export_examples(name={name})")))?,
        );
    }
    Ok(compiled)
}

/// Run one [`Editor`] until stdin closes: every line is a command, and the
/// answer is one line — the update as JSON, or `{"fatal": "..."}` if the
/// command could not be read at all. A panic inside the kernel is answered
/// `{"crashed": "<what panicked, where>"}`, and the process ends: the editor
/// is then in an unknown state. The front end starts another with the
/// program it holds (`vscode-extension/src/server.ts`), rather than waiting
/// for an answer that never comes.
fn serve(_: &ServeArgs) -> GeopResult<()> {
    let io_err = |e: std::io::Error| GeopError::new(format!("serving: {e}"));
    // What panicked and where, as the default hook prints it to stderr.
    let panicked = std::sync::Arc::new(std::sync::Mutex::new(String::new()));
    let default_hook = std::panic::take_hook();
    let noted = panicked.clone();
    std::panic::set_hook(Box::new(move |info| {
        if let Ok(mut message) = noted.lock() {
            *message = info.to_string();
        }
        default_hook(info);
    }));
    let mut editor = Editor::new();
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
            Err(_) => {
                let message = panicked.lock().map(|m| m.clone()).unwrap_or_default();
                (serde_json::json!({ "crashed": message }).to_string(), false)
            }
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
        Command::Serve(args) => serve(&args),
        Command::Compile(args) => compile(&args).map(|c| {
            if !c.solved_for.is_empty() {
                eprintln!(
                    "warning: the program's parts are not where its mates hold them ({}); compiled where they do",
                    c.solved_for.join(", ")
                );
            }
            eprintln!(
                "{} steps, {} bytes -> {}",
                c.steps,
                c.bytes,
                c.output.display()
            );
        }),
        Command::Examples(args) => export_examples(&args).map(|compiled| {
            for c in &compiled {
                eprintln!("{} bytes -> {}", c.bytes, c.output.display());
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

    use geop_core_math::{primitives::Pose, scalars::Scalar};
    use geop_core_program::values::Design;

    use super::*;

    /// A fresh directory for one test's files.
    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("geop-cli-{test}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// How many triangles the binary STL file `compiled` wrote holds: what
    /// its header says, checked against its size.
    fn triangles(compiled: &Compiled) -> usize {
        let bytes = std::fs::read(&compiled.output).unwrap();
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        assert_eq!(
            bytes.len(),
            84 + 50 * count,
            "{}",
            compiled.output.display()
        );
        count
    }

    fn args(program: PathBuf) -> CompileArgs {
        CompileArgs {
            program,
            output: None,
            options: Vec::new(),
        }
    }

    #[test]
    fn default_output_replaces_the_program_extension() {
        let output = |path: &str| default_output(Path::new(path)).unwrap();
        assert_eq!(output("a/part.geop"), Path::new("a/part.stl"));
        assert_eq!(output("part.json"), Path::new("part.stl"));
        assert_eq!(output("part"), Path::new("part.stl"));
    }

    #[test]
    fn options_are_json_or_else_text() {
        let pairs = ["quality=8", r#"solids=["a(b)"]"#, "name=x=y"].map(String::from);
        assert_eq!(
            options(&pairs).unwrap(),
            serde_json::json!({"quality": 8, "solids": ["a(b)"], "name": "x=y"})
        );
        assert!(options(&["quality".into()]).is_err());
    }

    /// A command that cannot be read is answered, not fatal: the front end
    /// is waiting for exactly one line per line it sent.
    #[test]
    fn serve_answers_every_command_with_one_line() {
        let mut editor = Editor::new();
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
            .1();
        let path = dir.join("box.geop");
        std::fs::write(&path, program.to_json().unwrap()).unwrap();
        let compiled = compile(&CompileArgs {
            output: Some(dir.join("box.step")),
            ..args(path.clone())
        })
        .unwrap();
        let text = std::fs::read_to_string(&compiled.output).unwrap();
        assert!(text.starts_with("ISO-10303-21;"), "{}", &text[..40]);
        assert!(text.contains("MANIFOLD_SOLID_BREP"));

        let mut import = Program::default();
        import.push(
            "imp",
            geop_ops_step::ImportStepArgs {
                file: "box.step".into(),
            },
        );
        let imported = dir.join("import.geop");
        std::fs::write(&imported, import.to_json().unwrap()).unwrap();
        let solids = |path: &Path| {
            let (_, runner, _) = build(path).unwrap();
            let part = runner.part.current();
            geop_ops_model::read_model(&part)
                .unwrap()
                .solid_names()
                .len()
        };
        assert_eq!(solids(&imported), solids(&path));
        let back = compile(&args(imported)).unwrap();
        assert!(triangles(&back) > 0);
    }

    #[test]
    #[ignore = "slow: 7 s — run with `cargo test -- --ignored`"]
    fn compiles_every_example() {
        let dir = scratch("examples");
        for (name, program) in examples::all() {
            let path = dir.join(format!("{name}.geop"));
            std::fs::write(&path, program().to_json().unwrap()).unwrap();
            let compiled = compile(&args(path)).unwrap();
            assert_eq!(compiled.output, dir.join(format!("{name}.stl")));
            assert!(triangles(&compiled) > 0, "{name}: no triangles");
        }
    }

    #[test]
    #[ignore = "slow: 10 s — run with `cargo test -- --ignored`"]
    fn export_examples_writes_every_example_as_json_and_stl() {
        let dir = scratch("export-examples");
        let compiled = export_examples(&ExamplesArgs {
            out_dir: dir.clone(),
            only: Vec::new(),
            options: Vec::new(),
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
            for (file, _) in files() {
                assert!(dir.join(name).join(file).is_file(), "{name}/{file}");
            }
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
    }

    /// The arm, compiled to a `.zip`: its URDF robot — `robot.urdf` and
    /// the mesh of its link — in one ZIP archive.
    #[test]
    fn compiles_the_arm_as_urdf() {
        let dir = scratch("urdf");
        let (_, files) = examples::workspaces()
            .into_iter()
            .find(|(name, _)| *name == "arm")
            .unwrap();
        for (file, program) in &files() {
            std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
        }
        let compiled = compile(&CompileArgs {
            output: Some(dir.join("arm.zip")),
            options: vec!["quality=8".into()],
            ..args(dir.join("arm.geop"))
        })
        .unwrap();
        let bytes = std::fs::read(compiled.output).unwrap();
        assert!(bytes.starts_with(b"PK\x03\x04"));
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("robot.urdf") && text.contains("meshes/link.stl"));
        assert!(text.contains(r#"<robot name="arm">"#), "{text}");
    }

    /// The bolted plate's bill of materials, compiled to a `.csv`: its
    /// plate, screw and nut, the standard parts by their norms.
    #[test]
    fn writes_the_bill_of_materials_of_the_bolted_plate() {
        let dir = scratch("bom");
        let (_, files) = examples::workspaces()
            .into_iter()
            .find(|(name, _)| *name == "bolted_plate")
            .unwrap();
        for (file, program) in &files() {
            std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
        }
        let output = dir.join("bom.csv");
        compile(&CompileArgs {
            output: Some(output.clone()),
            ..args(dir.join("bolted_plate.geop"))
        })
        .unwrap();
        let csv = std::fs::read_to_string(&output).unwrap();
        let rows: Vec<&str> = csv.lines().collect();
        assert_eq!(rows.len(), 5, "{csv}");
        assert!(rows[0].starts_with("Item,Level,Quantity,Name,Designation,File"));
        assert!(rows[1].starts_with("1,0,1,plate,,"), "{csv}");
        let screw = ",ISO 4762 M4x12,std:iso4762_socket_head_cap_screw.geop,size=M4x12,Steel,";
        assert!(rows[2].contains(screw), "{csv}");
        assert!(rows[3].contains(",ISO 4032 M4,"), "{csv}");
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
        let files = files();
        {
            for (file, program) in &files {
                std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
            }
            for (file, _) in files {
                sizes.insert(file, triangles(&compile(&args(dir.join(file))).unwrap()));
            }
        }
        assert_eq!(
            sizes["assembly.geop"],
            sizes["plate.geop"] + sizes["pin.geop"]
        );
        let pin = compile(&CompileArgs {
            options: vec![r#"solids=["pin/extrude(pin)"]"#.into()],
            ..args(dir.join("assembly.geop"))
        })
        .unwrap();
        assert_eq!(triangles(&pin), sizes["pin.geop"]);

        // Its state stale, it is solved for before it is written.
        let mut stale = examples::pin_in_plate_assembly();
        stale.state.insert(
            geop_core_program::values::pose_state("pin"),
            geop_core_program::values::StateValue::Pose(
                Pose::from_euler(
                    geop_core_math::vector::Vector3::from_array(
                        [3.5, 1.0, 0.0].map(Design::from_f64),
                    ),
                    [Design::ZERO; 3],
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
        assert_eq!(triangles(&compiled), sizes["assembly.geop"]);
    }

    #[test]
    fn export_examples_only_writes_the_named_ones() {
        let dir = scratch("export-examples-only");
        let compiled = export_examples(&ExamplesArgs {
            out_dir: dir.clone(),
            only: vec!["cross_drilled_shaft".into(), "luggage_tag".into()],
            options: Vec::new(),
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
            options: Vec::new(),
        })
        .unwrap_err();
        assert!(err.to_string().contains("luggage_tag"), "{err}");
    }

    /// `geop compile bracket.geop -o bracket.dxf`: a drawing of the whole
    /// part, written in the format its name says, by the drawing exporter;
    /// named after its file in the title block.
    #[test]
    fn draws_a_program() {
        let dir = scratch("drawing");
        let path = dir.join("bracket.geop");
        std::fs::write(&path, examples::bracket().to_json().unwrap()).unwrap();
        let to = |output: &str| CompileArgs {
            output: Some(dir.join(output)),
            ..args(path.clone())
        };
        let written = compile(&to("bracket.dxf")).unwrap();
        let dxf = std::fs::read_to_string(written.output).unwrap();
        assert!(dxf.contains("\nAC1009\n") && dxf.ends_with("EOF\n"));
        for text in ["bracket", "THIRD ANGLE"] {
            assert!(dxf.contains(&format!("\n{text}\n")), "{text}");
        }
        let svg = std::fs::read_to_string(compile(&to("bracket.svg")).unwrap().output).unwrap();
        assert!(svg.starts_with("<svg"));
        let err = compile(&to("bracket.png")).unwrap_err();
        assert!(err.to_string().contains("svg, dxf"), "{err}");
    }

    /// The bracket's flat pattern for laser cutting, written by the
    /// exporter the editor offers for it, from the program built as
    /// `compile` builds it: its outline and holes on the CUT layer, its two
    /// bends on the BEND layer.
    #[test]
    fn writes_a_flat_pattern_for_cutting() {
        let dir = scratch("flat-pattern");
        let path = dir.join("sheet_metal_bracket.geop");
        std::fs::write(&path, examples::sheet_metal_bracket().to_json().unwrap()).unwrap();
        let (program, mut built_part, _) = build(&path).unwrap();
        let outputs = outputs().unwrap();
        let exporter = outputs.get_exporter("flat_pattern").unwrap();
        let mut built = Output {
            runner: &built_part.runner,
            part: &mut built_part.part,
            file: &program,
            editing: None,
            date: "",
        };
        let export = exporter
            .export(&mut built, None, &serde_json::Value::Null)
            .unwrap();
        assert_eq!(export.name, "sheet_metal_bracket_flat.dxf");
        let dxf = export.text().unwrap();
        assert_eq!(dxf.matches("\nCIRCLE\n8\nCUT\n").count(), 2, "{dxf}");
        assert_eq!(dxf.matches("\nLINE\n8\nBEND\n").count(), 2, "{dxf}");
        let err = exporter
            .export(&mut built, Some("svg"), &serde_json::Value::Null)
            .unwrap_err();
        assert!(err.to_string().contains("dxf"), "{err}");
    }

    #[test]
    fn today_is_a_date() {
        let today = today();
        assert_eq!(today.len(), 10, "{today}");
        assert_eq!(today.matches('-').count(), 2, "{today}");
    }

    #[test]
    fn an_unknown_solid_names_the_known_ones() {
        let dir = scratch("unknown-solid");
        let path = dir.join("box.geop");
        std::fs::write(&path, examples::box_with_drill_hole().to_json().unwrap()).unwrap();
        let err = compile(&CompileArgs {
            options: vec![r#"solids=["nothing"]"#.into()],
            ..args(path)
        })
        .unwrap_err();
        assert!(err.to_string().contains("extrude(hole)"), "{err}");
    }
}
