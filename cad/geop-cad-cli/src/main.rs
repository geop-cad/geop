//! `geop`: the kernel on the command line.
//!
//! `geop compile part.geop` builds a program — the JSON the web
//! editor saves (see `geop_cad_base::Program`) — and writes the part it
//! makes as an STL mesh. The mesh is the one the editor draws (see
//! `geop_ops_rasterize::stl`), so a compiled file looks exactly like the
//! part on screen.
//!
//! `geop serve` is the other way in: the same editor the web app runs, as a
//! process a front end spawns and talks to over stdin/stdout (see
//! [`serve`]).

use std::{
    fs::File,
    io::{BufRead, BufWriter, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

use clap::{Parser, Subcommand};
use geop_cad_base::{Editor, Program};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::scal_in_f64::ScalInF64,
};
use geop_ops_rasterize::{
    rasterize,
    stl::{StlFormat, stl_triangles, write_stl},
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
    /// Where to write the mesh. Defaults to the program's path with
    /// `.geop` replaced by `.stl`.
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
}

/// `program`'s path with its extension replaced by `.stl`: `part.geop`
/// becomes `part.stl`.
fn default_output(program: &Path) -> PathBuf {
    program.with_extension("stl")
}

fn compile(args: &CompileArgs) -> GeopResult<Compiled> {
    let io_err = |what: &str, path: &Path| {
        let what = what.to_string();
        let path = path.display().to_string();
        move |e: std::io::Error| GeopError::new(format!("{what} {path}: {e}"))
    };
    let json = std::fs::read_to_string(&args.program).map_err(io_err("reading", &args.program))?;
    let program = Program::from_json(&json)?;
    let part = program.build::<S>()?;
    let model = part.topology();

    // The solids to write, in name order so the file does not depend on
    // how the part stores them.
    let solids = if args.solids.is_empty() {
        let mut solids: Vec<_> = model.solids.keys().copied().collect();
        solids.sort_by(|a, b| part.name_of(*a).cmp(&part.name_of(*b)));
        solids
    } else {
        args.solids
            .iter()
            .map(|name| {
                part.solid_id(name).map_err(|e| {
                    let mut known: Vec<_> = model
                        .solids
                        .keys()
                        .filter_map(|&s| part.name_of(s))
                        .collect();
                    known.sort();
                    e.with_context(format!("the part's solids are: {}", known.join(", ")))
                })
            })
            .collect::<GeopResult<_>>()?
    };
    let mut faces = Vec::new();
    for &solid in &solids {
        let mut of_solid = model.solid_faces(solid)?;
        of_solid.sort_by_key(|f| f.0);
        faces.extend(of_solid);
    }

    let raster = rasterize(model, usize::from(args.quality))?;
    let triangles = stl_triangles(&raster, &faces);

    let output = args
        .output
        .clone()
        .unwrap_or_else(|| default_output(&args.program));
    let name = output
        .file_stem()
        .and_then(|n| n.to_str())
        .unwrap_or("part");
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
        solids: solids.len(),
        triangles: triangles.len(),
    })
}

/// Write every built-in example as `<out_dir>/<name>.geop` and
/// `<out_dir>/<name>.stl`, via [`compile`] — so an example's mesh is
/// generated exactly the way any other program's would be.
fn export_examples(args: &ExamplesArgs) -> GeopResult<Vec<Compiled>> {
    std::fs::create_dir_all(&args.out_dir)
        .map_err(|e| GeopError::new(format!("creating {}: {e}", args.out_dir.display())))?;
    let all = geop_cad_base::examples::all();
    let selected: Vec<_> = if args.only.is_empty() {
        all
    } else {
        args.only
            .iter()
            .map(|name| {
                all.iter().find(|(n, _)| n == name).cloned().ok_or_else(|| {
                    let known: Vec<_> = all.iter().map(|(n, _)| *n).collect();
                    GeopError::new(format!(
                        "no example named {name:?}; the built-in examples are: {}",
                        known.join(", ")
                    ))
                })
            })
            .collect::<GeopResult<_>>()?
    };
    selected
        .into_iter()
        .map(|(name, program)| {
            let json_path = args.out_dir.join(format!("{name}.geop"));
            std::fs::write(&json_path, program.to_json()?)
                .map_err(|e| GeopError::new(format!("writing {}: {e}", json_path.display())))?;
            compile(&CompileArgs {
                program: json_path,
                output: Some(args.out_dir.join(format!("{name}.stl"))),
                solids: Vec::new(),
                ascii: args.ascii,
                quality: args.quality,
            })
            .map_err(|e| e.with_context(format!("export_examples(name={name})")))
        })
        .collect()
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
            eprintln!(
                "{} steps, {} solid{}, {} triangles -> {}",
                c.steps,
                c.solids,
                if c.solids == 1 { "" } else { "s" },
                c.triangles,
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
        assert_eq!(compiled.len(), examples::all().len());
        for (name, _) in examples::all() {
            assert!(dir.join(format!("{name}.geop")).is_file(), "{name}");
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
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
