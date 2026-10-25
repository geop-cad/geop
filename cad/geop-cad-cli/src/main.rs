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
//! [`serve`]). `geop mcp` is that editor as a Model Context Protocol server,
//! for an agent to drive (see [`mcp`]). `geop open` serves that editor to
//! a browser, editing a folder of files (see [`open`]), and `geop vscode`
//! installs the VS Code extension of this release (see [`vscode`]).

mod mcp;
mod open;
mod release;
mod session;
mod vscode;

use clap::{Parser, Subcommand};
use geop_cad_base::{BuiltPart, outputs, workspace_with};
use geop_core_editor::editor::Rerun;
use geop_core_editor::output::Output;
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    cache::{CACHE_EXTENSION, Cache, CacheStore},
    program::Program,
    workspace::FileStore,
};
use geop_ops_assembly::solving::{check as check_mates, solve};
use session::Session;
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
    /// Write every built-in example (see `geop_cad_base::registry`) as a program and its part, in the default format.
    Examples(ExamplesArgs),
    /// Run the editor (see `geop_core_editor::editor`) as a host process for a
    /// front end: one JSON command per line on stdin, one JSON update per
    /// line on stdout. This is how the VS Code extension drives the kernel.
    Serve(ServeArgs),
    /// Run the editor as a Model Context Protocol server over stdin/stdout,
    /// for an agent to drive: `claude mcp add geop -- geop mcp` (Claude Code)
    /// or `codex mcp add geop -- geop mcp` (Codex). See
    /// docs/mcp.md.
    Mcp(McpArgs),
    /// Edit the files of a folder in the web editor, in your browser, on
    /// the kernel running here: `geop open my-project`. The files stay on
    /// disk, written as they are edited.
    Open(open::OpenArgs),
    /// Install the VS Code extension of this version of geop into VS Code
    /// (or Cursor, or VSCodium), downloaded from its GitHub release.
    Vscode(vscode::VscodeArgs),
}

#[derive(clap::Args)]
struct ServeArgs {
    /// The folder the files the front end sends are named relative to:
    /// what they are made into is kept there, beside each, in a `.geopc`
    /// file (see `geop_core_program::cache`). Kept in memory only, if not
    /// given.
    #[arg(long, value_name = "DIR")]
    cache_root: Option<PathBuf>,
}

#[derive(clap::Args)]
struct McpArgs {}

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

/// Cache files on disk: `path.geopc` under `root` for the file `path`, of
/// this build of the kernel (see `geop_core_program::cache`).
struct DiskCache {
    root: PathBuf,
    build: String,
}

impl DiskCache {
    /// The cache files under `root`, of the executable running: its version,
    /// and its size and time on disk, so that a kernel built again makes
    /// again what an older one kept. None if the executable cannot be told
    /// apart: nothing is then kept on disk.
    fn of(root: PathBuf) -> Option<Self> {
        let exe = std::fs::metadata(std::env::current_exe().ok()?).ok()?;
        let modified = exe
            .modified()
            .ok()?
            .duration_since(std::time::UNIX_EPOCH)
            .ok()?;
        let build = format!(
            "{} {} {}",
            env!("CARGO_PKG_VERSION"),
            exe.len(),
            modified.as_nanos()
        );
        Some(Self { root, build })
    }

    /// What `self` makes of files: kept in it, if there is one, and in
    /// memory.
    fn cache(this: Option<Self>) -> Cache {
        match this {
            Some(store) => Cache::stored(Box::new(store)),
            None => Cache::new(),
        }
    }
}

impl CacheStore for DiskCache {
    fn build(&self) -> &str {
        &self.build
    }

    fn read(&self, path: &str) -> Option<String> {
        std::fs::read_to_string(self.root.join(path)).ok()
    }

    /// Written whole, then put in place: a run reading it meanwhile, or one
    /// cut short, sees the old file or the new, never half of one. Not
    /// written is only made again next time, so it is said on stderr and
    /// nothing else.
    fn write(&self, path: &str, text: &str) {
        let target = self.root.join(path);
        let partial = target.with_extension(format!("{CACHE_EXTENSION}.{}", std::process::id()));
        let written =
            std::fs::write(&partial, text).and_then(|()| std::fs::rename(&partial, &target));
        if let Err(e) = written {
            let _ = std::fs::remove_file(&partial);
            eprintln!("geop: could not keep {}: {e}", target.display());
        }
    }
}

/// The program at `path`, built, its parts where its mates hold them, and
/// the mates the program's state did not hold. A stale state — a file it
/// places changed since it was saved — is solved for here, not in the
/// file: that is the editor's to write.
fn build(path: &Path) -> GeopResult<(Program, BuiltPart, Vec<String>)> {
    let path = path.to_string_lossy();
    // A standard part — `std:iso4032_hex_nut.geop` — builds too.
    let workspace = workspace_with(Disk, DiskCache::cache(DiskCache::of(PathBuf::new())));
    let mut program = Program::from_json(&workspace.files().read(&path)?)?;
    let mut built = geop_cad_base::build(&program, &path, workspace.clone())?;
    let report = check_mates(&built.part.current())?;
    if !report.converged {
        let mut rerun = Rerun {
            runner: &built.runner,
            part: &mut built.part,
            program: &program,
        };
        let (moved, _) = solve(&mut rerun, &program.state, &[], None)?;
        program.state.extend(moved);
        program.run(&built.runner, &mut built.part)?;
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
    let offered = geop_cad_base::registry()?;
    let singles: Vec<&str> = offered.example_names().collect();
    let workspaces: Vec<&str> = offered.workspace_example_names().collect();
    let known = || -> Vec<&str> { singles.iter().chain(&workspaces).copied().collect() };
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
    for name in singles.iter().filter(|n| chosen(n)) {
        let path = args.out_dir.join(format!("{name}.geop"));
        write(&path, &offered.example(name)?)?;
        let output = args.out_dir.join(name);
        compiled.push(
            compile_to(path, output)
                .map_err(|e| e.with_context(format!("export_examples(name={name})")))?,
        );
    }
    for name in workspaces.iter().filter(|n| chosen(n)) {
        let dir = args.out_dir.join(name);
        std::fs::create_dir_all(&dir).map_err(dir_err(&dir))?;
        let files = offered.workspace_example(name)?;
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

/// Run one [`Session`] until stdin closes: every line is a command, and the
/// answer is one line (see [`session`]). After a panic the process ends:
/// the front end starts another with the program it holds
/// (`vscode-extension/src/server.ts`), rather than waiting for an answer
/// that never comes.
fn serve(args: &ServeArgs) -> GeopResult<()> {
    let io_err = |e: std::io::Error| GeopError::new(format!("serving: {e}"));
    let mut session = Session::new(args.cache_root.clone());
    let stdin = std::io::stdin().lock();
    let mut stdout = std::io::stdout().lock();
    for line in stdin.lines() {
        let line = line.map_err(io_err)?;
        if line.trim().is_empty() {
            continue;
        }
        let answer = session.answer(&line);
        writeln!(stdout, "{}", answer.reply)
            .and_then(|()| stdout.flush())
            .map_err(io_err)?;
        if answer.crashed {
            return Err(GeopError::new("the kernel panicked"));
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Serve(args) => serve(&args),
        Command::Mcp(McpArgs {}) => mcp::serve(),
        Command::Open(args) => open::open(&args),
        Command::Vscode(args) => vscode::install(&args),
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
        let mut editor = geop_cad_base::editor();
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
    #[ignore = "slow: 7 s — run with `cargo test -- --ignored`"]
    fn compiles_every_example() {
        let dir = scratch("examples");
        let offered = geop_cad_base::registry().unwrap();
        for name in offered.example_names() {
            let path = dir.join(format!("{name}.geop"));
            let program = offered.example(name).unwrap();
            std::fs::write(&path, program.to_json().unwrap()).unwrap();
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
        let offered = geop_cad_base::registry().unwrap();
        assert_eq!(
            compiled.len(),
            offered.example_names().count() + offered.workspace_example_names().count()
        );
        for name in offered.example_names() {
            assert!(dir.join(format!("{name}.geop")).is_file(), "{name}");
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
        for name in offered.workspace_example_names() {
            for (file, _) in offered.workspace_example(name).unwrap() {
                assert!(dir.join(name).join(file).is_file(), "{name}/{file}");
            }
            assert!(dir.join(format!("{name}.stl")).is_file(), "{name}");
        }
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

    #[test]
    fn open_and_vscode_take_their_arguments() {
        let parse = |args: &[&str]| Cli::try_parse_from([&["geop"], args].concat());
        let Command::Open(open) = parse(&["open"]).unwrap().command else {
            panic!("not open")
        };
        assert_eq!(
            (open.path, open.port, open.no_browser),
            (PathBuf::from("."), None, false)
        );
        let Command::Open(open) = parse(&["open", "proj", "--port", "8080", "--no-browser"])
            .unwrap()
            .command
        else {
            panic!("not open")
        };
        assert_eq!(
            (open.path, open.port, open.no_browser),
            (PathBuf::from("proj"), Some(8080), true)
        );
        assert!(parse(&["open", "--port", "70000"]).is_err());
        let Command::Vscode(vscode) = parse(&["vscode"]).unwrap().command else {
            panic!("not vscode")
        };
        assert_eq!(vscode.editor, None);
        let Command::Vscode(vscode) = parse(&["vscode", "--editor", "cursor"]).unwrap().command
        else {
            panic!("not vscode")
        };
        assert_eq!(vscode.editor.as_deref(), Some("cursor"));
    }

    #[test]
    fn today_is_a_date() {
        let today = today();
        assert_eq!(today.len(), 10, "{today}");
        assert_eq!(today.matches('-').count(), 2, "{today}");
    }
}
