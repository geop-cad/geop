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
//! `geop bom robot.geop -o bom.csv` writes an assembly's bill of materials
//! (see `geop_ops_bom`); `--indented` lists it by sub-assembly.
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
use geop_cad_base::{Editor, PartOperation, Program, Workspace, stdlib::WithStandardParts};
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
    /// Build a program and write a 2-D drawing of its part — views with
    /// hidden lines, dimensions, a title block — as SVG or DXF.
    Drawing(DrawingCliArgs),
    /// Build an assembly and write it as a URDF robot — links, joints,
    /// inertia and meshes — for simulators: a directory of `robot.urdf`
    /// and `meshes/*.stl`, or one `.zip` of them.
    Urdf(UrdfArgs),
    /// Build an assembly and write its bill of materials as CSV: every
    /// part with its quantity, designation, material and mass, and every
    /// wire of its harnesses cut to length.
    Bom(BomArgs),
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

#[derive(clap::Args)]
struct DrawingCliArgs {
    /// The program to draw, e.g. `part.geop`.
    program: PathBuf,
    /// Where to write the drawing: `.svg` or `.dxf`. Defaults to the
    /// program's path with `.geop` replaced by `.svg`.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// The drawing step to draw, by id. The program's last drawing step if
    /// not given; the default drawing of the whole part if it has none.
    #[arg(long)]
    step: Option<String>,
    /// The views, e.g. `front,top,right,iso` (also `left`, `bottom`,
    /// `back`). The drawing step's if not given.
    #[arg(long, value_delimiter = ',')]
    views: Vec<String>,
    /// Lay the views out in first-angle projection (ISO) rather than third
    /// angle (ASME).
    #[arg(long)]
    first_angle: bool,
    /// The scale, e.g. `1:2` or `5:1`; the largest standard one that fits
    /// if not given.
    #[arg(long)]
    scale: Option<String>,
    /// The paper: `a4` to `a0`, landscape.
    #[arg(long)]
    sheet: Option<String>,
    /// The part's name for the title block; the program's file name if
    /// neither this nor the drawing step gives one.
    #[arg(long)]
    name: Option<String>,
    /// What the part is made of, for the title block.
    #[arg(long)]
    material: Option<String>,
}

#[derive(clap::Args)]
struct UrdfArgs {
    /// The assembly to export, e.g. `robot.geop`.
    program: PathBuf,
    /// Where to write it: a directory — made if missing — or a `.zip`
    /// file. Defaults to the program's path with `.geop` replaced by
    /// `_urdf`, a directory.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// How finely curved faces are meshed: higher is smoother, and bigger.
    #[arg(short, long, default_value_t = DEFAULT_QUALITY, value_parser = clap::value_parser!(u16).range(2..))]
    quality: u16,
}

#[derive(clap::Args)]
struct BomArgs {
    /// The assembly to list, e.g. `robot.geop`.
    program: PathBuf,
    /// Where to write the CSV file. Written to the standard output if not
    /// given.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// List the tree of sub-assemblies, each part counted per one of the
    /// assembly placing it, rather than every part once with its count in
    /// the whole.
    #[arg(long)]
    indented: bool,
}

/// Today's date, `YYYY-MM-DD` (UTC), for a title block.
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

/// A scale as written, `1:2` or `5:1`, as paper length per model length.
fn parse_scale(text: &str) -> GeopResult<f64> {
    let bad = || GeopError::new(format!("the scale {text:?} is not like 1:2 or 5:1"));
    let (a, b) = text.split_once(':').ok_or_else(bad)?;
    let (a, b): (f64, f64) = (
        a.trim().parse().map_err(|_| bad())?,
        b.trim().parse().map_err(|_| bad())?,
    );
    if a > 0.0 && b > 0.0 {
        Ok(a / b)
    } else {
        Err(bad())
    }
}

/// Writes the drawing `args` ask for; returns where.
fn drawing(args: &DrawingCliArgs) -> GeopResult<PathBuf> {
    use geop_ops_drawing::{DrawingArgs, Format, Projection, SheetSize, ViewKind};
    let path = args.program.to_string_lossy();
    let program = Program::from_json(&Disk.read(&path)?)?;
    let workspace = Workspace::<S, Disk>::new(WithStandardParts(Disk));
    let library = workspace.scope(&path);
    let found = match &args.step {
        Some(id) => {
            let index = program.index_of(id)?;
            match &program.steps[index].operation {
                PartOperation::Drawing(d) => Some((index, d.clone())),
                _ => return Err(GeopError::new(format!("the step {id:?} is no drawing"))),
            }
        }
        None => program
            .steps
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, s)| match &s.operation {
                PartOperation::Drawing(d) => Some((i, d.clone())),
                _ => None,
            }),
    };
    let (index, mut spec) = found.unwrap_or((program.steps.len(), DrawingArgs::default()));
    let mut before = program.clone();
    before.steps.truncate(index);
    let part = before.build(&library)?;

    if !args.views.is_empty() {
        spec.views = args
            .views
            .iter()
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
    if args.first_angle {
        spec.projection = Projection::FirstAngle;
    }
    if let Some(scale) = &args.scale {
        spec.scale = Some(parse_scale(scale)?);
    }
    if let Some(sheet) = &args.sheet {
        spec.sheet = SheetSize::ALL
            .into_iter()
            .find(|s| s.name() == sheet.to_ascii_lowercase())
            .ok_or_else(|| GeopError::new(format!("no paper is named {sheet:?}: a4 to a0")))?;
    }
    if let Some(name) = &args.name {
        spec.name = name.clone();
    }
    if spec.name.is_empty() {
        spec.name = args
            .program
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
    }
    if let Some(material) = &args.material {
        spec.material = material.clone();
    }
    let output = args
        .output
        .clone()
        .unwrap_or_else(|| args.program.with_extension("svg"));
    let format = Format::of_path(&output.to_string_lossy()).ok_or_else(|| {
        GeopError::new(format!(
            "{} is neither .svg nor .dxf: name the drawing for the format to write",
            output.display()
        ))
    })?;
    let text = geop_ops_drawing::render(&part, &spec, &today(), format)?;
    std::fs::write(&output, text)
        .map_err(|e| GeopError::new(format!("writing {}: {e}", output.display())))?;
    Ok(output)
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

/// The program at `path`, built, its parts where its mates hold them, and
/// the mates the program's state did not hold. A stale state — a file it
/// places changed since it was saved — is solved for here, not in the
/// file: that is the editor's to write.
fn build(path: &Path) -> GeopResult<(Program, Part<S>, Vec<String>)> {
    let path = path.to_string_lossy();
    // A standard part — `std:iso4032_hex_nut.geop` — builds too.
    let workspace = Workspace::<S, Disk>::new(WithStandardParts(Disk));
    let mut program = Program::from_json(&workspace.files().read(&path)?)?;
    let library = workspace.scope(&path);
    let mut part = program.build(&library)?;
    let report = part.check_mates(|_| true)?;
    if !report.converged {
        let (moved, _) = part.solve_mates(None, &[], &[])?;
        program.state.extend(moved);
        part = program.build(&library)?;
    }
    Ok((program, part, report.failed))
}

/// Writes the robot `args` ask for; returns where, and how many links and
/// joints it has.
fn urdf(args: &UrdfArgs) -> GeopResult<(PathBuf, usize, usize)> {
    let (_, part, _) = build(&args.program)?;
    let name = args
        .program
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "robot".into());
    let robot = geop_ops_urdf::export(&part, &name, usize::from(args.quality))?;
    let output = args.output.clone().unwrap_or_else(|| {
        let mut dir = args.program.with_extension("").into_os_string();
        dir.push("_urdf");
        dir.into()
    });
    let write = |path: &Path, bytes: &[u8]| {
        std::fs::write(path, bytes)
            .map_err(|e| GeopError::new(format!("writing {}: {e}", path.display())))
    };
    if output.extension().is_some_and(|e| e == "zip") {
        write(&output, &robot.zip()?)?;
    } else {
        for (file, bytes) in robot.files() {
            let path = output.join(&file);
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)
                    .map_err(|e| GeopError::new(format!("creating {}: {e}", dir.display())))?;
            }
            write(&path, &bytes)?;
        }
    }
    Ok((output, robot.robot.links.len(), robot.robot.joints.len()))
}

/// The bill of materials `args` ask for: written as CSV where they say,
/// and returned.
fn bom(args: &BomArgs) -> GeopResult<geop_ops_bom::Bom> {
    let (_, part, _) = build(&args.program)?;
    let structure = match args.indented {
        true => geop_ops_bom::Structure::Indented,
        false => geop_ops_bom::Structure::Flat,
    };
    let file = args.program.to_string_lossy();
    let bom = geop_cad_base::inspect::bill_of_materials(&part, &file, structure)?;
    let csv = bom.to_csv();
    match &args.output {
        Some(path) => std::fs::write(path, csv)
            .map_err(|e| GeopError::new(format!("writing {}: {e}", path.display())))?,
        None => std::io::stdout()
            .write_all(csv.as_bytes())
            .map_err(|e| GeopError::new(format!("writing the bill of materials: {e}")))?,
    }
    Ok(bom)
}

fn compile(args: &CompileArgs) -> GeopResult<Compiled> {
    let io_err = |what: &str, path: &Path| {
        let what = what.to_string();
        let path = path.display().to_string();
        move |e: std::io::Error| GeopError::new(format!("{what} {path}: {e}"))
    };
    let (program, part, solved_for) = build(&args.program)?;

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
            solved_for,
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
        solved_for,
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
        Command::Drawing(args) => drawing(&args).map(|output| {
            eprintln!("drawing -> {}", output.display());
        }),
        Command::Urdf(args) => urdf(&args).map(|(output, links, joints)| {
            eprintln!("{links} links, {joints} joints -> {}", output.display());
        }),
        Command::Bom(args) => bom(&args).map(|bom| {
            if let Some(output) = &args.output {
                eprintln!("{} lines -> {}", bom.lines.len(), output.display());
            }
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

    /// The arm, exported as a URDF robot: by default into a directory
    /// beside it — `robot.urdf` and the mesh of its link — and as one ZIP
    /// archive of the same files when asked for a `.zip`.
    #[test]
    fn exports_the_arm_as_urdf() {
        let dir = scratch("urdf");
        let (_, files) = examples::workspaces()
            .into_iter()
            .find(|(name, _)| *name == "arm")
            .unwrap();
        for (file, program) in &files {
            std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
        }
        let args = UrdfArgs {
            program: dir.join("arm.geop"),
            output: None,
            quality: 8,
        };
        let (output, links, joints) = urdf(&args).unwrap();
        assert_eq!(output, dir.join("arm_urdf"));
        assert_eq!((links, joints), (3, 2));
        let text = std::fs::read_to_string(output.join("robot.urdf")).unwrap();
        assert!(text.contains(r#"<robot name="arm">"#), "{text}");
        assert!(output.join("meshes/link.stl").is_file());

        let zip = dir.join("arm.zip");
        urdf(&UrdfArgs {
            output: Some(zip.clone()),
            ..args
        })
        .unwrap();
        let bytes = std::fs::read(zip).unwrap();
        assert!(bytes.starts_with(b"PK\x03\x04"));
        let urdf_at = bytes.windows(10).position(|w| w == b"robot.urdf");
        assert!(urdf_at.is_some());
    }

    /// The bolted plate's bill of materials, written as CSV: its plate,
    /// screw and nut, the standard parts by their norms.
    #[test]
    fn writes_the_bill_of_materials_of_the_bolted_plate() {
        let dir = scratch("bom");
        let (_, files) = examples::workspaces()
            .into_iter()
            .find(|(name, _)| *name == "bolted_plate")
            .unwrap();
        for (file, program) in &files {
            std::fs::write(dir.join(file), program.to_json().unwrap()).unwrap();
        }
        let output = dir.join("bom.csv");
        let args = BomArgs {
            program: dir.join("bolted_plate.geop"),
            output: Some(output.clone()),
            indented: false,
        };
        let listed = bom(&args).unwrap();
        assert_eq!(listed.lines.len(), 3);
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

    /// `geop drawing bracket.geop -o bracket.dxf --views front,top`: the
    /// drawing written in the format its name says, the views asked for.
    #[test]
    fn draws_a_program() {
        let dir = scratch("drawing");
        let path = dir.join("bracket.geop");
        std::fs::write(&path, examples::bracket().to_json().unwrap()).unwrap();
        let args = |output: &str, views: &[&str]| DrawingCliArgs {
            program: path.clone(),
            output: Some(dir.join(output)),
            step: None,
            views: views.iter().map(|v| v.to_string()).collect(),
            first_angle: true,
            scale: Some("2:1".into()),
            sheet: None,
            name: None,
            material: Some("6061-T6".into()),
        };
        let written = drawing(&args("bracket.dxf", &["front", "top"])).unwrap();
        let dxf = std::fs::read_to_string(written).unwrap();
        assert!(dxf.contains("\nAC1009\n") && dxf.ends_with("EOF\n"));
        for text in ["bracket", "6061-T6", "2:1", "FIRST ANGLE"] {
            assert!(dxf.contains(&format!("\n{text}\n")), "{text}");
        }
        let svg = std::fs::read_to_string(drawing(&args("bracket.svg", &[])).unwrap()).unwrap();
        assert!(svg.starts_with("<svg"));
        let err = drawing(&args("bracket.png", &[])).unwrap_err();
        assert!(err.to_string().contains(".svg nor .dxf"), "{err}");
        let err = drawing(&args("bracket.svg", &["side"])).unwrap_err();
        assert!(err.to_string().contains("front"), "{err}");
        assert_eq!(parse_scale("1:5").unwrap(), 0.2);
        assert_eq!(today().len(), 10);
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
