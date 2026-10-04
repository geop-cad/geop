//! The corpus test: every STEP file `scripts/fetch_corpus.sh` downloaded,
//! imported and validated, with a table of what passed and why the rest
//! failed.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    time::Instant,
};

use geop_core_math::scalars::scal_in_f64::ScalInF64;
use geop_core_topology::validation::{ValidationParameters, validate};
use geop_ops::{Namer, Part};

use crate::{add_bodies, read_step};

type S = ScalInF64;

fn corpus_dir() -> PathBuf {
    std::env::var_os("STEP_CORPUS")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/step-corpus"))
}

fn step_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            step_files(&path, out);
        } else if crate::is_step_file(&path.to_string_lossy()) {
            out.push(path);
        }
    }
}

/// What kind of failure an error is: its root message with the numbers
/// and entity ids taken out, so alike failures count together.
fn cause(message: &str) -> String {
    let root = message
        .lines()
        .find_map(|l| l.strip_prefix("RootError: "))
        .unwrap_or(message);
    let mut out = String::new();
    let mut last_digit = false;
    for c in root.chars().take(140) {
        if c.is_ascii_digit() {
            if !last_digit {
                out.push('N');
            }
            last_digit = true;
        } else {
            last_digit = false;
            out.push(c);
        }
    }
    out
}

/// Imports one file: `Ok` with its counts of solids/sheets and faces, or the
/// error.
fn import_file(path: &Path, full: bool) -> Result<(usize, usize), String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let text = String::from_utf8_lossy(&bytes);
    let bodies = read_step::<S>(&text).map_err(|e| format!("{e:?}"))?;
    if bodies.is_empty() {
        return Err("RootError: no bodies".into());
    }
    let n = bodies.len();
    let mut part = Part::new();
    add_bodies(&mut part, &Namer::new("import", "i").unwrap(), bodies).map_err(|e| format!("{e:?}"))?;
    if full && let Err(errors) = validate(&ValidationParameters::default(), part.topology()) {
        return Err(format!("RootError: full validation: {}", cause(&format!("{:?}", errors[0]))));
    }
    Ok((n, part.topology().faces.len()))
}

#[test]
#[ignore = "slow: imports every file of the downloaded STEP corpus (scripts/fetch_corpus.sh) — run with `cargo test -- --ignored`"]
fn corpus() {
    let dir = corpus_dir();
    let mut files = Vec::new();
    step_files(&dir, &mut files);
    files.sort();
    assert!(!files.is_empty(), "no STEP files in {}: run scripts/fetch_corpus.sh", dir.display());
    // The full validation's pairwise searches are slow on big parts: only
    // with STEP_CORPUS_FULL set.
    let full = std::env::var_os("STEP_CORPUS_FULL").is_some();
    let only = std::env::var("STEP_CORPUS_ONLY").ok();
    let mut passed = 0;
    let mut total = 0;
    let mut causes: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for path in &files {
        let name = path.strip_prefix(&dir).unwrap_or(path).to_string_lossy().to_string();
        if only.as_ref().is_some_and(|o| !name.contains(o.as_str())) {
            continue;
        }
        total += 1;
        let start = Instant::now();
        let result = std::panic::catch_unwind(|| import_file(path, full))
            .unwrap_or_else(|_| Err("RootError: panicked".into()));
        let seconds = start.elapsed().as_secs_f64();
        match result {
            Ok((bodies, faces)) => {
                passed += 1;
                println!("PASS {seconds:7.2}s {bodies:3} bodies {faces:5} faces  {name}");
            }
            Err(e) => {
                let c = cause(&e);
                println!("FAIL {seconds:7.2}s  {name}: {c}");
                causes.entry(c).or_default().push(name);
            }
        }
    }
    let mut ranked: Vec<_> = causes.into_iter().collect();
    ranked.sort_by(|a, b| b.1.len().cmp(&a.1.len()));
    println!("\n{passed} of {total} files imported valid ({:.0}%)", 100.0 * passed as f64 / total as f64);
    println!("failure causes:");
    for (cause, files) in &ranked {
        println!("{:4}  {cause}   (e.g. {})", files.len(), files[0]);
    }
}
