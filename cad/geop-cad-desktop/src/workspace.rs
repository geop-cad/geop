//! The files the app edits: a folder on disk ([`geop_cad_base::disk::Folder`],
//! as `geop open` serves one), its programs and the data files programs
//! read, by path — `/`-separated, relative to the folder — and the one
//! edited. What the editor writes of a program is written back to its file
//! at once, as the web app keeps its files.

use std::{collections::BTreeMap, path::Path};

use geop_cad_base::disk::Folder;
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::program::Program;

/// A program as a file's text: one step per object, indented.
pub fn program_text(program: &Program) -> String {
    let mut text = serde_json::to_string_pretty(program).expect("a program is JSON");
    text.push('\n');
    text
}

/// The program a file's text holds, an empty file being an empty program.
pub fn parse_program(text: &str) -> GeopResult<Program> {
    if text.trim().is_empty() {
        return Ok(Program::default());
    }
    serde_json::from_str(text).map_err(|e| GeopError::new(format!("not a geop program: {e}")))
}

/// Whether `path` names a program: a `.geop` file.
pub fn is_program(path: &str) -> bool {
    path.ends_with(".geop")
}

/// What a program file is called, without its folder and extension.
pub fn program_name(path: &str) -> &str {
    let file = path.rsplit('/').next().unwrap_or(path);
    file.strip_suffix(".geop").unwrap_or(file)
}

/// The folder `path` is in, with its `/`: `""` at the top.
pub fn folder_of(path: &str) -> &str {
    path.rfind('/').map_or("", |i| &path[..=i])
}

/// `wanted`, or — if a file of that path is taken — the first of
/// `name 2.ext`, `name 3.ext`, … that is free.
pub fn free_path(taken: &BTreeMap<String, String>, wanted: &str) -> String {
    if !taken.contains_key(wanted) {
        return wanted.to_string();
    }
    let (stem, extension) = match wanted.rfind('.') {
        Some(i) if i > folder_of(wanted).len() => (&wanted[..i], &wanted[i..]),
        _ => (wanted, ""),
    };
    (2..)
        .map(|n| format!("{stem} {n}{extension}"))
        .find(|p| !taken.contains_key(p))
        .expect("some number is free")
}

/// `wanted`, or the first of `wanted 2`, `wanted 3`, … no file is in.
pub fn free_folder(taken: &BTreeMap<String, String>, wanted: &str) -> String {
    let used = |folder: &str| taken.keys().any(|p| p.starts_with(&format!("{folder}/")));
    if !used(wanted) {
        return wanted.to_string();
    }
    (2..)
        .map(|n| format!("{wanted} {n}"))
        .find(|f| !used(f))
        .expect("some number is free")
}

/// The files of a folder, and the program edited.
pub struct Workspace {
    folder: Folder,
    /// Each file's content, by path (see [`geop_core_program::workspace::content`]).
    pub files: BTreeMap<String, String>,
    pub active: String,
}

impl Workspace {
    /// The workspace of the folder `dir` (see `geop_cad_base::disk::workspace_of`),
    /// showing files of `extensions`, editing `active` — or the first
    /// program, or a new `part.geop` if it has none.
    pub fn open(dir: &Path, extensions: Vec<String>, active: Option<String>) -> GeopResult<Self> {
        let folder = Folder::new(dir, extensions)?;
        let files = folder.files()?;
        let first = || files.keys().find(|p| is_program(p)).cloned();
        let active = active
            .filter(|a| files.contains_key(a))
            .or_else(first)
            .unwrap_or_else(|| "part.geop".to_string());
        let mut workspace = Self {
            folder,
            files,
            active,
        };
        if !workspace.files.contains_key(&workspace.active) {
            let path = workspace.active.clone();
            workspace.write(&path, &program_text(&Program::default()))?;
        }
        Ok(workspace)
    }

    pub fn root(&self) -> &Path {
        self.folder.root()
    }

    /// Keep `content` as the file `path`, on disk too — unless it holds
    /// that already.
    pub fn write(&mut self, path: &str, content: &str) -> GeopResult<()> {
        if self.files.get(path).is_some_and(|c| c == content) {
            return Ok(());
        }
        self.folder.write(path, content)?;
        self.files.insert(path.to_string(), content.to_string());
        Ok(())
    }

    pub fn remove(&mut self, path: &str) -> GeopResult<()> {
        self.folder.remove(path)?;
        self.files.remove(path);
        Ok(())
    }

    /// Read the folder again: what changed outside the app, by path, each
    /// set to its content now, or — `None` — gone.
    pub fn reread(&mut self) -> GeopResult<BTreeMap<String, Option<String>>> {
        let now = self.folder.files()?;
        let mut changed: BTreeMap<String, Option<String>> = BTreeMap::new();
        for (path, content) in &now {
            if self.files.get(path) != Some(content) {
                changed.insert(path.clone(), Some(content.clone()));
            }
        }
        for path in self.files.keys() {
            if !now.contains_key(path) {
                changed.insert(path.clone(), None);
            }
        }
        self.files = now;
        Ok(changed)
    }

    /// Whether `path` can name a file of the folder.
    pub fn names_file(&self, path: &str) -> bool {
        self.folder.names_file(path)
    }

    /// The programs, by path.
    pub fn programs(&self) -> impl Iterator<Item = &str> {
        self.files
            .keys()
            .map(String::as_str)
            .filter(|p| is_program(p))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_taken_path_is_numbered() {
        let taken = BTreeMap::from([
            ("part.geop".to_string(), String::new()),
            ("part 2.geop".to_string(), String::new()),
            ("examples/gear/a.geop".to_string(), String::new()),
        ]);
        assert_eq!(free_path(&taken, "part.geop"), "part 3.geop");
        assert_eq!(free_path(&taken, "new.geop"), "new.geop");
        assert_eq!(free_folder(&taken, "examples/gear"), "examples/gear 2");
        assert_eq!(program_name("parts/bolt.geop"), "bolt");
        assert_eq!(folder_of("parts/bolt.geop"), "parts/");
    }

    #[test]
    fn a_program_reads_back_as_written() {
        let program = parse_program("").unwrap();
        assert_eq!(parse_program(&program_text(&program)).unwrap(), program);
        assert!(parse_program("{ nope").is_err());
    }
}
