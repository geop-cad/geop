//! A workspace on disk: the folder of programs and data files an editor
//! edits ([`Folder`]), the folder a path names ([`workspace_of`]), and
//! where what the files are made into is kept beside them ([`DiskCache`]).
//! What `geop open` serves to a browser and the desktop app edits alike.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::cache::{CACHE_EXTENSION, Cache, CacheStore};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

/// The most files, and bytes in all, of a folder the editor is sent: a
/// folder much larger is no workspace, but a home directory opened by
/// mistake.
pub const MOST_FILES: usize = 10_000;
pub const MOST_BYTES: u64 = 512 << 20;

/// The folder edited: the files under it of the extensions the explorer
/// shows — programs, and the data files operations read (see
/// `OperationInfo::reads`).
pub struct Folder {
    /// Canonical: what every file's own canonical path must start with.
    root: PathBuf,
    /// In lower case, `geop` among them.
    extensions: Vec<String>,
}

impl Folder {
    pub fn new(root: &Path, extensions: Vec<String>) -> GeopResult<Self> {
        let root = std::fs::canonicalize(root)
            .map_err(|e| GeopError::new(format!("opening {}: {e}", root.display())))?;
        if !root.is_dir() {
            return Err(GeopError::new(format!("{} is no folder", root.display())));
        }
        Ok(Self { root, extensions })
    }

    /// The folder, canonical.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Whether `path` — relative, `/`-separated — can name a file of the
    /// folder: no empty, `.` or `..` segment, nothing a platform reads as
    /// a root, a drive or a stream (`/`, `\`, `:`), and an extension shown.
    pub fn names_file(&self, path: &str) -> bool {
        let segments_ok = (path.split('/'))
            .all(|s| !s.is_empty() && s != "." && s != ".." && !s.contains(['\\', ':', '\0']));
        let lower = path.to_ascii_lowercase();
        let extension_ok = (self.extensions.iter()).any(|e| lower.ends_with(&format!(".{e}")));
        segments_ok && extension_ok
    }

    /// The file `path` names under the folder — checked, before it is read
    /// or written, to stay in it: where it is already, its own canonical
    /// path; where it is not yet, that of the nearest folder above it
    /// that is. A link out of the folder is refused like `..`.
    pub fn file(&self, path: &str) -> GeopResult<PathBuf> {
        if !self.names_file(path) {
            return Err(GeopError::new(format!(
                "{path:?} names no file of the workspace"
            )));
        }
        let file = path.split('/').fold(self.root.clone(), |p, s| p.join(s));
        let existing = (file.ancestors())
            .find(|p| p.symlink_metadata().is_ok())
            .ok_or_else(|| GeopError::new(format!("{path:?}: nothing of it exists")))?;
        let canonical = std::fs::canonicalize(existing)
            .map_err(|e| GeopError::new(format!("{path:?}: {e}")))?;
        if !canonical.starts_with(&self.root) {
            return Err(GeopError::new(format!(
                "{path:?} lies outside the workspace"
            )));
        }
        Ok(file)
    }

    /// Every file of the folder, by path, its content as a workspace holds
    /// it: its text, or a `data:` URL of bytes that are not (see
    /// [`geop_core_program::workspace::content`]). Hidden folders, `node_modules` and
    /// `target` are left out, and so is a link out of the folder.
    pub fn files(&self) -> GeopResult<BTreeMap<String, String>> {
        let mut files = BTreeMap::new();
        let mut bytes = 0;
        let mut folders = vec![(self.root.clone(), String::new())];
        while let Some((dir, prefix)) = folders.pop() {
            let entries = std::fs::read_dir(&dir)
                .map_err(|e| GeopError::new(format!("listing {}: {e}", dir.display())))?;
            for entry in entries.flatten() {
                let Some(name) = entry.file_name().to_str().map(String::from) else {
                    continue;
                };
                let path = format!("{prefix}{name}");
                let Ok(kind) = entry.file_type() else {
                    continue;
                };
                if kind.is_dir() {
                    if !name.starts_with('.') && name != "node_modules" && name != "target" {
                        folders.push((entry.path(), format!("{path}/")));
                    }
                    continue;
                }
                // A link to a file is followed only within the folder; to
                // a folder, never (it could lead round in a circle).
                let Ok(file) = self.file(&path) else { continue };
                if !file.is_file() {
                    continue;
                }
                let text = std::fs::read(&file)
                    .map_err(|e| GeopError::new(format!("reading {}: {e}", file.display())))?;
                bytes += text.len() as u64;
                files.insert(path, geop_core_program::workspace::content(text));
                if files.len() > MOST_FILES || bytes > MOST_BYTES {
                    return Err(GeopError::new(format!(
                        "{} holds more than {MOST_FILES} files or {} MB of programs and data: open a folder of one project",
                        self.root.display(),
                        MOST_BYTES >> 20
                    )));
                }
            }
        }
        Ok(files)
    }

    /// Write the file `path`, its content `content` as a workspace holds
    /// it (see [`Folder::files`]): its bytes, the decoded ones of a `data:`
    /// URL. Written whole, then put in place: a reader meanwhile sees the
    /// old file or the new, never half of one.
    pub fn write(&self, path: &str, content: &str) -> GeopResult<()> {
        let text = geop_core_program::workspace::bytes(content)?;
        let file = self.file(path)?;
        let io = |e: std::io::Error| GeopError::new(format!("writing {}: {e}", file.display()));
        let dir = file.parent().expect("a file of the folder is in it");
        std::fs::create_dir_all(dir).map_err(io)?;
        let name = file
            .file_name()
            .expect("a file has a name")
            .to_string_lossy();
        let partial = dir.join(format!(".{name}.{}.partial", std::process::id()));
        let written =
            std::fs::write(&partial, &text).and_then(|()| std::fs::rename(&partial, &file));
        if written.is_err() {
            let _ = std::fs::remove_file(&partial);
        }
        written.map_err(io)
    }

    /// Remove the file `path`; one already gone is no error.
    pub fn remove(&self, path: &str) -> GeopResult<()> {
        let file = self.file(path)?;
        match std::fs::remove_file(&file) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => {
                Err(GeopError::new(format!("removing {}: {e}", file.display())))
            }
            _ => Ok(()),
        }
    }
}


/// The folder `path` names, and the program in it to edit first if it
/// names one.
pub fn workspace_of(path: &Path) -> GeopResult<(PathBuf, Option<String>)> {
    if path.is_dir() {
        return Ok((path.to_path_buf(), None));
    }
    let is_program = path.extension().is_some_and(|e| e == "geop");
    let name = path.file_name().and_then(|n| n.to_str());
    match (is_program && path.is_file(), name) {
        (true, Some(name)) => {
            let dir = path.parent().filter(|p| !p.as_os_str().is_empty());
            Ok((
                dir.unwrap_or(Path::new(".")).to_path_buf(),
                Some(name.to_string()),
            ))
        }
        _ => Err(GeopError::new(format!(
            "{} is neither a folder nor a .geop program",
            path.display()
        ))),
    }
}

/// Cache files on disk: `path.geopc` under `root` for the file `path`, of
/// this build of the kernel (see `geop_core_program::cache`).
pub struct DiskCache {
    root: PathBuf,
    build: String,
}

impl DiskCache {
    /// The cache files under `root`, of the executable running: its version,
    /// and its size and time on disk, so that a kernel built again makes
    /// again what an older one kept. None if the executable cannot be told
    /// apart: nothing is then kept on disk.
    pub fn of(root: PathBuf) -> Option<Self> {
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
    pub fn cache(this: Option<Self>) -> Cache {
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


/// The extensions of the files an editor of `registry` shows in a
/// workspace, in lower case: programs (`geop`), and the data files its
/// operations read (see `OperationInfo::reads`).
pub fn extensions(registry: &geop_core_editor::registry::Registry) -> Vec<String> {
    let mut extensions: Vec<String> = vec!["geop".into()];
    for listed in registry.infos() {
        extensions.extend(listed.info.reads.iter().map(|e| e.to_ascii_lowercase()));
    }
    extensions.sort();
    extensions.dedup();
    extensions
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("geop-disk-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn folder(dir: &Path) -> Folder {
        Folder::new(dir, vec!["geop".into(), "step".into(), "stp".into()]).unwrap()
    }

    #[test]
    fn a_path_names_a_file_of_the_folder_only_relatively_and_by_a_shown_extension() {
        let dir = scratch("names");
        let f = folder(&dir);
        for ok in ["part.geop", "parts/bolt.geop", "a b/c.STEP", "x.stp"] {
            assert!(f.names_file(ok), "{ok}");
        }
        for bad in [
            "",
            "/etc/passwd.geop",
            "../up.geop",
            "a/../../up.geop",
            "./part.geop",
            "a//b.geop",
            "a\\..\\b.geop",
            "C:/x.geop",
            "c:x.geop",
            "part.geop:stream",
            "notes.txt",
            "part.geop/",
            "part.geopc",
        ] {
            assert!(!f.names_file(bad), "{bad}");
            assert!(f.file(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn bytes_that_are_no_text_travel_as_a_data_url() {
        let dir = scratch("bytes");
        let f = folder(&dir);
        let content = geop_core_program::workspace::content(vec![0xff, 0x00, 0x80]);
        f.write("mesh.step", &content).unwrap();
        assert_eq!(std::fs::read(dir.join("mesh.step")).unwrap(), [0xff, 0x00, 0x80]);
        assert_eq!(f.files().unwrap()["mesh.step"], content);
    }

    #[test]
    fn files_are_read_and_written_in_the_folder() {
        let dir = scratch("rw");
        let f = folder(&dir);
        f.write("parts/bolt.geop", "{\"steps\": []}\n").unwrap();
        f.write("part.geop", "{}").unwrap();
        std::fs::write(dir.join("notes.txt"), "not shown").unwrap();
        std::fs::create_dir_all(dir.join(".git")).unwrap();
        std::fs::write(dir.join(".git/hidden.geop"), "{}").unwrap();
        let files = f.files().unwrap();
        assert_eq!(
            files.keys().collect::<Vec<_>>(),
            ["part.geop", "parts/bolt.geop"]
        );
        assert_eq!(files["parts/bolt.geop"], "{\"steps\": []}\n");
        f.remove("part.geop").unwrap();
        f.remove("part.geop").unwrap();
        assert!(!dir.join("part.geop").exists());
        // No partial file is left behind.
        let left: Vec<_> = std::fs::read_dir(dir.join("parts")).unwrap().collect();
        assert_eq!(left.len(), 1);
    }

    #[cfg(unix)]
    #[test]
    fn a_link_out_of_the_folder_is_refused() {
        let dir = scratch("links");
        let outside = scratch("links-outside");
        std::fs::write(outside.join("secret.geop"), "secret").unwrap();
        let root = dir.join("root");
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join("inside.geop"), "inside").unwrap();
        std::os::unix::fs::symlink(&outside, root.join("out")).unwrap();
        std::os::unix::fs::symlink(outside.join("secret.geop"), root.join("secret.geop")).unwrap();
        std::os::unix::fs::symlink(root.join("inside.geop"), root.join("alias.geop")).unwrap();
        let f = folder(&root);
        assert!(f.file("out/secret.geop").is_err());
        assert!(f.file("out/new/new.geop").is_err());
        assert!(f.file("secret.geop").is_err());
        assert!(f.write("out/written.geop", "x").is_err());
        assert!(!outside.join("written.geop").exists());
        assert!(f.write("secret.geop", "x").is_err());
        assert_eq!(
            std::fs::read_to_string(outside.join("secret.geop")).unwrap(),
            "secret"
        );
        assert!(f.remove("secret.geop").is_err());
        assert!(outside.join("secret.geop").exists());
        // Listed: what is in the folder, a link within it included.
        let files = f.files().unwrap();
        assert_eq!(
            files.keys().collect::<Vec<_>>(),
            ["alias.geop", "inside.geop"]
        );
    }

    #[test]
    fn a_program_named_is_opened_in_its_folder() {
        let dir = scratch("workspace-of");
        std::fs::write(dir.join("a.geop"), "{}").unwrap();
        assert_eq!(workspace_of(&dir).unwrap(), (dir.clone(), None));
        assert_eq!(
            workspace_of(&dir.join("a.geop")).unwrap(),
            (dir.clone(), Some("a.geop".into()))
        );
        assert!(workspace_of(&dir.join("missing.geop")).is_err());
        assert!(workspace_of(&dir.join("missing")).is_err());
    }
}
