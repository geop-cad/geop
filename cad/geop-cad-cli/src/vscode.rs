//! `geop vscode`: install the VS Code extension of this very release —
//! the one whose kernel and page speak this build's protocol — into VS
//! Code, or an editor built on it (Cursor, VSCodium).
//!
//! The extension is downloaded from the release (see [`release`]), its
//! SHA-256 checked, and given to the editor's command line:
//! `code --install-extension <file>`.

use crate::release;
use geop_core_math::geop_error::{GeopError, GeopResult};
use std::{
    ffi::OsStr,
    path::{Path, PathBuf},
    process::Command,
};

#[derive(clap::Args)]
pub struct VscodeArgs {
    /// The editor's command line to install into: `code`, `cursor`,
    /// `codium`, or a path to one. The first of those on PATH, if not
    /// given.
    #[arg(long, value_name = "CMD")]
    pub editor: Option<String>,
}

/// The command lines of the editors an extension of VS Code installs into,
/// in the order they are looked for.
const EDITORS: [&str; 3] = ["code", "cursor", "codium"];

/// The file `name` names among the directories `path` (as `PATH` lists
/// them), with one of the extensions `extensions` (`""` for none): what a
/// shell would run for `name`. A name with a directory in it is taken as
/// it is.
pub fn find_command(name: &str, path: &OsStr, extensions: &[&str]) -> Option<PathBuf> {
    let candidates = |dir: &Path| -> Vec<PathBuf> {
        (extensions.iter())
            .map(|e| dir.join(format!("{name}{e}")))
            .collect()
    };
    if Path::new(name).components().count() > 1 {
        return candidates(Path::new("")).into_iter().find(|p| p.is_file());
    }
    std::env::split_paths(path)
        .flat_map(|dir| candidates(&dir))
        .find(|p| p.is_file())
}

/// The extensions a command is run with here: Windows runs `code.cmd` for
/// `code`.
fn command_extensions() -> Vec<String> {
    if cfg!(windows) {
        let listed = std::env::var("PATHEXT").unwrap_or_else(|_| ".COM;.EXE;.BAT;.CMD".into());
        (listed.split(';'))
            .filter(|e| !e.is_empty())
            .map(|e| e.to_ascii_lowercase())
            .collect()
    } else {
        vec![String::new()]
    }
}

/// The editor command line to install into: `chosen`, or the first of
/// [`EDITORS`] on PATH.
fn editor(chosen: Option<&str>) -> GeopResult<PathBuf> {
    let path = std::env::var_os("PATH").unwrap_or_default();
    let extensions = command_extensions();
    let extensions: Vec<&str> = extensions.iter().map(String::as_str).collect();
    if let Some(chosen) = chosen {
        return find_command(chosen, &path, &extensions)
            .ok_or_else(|| GeopError::new(format!("no command {chosen:?} found on PATH")));
    }
    (EDITORS.iter())
        .find_map(|name| find_command(name, &path, &extensions))
        .ok_or_else(|| {
            GeopError::new(format!(
                "none of the editor command lines {} is on PATH. In VS Code, run \
                 \"Shell Command: Install 'code' command in PATH\" from the command palette, \
                 or pass the editor's command with --editor",
                EDITORS.join(", ")
            ))
        })
}

pub fn install(args: &VscodeArgs) -> GeopResult<()> {
    let editor = editor(args.editor.as_deref())?;
    let version = env!("CARGO_PKG_VERSION");
    let (os, arch) = (std::env::consts::OS, std::env::consts::ARCH);
    let target = release::vscode_target(os, arch).ok_or_else(|| {
        GeopError::new(format!(
            "no geop VS Code extension is released for {os} {arch}"
        ))
    })?;
    let name = release::vsix_name(version, target);
    eprintln!("downloading {name}");
    let vsix = release::download(version, &name)?;

    let dir = std::env::temp_dir().join(format!("geop-vscode-{}", std::process::id()));
    let file = dir.join(&name);
    let written = std::fs::create_dir_all(&dir).and_then(|()| std::fs::write(&file, &vsix));
    let installed = written
        .map_err(|e| GeopError::new(format!("writing {}: {e}", file.display())))
        .and_then(|()| {
            eprintln!("installing it with {}", editor.display());
            let status = Command::new(&editor)
                .arg("--install-extension")
                .arg(&file)
                .arg("--force")
                .status()
                .map_err(|e| GeopError::new(format!("running {}: {e}", editor.display())))?;
            if !status.success() {
                return Err(GeopError::new(format!(
                    "{} --install-extension failed ({status})",
                    editor.display()
                )));
            }
            Ok(())
        });
    let _ = std::fs::remove_dir_all(&dir);
    installed?;
    eprintln!("installed the geop extension {version}: open a .geop file in the editor");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_command_is_found_on_path_with_its_extension() {
        let dir = std::env::temp_dir().join(format!("geop-cli-find-{}", std::process::id()));
        let (a, b) = (dir.join("a"), dir.join("b"));
        std::fs::create_dir_all(&a).unwrap();
        std::fs::create_dir_all(&b).unwrap();
        std::fs::write(b.join("code.cmd"), "").unwrap();
        std::fs::write(a.join("cursor"), "").unwrap();
        let path = std::env::join_paths([&a, &b]).unwrap();
        assert_eq!(
            find_command("code", &path, &[".exe", ".cmd"]),
            Some(b.join("code.cmd"))
        );
        assert_eq!(find_command("code", &path, &[""]), None);
        assert_eq!(find_command("cursor", &path, &[""]), Some(a.join("cursor")));
        // A path is taken as it is, not looked for.
        let given = b.join("code").to_string_lossy().into_owned();
        assert_eq!(
            find_command(&given, OsStr::new(""), &[".cmd"]),
            Some(b.join("code.cmd"))
        );
    }
}
