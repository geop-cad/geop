//! The files around a program, as a field picking one offers them: read
//! through the runner running the program (see
//! [`geop_core_program::workspace`]).

use geop_core_editor::editing::FileEntry;
use geop_core_program::runner::Runner;

/// What a field picking a file `runner`'s program reads offers: every file
/// it could name, sorted, those of the kind `kind` takes accepted. None,
/// without files.
pub fn file_choices(runner: &Runner, kind: impl Fn(&str) -> bool) -> Vec<FileEntry> {
    let mut paths = runner.files();
    paths.sort();
    (paths.into_iter())
        .map(|path| FileEntry {
            accepted: kind(&path),
            path,
        })
        .collect()
}
