//! The files around a program, as a field picking one offers them: read
//! through the runner running the program (see
//! [`geop_core_program::workspace`]).

use geop_core_program::program_runner::ProgramRunner;

/// What a field picking a file `runner`'s program reads offers: the files
/// of it `kind` takes, sorted, and `current`, if a file is picked that is
/// not among them. None, without files.
pub fn file_choices(
    runner: &ProgramRunner,
    kind: impl Fn(&str) -> bool,
    current: &str,
) -> Vec<String> {
    let mut choices: Vec<String> = (runner.files().into_iter()).filter(|f| kind(f)).collect();
    choices.sort();
    if !current.is_empty() && !choices.iter().any(|f| f == current) {
        choices.push(current.to_string());
    }
    choices
}
