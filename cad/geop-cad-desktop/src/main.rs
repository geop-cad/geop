//! `geop-desktop [FOLDER | PROGRAM.geop]`: geop's desktop app, editing the
//! files of a folder — the one named, the program's, or the current one.

fn main() -> std::process::ExitCode {
    let path = (std::env::args_os().nth(1))
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| ".".into());
    match geop_cad_desktop::ui::run(path) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("geop-desktop: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}
