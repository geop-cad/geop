//! Icons: the kernel's — an operation's, an inspector's, a tool's, a
//! dialog action's — sent as SVG on a 20 × 20 grid drawn in the current
//! colour, and the app's own on the same grid. Each is drawn white and
//! tinted as its button is, so a button's state colours its icon too.

use std::hash::{Hash, Hasher};

use eframe::egui::{self, Color32, Image, Vec2};

/// The app's own icons, by name (as `web/src/icons.tsx` draws them).
fn own(name: &str) -> Option<&'static str> {
    Some(match name {
        "pick" => r#"<path d="M5 3.5 L5 15 L8.2 12 L10.5 16.5 L12.5 15.5 L10.3 11 L14.5 11 Z"/>"#,
        "undo" => {
            r#"<path d="M7 5 L3.5 8.5 L7 12"/><path d="M3.5 8.5 L12 8.5 A4.5 4.5 0 0 1 12 17.5 L9 17.5"/>"#
        }
        "redo" => {
            r#"<path d="M13 5 L16.5 8.5 L13 12"/><path d="M16.5 8.5 L8 8.5 A4.5 4.5 0 0 0 8 17.5 L11 17.5"/>"#
        }
        "save" => {
            r#"<path d="M10 3.5 L10 12.5 M6.5 9 L10 12.5 L13.5 9"/><path d="M4 13.5 L4 16.5 L16 16.5 L16 13.5"/>"#
        }
        "open" => {
            r#"<path d="M3 6 L3 15.5 L16 15.5 L17.5 8.5 L6 8.5 L4.5 15.5"/><path d="M3 6 L3 4.5 L7.5 4.5 L9 6 L14.5 6 L14.5 8.5"/>"#
        }
        "new" => {
            r#"<path d="M5 3 L12 3 L15.5 6.5 L15.5 17 L5 17 Z"/><path d="M12 3 L12 6.5 L15.5 6.5"/><path d="M10.2 9 L10.2 14 M7.7 11.5 L12.7 11.5"/>"#
        }
        "example" => {
            r#"<path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z"/><path d="M4 6.5 L10 10 L16 6.5 M10 10 L10 17"/>"#
        }
        "section" => {
            r#"<path d="M3 7 L10 3.5 L17 7 L17 13 L10 16.5 L3 13 Z" stroke-dasharray="1.6 1.6"/><path d="M3 10 L17 10" stroke-width="2"/>"#
        }
        "check" => r#"<path d="M4.5 10.5 L8.5 14.5 L15.5 6"/>"#,
        "chevron-down" => r#"<path d="M6 8 L10 12 L14 8"/>"#,
        "chevron-right" => r#"<path d="M8 6 L12 10 L8 14"/>"#,
        "chevron-up" => r#"<path d="M6 12 L10 8 L14 12"/>"#,
        "plus" => r#"<path d="M10 4.5 L10 15.5 M4.5 10 L15.5 10"/>"#,
        "minus" => r#"<path d="M4.5 10 L15.5 10"/>"#,
        "dot" => r#"<circle cx="10" cy="10" r="3.5" fill="currentColor" stroke="none"/>"#,
        "search" => r#"<circle cx="8.5" cy="8.5" r="5"/><path d="M12.2 12.2 L16.5 16.5"/>"#,
        "eye" => {
            r#"<path d="M2 10 C5 4.5, 15 4.5, 18 10 C15 15.5, 5 15.5, 2 10 Z"/><circle cx="10" cy="10" r="2.5" fill="currentColor"/>"#
        }
        "eye-shut" => {
            r#"<path d="M2 10 C5 4.5, 15 4.5, 18 10 C15 15.5, 5 15.5, 2 10 Z"/><path d="M3 17 L17 3"/>"#
        }
        "copy" => {
            r#"<rect x="6.5" y="6.5" width="9" height="10" rx="1.2"/><path d="M4.5 13.5 L4.5 4.5 A1 1 0 0 1 5.5 3.5 L12.5 3.5"/>"#
        }
        "close" => r#"<path d="M5 5 L15 15 M15 5 L5 15"/>"#,
        "folder" => r#"<path d="M2.5 5 L2.5 15.5 L17.5 15.5 L17.5 7 L9.5 7 L8 5 Z"/>"#,
        "file" => {
            r#"<path d="M5 2.5 L12 2.5 L15.5 6 L15.5 17.5 L5 17.5 Z"/><path d="M12 2.5 L12 6 L15.5 6"/>"#
        }
        "fit" => {
            r#"<path d="M3 7 L3 3 L7 3 M13 3 L17 3 L17 7 M17 13 L17 17 L13 17 M7 17 L3 17 L3 13"/><rect x="7" y="7" width="6" height="6"/>"#
        }
        "home" => r#"<path d="M3 10 L10 3.5 L17 10"/><path d="M5 8.5 L5 16.5 L15 16.5 L15 8.5"/>"#,
        "sun" => {
            r#"<circle cx="10" cy="10" r="3.5"/><path d="M10 2 L10 4 M10 16 L10 18 M2 10 L4 10 M16 10 L18 10 M4.3 4.3 L5.7 5.7 M14.3 14.3 L15.7 15.7 M4.3 15.7 L5.7 14.3 M14.3 5.7 L15.7 4.3"/>"#
        }
        "moon" => r#"<path d="M15.5 12.5 A6.5 6.5 0 1 1 8 3 A5 5 0 0 0 15.5 12.5 Z"/>"#,
        "command" => {
            r#"<rect x="3" y="4" width="14" height="12" rx="2"/><path d="M6 8 L8.5 10 L6 12 M10 12.5 L14 12.5"/>"#
        }
        "perspective" => r#"<path d="M4 6 L16 3 L16 17 L4 14 Z"/><path d="M4 10 L16 10"/>"#,
        "orthographic" => r#"<rect x="4" y="4" width="12" height="12"/><path d="M4 10 L16 10"/>"#,
        "upload" => {
            r#"<path d="M10 13 L10 4 M6.5 7.5 L10 4 L13.5 7.5"/><path d="M4 13.5 L4 16.5 L16 16.5 L16 13.5"/>"#
        }
        "rename" => r#"<path d="M4 16 L4 13 L13 4 L16 7 L7 16 Z"/>"#,
        "suppress" => r#"<circle cx="10" cy="10" r="6"/><path d="M5.8 14.2 L14.2 5.8"/>"#,
        "trash" => {
            r#"<path d="M4 6 L16 6 M8 6 L8 4 L12 4 L12 6 M5.5 6 L6.5 17 L13.5 17 L14.5 6"/>"#
        }
        _ => return None,
    })
}

/// The SVG of `icon` — the kernel's markup, starting with `<`, or the name
/// of one of the app's own — drawn white on the 20 × 20 grid.
fn svg(icon: &str) -> Option<String> {
    let markup = if icon.starts_with('<') {
        icon
    } else {
        own(icon)?
    };
    Some(format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" width="40" height="40" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" color="white" style="color: white">{}</svg>"#,
        markup.replace("currentColor", "white")
    ))
}

/// The icon `icon` as an image `size` points square, tinted `tint`.
pub fn image(icon: &str, size: f32, tint: Color32) -> Option<Image<'static>> {
    let svg = svg(icon)?;
    let mut hasher = std::hash::DefaultHasher::new();
    svg.hash(&mut hasher);
    let uri = format!("bytes://geop-icon-{:016x}.svg", hasher.finish());
    Some(
        Image::from_bytes(uri, svg.into_bytes())
            .fit_to_exact_size(Vec2::splat(size))
            .tint(tint),
    )
}

/// A small button of the icon `icon` alone, `tip` said while pointed at.
pub fn button(ui: &mut egui::Ui, icon: &str, tip: &str) -> egui::Response {
    let tint = ui.visuals().text_color().gamma_multiply(0.8);
    match image(icon, 14.0, tint) {
        Some(image) => ui
            .add(egui::Button::image(image).frame(false).small())
            .on_hover_text(tip),
        None => ui.small_button(icon).on_hover_text(tip),
    }
}

/// A small button of the icon `icon` alone, enabled or not.
pub fn button_enabled(ui: &mut egui::Ui, enabled: bool, icon: &str, tip: &str) -> egui::Response {
    let tint = ui.visuals().text_color().gamma_multiply(0.8);
    match image(icon, 14.0, tint) {
        Some(image) => (ui.add_enabled(enabled, egui::Button::image(image).frame(false).small()))
            .on_hover_text(tip),
        None => ui
            .add_enabled(enabled, egui::Button::new(icon).small())
            .on_hover_text(tip),
    }
}

/// The icon `icon` in `ui`, `size` points square, in the text's colour.
pub fn show(ui: &mut egui::Ui, icon: &str, size: f32) -> Option<egui::Response> {
    let tint = ui.visuals().text_color();
    image(icon, size, tint).map(|i| ui.add(i))
}
