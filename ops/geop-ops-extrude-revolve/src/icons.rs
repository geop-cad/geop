//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const EXTRUDE: &str = r#"<path d="M4 12 L10 15 L16 12 L10 9 Z"/><path d="M10 8 L10 2.5 M7.8 4.7 L10 2.5 L12.2 4.7"/>"#;
pub(crate) const REVOLVE: &str = r#"<path d="M10 2.5 L10 17.5" stroke-dasharray="1.5 1.5"/><path d="M15.5 10 A5.5 2.5 0 1 1 13 7.9"/><path d="M12 6 L13.4 8 L11.2 8.8"/>"#;
pub(crate) const SWEEP: &str = r#"<path d="M3 15 C3 9 7 6 11 6 L16.5 6" stroke-dasharray="1.5 1.5"/><ellipse cx="3" cy="15" rx="1.8" ry="2.5"/><path d="M14.5 3.8 L16.8 6 L14.5 8.2"/>"#;
pub(crate) const LOFT: &str = r#"<rect x="4" y="12" width="12" height="5"/><ellipse cx="10" cy="4.5" rx="3.5" ry="1.8"/><path d="M4 12 L6.5 4.5 M16 12 L13.5 4.5"/>"#;
