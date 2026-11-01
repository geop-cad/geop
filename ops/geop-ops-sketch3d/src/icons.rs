//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const ADD_SKETCH3D: &str = r#"<path d="M3 16 L8 13 L8 6 L16 3"/><path d="M3 16 L3 11 M3 16 L7 17" stroke-dasharray="1.2 1.2"/><circle cx="8" cy="13" r="1.4" fill="currentColor" stroke="none"/><circle cx="8" cy="6" r="1.4" fill="currentColor" stroke="none"/>"#;

pub(crate) const HELIX: &str = r#"<path d="M5 15.5 C9 18 15 17 15 14.5 C15 12.5 5 13 5 10.5 C5 8.5 15 9 15 6.5 C15 4 9 2.5 5 4.5"/>"#;
