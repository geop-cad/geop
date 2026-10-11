//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_program::operation::OperationInfo::icon`]).

pub(crate) const DELETE_BODY: &str = r#"<path d="M4 6 L16 6 M8 6 L8 4 L12 4 L12 6"/><path d="M5.5 6 L6.5 16.5 L13.5 16.5 L14.5 6"/>"#;
pub(crate) const EXTRACT_FACE: &str = r#"<path d="M3 10 L8 12.5 L13 10 L8 7.5 Z" stroke-dasharray="1.5 1.5"/><path d="M7 5.5 L12 8 L17 5.5 L12 3 Z"/>"#;
pub(crate) const PROJECT_CURVE: &str = r#"<path d="M4 4 C7 2 10 6 14 3.5"/><path d="M3 13 L10 16.5 L17 13 L10 9.5 Z"/><path d="M9 5 L9 11.5 M7.5 10 L9 11.5 L10.5 10" stroke-width="1"/>"#;
