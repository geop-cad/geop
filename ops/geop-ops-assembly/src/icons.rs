//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_program::operation::OperationInfo::icon`]).

pub(crate) const ADD_PART: &str = r#"<path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z"/><path d="M10 10 L16 6.5 M10 10 L4 6.5 M10 10 L10 17"/><path d="M13 1.5 L13 4.8" stroke-width="1"/>"#;
pub(crate) const PART_PATTERN: &str = r#"<circle cx="10" cy="10" r="6.5" stroke-dasharray="1.5 1.5" stroke-width="1"/><circle cx="10" cy="3.5" r="1.6" fill="currentColor" stroke="none"/><circle cx="15.6" cy="13.2" r="1.6" fill="currentColor" stroke="none"/><circle cx="4.4" cy="13.2" r="1.6" fill="currentColor" stroke="none"/>"#;
