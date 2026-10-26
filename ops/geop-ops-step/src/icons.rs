//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const IMPORT_STEP: &str = r#"<path d="M4 3 L12 3 L16 7 L16 17 L4 17 Z"/><path d="M12 3 L12 7 L16 7" stroke-width="1"/><path d="M10 8 L10 14 M7.5 11.5 L10 14 L12.5 11.5"/>"#;
pub(crate) const IMPORT_STL: &str = r#"<path d="M3 16 L10 4 L17 16 Z"/><path d="M10 4 L10 16 M3 16 L10 11 L17 16" stroke-width="1"/>"#;
