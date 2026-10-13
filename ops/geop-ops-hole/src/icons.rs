//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const HOLE: &str = r#"<path d="M2.5 6 L17.5 6 M2.5 6 L2.5 17 M17.5 6 L17.5 17"/><path d="M5 6 L5 9 L7.5 9 L7.5 17 M15 6 L15 9 L12.5 9 L12.5 17"/><path d="M10 2 L10 17" stroke-width="1" stroke-dasharray="1.5 1.5"/>"#;
pub(crate) const THREAD: &str = r#"<path d="M6 3 L6 17 M14 3 L14 17"/><path d="M6 5 L14 7 M6 9 L14 11 M6 13 L14 15" stroke-width="1"/>"#;
