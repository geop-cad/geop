//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const DRAFT: &str = r#"<path d="M3 16 L17 16 L14 4 L6 4 Z"/><path d="M6 4 L3 4 L3 16" stroke-dasharray="1.5 1.5"/>"#;
pub(crate) const LIP: &str = r#"<path d="M3 16 L3 8 L9 8 L9 4 L12 4 L12 8 L17 8 L17 16"/>"#;
pub(crate) const GROOVE: &str = r#"<path d="M3 16 L3 6 L8 6 L8 10 L12 10 L12 6 L17 6 L17 16"/>"#;
pub(crate) const RIB: &str =
    r#"<path d="M3 4 L3 16 L17 16 L17 4"/><path d="M9 16 L9 8 L11 8 L11 16"/>"#;
