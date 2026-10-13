//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const BOOLEAN: &str =
    r#"<rect x="3" y="3" width="9" height="9"/><rect x="8" y="8" width="9" height="9"/>"#;
pub(crate) const SPLIT: &str =
    r#"<path d="M3 4 L10 4 L8 16 L3 16 Z"/><path d="M12.5 4 L17 4 L17 16 L10.5 16 Z"/>"#;
