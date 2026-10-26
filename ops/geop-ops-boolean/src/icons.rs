//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const BOOLEAN: &str =
    r#"<rect x="3" y="3" width="9" height="9"/><rect x="8" y="8" width="9" height="9"/>"#;
pub(crate) const SPLIT: &str =
    r#"<path d="M3 4 L10 4 L8 16 L3 16 Z"/><path d="M12.5 4 L17 4 L17 16 L10.5 16 Z"/>"#;
pub(crate) const IMPRINT: &str = r#"<rect x="3" y="3" width="9" height="9"/><rect x="8" y="8" width="9" height="9"/><path d="M8 12 L12 12 L12 8"/>"#;
pub(crate) const SECTION: &str =
    r#"<rect x="4" y="4" width="12" height="12"/><path d="M1 10 L19 10"/>"#;
