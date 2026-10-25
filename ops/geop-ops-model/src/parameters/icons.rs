//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

/// A table: a header row, and a column of row names.
pub(crate) const LOOKUP_TABLE: &str = r#"<rect x="3" y="4" width="14" height="12" rx="1"/><path d="M3 8 L17 8 M8 8 L8 16 M8 12 L17 12"/>"#;

/// Parameters: two sliders, their knobs at different values.
pub(crate) const PARAMETERS: &str = r#"<path d="M3 7 L17 7 M3 13 L17 13"/><circle cx="12" cy="7" r="2"/><circle cx="7" cy="13" r="2"/>"#;

/// Properties: a tag on a string, with its eyelet.
pub(crate) const PROPERTIES: &str =
    r#"<path d="M3 4 L10 4 L17 11 L10 18 L3 11 Z"/><circle cx="7" cy="8" r="1.3"/>"#;
