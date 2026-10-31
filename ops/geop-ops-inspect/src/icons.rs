//! The icons of this crate's inspectors: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const MEASURE: &str = r#"<path d="M2.5 13 L13 2.5 L17.5 7 L7 17.5 Z"/><path d="M6 9.5 L7.5 11 M8.5 7 L10.5 9 M11 4.5 L12.5 6" stroke-width="1"/>"#;
pub(crate) const INTERFERENCE: &str = r#"<rect x="3" y="3" width="9" height="9"/><rect x="8" y="8" width="9" height="9"/><path d="M8 12 L12 8" stroke-width="1"/>"#;
pub(crate) const MASS: &str =
    r#"<path d="M5 8 L15 8 L17 17 L3 17 Z"/><circle cx="10" cy="5" r="2.2"/>"#;
pub(crate) const CLEARANCE: &str = r#"<rect x="2.5" y="4" width="6" height="12"/><rect x="11.5" y="4" width="6" height="12"/><path d="M8.5 10 L11.5 10" stroke-width="1"/>"#;
