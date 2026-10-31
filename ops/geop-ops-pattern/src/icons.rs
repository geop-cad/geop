//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const MOVE_BODY: &str = r#"<rect x="3" y="9" width="6" height="6" stroke-dasharray="1.5 1.5"/><rect x="11" y="4" width="6" height="6"/><path d="M7 8 L11.5 4.5 M9 4.5 L11.5 4.5 L11.5 7" stroke-width="1"/>"#;
pub(crate) const SCALE_BODY: &str = r#"<rect x="3" y="9" width="5" height="5" stroke-dasharray="1.5 1.5"/><rect x="3" y="4" width="13" height="10"/><path d="M9 11 L14 6 M11.5 6 L14 6 L14 8.5" stroke-width="1"/>"#;
pub(crate) const DEFORM_BODY: &str = r#"<path d="M4 16 L4 4 M10 16 L10 4" stroke-dasharray="1.5 1.5"/><path d="M4 16 C4 10 8 8 12 4 M10 16 C10 12 13 10 16 8 M12 4 L16 8"/>"#;
pub(crate) const PATTERN: &str = r#"<path d="M3 15 C6 6, 13 6, 17 13" stroke-width="1"/><rect x="2" y="12.5" width="3" height="3"/><rect x="8.5" y="6" width="3" height="3" stroke-dasharray="1.2 1.2"/><rect x="15" y="10.5" width="3" height="3" stroke-dasharray="1.2 1.2"/>"#;
