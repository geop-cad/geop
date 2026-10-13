//! The icons of this crate's operation and its tools: SVG on a 20 × 20
//! grid, drawn in the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const DRAWING: &str = r#"<rect x="2.5" y="3.5" width="15" height="13"/><rect x="5" y="6" width="5" height="5"/><path d="M11 16.5 L11 13.5 L17.5 13.5"/>"#;
pub(crate) const NOTE: &str = r#"<path d="M9 5 L17 5 M9 8 L15 8"/><path d="M9 8 L4 15" stroke-width="1"/><circle cx="4" cy="15" r="1.2" fill="currentColor" stroke="none"/>"#;
pub(crate) const CENTER_MARK: &str = r#"<circle cx="10" cy="10" r="5"/><path d="M2.5 10 L17.5 10 M10 2.5 L10 17.5" stroke-width="1" stroke-dasharray="3 1 1 1"/>"#;
