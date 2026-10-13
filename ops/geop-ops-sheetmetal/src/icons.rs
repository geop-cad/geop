//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const BASE_FLANGE: &str = r#"<path d="M3 13 L10 16.5 L17 13 L10 9.5 Z"/><path d="M3 13 L3 14.5 L10 18 L17 14.5 L17 13" stroke-width="1"/>"#;
pub(crate) const EDGE_FLANGE: &str =
    r#"<path d="M3 15 L12 15 Q15 15 15 12 L15 4"/><path d="M3 17 L12 17 Q17 17 17 12 L17 4"/>"#;
pub(crate) const FLAT_PATTERN: &str = r#"<rect x="3" y="7" width="14" height="6"/><path d="M8 7 L8 13 M12 7 L12 13" stroke-dasharray="1.5 1.5"/>"#;
pub(crate) const SHEET_CUT: &str = r#"<path d="M3 5 L17 5 L17 15 L12 15 L12 12 L9 12 L9 15 L3 15 Z"/><circle cx="7" cy="9" r="1.8"/>"#;
pub(crate) const HEM: &str = r#"<path d="M17 13 L6 13 A2.5 2.5 0 0 1 6 8 L13 8"/><path d="M17 15 L6 15 A4.5 4.5 0 0 1 6 6 L13 6" stroke-width="1"/>"#;
