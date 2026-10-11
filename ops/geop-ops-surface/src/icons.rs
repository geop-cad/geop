//! The icons of this crate's operations: SVG on a 20 × 20 grid, drawn in
//! the current colour as a front end draws every icon (see
//! [`geop_core_program::operation::OperationInfo::icon`]).

pub(crate) const BOUNDARY_SURFACE: &str = r#"<path d="M3 15 C6 13 10 16 13 14 L17 6 C14 7 10 4 7 6 Z"/><path d="M5 10.5 C8 9 11 11.5 15 10" stroke-dasharray="1.5 1.5"/>"#;
pub(crate) const OFFSET_SURFACE: &str = r#"<path d="M3 14 C7 10 12 16 17 12"/><path d="M3 8 C7 4 12 10 17 6" stroke-dasharray="1.5 1.5"/><path d="M10 12.5 L10 7.5 M8.5 9 L10 7.5 L11.5 9" stroke-width="1"/>"#;
pub(crate) const THICKEN: &str =
    r#"<path d="M3 13 C7 9 12 15 17 11 L17 7.5 C12 11.5 7 5.5 3 9.5 Z"/>"#;
pub(crate) const KNIT: &str = r#"<path d="M3 5 L9 5 L9 15 L3 15 Z"/><path d="M11 5 L17 5 L17 15 L11 15 Z"/><path d="M8 8 L12 8 M8 12 L12 12" stroke-width="1"/>"#;
pub(crate) const TRIM_SURFACE: &str = r#"<path d="M3 6 L10 6 L10 16 L3 16 Z"/><path d="M10 6 L17 6 L17 16 L10 16" stroke-dasharray="1.5 1.5"/><path d="M10 3 L10 18.5" stroke-width="1"/>"#;
pub(crate) const EXTEND_SURFACE: &str = r#"<path d="M3 5 L11 5 L11 15 L3 15 Z"/><path d="M11 5 L16 5 L16 15 L11 15" stroke-dasharray="1.5 1.5"/><path d="M12.5 10 L17.5 10 M15.5 8 L17.5 10 L15.5 12" stroke-width="1"/>"#;
pub(crate) const NETWORK_SURFACE: &str = r#"<path d="M3 15 C6 13 10 16 13 14 L17 6 C14 7 10 4 7 6 Z"/><path d="M5 10.5 C8 9 11 11.5 15 10" stroke-width="1"/><path d="M8 14.6 L10 5" stroke-width="1"/>"#;
