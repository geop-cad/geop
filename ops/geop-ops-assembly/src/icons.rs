//! The icons of this crate's operations and of its drag tool: SVG on a
//! 20 × 20 grid, drawn in the current colour as a front end draws every
//! icon (see
//! [`geop_core_editor::editable::OperationInfo::icon`]).

pub(crate) const ADD_PART: &str = r#"<path d="M10 3 L16 6.5 L16 13.5 L10 17 L4 13.5 L4 6.5 Z"/><path d="M10 10 L16 6.5 M10 10 L4 6.5 M10 10 L10 17"/><path d="M13 1.5 L13 4.8" stroke-width="1"/>"#;
pub(crate) const PART_PATTERN: &str = r#"<circle cx="10" cy="10" r="6.5" stroke-dasharray="1.5 1.5" stroke-width="1"/><circle cx="10" cy="3.5" r="1.6" fill="currentColor" stroke="none"/><circle cx="15.6" cy="13.2" r="1.6" fill="currentColor" stroke="none"/><circle cx="4.4" cy="13.2" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const DRAG: &str = r#"<path d="M10 2.5 L10 17.5 M2.5 10 L17.5 10"/><path d="M8 4.5 L10 2.5 L12 4.5 M8 15.5 L10 17.5 L12 15.5 M4.5 8 L2.5 10 L4.5 12 M15.5 8 L17.5 10 L15.5 12"/>"#;
pub(crate) const EXPLODE: &str = r#"<rect x="7" y="8" width="6" height="4" rx="0.5"/><path d="M10 6 L10 2.5 M8.5 4 L10 2.5 L11.5 4 M10 14 L10 17.5 M8.5 16 L10 17.5 L11.5 16" stroke-width="1"/><path d="M5 10 L2.5 10 M15 10 L17.5 10" stroke-dasharray="1 1" stroke-width="1"/>"#;
pub(crate) const MOTION: &str = r#"<circle cx="6" cy="14" r="2.5"/><path d="M8 12.5 L14.5 5.5" /><circle cx="15.5" cy="4.5" r="1.3" fill="currentColor" stroke="none"/><path d="M10.5 3 A7 7 0 0 1 17 9.5" stroke-dasharray="1.2 1.2" stroke-width="1"/>"#;
pub(crate) const DERIVED_PART: &str = r#"<path d="M7 3 L12 6 L12 11 L7 14 L2 11 L2 6 Z"/><path d="M13 8 L18 11 L18 16 L13 19" stroke-dasharray="1.5 1.5"/><path d="M12 8.5 L15 8.5 L13.8 7.3 M15 8.5 L13.8 9.7"/>"#;
pub(crate) const MIRROR_PART: &str = r#"<path d="M10 2 L10 18" stroke-dasharray="1.5 1.5"/><path d="M8 5 L3 8 L3 14 L8 16 Z"/><path d="M12 5 L17 8 L17 14 L12 16 Z"/>"#;
