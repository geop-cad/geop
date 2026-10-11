//! The icons of what several operations draw — the tools of a sketch and of
//! a 3-D sketch, the dimensions of a drawing, what a pattern repeats or a
//! blend rounds: SVG on a 20 × 20 grid, drawn in the current colour as a
//! front end draws every icon (see
//! [`geop_core_program::operation::OperationInfo::icon`]).

pub const POINT: &str = r#"<circle cx="10" cy="10" r="2.4" fill="currentColor" stroke="none"/>"#;
pub const LINE: &str = r#"<path d="M4 16 L16 4"/><circle cx="4" cy="16" r="1.6" fill="currentColor" stroke="none"/><circle cx="16" cy="4" r="1.6" fill="currentColor" stroke="none"/>"#;
pub const ARC: &str = r#"<path d="M3.5 15 A7 7 0 0 1 16.5 15"/><circle cx="3.5" cy="15" r="1.6" fill="currentColor" stroke="none"/><circle cx="10" cy="8.6" r="1.6" fill="currentColor" stroke="none"/><circle cx="16.5" cy="15" r="1.6" fill="currentColor" stroke="none"/>"#;
pub const SPLINE: &str = r#"<path d="M3 15 C6 3, 10 17, 17 5"/><circle cx="3" cy="15" r="1.6" fill="currentColor" stroke="none"/><circle cx="17" cy="5" r="1.6" fill="currentColor" stroke="none"/>"#;
pub const DISTANCE: &str = r#"<path d="M3 5 L3 15 M17 5 L17 15" stroke-width="1"/><path d="M3 10 L17 10 M3 10 L6 8 M3 10 L6 12 M17 10 L14 8 M17 10 L14 12"/>"#;
pub const DISTANCE_X: &str = r#"<path d="M3 4 L3 16 M17 4 L17 16" stroke-width="1"/><path d="M3 13 L17 13 M3 13 L6 11 M3 13 L6 15 M17 13 L14 11 M17 13 L14 15"/><text x="10" y="9" font-size="6" text-anchor="middle" fill="currentColor" stroke="none">x</text>"#;
pub const DISTANCE_Y: &str = r#"<path d="M4 3 L16 3 M4 17 L16 17" stroke-width="1"/><path d="M13 3 L13 17 M13 3 L11 6 M13 3 L15 6 M13 17 L11 14 M13 17 L15 14"/><text x="7" y="12" font-size="6" text-anchor="middle" fill="currentColor" stroke="none">y</text>"#;
pub const RADIUS: &str = r#"<path d="M17 10 A7 7 0 0 1 3 10"/><path d="M10 10 L15 5.5"/><circle cx="10" cy="10" r="1.2" fill="currentColor" stroke="none"/><text x="7" y="8" font-size="6" text-anchor="middle" fill="currentColor" stroke="none">R</text>"#;
pub const DIAMETER: &str = r#"<circle cx="10" cy="10" r="6.5"/><path d="M5.4 14.6 L14.6 5.4"/>"#;
pub const ANGLE: &str =
    r#"<path d="M3 16 L17 16 M3 16 L13 5"/><path d="M10 16 A7 7 0 0 0 7.8 10.8"/>"#;
pub const FILLET: &str = r#"<path d="M4 16 L4 10 A6 6 0 0 1 10 4 L16 4"/><path d="M4 10 L4 4 L10 4" stroke-dasharray="1.5 1.5"/>"#;
pub const CHAMFER: &str =
    r#"<path d="M4 16 L4 10 L10 4 L16 4"/><path d="M4 10 L4 4 L10 4" stroke-dasharray="1.5 1.5"/>"#;
pub const MIRROR: &str = r#"<path d="M10 2.5 L10 17.5" stroke-dasharray="1.5 1.5"/><path d="M8 5 L3 7 L3 14 L8 15 Z"/><path d="M12 5 L17 7 L17 14 L12 15 Z" stroke-dasharray="1.5 1.5"/>"#;
pub const LINEAR_PATTERN: &str = r#"<rect x="2.5" y="7.5" width="4" height="5"/><rect x="8" y="7.5" width="4" height="5" stroke-dasharray="1.5 1.5"/><rect x="13.5" y="7.5" width="4" height="5" stroke-dasharray="1.5 1.5"/><path d="M3 16 L17 16 M15.5 14.5 L17 16 L15.5 17.5" stroke-width="1"/>"#;
pub const CIRCULAR_PATTERN: &str = r#"<circle cx="10" cy="10" r="6" stroke-width="1"/><rect x="8.5" y="2" width="3" height="3"/><rect x="14.5" y="8.5" width="3" height="3" stroke-dasharray="1.2 1.2"/><rect x="8.5" y="15" width="3" height="3" stroke-dasharray="1.2 1.2"/><rect x="2.5" y="8.5" width="3" height="3" stroke-dasharray="1.2 1.2"/>"#;
