//! The icons of a sketch's step and its tools: SVG on a 20 × 20 grid, drawn
//! in the current colour as a front end draws every icon (see
//! [`geop_core_program::operation::OperationInfo::icon`]).

pub(crate) const RECTANGLE: &str = r#"<rect x="3.5" y="5.5" width="13" height="9"/><circle cx="3.5" cy="14.5" r="1.6" fill="currentColor" stroke="none"/><circle cx="16.5" cy="5.5" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const CENTER_RECTANGLE: &str = r#"<rect x="3.5" y="5.5" width="13" height="9"/><path d="M3.5 14.5 L16.5 5.5" stroke-dasharray="1.5 1.5"/><circle cx="10" cy="10" r="1.6" fill="currentColor" stroke="none"/><circle cx="16.5" cy="5.5" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const THREE_POINT_RECTANGLE: &str = r#"<path d="M3 12 L11 4 L17 10 L9 18 Z"/><circle cx="3" cy="12" r="1.6" fill="currentColor" stroke="none"/><circle cx="11" cy="4" r="1.6" fill="currentColor" stroke="none"/><circle cx="17" cy="10" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const CIRCLE: &str = r#"<circle cx="10" cy="10" r="6.5"/><circle cx="10" cy="10" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const THREE_POINT_CIRCLE: &str = r#"<circle cx="10" cy="10" r="6.5"/><circle cx="3.5" cy="10" r="1.6" fill="currentColor" stroke="none"/><circle cx="13.25" cy="4.4" r="1.6" fill="currentColor" stroke="none"/><circle cx="13.25" cy="15.6" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const CENTER_ARC: &str = r#"<path d="M16.5 14 A6.5 6.5 0 0 0 10 7.5"/><path d="M10 14 L16.5 14 M10 14 L10 7.5" stroke-dasharray="1.5 1.5"/><circle cx="10" cy="14" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const TANGENT_ARC: &str = r#"<path d="M2.5 15.5 L9 15.5 A5 5 0 0 0 9 5.5"/><circle cx="9" cy="15.5" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const POLYGON: &str = r#"<path d="M10 3.5 L16.2 8 L13.8 15.3 L6.2 15.3 L3.8 8 Z"/><circle cx="10" cy="10" r="1.2" fill="currentColor" stroke="none"/>"#;
pub(crate) const SLOT: &str = r#"<path d="M6 6.5 L14 6.5 A3.5 3.5 0 0 1 14 13.5 L6 13.5 A3.5 3.5 0 0 1 6 6.5 Z"/><circle cx="6" cy="10" r="1.2" fill="currentColor" stroke="none"/><circle cx="14" cy="10" r="1.2" fill="currentColor" stroke="none"/>"#;
pub(crate) const ARC_SLOT: &str = r#"<path d="M3.5 14 A6.5 6.5 0 0 1 16.5 14 A1.75 1.75 0 0 1 13 14 A3 3 0 0 0 7 14 A1.75 1.75 0 0 1 3.5 14 Z"/><circle cx="10" cy="14" r="1.2" fill="currentColor" stroke="none"/>"#;
pub(crate) const TRIM: &str = r#"<path d="M3 10 L17 10" stroke-dasharray="1.5 1.5"/><path d="M7 3 L7 17 M13 3 L13 17"/><path d="M8.5 6 L11.5 14 M11.5 6 L8.5 14" stroke-width="1.2"/>"#;
pub(crate) const CONSTRUCTION: &str = r#"<path d="M3 17 L17 3" stroke-dasharray="2.5 2"/>"#;
pub(crate) const OFFSET: &str = r#"<path d="M4 16 L4 9 A5 5 0 0 1 9 4 L16 4"/><path d="M7.5 16 L7.5 9.5 A2 2 0 0 1 9.5 7.5 L16 7.5" stroke-dasharray="1.5 1.5"/>"#;
pub(crate) const COINCIDENT: &str = r#"<circle cx="10" cy="10" r="4"/><circle cx="10" cy="10" r="1.6" fill="currentColor" stroke="none"/>"#;
pub(crate) const HORIZONTAL: &str =
    r#"<path d="M3 10 L17 10"/><path d="M7 6.5 L7 13.5 M13 6.5 L13 13.5" stroke-width="1"/>"#;
pub(crate) const VERTICAL: &str =
    r#"<path d="M10 3 L10 17"/><path d="M6.5 7 L13.5 7 M6.5 13 L13.5 13" stroke-width="1"/>"#;
pub(crate) const PARALLEL: &str = r#"<path d="M5 16 L11 4 M9 16 L15 4"/>"#;
pub(crate) const PERPENDICULAR: &str = r#"<path d="M4 16 L16 16 M10 16 L10 4"/>"#;
pub(crate) const TANGENT: &str = r#"<circle cx="10" cy="11.5" r="4.5"/><path d="M3 7 L17 7"/>"#;
pub(crate) const COLLINEAR: &str = r#"<path d="M3 15 L8 10 M11 7 L17 3"/><path d="M8 10 L11 7" stroke-dasharray="1.2 1.2" stroke-width="1"/>"#;
pub(crate) const EQUAL: &str = r#"<path d="M5 8 L15 8 M5 12 L15 12"/>"#;
pub(crate) const CONCENTRIC: &str =
    r#"<circle cx="10" cy="10" r="6.5"/><circle cx="10" cy="10" r="3.2"/>"#;
pub(crate) const MIDPOINT: &str =
    r#"<path d="M3 13 L17 13"/><path d="M10 6 L7 10 L13 10 Z" fill="currentColor"/>"#;
pub(crate) const SYMMETRIC: &str = r#"<path d="M10 3 L10 17" stroke-dasharray="1.5 1.5"/><circle cx="5" cy="10" r="1.6" fill="currentColor" stroke="none"/><circle cx="15" cy="10" r="1.6" fill="currentColor" stroke="none"/><path d="M6.5 10 L8.5 10 M11.5 10 L13.5 10" stroke-width="1"/>"#;
pub(crate) const FIX: &str = r#"<rect x="5.5" y="9" width="9" height="7.5" rx="1"/><path d="M7.5 9 L7.5 6.5 A2.5 2.5 0 0 1 12.5 6.5 L12.5 9"/>"#;
pub(crate) const ADD_SKETCH: &str =
    r#"<path d="M3.5 16.5 L4.5 12.5 L13 4 L16 7 L7.5 15.5 Z"/><path d="M11.5 5.5 L14.5 8.5"/>"#;

/// The icon of the drawing tool or the constraint named `name` in events:
/// its own, or one it shares with other operations.
pub(crate) fn named(name: &str) -> Option<&'static str> {
    use geop_ops::ui::icons as shared;
    Some(match name {
        "rectangle" => RECTANGLE,
        "center_rectangle" => CENTER_RECTANGLE,
        "three_point_rectangle" => THREE_POINT_RECTANGLE,
        "circle" => CIRCLE,
        "three_point_circle" => THREE_POINT_CIRCLE,
        "center_arc" => CENTER_ARC,
        "tangent_arc" => TANGENT_ARC,
        "polygon" => POLYGON,
        "slot" => SLOT,
        "arc_slot" => ARC_SLOT,
        "trim" => TRIM,
        "construction" => CONSTRUCTION,
        "offset" => OFFSET,
        "coincident" => COINCIDENT,
        "horizontal" => HORIZONTAL,
        "vertical" => VERTICAL,
        "parallel" => PARALLEL,
        "perpendicular" => PERPENDICULAR,
        "tangent" => TANGENT,
        "collinear" => COLLINEAR,
        "equal" => EQUAL,
        "concentric" => CONCENTRIC,
        "midpoint" => MIDPOINT,
        "symmetric" => SYMMETRIC,
        "fix" => FIX,
        "point" => shared::POINT,
        "line" => shared::LINE,
        "arc" => shared::ARC,
        "spline" => shared::SPLINE,
        "fillet" => shared::FILLET,
        "chamfer" => shared::CHAMFER,
        "mirror" => shared::MIRROR,
        "linear_pattern" => shared::LINEAR_PATTERN,
        "circular_pattern" => shared::CIRCULAR_PATTERN,
        "distance" => shared::DISTANCE,
        "distance_x" => shared::DISTANCE_X,
        "distance_y" => shared::DISTANCE_Y,
        "radius" => shared::RADIUS,
        "diameter" => shared::DIAMETER,
        "angle" => shared::ANGLE,
        _ => return None,
    })
}
