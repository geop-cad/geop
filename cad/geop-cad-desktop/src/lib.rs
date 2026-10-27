//! geop's desktop app: the editor of `geop_core_editor`, the same one the
//! web app talks to, linked in and run on a thread of its own, drawn in a
//! native window — egui for the panels and dialogs, wgpu for the 3-D view.
//! It sends the editor the same [`geop_core_editor::editor::Command`]s the
//! web app does and draws the same updates; what the window does is in
//! [`ui`], what it holds and sends in [`controller::Controller`], which
//! runs without a window too (its tests drive it as the window does).

pub mod camera;
pub mod controller;
pub mod draw;
pub mod kernel;
pub mod math;
pub mod render;
pub mod scene;
pub mod ui;
pub mod workspace;

#[cfg(test)]
mod tests;
