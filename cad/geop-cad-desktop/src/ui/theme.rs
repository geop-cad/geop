//! The look: a dark theme and a light one, one accent, one spacing scale,
//! and the tones a dialog's text reads in.

use eframe::egui::{self, Color32, CornerRadius, FontId, Frame, Margin, Stroke, TextStyle};
use geop_core_editor::editing::Tone;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Theme {
    Dark,
    Light,
}

impl Theme {
    pub fn other(self) -> Self {
        match self {
            Theme::Dark => Theme::Light,
            Theme::Light => Theme::Dark,
        }
    }

    pub fn dark(self) -> bool {
        self == Theme::Dark
    }

    pub fn accent(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0x5a, 0xa8, 0xff),
            Theme::Light => Color32::from_rgb(0x1f, 0x6f, 0xd6),
        }
    }

    pub fn ok(self) -> Color32 {
        Color32::from_rgb(0x4c, 0xc3, 0x6b)
    }

    pub fn error(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0xff, 0x6b, 0x6b),
            Theme::Light => Color32::from_rgb(0xc6, 0x28, 0x28),
        }
    }

    pub fn tone(self, tone: Tone) -> Option<Color32> {
        match tone {
            Tone::Normal => None,
            Tone::Hint => Some(self.weak()),
            Tone::Error => Some(self.error()),
            Tone::Success => Some(self.ok()),
        }
    }

    pub fn weak(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_gray(0x9a),
            Theme::Light => Color32::from_gray(0x6a),
        }
    }

    /// The viewport's backdrop, sRGB.
    pub fn backdrop(self) -> [f32; 4] {
        match self {
            Theme::Dark => [0.105, 0.115, 0.13, 1.0],
            Theme::Light => [0.90, 0.915, 0.935, 1.0],
        }
    }

    fn panel(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0x1e, 0x20, 0x24),
            Theme::Light => Color32::from_rgb(0xf6, 0xf7, 0xf9),
        }
    }

    fn bar(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0x17, 0x19, 0x1c),
            Theme::Light => Color32::from_rgb(0xec, 0xee, 0xf1),
        }
    }

    /// What text over the view reads in.
    pub fn text(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_gray(0xe6),
            Theme::Light => Color32::from_gray(0x22),
        }
    }

    /// The backdrop of a label over the view.
    pub fn label_fill(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0x2a, 0x2d, 0x33),
            Theme::Light => Color32::from_rgb(0xfa, 0xfb, 0xfc),
        }
    }

    pub fn line(self) -> Color32 {
        match self {
            Theme::Dark => Color32::from_rgb(0x2e, 0x31, 0x37),
            Theme::Light => Color32::from_rgb(0xd5, 0xd8, 0xdd),
        }
    }

    /// A panel beside the viewport.
    pub fn side_frame(self) -> Frame {
        Frame::new()
            .fill(self.panel())
            .inner_margin(Margin::symmetric(10, 8))
    }

    /// The ribbon and the status bar.
    pub fn bar_frame(self) -> Frame {
        Frame::new()
            .fill(self.bar())
            .inner_margin(Margin::symmetric(8, 4))
    }

    /// A card floating over the viewport.
    pub fn card(self) -> Frame {
        Frame::new()
            .fill(self.panel().gamma_multiply(0.97))
            .stroke(Stroke::new(1.0, self.line()))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::same(10))
            .shadow(egui::epaint::Shadow {
                offset: [0, 4],
                blur: 16,
                spread: 0,
                color: Color32::from_black_alpha(if self.dark() { 110 } else { 40 }),
            })
    }

    /// Set egui's look to the theme's.
    pub fn apply(self, ctx: &egui::Context) {
        ctx.set_theme(match self {
            Theme::Dark => egui::Theme::Dark,
            Theme::Light => egui::Theme::Light,
        });
        let accent = self.accent();
        let panel = self.panel();
        ctx.all_styles_mut(move |style| {
            style.spacing.item_spacing = egui::vec2(6.0, 5.0);
            style.spacing.button_padding = egui::vec2(7.0, 3.0);
            style.spacing.interact_size.y = 22.0;
            style.spacing.combo_width = 120.0;
            style
                .text_styles
                .insert(TextStyle::Body, FontId::proportional(13.0));
            style
                .text_styles
                .insert(TextStyle::Button, FontId::proportional(13.0));
            style
                .text_styles
                .insert(TextStyle::Small, FontId::proportional(11.0));
            style
                .text_styles
                .insert(TextStyle::Heading, FontId::proportional(15.5));
            style
                .text_styles
                .insert(TextStyle::Monospace, FontId::monospace(12.5));
            let visuals = &mut style.visuals;
            visuals.selection.bg_fill = accent.gamma_multiply(0.45);
            visuals.selection.stroke = Stroke::new(1.0, accent);
            visuals.hyperlink_color = accent;
            visuals.panel_fill = panel;
            visuals.window_corner_radius = CornerRadius::same(8);
            visuals.menu_corner_radius = CornerRadius::same(6);
            for w in [
                &mut visuals.widgets.inactive,
                &mut visuals.widgets.hovered,
                &mut visuals.widgets.active,
                &mut visuals.widgets.open,
                &mut visuals.widgets.noninteractive,
            ] {
                w.corner_radius = CornerRadius::same(5);
            }
        });
    }
}
