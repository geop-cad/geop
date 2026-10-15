//! [`Exporter`]: a file written from what a program built.

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Serialize, de::DeserializeOwned};

use super::{Output, read_options};

/// What an exporter is to a front end: how it is asked for, what it is
/// offered as, and the formats it writes.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ExporterInfo {
    /// The name it is asked for by: `stl`.
    pub kind: &'static str,
    /// What it is offered as: `STL`.
    pub label: &'static str,
    /// What the file is for, in a few words: `mesh, for 3-D printing`.
    pub doc: &'static str,
    /// The formats it writes, by the extension of their files, the one
    /// written unless asked for another first: `["svg", "dxf"]`.
    pub formats: &'static [&'static str],
}

/// A kind of file written from what a program built: a mesh, a STEP file,
/// a drawing, a robot. An exporter is a unit struct; what it is told
/// beyond the part is its [`Exporter::Options`].
pub trait Exporter: 'static {
    const INFO: ExporterInfo;
    /// What it is told besides the part and the format, as a front end
    /// sends it (JSON); the defaults where it sends none.
    type Options: DeserializeOwned + Default;

    /// The file of `output` in `format`, one of [`ExporterInfo::formats`].
    fn export(output: &mut Output, format: &str, options: Self::Options) -> GeopResult<Export>;
}

/// An [`Exporter`] with its options erased, so that exporters of all kinds
/// can be kept in one [`super::Outputs`]. Every exporter is one, through
/// its value.
pub trait ErasedExporter {
    fn info(&self) -> &'static ExporterInfo;

    /// The file of `output` in `format` — its first, if `None` — with
    /// `options` as sent.
    fn export(
        &self,
        output: &mut Output,
        format: Option<&str>,
        options: &serde_json::Value,
    ) -> GeopResult<Export>;
}

impl<E: Exporter> ErasedExporter for E {
    fn info(&self) -> &'static ExporterInfo {
        &E::INFO
    }

    fn export(
        &self,
        output: &mut Output,
        format: Option<&str>,
        options: &serde_json::Value,
    ) -> GeopResult<Export> {
        let info = &E::INFO;
        let format = match format {
            None => info.formats.first().copied().ok_or_else(|| {
                GeopError::new(format!("exporter `{}` writes no format", info.kind))
            })?,
            Some(format) => (info.formats.iter().copied())
                .find(|f| f.eq_ignore_ascii_case(format))
                .ok_or_else(|| {
                    GeopError::new(format!(
                        "exporter `{}` writes no `{format}`; it writes: {}",
                        info.kind,
                        info.formats.join(", ")
                    ))
                })?,
        };
        let options = read_options::<E::Options>(info.kind, options)?;
        let file = output.file_name().to_string();
        E::export(output, format, options).map_err(|e| {
            e.with_context(format!(
                "export(exporter={}, format={format}, file={file:?})",
                info.kind
            ))
        })
    }
}

/// A file written for a front end to save.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Export {
    /// What to call it: `part.stl`.
    pub name: String,
    /// Its media type, to save it as: `model/stl`.
    pub media_type: String,
    #[serde(flatten)]
    pub content: Content,
}

impl Export {
    /// Its text, if it is a text file.
    pub fn text(&self) -> Option<&str> {
        match &self.content {
            Content::Text(text) => Some(text),
            Content::Bytes(_) => None,
        }
    }

    /// What it holds, as bytes.
    pub fn bytes(&self) -> &[u8] {
        match &self.content {
            Content::Text(text) => text.as_bytes(),
            Content::Bytes(bytes) => bytes,
        }
    }
}

/// What an exported file holds: text — sent as `text` — or bytes, sent as
/// `bytes`, in base64.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Content {
    Text(String),
    #[serde(serialize_with = "base64")]
    Bytes(Vec<u8>),
}

/// `bytes` as base64 text (RFC 4648, padded).
fn base64<Ser: serde::Serializer>(bytes: &[u8], serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut text = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [0, 1, 2].map(|k| chunk.get(k).copied().unwrap_or(0) as u32);
        let n = (b[0] << 16) | (b[1] << 8) | b[2];
        for k in 0..4 {
            text.push(if k <= chunk.len() {
                ALPHABET[(n >> (18 - 6 * k) & 63) as usize] as char
            } else {
                '='
            });
        }
    }
    serializer.serialize_str(&text)
}
