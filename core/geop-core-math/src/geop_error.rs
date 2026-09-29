use std::backtrace::Backtrace;

/// A renderable diagnostic artefact that can be attached to any
/// frame of a `GeopError`.
pub trait DebugContext: Send + Sync + core::fmt::Debug {
    /// One-line label shown in the error chain display.
    fn label(&self) -> &str;
}

pub enum GeopError {
    Root {
        message: String,
        backtrace: Backtrace,
    },
    Context {
        inner: Box<GeopError>,
        context: Option<Box<dyn DebugContext>>,
    },
}

impl GeopError {
    /// Create a root error. Captures a backtrace at the call site.
    pub fn new(message: impl Into<String>) -> Self {
        let backtrace = Backtrace::capture();
        GeopError::Root {
            message: message.into(),
            backtrace,
        }
    }

    /// Wrap this error in a Context frame with a string message.
    pub fn with_context(self, message: impl Into<String>) -> Self {
        let label = message.into();
        GeopError::Context {
            inner: Box::new(self),
            context: Some(Box::new(StringContext(label))),
        }
    }

    /// Wrap this error in a Context frame with a debug scene.
    pub fn with_scene(self, scene: impl DebugContext + 'static) -> Self {
        GeopError::Context {
            inner: Box::new(self),
            context: Some(Box::new(scene)),
        }
    }

    /// What went wrong at the root, without the context it was reported
    /// through: what a user is shown.
    pub fn root_message(&self) -> &str {
        match self {
            GeopError::Root { message, .. } => message,
            GeopError::Context { inner, .. } => inner.root_message(),
        }
    }

    /// Walk the chain and collect labels of all attached scenes, root-first.
    pub fn scene_labels(&self) -> Vec<&str> {
        match self {
            GeopError::Root { .. } => vec![],
            GeopError::Context { inner, context } => {
                let mut labels = inner.scene_labels();
                if let Some(ctx) = context {
                    labels.push(ctx.label());
                }
                labels
            }
        }
    }
}

impl std::fmt::Display for GeopError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            GeopError::Root { message, backtrace } => {
                writeln!(f, "GeopError")?;
                writeln!(f, "Backtrace: {}", backtrace)?;
                writeln!(f, "RootError: {}", message)
            }
            GeopError::Context { inner, context } => {
                write!(f, "{}", inner)?;
                match context {
                    Some(ctx) => writeln!(f, "Context:\n{}", ctx.label()),
                    None => writeln!(f, "Context: (no details)"),
                }
            }
        }
    }
}

impl std::fmt::Debug for GeopError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}

impl std::error::Error for GeopError {}

impl From<&str> for GeopError {
    fn from(message: &str) -> Self {
        GeopError::new(message)
    }
}

impl From<String> for GeopError {
    fn from(message: String) -> Self {
        GeopError::new(message)
    }
}

pub type GeopResult<T> = Result<T, GeopError>;

/// Something that can turn one `GeopError` into another, more-annotated one
/// — either a plain message (wrapped via `GeopError::with_context`) or a
/// callback that builds the wrapped error itself (e.g. to embed live state
/// captured at the call site). Lets [`WithContext::with_context`] accept
/// either `.with_context("a message")` or `.with_context(&|e| ...)`.
pub trait ContextSource {
    fn apply(&self, err: GeopError) -> GeopError;
}

impl ContextSource for str {
    fn apply(&self, err: GeopError) -> GeopError {
        err.with_context(self)
    }
}

impl ContextSource for String {
    fn apply(&self, err: GeopError) -> GeopError {
        err.with_context(self.as_str())
    }
}

impl<F: Fn(GeopError) -> GeopError + ?Sized> ContextSource for F {
    fn apply(&self, err: GeopError) -> GeopError {
        self(err)
    }
}

pub trait WithContext<T> {
    fn with_context(self, ctx: &(impl ContextSource + ?Sized)) -> GeopResult<T>;
}

impl<T> WithContext<T> for GeopResult<T> {
    fn with_context(self, ctx: &(impl ContextSource + ?Sized)) -> GeopResult<T> {
        match self {
            Ok(v) => Ok(v),
            Err(err) => Err(ctx.apply(err)),
        }
    }
}

/// Like [`format!`], but for [`WithContext::with_context`]: builds a
/// `ContextSource` that formats its message lazily, only if the result is
/// actually an `Err` — `.with_context(with_context!("failed at i={i}"))`
/// costs nothing on the success path, unlike `.with_context(&format!(...))`
/// (eagerly formats every time, even when nothing is wrong).
#[macro_export]
macro_rules! with_context {
    ($($arg:tt)*) => {
        &|e: $crate::geop_error::GeopError| e.with_context(format!($($arg)*))
    };
}

// ── internal helper ──────────────────────────────────────────────────────────

#[derive(Debug)]
struct StringContext(String);

impl DebugContext for StringContext {
    fn label(&self) -> &str {
        &self.0
    }
}
