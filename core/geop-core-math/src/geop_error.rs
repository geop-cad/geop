use std::{backtrace::Backtrace, sync::Arc};

/// An error: what went wrong at the root, and the context it was reported
/// through. A copy of one is the same error — a step's failure kept with
/// what it built, and reported again with more context — so the backtrace
/// is shared, not captured again.
#[derive(Clone)]
pub enum GeopError {
    Root {
        message: String,
        backtrace: Arc<Backtrace>,
    },
    /// `inner`, reported through a frame that says what was being done:
    /// one line of the chain that [`GeopError::with_context`] builds.
    Context {
        inner: Box<GeopError>,
        message: String,
    },
}

impl GeopError {
    /// Create a root error. Captures a backtrace at the call site.
    pub fn new(message: impl Into<String>) -> Self {
        let backtrace = Arc::new(Backtrace::capture());
        GeopError::Root {
            message: message.into(),
            backtrace,
        }
    }

    /// Wrap this error in a Context frame with a string message.
    pub fn with_context(self, message: impl Into<String>) -> Self {
        GeopError::Context {
            inner: Box::new(self),
            message: message.into(),
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
}

impl std::fmt::Display for GeopError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            GeopError::Root { message, backtrace } => {
                writeln!(f, "GeopError")?;
                writeln!(f, "Backtrace: {}", backtrace)?;
                writeln!(f, "RootError: {}", message)
            }
            GeopError::Context { inner, message } => {
                write!(f, "{}", inner)?;
                writeln!(f, "Context:\n{}", message)
            }
        }
    }
}

/// Two errors are the same when they say the same: their messages, root
/// and context — not where they were captured.
impl PartialEq for GeopError {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (GeopError::Root { message: a, .. }, GeopError::Root { message: b, .. }) => a == b,
            (
                GeopError::Context {
                    inner: a,
                    message: m,
                },
                GeopError::Context {
                    inner: b,
                    message: n,
                },
            ) => m == n && a == b,
            _ => false,
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
