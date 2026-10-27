use std::backtrace::Backtrace;

pub enum GeopError {
    Root {
        message: String,
        backtrace: Backtrace,
    },
    /// `inner`, reported through a frame that says what was being done:
    /// one line of the chain that [`GeopError::with_context`] builds.
    Context {
        inner: Box<GeopError>,
        message: String,
    },
    /// `inner`, caused by the argument `key` of the step it was raised in:
    /// what an editor shows the error beside — the field of that key (see
    /// [`GeopError::concerning`]).
    Argument { inner: Box<GeopError>, key: String },
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
        GeopError::Context {
            inner: Box::new(self),
            message: message.into(),
        }
    }

    /// Caused by the argument `key` of the step it is raised in — its
    /// sketch, its distance — by a value that argument holds or an element
    /// it refers to: an editor shows it beside that argument's field.
    pub fn concerning(self, key: impl Into<String>) -> Self {
        GeopError::Argument {
            inner: Box::new(self),
            key: key.into(),
        }
    }

    /// The argument it was caused by (see [`GeopError::concerning`]): the
    /// outermost one said, which the step that failed said — an argument of
    /// a step it read from is that step's.
    pub fn argument(&self) -> Option<&str> {
        match self {
            GeopError::Root { .. } => None,
            GeopError::Context { inner, .. } => inner.argument(),
            GeopError::Argument { key, .. } => Some(key),
        }
    }

    /// What went wrong at the root, without the context it was reported
    /// through: what a user is shown.
    pub fn root_message(&self) -> &str {
        match self {
            GeopError::Root { message, .. } => message,
            GeopError::Context { inner, .. } | GeopError::Argument { inner, .. } => {
                inner.root_message()
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
            GeopError::Context { inner, message } => {
                write!(f, "{}", inner)?;
                writeln!(f, "Context:\n{}", message)
            }
            GeopError::Argument { inner, key } => {
                write!(f, "{}", inner)?;
                writeln!(f, "Argument: {}", key)
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

    /// Its error caused by the step's argument `key` (see
    /// [`GeopError::concerning`]).
    fn concerning(self, key: &str) -> GeopResult<T>;
}

impl<T> WithContext<T> for GeopResult<T> {
    fn with_context(self, ctx: &(impl ContextSource + ?Sized)) -> GeopResult<T> {
        match self {
            Ok(v) => Ok(v),
            Err(err) => Err(ctx.apply(err)),
        }
    }

    fn concerning(self, key: &str) -> GeopResult<T> {
        self.map_err(|e| e.concerning(key))
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
