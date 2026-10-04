//! ISO 10303-21 ("Part 21"), the clear-text encoding every STEP file is
//! written in: a header, then a data section of entity instances, each
//! `#id = NAME(parameters);` — or, for an instance of several entity types
//! at once, a *complex* instance `#id = (NAME1(...) NAME2(...));`.
//!
//! This module only knows the encoding, not what any entity means: it reads
//! a file into an [`Exchange`] and writes one back. What the entities say
//! about geometry is [`crate::import`]'s and [`crate::export`]'s business.

use std::collections::BTreeMap;
use std::fmt::Write;

use geop_core_math::geop_error::{GeopError, GeopResult};

/// One parameter of an entity instance.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// `$`: no value.
    Null,
    /// `*`: a value derived from others, not written.
    Derived,
    Integer(i64),
    Real(f64),
    String(String),
    /// `.NAME.`, held without the dots: `.T.` is `Enum("T")`.
    Enum(String),
    /// `#id`: another instance.
    Ref(u64),
    /// `"..."`: a bit string, held as written.
    Binary(String),
    List(Vec<Value>),
    /// `NAME(value)`: a value of a defined type, as `LENGTH_MEASURE(1.5)`.
    Typed(String, Box<Value>),
}

/// One entity type's part of an instance: its name and parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Record {
    pub name: String,
    pub args: Vec<Value>,
}

/// An entity instance: of one type, or complex — of several at once, each
/// with its own parameters, as `(B_SPLINE_CURVE(...) RATIONAL_B_SPLINE_CURVE(...))`.
#[derive(Clone, Debug, PartialEq)]
pub enum Instance {
    Simple(Record),
    Complex(Vec<Record>),
}

impl Instance {
    /// The record of the type `name` this instance is of — itself, if
    /// simple and of that type.
    pub fn record(&self, name: &str) -> Option<&Record> {
        match self {
            Instance::Simple(r) => (r.name == name).then_some(r),
            Instance::Complex(rs) => rs.iter().find(|r| r.name == name),
        }
    }

    /// Whether this instance is of the type `name`.
    pub fn is(&self, name: &str) -> bool {
        self.record(name).is_some()
    }

    /// What the instance is, for messages: its type, or its types joined
    /// by `+` for a complex one.
    pub fn type_name(&self) -> String {
        match self {
            Instance::Simple(r) => r.name.clone(),
            Instance::Complex(rs) => rs
                .iter()
                .map(|r| r.name.as_str())
                .collect::<Vec<_>>()
                .join("+"),
        }
    }
}

/// A whole exchange file: its header records and its instances by id.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Exchange {
    pub header: Vec<Record>,
    pub instances: BTreeMap<u64, Instance>,
}

impl Exchange {
    /// The instance `#id`.
    pub fn get(&self, id: u64) -> GeopResult<&Instance> {
        self.instances
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("the file refers to #{id}, which it does not have")))
    }

    /// The ids of every instance of the type `name`, in id order.
    pub fn all_of(&self, name: &str) -> Vec<u64> {
        self.instances
            .iter()
            .filter(|(_, instance)| instance.is(name))
            .map(|(&id, _)| id)
            .collect()
    }

    /// Reads the text of an exchange file.
    pub fn parse(text: &str) -> GeopResult<Self> {
        Parser {
            lexer: Lexer {
                bytes: text.as_bytes(),
                pos: 0,
            },
        }
        .exchange()
    }

    /// The text of the exchange file, one instance per line.
    pub fn write(&self) -> String {
        let mut out = String::from("ISO-10303-21;\nHEADER;\n");
        for record in &self.header {
            write_record(&mut out, record);
            out.push_str(";\n");
        }
        out.push_str("ENDSEC;\nDATA;\n");
        for (id, instance) in &self.instances {
            let _ = write!(out, "#{id}=");
            match instance {
                Instance::Simple(record) => write_record(&mut out, record),
                Instance::Complex(records) => {
                    out.push('(');
                    for (k, record) in records.iter().enumerate() {
                        if k > 0 {
                            out.push(' ');
                        }
                        write_record(&mut out, record);
                    }
                    out.push(')');
                }
            }
            out.push_str(";\n");
        }
        out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
        out
    }
}

fn write_record(out: &mut String, record: &Record) {
    out.push_str(&record.name);
    write_list(out, &record.args);
}

fn write_list(out: &mut String, values: &[Value]) {
    out.push('(');
    for (k, value) in values.iter().enumerate() {
        if k > 0 {
            out.push(',');
        }
        write_value(out, value);
    }
    out.push(')');
}

fn write_value(out: &mut String, value: &Value) {
    match value {
        Value::Null => out.push('$'),
        Value::Derived => out.push('*'),
        Value::Integer(i) => {
            let _ = write!(out, "{i}");
        }
        Value::Real(r) => out.push_str(&real(*r)),
        Value::String(s) => {
            out.push('\'');
            for c in s.chars() {
                match c {
                    '\'' => out.push_str("''"),
                    '\\' => out.push_str("\\\\"),
                    ' '..='~' => out.push(c),
                    c => {
                        let mut units = [0u16; 2];
                        for unit in c.encode_utf16(&mut units) {
                            let _ = write!(out, "\\X2\\{unit:04X}\\X0\\");
                        }
                    }
                }
            }
            out.push('\'');
        }
        Value::Enum(e) => {
            let _ = write!(out, ".{e}.");
        }
        Value::Ref(id) => {
            let _ = write!(out, "#{id}");
        }
        Value::Binary(b) => {
            let _ = write!(out, "\"{b}\"");
        }
        Value::List(values) => write_list(out, values),
        Value::Typed(name, value) => {
            out.push_str(name);
            out.push('(');
            write_value(out, value);
            out.push(')');
        }
    }
}

/// `r` as a Part 21 real, which needs a decimal point and an upper-case
/// exponent: `1.`, `-0.25`, `1.5E-07`. Written as the shortest text that
/// reads back to exactly `r`.
pub fn real(r: f64) -> String {
    let text = format!("{r:?}");
    let (mantissa, exponent) = match text.split_once('e') {
        Some((m, e)) => (m.to_string(), Some(e.to_string())),
        None => (text, None),
    };
    let mantissa = if mantissa.contains('.') {
        mantissa
    } else {
        format!("{mantissa}.")
    };
    match exponent {
        Some(e) => format!("{mantissa}E{e}"),
        None => mantissa,
    }
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    /// A keyword: an entity or type name, or a section name like `DATA`
    /// (which may contain `-`, as `END-ISO-10303-21`).
    Keyword(String),
    Ref(u64),
    Integer(i64),
    Real(f64),
    String(String),
    Enum(String),
    Binary(String),
    Null,
    Derived,
    Open,
    Close,
    Comma,
    Equals,
    Semicolon,
    End,
}

struct Lexer<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl Lexer<'_> {
    fn error(&self, what: impl std::fmt::Display) -> GeopError {
        let line = 1 + self.bytes[..self.pos.min(self.bytes.len())]
            .iter()
            .filter(|&&b| b == b'\n')
            .count();
        GeopError::new(format!("reading the STEP file, line {line}: {what}"))
    }

    fn skip_space(&mut self) -> GeopResult<()> {
        loop {
            match self.bytes.get(self.pos) {
                Some(b) if b.is_ascii_whitespace() => self.pos += 1,
                Some(b'/') if self.bytes.get(self.pos + 1) == Some(&b'*') => {
                    let start = self.pos;
                    self.pos += 2;
                    loop {
                        match self.bytes.get(self.pos) {
                            None => {
                                self.pos = start;
                                return Err(self.error("a comment that never ends"));
                            }
                            Some(b'*') if self.bytes.get(self.pos + 1) == Some(&b'/') => {
                                self.pos += 2;
                                break;
                            }
                            Some(_) => self.pos += 1,
                        }
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    fn next(&mut self) -> GeopResult<Token> {
        self.skip_space()?;
        let Some(&b) = self.bytes.get(self.pos) else {
            return Ok(Token::End);
        };
        let single = |lexer: &mut Self, token| {
            lexer.pos += 1;
            Ok(token)
        };
        match b {
            b'(' => single(self, Token::Open),
            b')' => single(self, Token::Close),
            b',' => single(self, Token::Comma),
            b'=' => single(self, Token::Equals),
            b';' => single(self, Token::Semicolon),
            b'$' => single(self, Token::Null),
            b'*' => single(self, Token::Derived),
            b'#' => {
                self.pos += 1;
                let start = self.pos;
                while self.bytes.get(self.pos).is_some_and(u8::is_ascii_digit) {
                    self.pos += 1;
                }
                let digits = std::str::from_utf8(&self.bytes[start..self.pos]).unwrap_or("");
                digits
                    .parse()
                    .map(Token::Ref)
                    .map_err(|_| self.error("a `#` that is not followed by an instance number"))
            }
            b'\'' => self.string(),
            b'"' => {
                self.pos += 1;
                let start = self.pos;
                while self.bytes.get(self.pos).is_some_and(|&c| c != b'"') {
                    self.pos += 1;
                }
                let text = String::from_utf8_lossy(&self.bytes[start..self.pos]).into_owned();
                self.pos += 1;
                Ok(Token::Binary(text))
            }
            b'.' if self
                .bytes
                .get(self.pos + 1)
                .is_some_and(|c| c.is_ascii_alphabetic() || *c == b'_') =>
            {
                self.pos += 1;
                let start = self.pos;
                while self
                    .bytes
                    .get(self.pos)
                    .is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_')
                {
                    self.pos += 1;
                }
                let name = String::from_utf8_lossy(&self.bytes[start..self.pos]).to_uppercase();
                if self.bytes.get(self.pos) != Some(&b'.') {
                    return Err(self.error(format!("the enumeration value .{name} has no closing `.`")));
                }
                self.pos += 1;
                Ok(Token::Enum(name))
            }
            b'+' | b'-' | b'.' | b'0'..=b'9' => self.number(),
            b'!' | b'A'..=b'Z' | b'a'..=b'z' | b'_' => {
                let start = self.pos;
                self.pos += 1;
                while self
                    .bytes
                    .get(self.pos)
                    .is_some_and(|c| c.is_ascii_alphanumeric() || *c == b'_' || *c == b'-')
                {
                    self.pos += 1;
                }
                Ok(Token::Keyword(
                    String::from_utf8_lossy(&self.bytes[start..self.pos]).to_uppercase(),
                ))
            }
            other => Err(self.error(format!("an unexpected character {:?}", other as char))),
        }
    }

    fn number(&mut self) -> GeopResult<Token> {
        let start = self.pos;
        if matches!(self.bytes.get(self.pos), Some(b'+' | b'-')) {
            self.pos += 1;
        }
        let mut real = false;
        while let Some(&c) = self.bytes.get(self.pos) {
            match c {
                b'0'..=b'9' => self.pos += 1,
                b'.' => {
                    real = true;
                    self.pos += 1;
                }
                b'E' | b'e' => {
                    real = true;
                    self.pos += 1;
                    if matches!(self.bytes.get(self.pos), Some(b'+' | b'-')) {
                        self.pos += 1;
                    }
                }
                _ => break,
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos]).unwrap_or("");
        if real {
            // Rust does not read `1.E5` or `1.`: put a zero after a bare point.
            let text = text.replace(".E", ".0E").replace(".e", ".0e");
            let text = if text.ends_with('.') {
                format!("{text}0")
            } else {
                text
            };
            text.parse()
                .map(Token::Real)
                .map_err(|_| self.error(format!("{text:?} is not a number")))
        } else {
            text.parse()
                .map(Token::Integer)
                .map_err(|_| self.error(format!("{text:?} is not a number")))
        }
    }

    /// A string: `'...'`, a quote written twice, with the `\X2\...\X0\`
    /// (UTF-16), `\X\hh` (Latin-1) and `\\` escapes decoded and any other
    /// escape kept as written.
    fn string(&mut self) -> GeopResult<Token> {
        self.pos += 1;
        let mut raw = Vec::new();
        loop {
            match self.bytes.get(self.pos) {
                None => return Err(self.error("a string that never ends")),
                Some(b'\'') if self.bytes.get(self.pos + 1) == Some(&b'\'') => {
                    raw.push(b'\'');
                    self.pos += 2;
                }
                Some(b'\'') => {
                    self.pos += 1;
                    break;
                }
                Some(&c) => {
                    raw.push(c);
                    self.pos += 1;
                }
            }
        }
        let raw = String::from_utf8_lossy(&raw).into_owned();
        Ok(Token::String(decode(&raw)))
    }
}

fn decode(raw: &str) -> String {
    let mut out = String::new();
    let mut rest = raw;
    while let Some(k) = rest.find('\\') {
        out.push_str(&rest[..k]);
        rest = &rest[k..];
        if let Some(after) = rest.strip_prefix("\\\\") {
            out.push('\\');
            rest = after;
        } else if let Some(after) = rest.strip_prefix("\\X2\\") {
            let end = after.find("\\X0\\").unwrap_or(after.len());
            let hex = &after[..end];
            let units: Vec<u16> = (0..hex.len() / 4)
                .filter_map(|i| u16::from_str_radix(&hex[4 * i..4 * i + 4], 16).ok())
                .collect();
            out.push_str(&String::from_utf16_lossy(&units));
            rest = after.get(end + 4..).unwrap_or("");
        } else if let Some(after) = rest.strip_prefix("\\X\\") {
            match after.get(..2).and_then(|h| u8::from_str_radix(h, 16).ok()) {
                Some(byte) => {
                    out.push(byte as char);
                    rest = &after[2..];
                }
                None => {
                    out.push_str("\\X\\");
                    rest = after;
                }
            }
        } else {
            out.push('\\');
            rest = &rest[1..];
        }
    }
    out.push_str(rest);
    out
}

struct Parser<'a> {
    lexer: Lexer<'a>,
}

impl Parser<'_> {
    fn expect(&mut self, expected: Token) -> GeopResult<()> {
        let token = self.lexer.next()?;
        if token == expected {
            Ok(())
        } else {
            Err(self
                .lexer
                .error(format!("expected {expected:?}, found {token:?}")))
        }
    }

    fn exchange(mut self) -> GeopResult<Exchange> {
        let mut exchange = Exchange::default();
        loop {
            match self.lexer.next()? {
                Token::End => break,
                Token::Keyword(k) if k == "ISO-10303-21" => self.expect(Token::Semicolon)?,
                Token::Keyword(k) if k == "END-ISO-10303-21" => {
                    self.expect(Token::Semicolon)?;
                    break;
                }
                Token::Keyword(k) if k == "HEADER" => {
                    self.expect(Token::Semicolon)?;
                    loop {
                        match self.lexer.next()? {
                            Token::Keyword(k) if k == "ENDSEC" => {
                                self.expect(Token::Semicolon)?;
                                break;
                            }
                            Token::Keyword(name) => {
                                let record = self.record_after(name)?;
                                exchange.header.push(record);
                                self.expect(Token::Semicolon)?;
                            }
                            other => {
                                return Err(self
                                    .lexer
                                    .error(format!("expected a header entry, found {other:?}")));
                            }
                        }
                    }
                }
                Token::Keyword(k) if k == "DATA" => {
                    // `DATA('name', ('schema'));` names a section in files of
                    // several; there is no need to tell them apart here.
                    match self.lexer.next()? {
                        Token::Semicolon => {}
                        Token::Open => {
                            self.list_after_open()?;
                            self.expect(Token::Semicolon)?;
                        }
                        other => {
                            return Err(self
                                .lexer
                                .error(format!("expected `;` after DATA, found {other:?}")));
                        }
                    }
                    self.data(&mut exchange)?;
                }
                other => {
                    return Err(self
                        .lexer
                        .error(format!("expected a section, found {other:?}")));
                }
            }
        }
        Ok(exchange)
    }

    fn data(&mut self, exchange: &mut Exchange) -> GeopResult<()> {
        loop {
            match self.lexer.next()? {
                Token::Keyword(k) if k == "ENDSEC" => {
                    self.expect(Token::Semicolon)?;
                    return Ok(());
                }
                Token::Ref(id) => {
                    self.expect(Token::Equals)?;
                    let instance = match self.lexer.next()? {
                        Token::Keyword(name) => Instance::Simple(self.record_after(name)?),
                        Token::Open => {
                            let mut records = Vec::new();
                            loop {
                                match self.lexer.next()? {
                                    Token::Close => break,
                                    Token::Keyword(name) => records.push(self.record_after(name)?),
                                    other => {
                                        return Err(self.lexer.error(format!(
                                            "#{id}: expected an entity of a complex instance, found {other:?}"
                                        )));
                                    }
                                }
                            }
                            Instance::Complex(records)
                        }
                        other => {
                            return Err(self.lexer.error(format!(
                                "#{id}: expected an entity, found {other:?}"
                            )));
                        }
                    };
                    self.expect(Token::Semicolon)?;
                    if exchange.instances.insert(id, instance).is_some() {
                        return Err(self.lexer.error(format!("#{id} is defined twice")));
                    }
                }
                other => {
                    return Err(self
                        .lexer
                        .error(format!("expected an instance `#id = ...`, found {other:?}")));
                }
            }
        }
    }

    /// The parameters of the entity `name`, whose name was just read.
    fn record_after(&mut self, name: String) -> GeopResult<Record> {
        self.expect(Token::Open)?;
        Ok(Record {
            name,
            args: self.list_after_open()?,
        })
    }

    /// The values of a list whose `(` was just read, up to its `)`.
    fn list_after_open(&mut self) -> GeopResult<Vec<Value>> {
        let mut values = Vec::new();
        let mut token = self.lexer.next()?;
        if token == Token::Close {
            return Ok(values);
        }
        loop {
            values.push(self.value(token)?);
            match self.lexer.next()? {
                Token::Comma => token = self.lexer.next()?,
                Token::Close => return Ok(values),
                other => {
                    return Err(self
                        .lexer
                        .error(format!("expected `,` or `)` in a list, found {other:?}")));
                }
            }
        }
    }

    fn value(&mut self, token: Token) -> GeopResult<Value> {
        Ok(match token {
            Token::Null => Value::Null,
            Token::Derived => Value::Derived,
            Token::Integer(i) => Value::Integer(i),
            Token::Real(r) => Value::Real(r),
            Token::String(s) => Value::String(s),
            Token::Enum(e) => Value::Enum(e),
            Token::Ref(id) => Value::Ref(id),
            Token::Binary(b) => Value::Binary(b),
            Token::Open => Value::List(self.list_after_open()?),
            Token::Keyword(name) => {
                self.expect(Token::Open)?;
                let mut inner = self.list_after_open()?;
                let value = if inner.len() == 1 {
                    inner.pop().expect("one value")
                } else {
                    Value::List(inner)
                };
                Value::Typed(name, Box::new(value))
            }
            other => {
                return Err(self
                    .lexer
                    .error(format!("expected a value, found {other:?}")));
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "ISO-10303-21;
HEADER;
/* a comment */
FILE_DESCRIPTION(('a test'),'2;1');
FILE_NAME('it''s.stp','2026-10-04T00:00:00',('me'),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(0.,1.5,-2.E-3));
#2 = DIRECTION ( 'd' , ( 0.0 , 0.0 , 1.0 ) ) ;
#3=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#4=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#3,'distance_accuracy_value','');
#5=B_SPLINE_CURVE_WITH_KNOTS('',2,(#1,#1,#1),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);
#6=PRODUCT('caf\\X2\\00E9\\X0\\','',$,(#7));
ENDSEC;
END-ISO-10303-21;
";

    #[test]
    fn a_file_reads_with_complex_instances_and_typed_values() {
        let exchange = Exchange::parse(SAMPLE).unwrap();
        assert_eq!(exchange.header.len(), 3);
        assert_eq!(
            exchange.header[1].args[0],
            Value::String("it's.stp".into())
        );
        let point = exchange.get(1).unwrap().record("CARTESIAN_POINT").unwrap();
        assert_eq!(
            point.args[1],
            Value::List(vec![Value::Real(0.0), Value::Real(1.5), Value::Real(-2e-3)])
        );
        let unit = exchange.get(3).unwrap();
        assert!(unit.is("SI_UNIT") && unit.is("LENGTH_UNIT"));
        assert_eq!(
            unit.record("SI_UNIT").unwrap().args,
            vec![Value::Enum("MILLI".into()), Value::Enum("METRE".into())]
        );
        assert_eq!(
            exchange.get(4).unwrap().record("UNCERTAINTY_MEASURE_WITH_UNIT").unwrap().args[0],
            Value::Typed("LENGTH_MEASURE".into(), Box::new(Value::Real(1e-5)))
        );
        assert_eq!(
            exchange.get(6).unwrap().record("PRODUCT").unwrap().args[0],
            Value::String("café".into())
        );
        assert!(exchange.get(7).is_err());
    }

    #[test]
    fn a_written_file_reads_back_the_same() {
        let exchange = Exchange::parse(SAMPLE).unwrap();
        let again = Exchange::parse(&exchange.write()).unwrap();
        assert_eq!(exchange, again);
    }

    #[test]
    fn reals_are_written_as_part_21_reads_them() {
        assert_eq!(real(1.0), "1.0");
        assert_eq!(real(-0.25), "-0.25");
        assert_eq!(real(1.5e-7), "1.5E-7");
        assert_eq!(real(1e20), "1.E20");
        for r in [0.1, 1.0 / 3.0, -123456.789e-30, 2.5e300] {
            let Value::Real(back) = Exchange::parse(&format!(
                "DATA;#1=A({});ENDSEC;",
                real(r)
            ))
            .unwrap()
            .get(1)
            .unwrap()
            .record("A")
            .unwrap()
            .args[0]
            else {
                panic!()
            };
            assert_eq!(back, r);
        }
    }

    #[test]
    fn errors_say_where() {
        let error = Exchange::parse("ISO-10303-21;\nDATA;\n#1=A(1,;\nENDSEC;").unwrap_err();
        assert!(error.to_string().contains("line 3"), "{error}");
    }
}
