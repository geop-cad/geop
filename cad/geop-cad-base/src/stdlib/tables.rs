//! The dimensions of the standard parts, in millimetres, from the norms'
//! tables: nominal values (the middle of a tolerance, or the nominal size
//! where a norm gives only limits) — what a part is drawn and mated with.
//!
//! Each family is a list of sizes, a size a name and its dimensions in the
//! order of the family's columns. Fasteners come in the lengths the norm
//! lists for their size; a [`Table`] has one row per size and length.

/// The rows a family's file offers, as a table parameter holds them: the
/// columns' names, and per row its name and a value per column.
pub struct Table {
    pub columns: &'static [&'static str],
    pub rows: Vec<(String, Vec<f64>)>,
    /// The row a file is built with unless told otherwise.
    pub selected: &'static str,
}

impl Table {
    /// One row per size.
    fn of<const N: usize>(
        columns: &'static [&'static str],
        sizes: &[(&str, [f64; N])],
        selected: &'static str,
    ) -> Self {
        assert_eq!(columns.len(), N, "a value per column");
        let rows = sizes
            .iter()
            .map(|(name, values)| (name.to_string(), values.to_vec()))
            .collect();
        Self {
            columns,
            rows,
            selected,
        }
    }

    /// One row per size and length — `M3x10` — the length its last
    /// column, `l`.
    fn by_length<const N: usize>(
        columns: &'static [&'static str],
        sizes: &[(&str, [f64; N], &[f64])],
        selected: &'static str,
    ) -> Self {
        assert_eq!(columns.len(), N + 1, "a value per column, and the length");
        assert_eq!(columns[N], "l", "the length is the last column");
        let rows = sizes
            .iter()
            .flat_map(|(name, values, lengths)| {
                lengths.iter().map(move |&l| {
                    let mut row = values.to_vec();
                    row.push(l);
                    (format!("{name}x{}", number(l)), row)
                })
            })
            .collect();
        Self {
            columns,
            rows,
            selected,
        }
    }

    /// The value of `column` in the row `row`.
    pub fn value(&self, row: &str, column: &str) -> Option<f64> {
        let k = self.columns.iter().position(|c| *c == column)?;
        let (_, values) = self.rows.iter().find(|(name, _)| name == row)?;
        Some(values[k])
    }
}

/// `l` as a row's name shows it: `10`, `2.5`.
fn number(l: f64) -> String {
    format!("{l}")
}

/// ISO 4762 socket head cap screws: thread `d`, pitch `p`, head diameter
/// `dk` and height `k`, socket across flats `s` and depth `t`, length `l`
/// under the head.
pub fn iso4762() -> Table {
    Table::by_length(
        &["d", "p", "dk", "k", "s", "t", "l"],
        &[
            ("M2", [2.0, 0.4, 3.8, 2.0, 1.5, 1.0], &[3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0]),
            ("M2.5", [2.5, 0.45, 4.5, 2.5, 2.0, 1.1], &[4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0]),
            ("M3", [3.0, 0.5, 5.5, 3.0, 2.5, 1.3], &[4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0]),
            ("M4", [4.0, 0.7, 7.0, 4.0, 3.0, 2.0], &[5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
            ("M5", [5.0, 0.8, 8.5, 5.0, 4.0, 2.5], &[6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0]),
            ("M6", [6.0, 1.0, 10.0, 6.0, 5.0, 3.0], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0]),
            ("M8", [8.0, 1.25, 13.0, 8.0, 6.0, 4.0], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0]),
            ("M10", [10.0, 1.5, 16.0, 10.0, 8.0, 5.0], &[12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0]),
            ("M12", [12.0, 1.75, 18.0, 12.0, 10.0, 6.0], &[16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]),
        ],
        "M3x10",
    )
}

/// ISO 7380-1 button head screws: as [`iso4762`], the head a dome of
/// diameter `dk` and height `k`.
pub fn iso7380() -> Table {
    Table::by_length(
        &["d", "p", "dk", "k", "s", "t", "l"],
        &[
            ("M3", [3.0, 0.5, 5.7, 1.65, 2.0, 1.04], &[6.0, 8.0, 10.0, 12.0]),
            ("M4", [4.0, 0.7, 7.6, 2.2, 2.5, 1.3], &[8.0, 10.0, 12.0, 16.0]),
            ("M5", [5.0, 0.8, 9.5, 2.75, 3.0, 1.56], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0]),
            ("M6", [6.0, 1.0, 10.5, 3.3, 4.0, 2.08], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0]),
            ("M8", [8.0, 1.25, 14.0, 4.4, 5.0, 2.6], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
            ("M10", [10.0, 1.5, 17.5, 5.5, 6.0, 3.12], &[16.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
            ("M12", [12.0, 1.75, 21.0, 6.6, 8.0, 4.16], &[16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0]),
        ],
        "M3x10",
    )
}

/// ISO 10642 countersunk screws, 90°: as [`iso4762`], the head a cone
/// from `dk` at the top down to `d`, `k` deep; the length `l` overall,
/// head included.
pub fn iso10642() -> Table {
    Table::by_length(
        &["d", "p", "dk", "k", "s", "t", "l"],
        &[
            ("M3", [3.0, 0.5, 6.72, 1.86, 2.0, 1.1], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0]),
            ("M4", [4.0, 0.7, 8.96, 2.48, 2.5, 1.5], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
            ("M5", [5.0, 0.8, 11.2, 3.1, 3.0, 1.9], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0]),
            ("M6", [6.0, 1.0, 13.44, 3.72, 4.0, 2.2], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0]),
            ("M8", [8.0, 1.25, 17.92, 4.96, 5.0, 3.0], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0]),
            ("M10", [10.0, 1.5, 22.4, 6.2, 6.0, 3.6], &[12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0]),
            ("M12", [12.0, 1.75, 26.88, 7.44, 8.0, 4.3], &[20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0]),
        ],
        "M4x12",
    )
}

/// ISO 4017 hex head screws, threaded to the head: thread `d`, pitch `p`,
/// head across flats `s` and height `k`, length `l` under the head.
pub fn iso4017() -> Table {
    Table::by_length(
        &["d", "p", "s", "k", "l"],
        &[
            ("M2", [2.0, 0.4, 4.0, 1.4], &[4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0]),
            ("M2.5", [2.5, 0.45, 5.0, 1.7], &[5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0]),
            ("M3", [3.0, 0.5, 5.5, 2.0], &[6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0]),
            ("M4", [4.0, 0.7, 7.0, 2.8], &[8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
            ("M5", [5.0, 0.8, 8.0, 3.5], &[10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0]),
            ("M6", [6.0, 1.0, 10.0, 4.0], &[12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0]),
            ("M8", [8.0, 1.25, 13.0, 5.3], &[16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0]),
            ("M10", [10.0, 1.5, 16.0, 6.4], &[20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0]),
            ("M12", [12.0, 1.75, 18.0, 7.5], &[25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0]),
        ],
        "M5x20",
    )
}

/// ISO 4032 hex nuts: thread `d`, pitch `p`, across flats `s`, height
/// `m`, bearing face diameter `dw`.
pub fn iso4032() -> Table {
    Table::of(
        &["d", "p", "s", "m", "dw"],
        &[
            ("M2", [2.0, 0.4, 4.0, 1.6, 3.1]),
            ("M2.5", [2.5, 0.45, 5.0, 2.0, 4.1]),
            ("M3", [3.0, 0.5, 5.5, 2.4, 4.6]),
            ("M4", [4.0, 0.7, 7.0, 3.2, 5.9]),
            ("M5", [5.0, 0.8, 8.0, 4.7, 6.9]),
            ("M6", [6.0, 1.0, 10.0, 5.2, 8.9]),
            ("M8", [8.0, 1.25, 13.0, 6.8, 11.6]),
            ("M10", [10.0, 1.5, 16.0, 8.4, 14.6]),
            ("M12", [12.0, 1.75, 18.0, 10.8, 16.6]),
        ],
        "M5",
    )
}

/// ISO 10511 prevailing torque (nylon insert) nuts, as an envelope: as
/// [`iso4032`], `h` high overall, the hex `m` of it — taken as two thirds,
/// the norm giving only a minimum — and the insert's collar `dw` across
/// above it.
pub fn iso10511() -> Table {
    Table::of(
        &["d", "p", "s", "h", "m", "dw"],
        &[
            ("M3", [3.0, 0.5, 5.5, 4.0, 2.7, 4.6]),
            ("M4", [4.0, 0.7, 7.0, 5.0, 3.3, 5.9]),
            ("M5", [5.0, 0.8, 8.0, 5.0, 3.3, 6.9]),
            ("M6", [6.0, 1.0, 10.0, 6.0, 4.0, 8.9]),
            ("M8", [8.0, 1.25, 13.0, 8.0, 5.3, 11.6]),
            ("M10", [10.0, 1.5, 16.0, 10.0, 6.7, 14.6]),
            ("M12", [12.0, 1.75, 18.0, 12.0, 8.0, 16.6]),
        ],
        "M5",
    )
}

/// ISO 7089 plain washers, normal series: hole `d1`, outside `d2`,
/// thickness `h`.
pub fn iso7089() -> Table {
    Table::of(
        &["d1", "d2", "h"],
        &[
            ("M2", [2.2, 5.0, 0.3]),
            ("M2.5", [2.7, 6.0, 0.5]),
            ("M3", [3.2, 7.0, 0.5]),
            ("M4", [4.3, 9.0, 0.8]),
            ("M5", [5.3, 10.0, 1.0]),
            ("M6", [6.4, 12.0, 1.6]),
            ("M8", [8.4, 16.0, 1.6]),
            ("M10", [10.5, 20.0, 2.0]),
            ("M12", [13.0, 24.0, 2.5]),
        ],
        "M5",
    )
}

/// ISO 7090 plain washers, chamfered, normal series: as [`iso7089`],
/// which the norm starts at M5.
pub fn iso7090() -> Table {
    Table::of(
        &["d1", "d2", "h"],
        &[
            ("M5", [5.3, 10.0, 1.0]),
            ("M6", [6.4, 12.0, 1.6]),
            ("M8", [8.4, 16.0, 1.6]),
            ("M10", [10.5, 20.0, 2.0]),
            ("M12", [13.0, 24.0, 2.5]),
        ],
        "M5",
    )
}

/// ISO 8734 dowel pins, hardened: diameter `d`, end chamfer `c`, length
/// `l` — the preferred lengths in the norm's range for each diameter.
pub fn iso8734() -> Table {
    const PREFERRED: [f64; 15] = [
        4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 24.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0,
    ];
    let sizes: [(&str, [f64; 2], f64, f64); 10] = [
        ("1.5", [1.5, 0.3], 4.0, 16.0),
        ("2", [2.0, 0.35], 5.0, 20.0),
        ("2.5", [2.5, 0.4], 6.0, 24.0),
        ("3", [3.0, 0.5], 8.0, 30.0),
        ("4", [4.0, 0.63], 10.0, 40.0),
        ("5", [5.0, 0.8], 12.0, 50.0),
        ("6", [6.0, 1.2], 14.0, 60.0),
        ("8", [8.0, 1.6], 18.0, 80.0),
        ("10", [10.0, 2.0], 22.0, 100.0),
        ("12", [12.0, 2.5], 26.0, 100.0),
    ];
    let lengths: Vec<Vec<f64>> = sizes
        .iter()
        .map(|&(_, _, lo, hi)| {
            PREFERRED
                .into_iter()
                .filter(|l| (lo..=hi).contains(l))
                .collect()
        })
        .collect();
    let sizes: Vec<(&str, [f64; 2], &[f64])> = sizes
        .iter()
        .zip(&lengths)
        .map(|(&(name, values, ..), lengths)| (name, values, &lengths[..]))
        .collect();
    Table::by_length(&["d", "c", "l"], &sizes, "4x20")
}

/// Deep-groove ball bearings, open: bore `d`, outside `D`, width `B`,
/// edge chamfer `r`.
pub fn ball_bearings() -> Table {
    Table::of(
        &["d", "D", "B", "r"],
        &[
            ("623", [3.0, 10.0, 4.0, 0.15]),
            ("625", [5.0, 16.0, 5.0, 0.3]),
            ("608", [8.0, 22.0, 7.0, 0.3]),
            ("6000", [10.0, 26.0, 8.0, 0.3]),
            ("6001", [12.0, 28.0, 8.0, 0.3]),
            ("6002", [15.0, 32.0, 9.0, 0.3]),
            ("6003", [17.0, 35.0, 10.0, 0.3]),
            ("6004", [20.0, 42.0, 12.0, 0.6]),
            ("6005", [25.0, 47.0, 12.0, 0.6]),
            ("6800", [10.0, 19.0, 5.0, 0.3]),
            ("6801", [12.0, 21.0, 5.0, 0.3]),
            ("6802", [15.0, 24.0, 5.0, 0.3]),
            ("6803", [17.0, 26.0, 5.0, 0.3]),
            ("6804", [20.0, 32.0, 7.0, 0.3]),
            ("6805", [25.0, 37.0, 7.0, 0.3]),
        ],
        "608",
    )
}

/// Hex standoffs, threaded through (female at both ends): thread `d`,
/// across flats `s`, length `l`.
pub fn hex_standoffs() -> Table {
    Table::by_length(
        &["d", "s", "l"],
        &[
            ("M2.5", [2.5, 5.0], &[5.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0]),
            ("M3", [3.0, 5.5], &[5.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0]),
        ],
        "M3x10",
    )
}

/// NEMA 17 stepper motors, as an envelope: body length `L`.
pub fn nema17() -> Table {
    Table::of(
        &["L"],
        &[("34", [34.0]), ("40", [40.0]), ("48", [48.0])],
        "40",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_are_named_by_size_and_length() {
        let screws = iso4762();
        assert_eq!(screws.value("M3x10", "dk"), Some(5.5));
        assert_eq!(screws.value("M3x10", "l"), Some(10.0));
        assert_eq!(iso8734().value("4x20", "l"), Some(20.0));
        assert_eq!(iso8734().value("2.5x6", "d"), Some(2.5));
    }
}
