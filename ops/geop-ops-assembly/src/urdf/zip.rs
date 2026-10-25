//! [`zip`]: files packed into one ZIP archive, stored as they are.
//!
//! A robot is a URDF file and its meshes; a browser can offer one file to
//! save, so they go into one archive. Every unzip tool reads a stored
//! (uncompressed) entry, and that is all this writes: a local header and
//! the bytes per file, then the central directory — no compression, no
//! ZIP64 (a robot's meshes are far below 4 GB), and every entry dated
//! 1980-01-01, so the same robot packs into the same bytes.

use geop_core_math::geop_error::{GeopError, GeopResult};

/// The CRC-32 (IEEE 802.3, reflected) of `bytes`, as ZIP checks entries by.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// `files` — each a path inside the archive, `/`-separated, and its bytes
/// — as a ZIP archive. Fails for a file or an archive too large for a
/// plain (non-ZIP64) archive.
pub fn zip(files: &[(String, Vec<u8>)]) -> GeopResult<Vec<u8>> {
    let small = |n: usize, what: &str| {
        u32::try_from(n)
            .map_err(|_| GeopError::new(format!("{what} is too large for a ZIP archive")))
    };
    let mut out = Vec::new();
    let mut central = Vec::new();
    for (path, bytes) in files {
        let offset = small(out.len(), "the archive")?;
        let size = small(bytes.len(), path)?;
        let name_len = u16::try_from(path.len()).map_err(|_| {
            GeopError::new(format!("the path {path:?} is too long for a ZIP archive"))
        })?;
        let crc = crc32(bytes);
        // What the local header and the central directory's entry share:
        // version needed (1.0), flags (UTF-8 names), method (stored), time
        // and date (1980-01-01 00:00), CRC, sizes, name length.
        let mut common = Vec::new();
        common.extend(10u16.to_le_bytes());
        common.extend((1u16 << 11).to_le_bytes());
        common.extend(0u16.to_le_bytes());
        common.extend(0u16.to_le_bytes());
        common.extend(((1u16 << 5) | 1).to_le_bytes());
        common.extend(crc.to_le_bytes());
        common.extend(size.to_le_bytes());
        common.extend(size.to_le_bytes());
        common.extend(name_len.to_le_bytes());

        out.extend(0x0403_4b50u32.to_le_bytes());
        out.extend(&common);
        out.extend(0u16.to_le_bytes()); // extra field length
        out.extend(path.as_bytes());
        out.extend(bytes);

        central.extend(0x0201_4b50u32.to_le_bytes());
        central.extend(20u16.to_le_bytes()); // made by: 2.0, MS-DOS
        central.extend(&common);
        central.extend([0u8; 12]); // extra, comment, disk, attributes
        central.extend(offset.to_le_bytes());
        central.extend(path.as_bytes());
    }
    let count = u16::try_from(files.len())
        .map_err(|_| GeopError::new("too many files for a ZIP archive"))?;
    let start = small(out.len(), "the archive")?;
    let length = small(central.len(), "the archive's directory")?;
    out.extend(central);
    out.extend(0x0605_4b50u32.to_le_bytes());
    out.extend([0u8; 4]); // this disk, the directory's disk
    out.extend(count.to_le_bytes());
    out.extend(count.to_le_bytes());
    out.extend(length.to_le_bytes());
    out.extend(start.to_le_bytes());
    out.extend(0u16.to_le_bytes()); // comment length
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn u16_at(bytes: &[u8], at: usize) -> usize {
        u16::from_le_bytes([bytes[at], bytes[at + 1]]) as usize
    }

    fn u32_at(bytes: &[u8], at: usize) -> usize {
        u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as usize
    }

    /// The check value every CRC-32 implementation is tested against.
    #[test]
    fn crc_of_the_check_string() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    /// Read back the way an unzip tool reads: from the end record to the
    /// central directory, from each entry to its local header and bytes.
    #[test]
    fn files_are_found_through_the_central_directory() {
        let files = vec![
            ("robot.urdf".to_string(), b"<robot/>".to_vec()),
            ("meshes/link.stl".to_string(), vec![0, 1, 2, 255]),
        ];
        let archive = zip(&files).unwrap();
        let end = archive.len() - 22;
        assert_eq!(u32_at(&archive, end), 0x0605_4b50);
        assert_eq!(u16_at(&archive, end + 10), 2);
        let mut entry = u32_at(&archive, end + 16);
        for (path, bytes) in &files {
            assert_eq!(u32_at(&archive, entry), 0x0201_4b50);
            assert_eq!(u32_at(&archive, entry + 16), crc32(bytes) as usize);
            let name_len = u16_at(&archive, entry + 28);
            assert_eq!(&archive[entry + 46..entry + 46 + name_len], path.as_bytes());
            let local = u32_at(&archive, entry + 42);
            assert_eq!(u32_at(&archive, local), 0x0403_4b50);
            let size = u32_at(&archive, local + 22);
            let data = local + 30 + u16_at(&archive, local + 26) + u16_at(&archive, local + 28);
            assert_eq!(&archive[data..data + size], bytes.as_slice());
            entry += 46 + name_len;
        }
        assert_eq!(entry, end);
    }
}
