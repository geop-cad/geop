//! The GitHub release this build of `geop` belongs to, and its files, as
//! `.github/workflows/release.yml` names them and `homepage/install.sh`
//! fetches them: every file listed with its SHA-256 in `SHA256SUMS`, and
//! nothing downloaded is used before its sum is checked.

use geop_core_math::geop_error::{GeopError, GeopResult};
use sha2::{Digest, Sha256};
use std::io::Read;

/// Where releases are downloaded from: `<base>/v<version>/<file>`.
/// `GEOP_RELEASE_BASE_URL` replaces it, for a mirror or a test.
const RELEASES: &str = "https://github.com/geop-cad/geop/releases/download";

/// The largest file a release holds that is read: an extension, with the
/// kernel in it, is some tens of megabytes.
const LARGEST: u64 = 512 << 20;

/// The URL of the file `file` of the release `version`, under `base`.
pub fn url(base: &str, version: &str, file: &str) -> String {
    format!("{}/v{version}/{file}", base.trim_end_matches('/'))
}

/// The base URL releases are downloaded from (see [`RELEASES`]).
pub fn base() -> String {
    std::env::var("GEOP_RELEASE_BASE_URL").unwrap_or_else(|_| RELEASES.to_string())
}

/// VS Code's name of the platform `os`/`arch` (as `std::env::consts`
/// names them), which an extension with a native program in it is built
/// for: `vsce package --target`.
pub fn vscode_target(os: &str, arch: &str) -> Option<&'static str> {
    Some(match (os, arch) {
        ("linux", "x86_64") => "linux-x64",
        ("linux", "aarch64") => "linux-arm64",
        ("macos", "x86_64") => "darwin-x64",
        ("macos", "aarch64") => "darwin-arm64",
        ("windows", "x86_64") => "win32-x64",
        _ => return None,
    })
}

/// The VS Code extension of the release `version` for the VS Code platform
/// `target`: it carries the kernel built for that platform.
pub fn vsix_name(version: &str, target: &str) -> String {
    format!("geop-vscode-{version}-{target}.vsix")
}

/// The SHA-256 `sums` (a `SHA256SUMS` file, as `sha256sum` writes it:
/// `<hex>  <name>`, or `<hex> *<name>` for one read as binary) gives
/// `name`, in lower case.
pub fn listed_sum(sums: &str, name: &str) -> Option<String> {
    sums.lines().find_map(|line| {
        let (sum, file) = line.trim_end().split_once(' ')?;
        let file = file.strip_prefix([' ', '*'])?;
        let valid = sum.len() == 64 && sum.bytes().all(|b| b.is_ascii_hexdigit());
        (valid && file == name).then(|| sum.to_ascii_lowercase())
    })
}

/// The SHA-256 of `bytes`, in lower-case hex.
pub fn sum_of(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// `bytes`, the file `name`, if `sums` lists it with their SHA-256.
pub fn verified(bytes: Vec<u8>, sums: &str, name: &str) -> GeopResult<Vec<u8>> {
    let listed = listed_sum(sums, name)
        .ok_or_else(|| GeopError::new(format!("SHA256SUMS lists no {name}")))?;
    let actual = sum_of(&bytes);
    if actual != listed {
        return Err(GeopError::new(format!(
            "{name} is not the file released: its SHA-256 is {actual}, SHA256SUMS says {listed}"
        )));
    }
    Ok(bytes)
}

/// The file at `url`, downloaded whole.
pub fn fetch(url: &str) -> GeopResult<Vec<u8>> {
    let failed = |e: ureq::Error| match e {
        ureq::Error::StatusCode(404) => GeopError::new(format!("{url} does not exist")),
        e => GeopError::new(format!("downloading {url}: {e}")),
    };
    let response = ureq::get(url)
        .header("User-Agent", concat!("geop/", env!("CARGO_PKG_VERSION")))
        .call()
        .map_err(failed)?;
    let mut bytes = Vec::new();
    response
        .into_body()
        .into_reader()
        .take(LARGEST + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| GeopError::new(format!("downloading {url}: {e}")))?;
    if bytes.len() as u64 > LARGEST {
        return Err(GeopError::new(format!(
            "{url} is larger than {LARGEST} bytes"
        )));
    }
    Ok(bytes)
}

/// The file `file` of the release `version`, checked against the
/// release's `SHA256SUMS`.
pub fn download(version: &str, file: &str) -> GeopResult<Vec<u8>> {
    let base = base();
    let sums = fetch(&url(&base, version, "SHA256SUMS")).map_err(|e| {
        e.with_context(format!(
            "download(version={version}, file={file}): no release v{version} to download from \
             (is this a development build? a release is cut with scripts/release.sh)"
        ))
    })?;
    let sums = String::from_utf8_lossy(&sums);
    verified(fetch(&url(&base, version, file))?, &sums, file)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_release_file_is_under_its_tag() {
        assert_eq!(
            url("https://example.org/dl/", "0.2.0", "SHA256SUMS"),
            "https://example.org/dl/v0.2.0/SHA256SUMS"
        );
        assert_eq!(
            url(RELEASES, "0.2.0-rc1", &vsix_name("0.2.0-rc1", "linux-x64")),
            "https://github.com/geop-cad/geop/releases/download/v0.2.0-rc1/geop-vscode-0.2.0-rc1-linux-x64.vsix"
        );
    }

    /// Every platform the release builds `geop` for has an extension, and
    /// no other one is guessed at.
    #[test]
    fn every_released_platform_has_a_vscode_target() {
        for (os, arch, target) in [
            ("linux", "x86_64", "linux-x64"),
            ("linux", "aarch64", "linux-arm64"),
            ("macos", "x86_64", "darwin-x64"),
            ("macos", "aarch64", "darwin-arm64"),
            ("windows", "x86_64", "win32-x64"),
        ] {
            assert_eq!(vscode_target(os, arch), Some(target), "{os} {arch}");
        }
        assert_eq!(vscode_target("windows", "aarch64"), None);
        assert_eq!(vscode_target("freebsd", "x86_64"), None);
        assert!(vscode_target(std::env::consts::OS, std::env::consts::ARCH).is_some());
    }

    #[test]
    fn sums_are_read_as_sha256sum_writes_them() {
        let a = "a".repeat(64);
        let b = "B".repeat(64);
        let sums = format!(
            "{a}  geop-0.2.0-x86_64-unknown-linux-gnu.tar.gz\n{b} *geop-vscode-0.2.0-linux-x64.vsix\r\n"
        );
        assert_eq!(
            listed_sum(&sums, "geop-0.2.0-x86_64-unknown-linux-gnu.tar.gz"),
            Some(a)
        );
        assert_eq!(
            listed_sum(&sums, "geop-vscode-0.2.0-linux-x64.vsix"),
            Some("b".repeat(64))
        );
        // Not a prefix, not a suffix: the name itself.
        assert_eq!(listed_sum(&sums, "geop-vscode-0.2.0-linux-x64"), None);
        assert_eq!(listed_sum(&sums, "x64.vsix"), None);
        assert_eq!(listed_sum("nothex  file\n", "file"), None);
    }

    #[test]
    fn a_file_is_used_only_if_its_sum_is_listed_and_matches() {
        let bytes = b"the extension".to_vec();
        let sums = format!("{}  geop.vsix\n", sum_of(&bytes));
        assert_eq!(verified(bytes.clone(), &sums, "geop.vsix").unwrap(), bytes);
        let err = verified(b"something else".to_vec(), &sums, "geop.vsix").unwrap_err();
        assert!(err.to_string().contains("not the file released"), "{err}");
        let err = verified(bytes, &sums, "other.vsix").unwrap_err();
        assert!(err.to_string().contains("lists no other.vsix"), "{err}");
    }

    #[test]
    fn sha256_is_the_standard_one() {
        assert_eq!(
            sum_of(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
