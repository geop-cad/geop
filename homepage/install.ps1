# Install the geop command line tool from its GitHub release, on Windows:
#
#   irm https://geop-cad.dev/install.ps1 | iex
#
# Downloads the archive for this machine and the release's SHA256SUMS,
# checks the archive's SHA-256 (and stops if it does not match), and puts
# geop.exe in %LOCALAPPDATA%\geop\bin. Nothing runs as administrator and
# PATH is not changed: if that folder is not on it, the command to add it
# is printed.
#
# Set before running, e.g. `$env:GEOP_VERSION = "0.2.0"`:
#   GEOP_VERSION           the version to install (default: the latest release)
#   GEOP_INSTALL_DIR       where to put geop.exe (default: %LOCALAPPDATA%\geop\bin)
#   GEOP_RELEASE_BASE_URL  where releases are downloaded from (default:
#                          https://github.com/geop-cad/geop/releases/download),
#                          a file is <base>/v<version>/<name>

& {
  $ErrorActionPreference = "Stop"
  $ProgressPreference = "SilentlyContinue"
  [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

  $repo = "geop-cad/geop"
  $base = if ($env:GEOP_RELEASE_BASE_URL) { $env:GEOP_RELEASE_BASE_URL.TrimEnd("/") } else { "https://github.com/$repo/releases/download" }
  $installDir = if ($env:GEOP_INSTALL_DIR) { $env:GEOP_INSTALL_DIR } else { Join-Path $env:LOCALAPPDATA "geop\bin" }

  $arch = [System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString()
  # An x64 build runs on an Arm machine too, emulated.
  if ($arch -ne "X64" -and $arch -ne "Arm64") { throw "geop install: no geop is released for the processor $arch" }
  $target = "x86_64-pc-windows-msvc"

  $version = $env:GEOP_VERSION
  if (-not $version) {
    $latest = Invoke-RestMethod -UseBasicParsing -Headers @{ "User-Agent" = "geop-install" } "https://api.github.com/repos/$repo/releases/latest"
    $version = $latest.tag_name
    if (-not $version) { throw "geop install: could not find the latest release of $repo (set GEOP_VERSION to choose one)" }
  }
  $version = $version.TrimStart("v")

  $name = "geop-$version-$target"
  $archive = "$name.zip"
  $tmp = Join-Path ([IO.Path]::GetTempPath()) ("geop-install-" + [Guid]::NewGuid())
  New-Item -ItemType Directory -Path $tmp | Out-Null
  try {
    Write-Host "downloading geop $version for $target"
    $sums = Join-Path $tmp "SHA256SUMS"
    $zip = Join-Path $tmp $archive
    Invoke-WebRequest -UseBasicParsing "$base/v$version/SHA256SUMS" -OutFile $sums
    Invoke-WebRequest -UseBasicParsing "$base/v$version/$archive" -OutFile $zip

    $expected = $null
    foreach ($line in Get-Content $sums) {
      $parts = $line.Trim() -split "\s+", 2
      if ($parts.Count -eq 2 -and $parts[1].TrimStart("*") -eq $archive) { $expected = $parts[0].ToLowerInvariant() }
    }
    if (-not $expected) { throw "geop install: SHA256SUMS lists no $archive" }
    $actual = (Get-FileHash -Algorithm SHA256 $zip).Hash.ToLowerInvariant()
    if ($actual -ne $expected) { throw "geop install: $archive is not the file released: its SHA-256 is $actual, SHA256SUMS says $expected" }

    Expand-Archive -Path $zip -DestinationPath $tmp -Force
    $exe = Join-Path $tmp "$name\geop.exe"
    if (-not (Test-Path $exe)) { throw "geop install: $archive holds no $name\geop.exe" }
    New-Item -ItemType Directory -Force -Path $installDir | Out-Null
    Copy-Item -Force $exe (Join-Path $installDir "geop.exe")
    Write-Host "installed geop $version to $(Join-Path $installDir 'geop.exe')"
  } finally {
    Remove-Item -Recurse -Force $tmp -ErrorAction SilentlyContinue
  }

  $onPath = ($env:PATH -split ";") | Where-Object { $_.TrimEnd("\") -ieq $installDir.TrimEnd("\") }
  $geop = "geop"
  if (-not $onPath) {
    $geop = Join-Path $installDir "geop.exe"
    Write-Host ""
    Write-Host "$installDir is not on your PATH. To add it for your user, run:"
    Write-Host "  [Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';$installDir', 'User')"
    Write-Host "and open a new terminal."
  }

  Write-Host ""
  Write-Host "Next:"
  Write-Host "  $geop open                          edit the .geop files of this folder in your browser"
  Write-Host "  claude mcp add geop -- $geop mcp    let Claude Code model parts with geop"
  Write-Host "  $geop vscode                        install the VS Code extension"
}
