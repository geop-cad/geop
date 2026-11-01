#!/usr/bin/env bash
# Package a built geop (with the features `frontend,app,desktop`) as what a
# person double-clicks: an app in a disk image on macOS, a single program on
# Windows, a package that installs into the menu on Linux. Named without a
# version, so that the site can link to the latest release's.
#
#   scripts/package-desktop.sh <version> <target> <binary> <out-dir>
#
# Nothing here signs with a publisher's certificate: see docs/desktop.md for
# what that asks of a person the first time.
set -euo pipefail

version="$1"
target="$2"
binary="$3"
out="$4"
root="$(cd "$(dirname "$0")/.." && pwd)"
assets="$root/cad/geop-cad-cli/assets"
work="$(mktemp -d)"
mkdir -p "$out"

case "$target" in
  *apple-darwin)
    case "$target" in
      aarch64*) arch=arm64 ;;
      *) arch=x64 ;;
    esac
    app="$work/Geop.app"
    mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"
    cp "$binary" "$app/Contents/MacOS/Geop"
    cp "$assets/geop.icns" "$app/Contents/Resources/geop.icns"
    cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>Geop</string>
  <key>CFBundleDisplayName</key><string>Geop</string>
  <key>CFBundleIdentifier</key><string>dev.geop-cad.geop</string>
  <key>CFBundleExecutable</key><string>Geop</string>
  <key>CFBundleIconFile</key><string>geop</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$version</string>
  <key>CFBundleVersion</key><string>$version</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
PLIST
    # Apple silicon runs nothing unsigned: this signs it as no one in
    # particular, which is no publisher's signature.
    codesign --force --deep --sign - "$app"
    ln -s /Applications "$work/Applications"
    hdiutil create -volname "Geop" -srcfolder "$work" -ov -format UDZO \
      "$out/geop-desktop-macos-$arch.dmg"
    ;;
  *windows*)
    cp "$binary" "$out/geop-desktop-windows-x64.exe"
    ;;
  x86_64-unknown-linux-gnu)
    deb="$work/geop-desktop"
    mkdir -p "$deb/DEBIAN" "$deb/usr/bin" "$deb/usr/share/applications" \
      "$deb/usr/share/icons/hicolor/512x512/apps"
    install -m 755 "$binary" "$deb/usr/bin/geop-desktop"
    install -m 644 "$assets/geop.png" "$deb/usr/share/icons/hicolor/512x512/apps/geop.png"
    cat > "$deb/usr/share/applications/geop.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Geop
Comment=Parametric CAD
Exec=geop-desktop %f
Icon=geop
Terminal=false
Categories=Graphics;Engineering;
DESKTOP
    cat > "$deb/DEBIAN/control" <<CONTROL
Package: geop-desktop
Version: $version
Section: graphics
Priority: optional
Architecture: amd64
Depends: libwebkit2gtk-4.1-0, libgtk-3-0
Maintainer: geop <noreply@geop-cad.dev>
Homepage: https://geop-cad.dev
Description: Geop, a parametric CAD editor
 The geop editor in a window of its own.
CONTROL
    dpkg-deb --root-owner-group --build "$deb" "$out/geop-desktop-linux-x64.deb"
    ;;
  *)
    echo "no desktop package for $target" >&2
    exit 2
    ;;
esac
rm -rf "$work"
