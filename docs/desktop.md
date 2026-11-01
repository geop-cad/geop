# Geop on the desktop

For anyone who does not want a terminal: download the program, double-click
it, and the editor opens in a window of its own. It runs on your machine,
needs no account and no connection, and your parts are ordinary files.

| Your computer | Download | Then |
|---|---|---|
| Windows 10 or 11 | [geop-desktop-windows-x64.exe](https://github.com/geop-cad/geop/releases/latest/download/geop-desktop-windows-x64.exe) | Double-click it. It needs no installing. |
| Mac with Apple silicon (M1 and later) | [geop-desktop-macos-arm64.dmg](https://github.com/geop-cad/geop/releases/latest/download/geop-desktop-macos-arm64.dmg) | Open it and drag Geop to Applications. |
| Mac with an Intel chip | [geop-desktop-macos-x64.dmg](https://github.com/geop-cad/geop/releases/latest/download/geop-desktop-macos-x64.dmg) | Open it and drag Geop to Applications. |
| Ubuntu 22.04 or later, Debian 12 or later | [geop-desktop-linux-x64.deb](https://github.com/geop-cad/geop/releases/latest/download/geop-desktop-linux-x64.deb) | Double-click it to install, or `sudo apt install ./geop-desktop-linux-x64.deb`. Geop is then in your applications. |

## The first time you open it

Geop is not yet signed with a publisher's certificate, so your computer warns
you the first time. This is once only:

- **Windows** says "Windows protected your PC". Choose **More info**, then
  **Run anyway**.
- **macOS** says it cannot check the app for malicious software. Close that,
  open **System Settings → Privacy & Security**, scroll down and choose
  **Open Anyway** beside Geop.

Each release lists the SHA-256 of every download in `SHA256SUMS`, to check
against if you wish.

## Where your work goes

Geop opens the folder you last worked in. The first time, that is a folder
called `Geop` in your Documents, which it makes for you. **File → Open folder…**
opens another. Each part is a `.geop` file in that folder, to copy, back up or
send like any other file.

Windows needs the WebView2 runtime, which Windows 11 and current Windows 10
have already. On Linux the package brings in WebKitGTK.

## For developers

Each download is the `geop` command line tool built with the feature `desktop`
(`scripts/package-desktop.sh`, `.github/workflows/release.yml`): with no
arguments it opens the app as `geop app` does, and on Windows it opens no
console. To edit a folder from a terminal, or use the other commands, install
`geop` itself (see the README).
