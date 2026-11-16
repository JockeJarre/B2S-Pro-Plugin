# B2S Pro Plugin for Visual Pinball X

A Visual Pinball X plugin that shows B2S Pro backglasses (`.B2SPro`) made with the
[B2S Pro Designer](https://github.com/KWildman69/B2S-Pro), plus legacy `.directb2s` backglasses.
It is built on the B2S plugin that ships with VPX.

Version 1.0.0, for VPinballX_BGFX 10.8.1 (64-bit Windows). Please report problems on the [Issues](https://github.com/KWildman69/B2S-Pro-Plugin/issues) page.

## Install

1. Download `B2S-Pro-Plugin-1.0.0.zip` from the [Releases](https://github.com/KWildman69/B2S-Pro-Plugin/releases) page.
   To check the download, compare its SHA-256 with the `.sha256` file.
2. Close VPX.
3. Unzip it and copy the `b2spro` folder into VPX's `plugins` folder, so you have
   `<VPX folder>\plugins\b2spro\plugin-b2spro64.dll` and `plugin.cfg`.
4. Start VPX and open the plugin settings. Enable **B2S Pro** and disable the built-in **B2S** plugin.

To uninstall, delete the `b2spro` folder.

## How it finds a backglass

For each table, the plugin looks for a `.B2SPro` file before a `.directb2s` file: first by the
table's file name, then by the folder name. B2S Pro behaviour is used only for files made with the
B2S Pro layer system; legacy backglasses run on the original B2S plugin code.
The plugin never writes into `.B2SPro` files.

## Settings

Shown in VPX's plugin settings for B2S Pro:

- **Show Grill**: show the grill area (off by default).
- **Dual Mode**: Authentic or Fantasy mode for dual backglasses.
- **Upgraded segment rendering**: off by default, which draws LED and VFD score displays the way the
  B2S Server does. Turn it on to use VPX's upgraded segment renderer instead.
- **Used LED type**: force the LED display type (Auto uses the type declared by the backglass).
- **Global animation slowdown**: slowdown factor applied to all backglass animations.

## Build from source

Requirements: Visual Studio 2026 (version 18) with C++ desktop development and CMake, Python,
and network access for the first build (the files the plugin builds against are not hosted in
this repository, they are fetched; the fetch is one-time, re-running it verifies the local copies).

```
python fetch-upstream.py
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

The plugin is written to `build\Release` (`plugin-b2spro64.dll` and `plugin.cfg`). A fresh
configure without the fetch fails with the same hint. On Linux/macOS: `python3 fetch-upstream.py`
then the usual `cmake -B build` (single-config, add `-DCMAKE_BUILD_TYPE=Release`).

What `fetch-upstream.py` downloads into `upstream\` (gitignored), and why:

- `plugins\plugins\*.h` + `ResURIResolver.cpp` - the official plugin API headers (message plugin,
  VPX/scriptable/logging/controller/paged-UI APIs, plugin strings) and the resource URI resolver
  the plugin builds. Exactly the set the official `make/CMakeLists_plugin_B2S.txt` compiles,
  minus the plugin's own sources (those are ours in `src\`).
- `plugins\nixie\NixieLayoutFile.*` - the shared nixie layout file parser the segment renderer uses.
- `third-party\include\`: `tinyxml2\` (the `.directb2s` parser), `unordered_dense.h` with its
  companion `stl.h` (hash maps), `simple-uri-parser\uri_parser.h` (resource URIs), and
  `pinmame\PinMAMEPlugin.h` (the PinMAME
  plugin interface, from the vpinball/pinmame commit VPX pins).
- `LICENSE` - the Visual Pinball license notice.

Every file is pinned by the exact commit and by its SHA-256 in the manifest inside the script;
`python fetch-upstream.py --check` verifies the local copies. `--all` additionally fetches the
unchanged official `plugins\b2s` sources and `make\` files as the reference to diff `src\` against.

- `src\` is the B2S Pro version of VPX's `plugins/b2s`; every change is marked `B2S Pro:`.
- `upstream\UPSTREAM-COMMIT.txt` (written by the fetch) names the source commits:
  vpinball/vpinball commit `85914a8504689e65769148a2eb012d6022c74074` (VPinballX_BGFX 10.8.1
  revision 6533) and PinMAME from vpinball/pinmame `9dfcbd0f2539d96f8739f1c78250716135fa4b74`.
  Files under `upstream\` are unchanged copies.

## Credits and license

Based on the B2S plugin by the Visual Pinball team ([vpinball/vpinball](https://github.com/vpinball/vpinball)),
which reimplements the B2S Backglass Server by Herweh and the B2S team.

This plugin is free software under the GNU General Public License, version 3 or later. See `LICENSE`.
The Visual Pinball license notice is in `upstream\LICENSE`, fetched by `fetch-upstream.py`. Third-party
files under `upstream\third-party` (also fetched) keep their own license notices.
