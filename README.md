# B2S Pro Plugin for Visual Pinball X

A Visual Pinball X plugin that shows B2S Pro backglasses (`.B2SPro`) made with the
[B2S Pro Designer](https://github.com/KWildman69/B2S-Pro), plus legacy `.directb2s` backglasses.
It is built on the B2S plugin that ships with VPX.

Version 1.1.0, for VPinballX_BGFX 10.8.1 (64-bit Windows). Please report problems on the [Issues](https://github.com/KWildman69/B2S-Pro-Plugin/issues) page.

## Install

1. Download `B2S-Pro-Plugin-1.1.0.zip` from the [Releases](https://github.com/KWildman69/B2S-Pro-Plugin/releases) page.
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

Requirements: Visual Studio 2026 (version 18) with C++ desktop development and CMake.

```
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release
```

The plugin is written to `build\Release` (`plugin-b2spro64.dll` and `plugin.cfg`).

- `src\` is the B2S Pro version of VPX's `plugins/b2s`; every change is marked `B2S Pro:`.
- `upstream\` holds unchanged copies of the VPX files the plugin builds against, from
  vpinball/vpinball commit `85914a8504689e65769148a2eb012d6022c74074` (VPinballX_BGFX 10.8.1
  revision 6533), and PinMAME headers from vpinball/pinmame `9dfcbd0f2539d96f8739f1c78250716135fa4b74`.
  See `upstream\UPSTREAM-COMMIT.txt`. `fetch-upstream.py` records how they were downloaded.

## Credits and license

Based on the B2S plugin by the Visual Pinball team ([vpinball/vpinball](https://github.com/vpinball/vpinball)),
which reimplements the B2S Backglass Server by Herweh and the B2S team.

This plugin is free software under the GNU General Public License, version 3 or later. See `LICENSE`.
The Visual Pinball license notice is in `upstream\LICENSE`. Third-party files under `upstream\third-party`
keep their own license notices.
