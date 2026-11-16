# Fetch the upstream files this plugin builds against, from the official repositories.
# They are NOT hosted in this repository: run this script before the first build
# (and after changing the pinned commits), it needs network access:
#
#     python3 fetch-upstream.py            # the files the build needs (plus LICENSE)
#     python3 fetch-upstream.py --all      # also the official plugins/b2s sources, for reference diffs
#     python3 fetch-upstream.py --check    # verify the local copies, no network
#
# Every file is pinned by its exact commit and by the SHA-256 of its content: the
# raw.githubusercontent.com URLs are immutable, the manifest is the tamper check.
# The files are downloaded unchanged into upstream/ (gitignored); the CMakeLists
# of this repository builds the plugin against them (see the README).
import argparse, hashlib, os, sys, urllib.request

VPX_COMMIT = '85914a8504689e65769148a2eb012d6022c74074'   # VPinballX_BGFX 10.8.1 revision 6533
PINMAME_COMMIT = '9dfcbd0f2539d96f8739f1c78250716135fa4b74'  # PINMAME_SHA in platforms/config.sh at the VPX commit

# The files the build needs, under upstream/ (the source lists of make/CMakeLists_plugin_B2S.txt,
# minus the plugin's own sources which live in src/, plus the headers they include)
BUILD_FILES = [
   ('LICENSE',
    '638deeada443c15ec6e198730b3c63b7222edc4476e272672b9b4c875192999a'),
   ('plugins/nixie/NixieLayoutFile.cpp',
    'ebeaa912fe36c34cb8151b75ac164b954b97bed6425eac0fdd9ee0bcfcaa0d09'),
   ('plugins/nixie/NixieLayoutFile.h',
    '5b5632233ce9ecdc10a08397329685d5aeb5d8fd713e2bcf4669f8fe0c4392cf'),
   ('plugins/plugins/ControllerPlugin.h',
    'b9b48930d6ed63844c5e341aad91d3c8cff01f02ee45dfbba2a45c0c9bdf3f53'),
   ('plugins/plugins/LoggingPlugin.h',
    '46e98350f86e34936cc250824421de366e5dce5ecd1636ba904faddae537f4b0'),
   ('plugins/plugins/MsgPlugin.h',
    'cdf932ac0b65c321be28eca24407a04c3009ae494806c6e30853fc347df61de3'),
   ('plugins/plugins/PagedUIPlugin.h',
    '79874a4c571e8db68490e25a7e3a9b49b9d3917c31f56fec4b3db2210804550d'),
   ('plugins/plugins/PluginStrings.h',
    '7c7962d52e5fc68b91d99f70192465f0577cdbf0eee9e59f12e3c814e8b23a79'),
   ('plugins/plugins/ResURIResolver.cpp',
    'fecaa162a3e9514ed72082304891b8a5e425f8420f46c98fd5d34a0dc27c39d3'),
   ('plugins/plugins/ResURIResolver.h',
    '093d6249e7979d80f406be6146180491359c08f859bd2cb19f2c788245490cdd'),
   ('plugins/plugins/ScriptablePlugin.h',
    'b947292e29f0f947177b45f09aecfb5db9e5fec5ae8c484c9535cdb90dab8a91'),
   ('plugins/plugins/VPXPlugin.h',
    '7f821c5f309c554e42db69c3afb295deed77a1dcc4579627dc38df20ca6cad19'),
   ('third-party/include/simple-uri-parser/uri_parser.h',
    'dc468e9cd882b6928635444e189e49d3b7add7b33b4ff8c3a66a19f5db0717d6'),
   # the companion of unordered_dense.h (the <stl> module replacement it includes)
   ('third-party/include/stl.h',
    'efc0170257b6148ee7655b22a34465f6ce105f1b720e557167ef27a59d6d5241'),
   ('third-party/include/tinyxml2/tinyxml2.cpp',
    '5deb62cd22013ba01505851811297d4f234347f9462fe52ddb121aebf935a55d'),
   ('third-party/include/tinyxml2/tinyxml2.h',
    '79923d1f8e56cc454c4edda228943d682c1581d2532219687c459ca2c8463070'),
   ('third-party/include/unordered_dense.h',
    'ad4e2fb7de67f8c7265028440b4250885c8a57b541565c3cd0ff9dbfaa942237'),
]

# The official B2S plugin sources and build files, kept as the reference the changes
# in src/ are diffed against; fetched with --all, not needed by the build
REFERENCE_FILES = [
   ('make/CMakeLists_plugin_B2S.txt',
    'e4bf377b6608d6c25086e02f8a84515a46fd24736c3af9f926afa60648e4da87'),
   ('make/plugin-b2s.vcxproj',
    'aef3384edb185933a891fe8cfc092bd44f120fa23cce8adb6ad5a1dcbd6dbdca'),
   ('make/plugin-b2s.vcxproj.filters',
    '577d3155c74255a8162271e0f9c41c54f912efa6d6ab928e704d1e756462459a'),
   ('plugins/b2s/B2SDataModel.cpp',
    '22dc66e94d4e2e1db2b074e2a6b06c1d1b5daf6e6caebea5edb1941c67d08474'),
   ('plugins/b2s/B2SDataModel.h',
    '2aacfde81836c2319dc3372454b26d7d9c34821e8efe6aa6b469b56e962eb63a'),
   ('plugins/b2s/B2SPlugin.cpp',
    'c715ad85595f0ea1837f03bd507c4cb6c34c1e33a008b5e228c94e2490d47a96'),
   ('plugins/b2s/B2SRenderer.cpp',
    'e20de8142d575230271c4440f58fcd2c1a5d4c9465a8f2ae3bc3ffdb4a2574bf'),
   ('plugins/b2s/B2SRenderer.h',
    '07dee127769b02c2fe06dd3a8b68f5e021ee4271c280a5ae5522394e13db12c1'),
   ('plugins/b2s/B2SServer.cpp',
    'c0f37410904c1762ed7afe44c7cf7b976e68c83764d3cefe9e4c3e8e6217fc8b'),
   ('plugins/b2s/B2SServer.h',
    'efb782a8b8a7a1cfe0c2f910e421eb80f6ba5dba975626747cd7f1ee1ea97df9'),
   ('plugins/b2s/DMDOverlay/DMDOverlay.cpp',
    '00476ebf1a6bfe1ceff44c42436c774a1db0265b011f9aaa1d5b368950308195'),
   ('plugins/b2s/DMDOverlay/DMDOverlay.h',
    '8f7c655be1d9a2236c1e31a5a3078782a6ad4fbdb9b458e9b9d8be81f0b8a40a'),
   ('plugins/b2s/DMDOverlay/OverlayDisplayOptions.h',
    'd18cbb774a615df3d988613a70d967a4da96b9d5a53e38416eff0ce01ac056e5'),
   ('plugins/b2s/base64.h',
    '2903f02d320cf8ecd03f2cc01e272d93118ea50cc9a6506c60d3cfda3906118f'),
   ('plugins/b2s/common.cpp',
    'ca54c9fa8c9bd686977c7cd9d2f53f184c4524994220549a3ae0c217c1c62976'),
   ('plugins/b2s/common.h',
    'f40380019cb2e06c3bcf0202e9e0ac1676f57aef1c0fc94336c8669de3d36ba7'),
   ('plugins/b2s/plugin.cfg',
    '8a0a484aeeb1ca25194c3616fcc196b317a9e4d198800ff4a6d4549a17b7516c'),
   ('plugins/b2s/seg/Brush.h',
    '0888af66b27c136a7281bc9b29abadce6149175b99f25e0d2612c2f228381e98'),
   ('plugins/b2s/seg/Dream7Display.cpp',
    '417e1feb57c2a20ff2f8b9d1b2d1c73b69474316732a25ecbec2f96825a96f38'),
   ('plugins/b2s/seg/Dream7Display.h',
    'd7dd754029376ea34a89eaf165da3bf743a0155b120f30405f2b2bddc830d510'),
   ('plugins/b2s/seg/GraphicsPath.cpp',
    'a02499b9055eb1aada2a469851f93df462c260508663215795f20a96574719de'),
   ('plugins/b2s/seg/GraphicsPath.h',
    '75c9d09cea9519216aa0748696128d256320bdbadb68c2d08fc132cfe8bc1416'),
   ('plugins/b2s/seg/LEDBox.cpp',
    '4ffa2f31eeee43a1d579f094860fd29237d01d9cdea23f63d716e508b1d95d22'),
   ('plugins/b2s/seg/LEDBox.h',
    '9bd07bdb5f4d9bcdb19ea701c59595d73271068d6f72fcee688e56262406d709'),
   ('plugins/b2s/seg/Matrix.cpp',
    '0a9275b1d7ee5e4d5aec0913f0b118ac99bd14b87a2fdbfa205a75d9d304bc16'),
   ('plugins/b2s/seg/Matrix.h',
    '4b2a4ebf4610c536fb2f27099cd22e8daec836c2c0db2a02c8ebe38294467ba8'),
   ('plugins/b2s/seg/SegDisplay.h',
    '747eeca7728f4e94214295d258f10e4be1ceab7350a9905524eb01d32897c115'),
   ('plugins/b2s/seg/SegGraphics.cpp',
    '8c77b90e2a223ebb5da93e1959ac1964c226d937bddf751dbdc00c4dbf850e54'),
   ('plugins/b2s/seg/SegGraphics.h',
    'a0fa8e91c3884f2baa28bc613c8bfb18a66651257c43f63a8a52ceeaeec97f82'),
   ('plugins/b2s/seg/SegTypes.h',
    'd9928db2a4b2c70802d239fde54efbce07ca44ec347ad80d481662ae77589590'),
   ('plugins/b2s/seg/Segment.cpp',
    '8f4cf9ba11f839e8be7edcffaa132e991b50a8b28c74fd95ecbba43a1bc1ed96'),
   ('plugins/b2s/seg/Segment.h',
    'cbaaaeace27dd2f354e2d8186bfdf28de25518b3030088858039ddb67a878c91'),
   ('plugins/b2s/seg/SegmentNumber.cpp',
    '758697506ed9084b1f657612c54e6c08b03515ace2168422ab616138e3b3bbdc'),
   ('plugins/b2s/seg/SegmentNumber.h',
    '686b52fac519b999a325e74e345b7066ff15bb3d4be1081efc8418e8b0200ffb'),
   ('plugins/b2s/seg/SegmentStyle.h',
    '378722634ea2310cc26a3d52ccfe75930d797b723386d03adf203837f26384a4'),
]

# The PinMAME plugin interface header, placed as platforms/windows-x64/external.sh
# extracts it (third-party/include/pinmame, from the vpinball/pinmame commit)
PINMAME_FILES = [
   ('src/libpinmame/PinMAMEPlugin.h', 'third-party/include/pinmame/PinMAMEPlugin.h',
    '69a0b80b421bcddabbaa20521b4b70e25e22ddeff1611fa18828f8e2b9b4bcf5'),
]

ROOT = os.path.dirname(os.path.abspath(__file__))
UP = os.path.join(ROOT, 'upstream')

def local(path):
    return os.path.join(UP, *path.split('/'))

def digest(path):
    return hashlib.sha256(open(local(path), 'rb').read()).hexdigest()

def url_of(rel, commit, repo):
    return 'https://raw.githubusercontent.com/%s/%s/%s' % (repo, commit, rel)

def write_commit_file():
    with open(os.path.join(UP, 'UPSTREAM-COMMIT.txt'), 'w') as f:
        f.write(
           'vpinball/vpinball %s\n'
           'VPinballX_BGFX 10.8.1 revision 6533 (VPinballX_BGFX64.exe 10.8.1.6533.85914a850)\n'
           'Files under upstream/ are unchanged copies, fetched by fetch-upstream.py (not hosted here).\n'
           'PinMAME headers: vpinball/pinmame %s, as platforms/windows-x64/external.sh extracts them.\n'
           % (VPX_COMMIT, PINMAME_COMMIT))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--all', action='store_true', help='also fetch the reference plugins/b2s sources')
    ap.add_argument('--check', action='store_true', help='only verify the local copies against the manifest')
    args = ap.parse_args()

    items = [(url_of(p, VPX_COMMIT, 'vpinball/vpinball'), p, p, sha) for p, sha in BUILD_FILES]
    items += [(url_of(rel, PINMAME_COMMIT, 'vpinball/pinmame'), rel, dest, sha) for rel, dest, sha in PINMAME_FILES]
    if args.all:
        items += [(url_of(p, VPX_COMMIT, 'vpinball/vpinball'), p, p, sha) for p, sha in REFERENCE_FILES]

    ok = down = 0
    for url, rel, dest, sha in items:
        if os.path.exists(local(dest)) and digest(dest) == sha:
            ok += 1
            continue
        if args.check:
            print('MISSING or MODIFIED: %s' % dest, file=sys.stderr)
            sys.exit(1)
        data = urllib.request.urlopen(url, timeout=60).read()
        if hashlib.sha256(data).hexdigest() != sha:
            print('SHA-256 MISMATCH for %s (the pinned commit should make this impossible: '
                  'refuse the download, report it)' % rel, file=sys.stderr)
            sys.exit(1)
        path = local(dest)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(data)
        print('fetched %s' % dest)
        down += 1

    write_commit_file()
    print('%d files up to date, %d fetched' % (ok, down))

if __name__ == '__main__':
    main()
