# Downloads the official VPX B2S plugin and what it builds against, unchanged,
# from vpinball/vpinball at the commit of VPinballX_BGFX 10.8.1 revision 6533.
import json, os, sys, urllib.request

COMMIT = '85914a8504689e65769148a2eb012d6022c74074'
ROOT = os.path.dirname(os.path.abspath(__file__))
UP = os.path.join(ROOT, 'upstream')
tree = json.load(open(sys.argv[1]))['tree']

wanted = []
for x in tree:
    p = x['path']
    if x['type'] != 'blob':
        continue
    if (p.startswith('plugins/b2s/') or p.startswith('plugins/plugins/')
            or p.startswith('third-party/include/tinyxml2/')
            or p in ('third-party/include/unordered_dense.h', 'third-party/include/stl.h',
                     'third-party/include/simple-uri-parser/uri_parser.h',
                     'make/CMakeLists_plugin_B2S.txt', 'make/plugin-b2s.vcxproj',
                     'make/plugin-b2s.vcxproj.filters', 'LICENSE')):
        wanted.append((p, x['size']))

total = 0
for p, size in wanted:
    url = 'https://raw.githubusercontent.com/vpinball/vpinball/%s/%s' % (COMMIT, p)
    dest = os.path.join(UP, *p.split('/'))
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    data = urllib.request.urlopen(url, timeout=60).read()
    assert len(data) == size, (p, len(data), size)
    open(dest, 'wb').write(data)
    total += len(data)
print('downloaded %d files, %d KB, all sizes match the commit tree' % (len(wanted), total // 1024))
open(os.path.join(UP, 'UPSTREAM-COMMIT.txt'), 'w').write(
    'vpinball/vpinball %s\nVPinballX_BGFX 10.8.1 revision 6533 (VPinballX_BGFX64.exe 10.8.1.6533.85914a850)\n'
    'Files under upstream/ are unchanged copies.\n' % COMMIT)
