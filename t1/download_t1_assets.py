"""Materialize only LFS data referenced by the pinned official Astribot T1 MJCF and dedicated collision meshes."""
import concurrent.futures, hashlib, json, pathlib, subprocess, urllib.request
import xml.etree.ElementTree as ET
ROOT = pathlib.Path(__file__).resolve().parent
REPO = (ROOT / 'descriptions').resolve()
COMMIT = 'cab872dc00081d2c5b096c7a19e563050eb7a2ff'
if subprocess.check_output(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip() != COMMIT:
    raise SystemExit('Description repository is not at the tested commit.')
entry = REPO / 'astribot_t1/model/mjcf/robot/t1.xml'
paths = set()
def visit(xml):
    for elem in ET.parse(xml).iter():
        if elem.tag == 'include': visit((xml.parent / elem.get('file')).resolve())
        elif elem.tag in ('mesh','texture') and elem.get('file'):
            paths.add((xml.parent / elem.get('file')).resolve())
visit(entry)
paths.update((REPO/'astribot_t1/mesh').rglob('*_collision_mesh.obj'))
files=[]
for path in sorted(paths):
    relative=path.relative_to(REPO).as_posix()
    pointer=subprocess.check_output(['git','-C',str(REPO),'show',f'{COMMIT}:{relative}'])
    if pointer.startswith(b'version https://git-lfs.github.com/spec/v1'):
        fields=dict(line.split(' ',1) for line in pointer.decode().splitlines())
        files.append((path,relative,fields['oid'].split(':')[1],int(fields['size'])))

def fetch(item):
    path, relative, expected_hash, size = item
    if path.stat().st_size == size and hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash:
        print('verified existing', relative, flush=True)
        return {'path': relative, 'sha256': expected_hash, 'bytes': size}
    url = f'https://media.githubusercontent.com/media/Astribot-Dev/astribot_descriptions/{COMMIT}/{relative}'
    temp = path.with_suffix(path.suffix + '.partial')
    print('downloading', relative, size, flush=True)
    with urllib.request.urlopen(url, timeout=120) as source, temp.open('wb') as target:
        while chunk := source.read(1024 * 1024): target.write(chunk)
    digest = hashlib.sha256(temp.read_bytes()).hexdigest()
    if temp.stat().st_size != size or digest != expected_hash:
        raise ValueError(f'LFS checksum mismatch: {relative}')
    temp.replace(path)
    print('verified', relative, flush=True)
    return {'path': relative, 'sha256': digest, 'bytes': size}
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    manifest = list(executor.map(fetch, files))
(ROOT / 'asset_manifest.json').write_text(json.dumps({'commit': COMMIT, 'lfs_assets': manifest}, indent=2) + '\n')
print('LFS asset bytes:', sum(x['bytes'] for x in manifest))
