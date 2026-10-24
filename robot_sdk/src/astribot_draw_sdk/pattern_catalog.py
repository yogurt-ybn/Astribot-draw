"""Stable zero-based pattern selection and hash checking, without SDK imports."""
import hashlib
import json
from pathlib import Path


def select_pattern(library, index):
    root = Path(library).resolve()
    query = json.loads((root/'pattern_lookup.json').read_text(encoding='utf-8'))
    records = sorted(query['patterns'].values(), key=lambda r: int(r['number']))
    if query.get('count') != 58 or len(records) != 58 or len({r['pattern_id'] for r in records}) != 58:
        raise ValueError('Expected the 58-pattern validated library')
    if [int(r['number']) for r in records] != [n for n in range(1,61) if n not in (20,24)]:
        raise ValueError('Pattern numbering differs from the validated 0..57 selection map')
    if type(index) is not int or not 0 <= index < len(records):
        raise ValueError('Pattern index must be an integer in 0..57')
    record = records[index]
    files = {}
    for name in ('source.svg', 'sdk-plan.npz', 'sdk-plan.validation.json'):
        asset = record['assets'][name]
        path = (root/asset['path']).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError('Missing or outside-library asset: '+name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != asset['sha256']:
            raise ValueError('Pattern asset SHA256 mismatch: '+name)
        files[name] = path
    return query, record, files
