"""Resolve the user-facing 0..57 index using standard Python only."""
import argparse
import importlib.util
import json
from pathlib import Path

path = Path(__file__).resolve().parents[1]/'src/astribot_draw_sdk/pattern_catalog.py'
spec = importlib.util.spec_from_file_location('pattern_catalog', path)
catalog = importlib.util.module_from_spec(spec); spec.loader.exec_module(catalog)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', type=Path)
    parser.add_argument('index', type=int, nargs='?')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.list:
        data = json.loads((args.library/'pattern_lookup.json').read_text(encoding='utf-8'))
        for i, row in enumerate(sorted(data['patterns'].values(), key=lambda r: int(r['number']))):
            print(f"{i:2d}  {row['pattern_id']:28s} {row['name_en']}")
    else:
        _, row, files = catalog.select_pattern(args.library, args.index)
        print(row['pattern_id'])
        print(row['name_en'])
        for name in ('source.svg', 'sdk-plan.npz', 'sdk-plan.validation.json'):
            print(files[name])
