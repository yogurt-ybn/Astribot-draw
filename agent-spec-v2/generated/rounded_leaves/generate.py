#!/usr/bin/env python3
"""Migrate supplied v1 geometry unchanged and render synchronized v2 design."""
from pathlib import Path
import json,sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT))
from drawing_contract import save_design
d=json.loads((HERE/'source-v1-paths.json').read_text())
d['schema_version']='astribot-drawing-v2'
d['execution']=json.loads((ROOT/'execution-contract.json').read_text())
save_design(d,HERE)
