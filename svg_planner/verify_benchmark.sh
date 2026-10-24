#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
"${PYTHON:-python}" - <<'PY'
from pathlib import Path
import json,hashlib,numpy as np
root=Path('.');run=root/'runs/frozen-v3-analytic-time';freeze=json.loads((run/'freeze.json').read_text());results=json.loads((run/'summary.json').read_text())
assert len(results)==3,'Benchmark incomplete'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(root/n)==h for n,h in freeze['source_sha256'].items())
assert all(sha(root/n)==h for n,h in freeze['canvas_sha256'].items())
assert all(sha(root/'inputs'/n)==h for n,h in freeze['input_sha256'].items())
common=None;checks=[]
for item in results:
 out=run/Path(item['input']).stem;r=item['report'];record={'input':item['input'],'status':r['status'],'stages':r['stages'],'source_input_unchanged':r['input_sha256']==freeze['input_sha256'][item['input']]}
 assert record['source_input_unchanged']
 if r['stages']['full_plan']=='passed':
  z=np.load(out/'plan.npz');home=z['home'];assert np.array_equal(z['qdes'][0],home) and np.array_equal(z['qdes'][-1],home)
  if common is None:common=home.copy()
  assert np.array_equal(home,common),'Different starting poses'
  record['identical_canonical_start_end']=True;record['plan_sha256']=sha(out/'plan.npz')
 if r['stages']['simulation']=='passed':
  z=np.load(out/'execution.npz');draw=z['phase']=='draw_both';err=np.linalg.norm(z['actual'][draw]-z['desired'][draw],axis=2)
  assert np.isfinite(z['qpos']).all() and np.isfinite(z['qvel']).all()
  assert err.max()<=.002;record['actual_max_tip_error_m']=float(err.max());record['execution_sha256']=sha(out/'execution.npz')
  record['actual_final_home_error_rad']=float(np.max(abs(z['qpos'][-1]-common)))
  assert np.array_equal(np.array(r['simulation']['initial_qpos']),common)
 checks.append(record)
(run/'verification.json').write_text(json.dumps({'source_canvas_inputs_hash_verified':True,'fresh_actual_simulations_no_reuse':True,'cases':checks,'canonical_home':common.tolist() if common is not None else None},indent=2));print(json.dumps(checks,indent=2))
PY
