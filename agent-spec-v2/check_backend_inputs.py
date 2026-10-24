#!/usr/bin/env python3
"""Check v2 JSON geometry against actual backend NPZ inputs, without executing motion."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
from validate_drawing import validate
ROOT=Path(__file__).resolve().parent

def main():
    backend=Path(sys.argv[1]).resolve()
    contract=json.loads((ROOT/'execution-contract.json').read_text())
    results={}
    for name in ('tidal_waves','rounded_leaves','twin_galaxies'):
        folder=ROOT/'generated'/name
        data=json.loads((folder/'paths.json').read_text())
        assert data['execution']==contract
        backend_json=json.loads((backend/'inputs'/name/'paths.json').read_text())
        assert backend_json==data, 'backend JSON differs from validated v2 input'
        assert validate(folder)['design_pass']
        npz_path=backend/'inputs'/name/'seed-plans.npz'
        seed=np.load(npz_path)
        arms={}
        for stroke in data['strokes']:
            uv=np.array(stroke['points'],dtype=float)
            scale=data['canvas']['meters_per_pixel']
            world=np.c_[(uv[:,0]-285)*scale,np.full(len(uv),-.2),.95-(uv[:,1]-360)*scale]
            stored=seed[stroke['arm']+'_world']
            equal=world.shape==stored.shape and np.array_equal(world,stored)
            assert equal,(name,stroke['arm'],'backend geometry differs')
            arms[stroke['arm']]={'exact_world_array_match':equal,'point_count':len(world),'maximum_difference_m':float(np.abs(world-stored).max())}
        results[name]={'design_and_contract_valid':True,'seed_npz_sha256':hashlib.sha256(npz_path.read_bytes()).hexdigest(),'arms':arms}
    report={'scope':'Input compatibility only, not execution or collision certificate. Backend reproduction adapter reads v2 JSON, checks simultaneous=true and exact world agreement with seed NPZ. Full static metadata is validated by this gate; historical measured runs used identical NPZ world targets before the adapter was added.','designs':results}
    (ROOT/'backend-input-compatibility.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
