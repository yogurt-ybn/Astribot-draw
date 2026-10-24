"""Freeze identical source/config, run untouched SVGs sequentially. No case-specific settings."""
import hashlib,json,os,subprocess,sys,time,platform,importlib.metadata
from pathlib import Path
ROOT=Path(__file__).resolve().parent
CONFIG={'coupled_ik':'deterministic branch-pair backtracking, max169 pairs, full3947pair checking every5 path knots','timing':'exact cubic derivative extrema bound','mode':'simulate','seeds':12,'candidate_order':['height_coupled','bend_0.45','bend_0.35','bend_0.55'],'ik_random_seed':36,'rrt_random_seed':12345,'maximum_steps':200000,'input_geometry_policy':'unchanged; no scaling or repair','initial_state':'same canonical whole-body neutral state; collision-checked 18DOF RRT approach and return'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    inputs=sorted((ROOT/'inputs').glob('*.svg')); run=ROOT/'runs'/'frozen-v3-analytic-time';run.mkdir(parents=True,exist_ok=True)
    if (run/'freeze.json').exists():raise SystemExit('Refusing to overwrite frozen benchmark')
    manifest={'config':CONFIG,'source_sha256':{p.name:sha(p) for p in sorted(ROOT.glob('*.py'))},'input_sha256':{p.name:sha(p) for p in inputs},'frozen_at_unix':time.time()}
    manifest['canvas_sha256']={str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'t1-canvas').rglob('*')) if p.is_file()}
    manifest['asset_manifest_sha256']=sha(ROOT/'asset_manifest.json') if (ROOT/'asset_manifest.json').exists() else None
    manifest['upstream_commit']='cab872dc00081d2c5b096c7a19e563050eb7a2ff'
    manifest['python']=platform.python_version()
    manifest['packages']={n:importlib.metadata.version(n) for n in ['numpy','scipy','pillow','mujoco']}
    (run/'freeze.json').write_text(json.dumps(manifest,indent=2));results=[]
    for p in inputs:
        out=run/p.stem;out.mkdir(exist_ok=True);start=time.time()
        cmd=[sys.executable,str(ROOT/'svg_plan.py'),str(p),'--out',str(out),'--mode',CONFIG['mode'],'--seeds',str(CONFIG['seeds'])]
        assert all(sha(ROOT/n)==h for n,h in manifest['source_sha256'].items())
        assert sha(p)==manifest['input_sha256'][p.name]
        print('START',p.name,flush=True)
        with (out/'run.log').open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
        assert sha(p)==manifest['input_sha256'][p.name]
        results.append({'input':p.name,'exit_code':r.returncode,'wall_seconds':time.time()-start,'report':json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else None})
        (run/'summary.json').write_text(json.dumps(results,indent=2));print('DONE',p.name,r.returncode,flush=True)
    assert all(sha(ROOT/n)==h for n,h in manifest['source_sha256'].items())
if __name__=='__main__':main()
