"""Build an SDK command file from the upstream T1 planner, without connecting a robot."""
import argparse
import importlib
import json
from pathlib import Path
import sys

from astribot_draw_sdk import export_model_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--t1-dir', type=Path, required=True, help='Astribot-draw/t1 from GitHub')
    parser.add_argument('--assets', type=Path, required=True, help='aos/descriptions root containing astribot_t1')
    parser.add_argument('--design', choices=['waves', 'leaves', 'spiral', 'wings'], default='waves')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.t1_dir.resolve()
    sys.path.insert(0, str(source))
    import mujoco
    from t1_model import build_t1_scene, CollisionChecker
    planner = importlib.import_module('t1_wings' if args.design == 'wings' else 't1_sync')
    args.output.mkdir(parents=True, exist_ok=True)
    model = build_t1_scene(args.output, assets=args.assets.resolve(), exclude_compound_elbow=True)
    home, q, dq, phase, _desired = planner.plan(model, args.design, args.output)
    data, checker = mujoco.MjData(model), CollisionChecker(model)

    def clearance(full_pose):
        data.qpos[:] = full_pose
        mujoco.mj_forward(model, data)
        return checker.check(data)['distance_m']

    paths = export_model_trajectory(args.output/'sdk-plan.npz', model, q, dq,
        float(model.opt.timestep), phase, final_home=home, validate_state=clearance,
        provenance={'planner_directory': str(source), 'design': args.design,
                    'model_revision': 'cab872dc00081d2c5b096c7a19e563050eb7a2ff',
                    'compound_elbow_excluded': True,
                    'collision_coverage': str(args.output/'t1-collision-model.json')})
    print(json.dumps({'plan': str(paths[0]), 'validation': str(paths[1])}))


if __name__ == '__main__':
    main()
