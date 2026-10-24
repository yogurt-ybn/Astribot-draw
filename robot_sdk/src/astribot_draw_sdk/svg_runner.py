"""SVG -> measured-start planning -> official simulation SDK -> measured FK audit."""
import hashlib
import importlib
import inspect
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from .contracts import T1_JOINTS, Trajectory
from .exporter import export_model_trajectory
from .geometry import CollisionChecker, checked_transfer
from .ros_feedback import AUXILIARY_PARTS


def validate_auxiliary(auxiliary, *, skip_gripper_checks=False):
    now = time.perf_counter()
    for part, size in AUXILIARY_PARTS.items():
        if skip_gripper_checks and 'gripper' in part:
            continue
        data = auxiliary.get(part, {})
        value = np.asarray(data.get('position', []), dtype=float)
        age = now-data.get('received_at', float('-inf'))
        if value.shape != (size,) or not np.isfinite(value).all() or not 0 <= age <= .25:
            raise RuntimeError(f'SVG planning requires fresh complete {part} simulation feedback')


class FixedPartsGuard:
    def __init__(self, baseline, *, skip_gripper_checks=False):
        self.skip_gripper_checks = skip_gripper_checks
        validate_auxiliary(baseline, skip_gripper_checks=skip_gripper_checks)
        self.baseline = {name: np.array(baseline[name]['position']) for name in AUXILIARY_PARTS
                         if not (skip_gripper_checks and 'gripper' in name)}

    def __call__(self, _state, auxiliary):
        validate_auxiliary(auxiliary, skip_gripper_checks=self.skip_gripper_checks)
        for name, expected in self.baseline.items():
            tolerance = .002 if 'gripper' in name else (.001 if 'chassis' in name else .005)
            if np.max(np.abs(np.asarray(auxiliary[name]['position'])-expected)) > tolerance:
                raise RuntimeError(f'Uncommanded part moved during SVG execution: {name}')


def full_pose(model, command, auxiliary, *, skip_gripper_checks=False):
    validate_auxiliary(auxiliary, skip_gripper_checks=skip_gripper_checks)
    q = model.qpos0.copy()
    for name, value in zip(T1_JOINTS, command):
        q[model.jnt_qposadr[model.joint(name).id]] = value
    for i, value in enumerate(auxiliary['astribot_head']['position'], 1):
        q[model.jnt_qposadr[model.joint(f'astribot_head_joint_{i}').id]] = value
    for side in (() if skip_gripper_checks else ('left', 'right')):
        value = auxiliary[f'astribot_gripper_{side}']['position'][0]
        if abs(value) > .002:
            raise RuntimeError('SVG fixed-gripper model requires nearly closed grippers; no automatic gripper motion')
        q[model.jnt_qposadr[model.joint(f'astribot_gripper_{side}_joint_L1').id]] = value
    return q


def validate_native_contract(model, q, dq, phase, desired, *, joint_speed=.2):
    """Vectorized numerical checks; geometry is checked on the final SDK poses."""
    if not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise RuntimeError('Nonfinite planned position/velocity')
    if not np.isfinite(desired[phase == 'draw_both']).all():
        raise RuntimeError('Nonfinite desired drawing target')
    limited = np.flatnonzero(model.jnt_limited)
    values = q[:, model.jnt_qposadr[limited]]
    margin = float(np.minimum(values-model.jnt_range[limited, 0],
                              model.jnt_range[limited, 1]-values).min())
    maximum = float(np.max(np.abs(dq)))
    if margin < 0 or maximum > joint_speed+1e-6:
        raise RuntimeError('Native trajectory violates joint limits or the configured speed ceiling')
    return dict(minimum_joint_margin_rad=margin, maximum_commanded_velocity_rad_s=maximum,
                geometry_validation='deferred to every final 125 Hz SDK command')


def motion_time_scale(minimum_duration, duration_s, speed_scale):
    target = (minimum_duration if duration_s is None else duration_s)/speed_scale
    if target < minimum_duration-1e-9:
        raise RuntimeError(f'Requested {target:.3f}s is below the {minimum_duration:.3f}s '
                           'schedule minimum under the configured speed ceilings; increase --duration or lower --speed-scale')
    return minimum_duration/target


class SvgStateGuard(FixedPartsGuard):
    def __init__(self, baseline, model, home, checker, *, skip_gripper_checks=False):
        super().__init__(baseline, skip_gripper_checks=skip_gripper_checks)
        import mujoco
        self.mujoco, self.model, self.home, self.checker = mujoco, model, home, checker
        self.data, self.last_check, self.minimum_clearance = mujoco.MjData(model), 0., .05
        self.indices = np.array([model.jnt_qposadr[model.joint(n).id] for n in T1_JOINTS])

    def __call__(self, state, auxiliary):
        super().__call__(state, auxiliary)
        now = time.perf_counter()
        if now-self.last_check >= .05:
            self.data.qpos[:] = full_pose(self.model, state.position, auxiliary,
                                          skip_gripper_checks=self.skip_gripper_checks)
            self.mujoco.mj_kinematics(self.model, self.data)
            distance = self.checker.minimum_distance(self.data)
            self.minimum_clearance = min(self.minimum_clearance, distance)
            self.last_check = now
            if distance < 0:
                raise RuntimeError(f'Measured SVG pose intersects collision proxies: {distance:.6f} m')


class SvgJob:
    def __init__(self, robot, svg, output, *, planner_dir=None, assets=None,
                 seeds=12, speed_scale=1., plan_only=False, pipeline_started_at=None,
                 joint_speed=1., tip_speed=.2, duration_s=None, skip_gripper_checks=False):
        now = time.perf_counter()
        self.started = now if pipeline_started_at is None else pipeline_started_at
        self.stage_started = now
        if robot.backend_kind != 'sdk-sim' or robot.transport != 'ros':
            raise ValueError('SVG drawing requires the official SDK simulation ROS backend')
        self.skip_gripper_checks = bool(skip_gripper_checks)
        if not np.isfinite(speed_scale) or not 0 < speed_scale <= 2.5 or not 0 <= seeds <= 100:
            raise ValueError('speed_scale must be finite and in (0,2.5] and seeds in [0,100]')
        if any(not np.isfinite(v) or v <= 0 for v in (joint_speed, tip_speed)):
            raise ValueError('joint_speed and tip_speed must be finite and positive')
        if duration_s is not None and (not np.isfinite(duration_s) or duration_s <= 0):
            raise ValueError('duration must be finite and positive')
        if joint_speed > robot.policy.max_speed_rad_s+1e-9:
            raise ValueError('joint_speed exceeds the session execution policy')
        self.joint_speed, self.tip_speed, self.duration_s = joint_speed, tip_speed, duration_s
        self.robot, self.svg, self.output = robot, Path(svg).resolve(), Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        original_svg = self.svg.read_bytes()
        (self.output/'input.svg').write_bytes(original_svg)
        self.planner_dir = Path(planner_dir or Path(__file__).resolve().parents[3]/'svg_planner').resolve()
        self.assets = Path(assets or Path(os.environ.get('ASTRIBOT_SIMU_ROOT', '/opt/astribot_simulation'))/'astribot_descriptions')
        self.seeds, self.speed_scale, self.plan_only = seeds, speed_scale, plan_only
        self.handle = None
        self.report = dict(schema='astribot-sdk-svg-run/v1', status='planning',
                           svg=str(self.svg), svg_sha256=hashlib.sha256(original_svg).hexdigest(),
                           model_assets=str(self.assets), model_revision=os.getenv('ASTRIBOT_DRAW_MODEL_REVISION', 'unknown'),
                           speed_scale=speed_scale, command_preview_s=robot.policy.command_preview_s,
                           requested_duration_s=duration_s, joint_speed_limit_rad_s=joint_speed,
                           tip_speed_limit_m_s=tip_speed,
                           command_speed_ceiling_rad_s=robot.policy.max_speed_rad_s,
                           gripper_checks='skipped_simulation_only' if self.skip_gripper_checks else 'enabled',
                           gripper_geometry='fixed_model_preset' if self.skip_gripper_checks else 'near_closed_feedback',
                           stages={}, completion_behavior='stop_at_drawing_end',
                           timings_s=dict(sdk_initialization=max(0., now-self.started)),
                           scope='Official simulation; free-space virtual light tip, not surface-contact drawing',
                           limitations=['Sampled collision checks and upstream compound-joint exclusions.',
                                        'Passive gripper joints use the near-closed model preset.',
                                        'Measured tip positions are FK from joint feedback, not camera measurements.'])
        if self.skip_gripper_checks:
            self.report['limitations'].append('Gripper feedback checks disabled for simulation testing; '
                                              'collision geometry uses the fixed model preset, not actual finger articulation.')

    def save_report(self):
        (self.output/'svg-run.json').write_text(json.dumps(self.report, indent=2, allow_nan=False)+'\n', encoding='utf-8')

    def checkpoint(self, stage):
        now = time.perf_counter()
        timings = self.report['timings_s']
        timings[stage] = timings.get(stage, 0.)+now-self.stage_started
        self.stage_started = now

    def run(self):
        try:
            return self._run()
        except BaseException as exc:
            self.report.update(status='cancelled' if isinstance(exc, KeyboardInterrupt) else 'failed',
                               error=f'{type(exc).__name__}: {exc}')
            raise
        finally:
            self.report['timings_s']['input_to_result'] = time.perf_counter()-self.started
            self.save_report()

    def _run(self):
        if not (self.planner_dir/'svg_plan.py').is_file():
            raise ValueError('Missing upstream svg_planner directory; update GitHub code first')
        sys.path.insert(0, str(self.planner_dir))
        planner = importlib.import_module('svg_plan')
        if Path(planner.__file__).resolve().parent != self.planner_dir:
            raise RuntimeError('Another svg_plan module is already loaded from a different directory')
        if 'initial_qpos' not in inspect.signature(planner.ik_candidate).parameters:
            raise RuntimeError('Apply robot_sdk/patches/svg-measured-start.patch to svg_planner/svg_plan.py')
        if 'return_home' not in inspect.signature(planner.schedule).parameters:
            raise RuntimeError('Update svg_planner: SDK drawing requires stop-at-end scheduling support')
        if 'timing' not in inspect.signature(planner.schedule).parameters:
            raise RuntimeError('Update svg_planner: SDK drawing requires progress-bound timing support')
        if 'sampling' not in inspect.signature(planner.shared_targets).parameters:
            raise RuntimeError('Update svg_planner: SDK drawing requires uniform arc-progress sampling support')
        self.planner = planner
        self.mujoco = planner.mujoco
        planner.CollisionChecker = lambda model: CollisionChecker(model, planner.collision_pairs(model))
        planner.rrt_transfer = checked_transfer
        planner.validate_calibration(self.planner_dir/'t1-canvas')
        parsed = planner.parse_svg(self.output/'input.svg')
        self.report['static'] = planner.validate_masks(parsed, self.planner_dir/'t1-canvas')
        (self.output/'ordered-paths.json').write_text(json.dumps(parsed, indent=2), encoding='utf-8')
        from PIL import Image, ImageDraw
        image = Image.new('RGB', (616, 560), 'white')
        draw = ImageDraw.Draw(image)
        for side, color in [('left', 'blue'), ('right', 'orange')]:
            draw.line([tuple(p) for p in parsed['arms'][side]['points_uv']], fill=color, width=2)
        image.save(self.output/'input-preview.png')
        self.report['stages']['static'] = 'passed'
        self.checkpoint('svg_import_and_static')
        self.save_report()
        print('SVG static check passed; compiling the current simulator model', flush=True)
        model = planner.build_t1_scene(self.output, assets=self.assets, exclude_compound_elbow=True)
        self.model = model
        coverage_path = self.output/'t1-collision-model.json'
        coverage = json.loads(coverage_path.read_text())
        coverage['planner_reference_commit'] = coverage.pop('upstream_commit', None)
        coverage['actual_assets_revision'] = self.report['model_revision']
        coverage_path.write_text(json.dumps(coverage, indent=2), encoding='utf-8')
        snapshot = self.robot.state()
        auxiliary = snapshot['auxiliary']
        initial = full_pose(model, snapshot['joints']['position'], auxiliary,
                            skip_gripper_checks=self.skip_gripper_checks)
        self.initial, self.auxiliary = initial, auxiliary
        self.report['initial_snapshot'] = snapshot
        self.checker = CollisionChecker(model, planner.collision_pairs(model))
        self.guard = SvgStateGuard(auxiliary, model, initial, self.checker,
                                   skip_gripper_checks=self.skip_gripper_checks)
        self.verify_kinematics()
        u, targets = planner.shared_targets(parsed, sampling='uniform')
        self.report['target_sampling'] = parsed['metadata']['shared_target_sampling']
        self.checkpoint('model_and_state')
        self.report['candidate_failures'] = []
        for name, torso in planner.torso_candidates(u, targets):
            try:
                print('SVG_CANDIDATE', name, flush=True)
                home, chains, solved, search = planner.ik_candidate(model, u, targets, torso, self.seeds,
                                                                  initial_qpos=initial)
                self.checkpoint('ik')
                q, dq, phase, desired = planner.schedule(model, u, targets, torso, home, chains, solved,
                                                        return_home=False, timing='feedrate',
                                                        joint_speed=self.joint_speed, tip_speed=self.tip_speed)
                self.checkpoint('scheduling')
                checked = validate_native_contract(model, q, dq, phase, desired, joint_speed=self.joint_speed)
                self.checkpoint('native_plan_validation')
                break
            except (planner.PlanningError, RuntimeError) as exc:
                self.checkpoint('rejected_candidate_work')
                self.report['candidate_failures'].append(dict(candidate=name, error=str(exc)))
                print('SVG_REJECT', name, str(exc), flush=True)
                self.save_report()
        else:
            raise RuntimeError('No checked SVG plan from measured state; see svg-run.json candidate_failures')
        self.report.update(selected_torso_candidate=name, ik_search=search, native_plan=checked)
        self.report['stages']['planning'] = 'passed'
        dt = float(model.opt.timestep)
        minimum_duration = (len(q)-1)*dt
        time_scale = motion_time_scale(minimum_duration, self.duration_s, self.speed_scale)
        self.report.update(minimum_schedule_duration_s=minimum_duration, effective_time_scale=time_scale)
        self.q, self.dq, self.phase, self.desired, self.dt = q, dq, phase, desired, dt/time_scale
        np.savez_compressed(self.output/'planner-trajectory.npz', qdes=q, dqdes=dq*time_scale,
                            phase=phase, desired=desired, dt=self.dt, home=initial)
        self.report['resampled_shape_error_m'] = 0.
        self.report['resampled_source_shape_error_bound_m'] = 0.
        source_sampling_error = self.report['target_sampling']['maximum_source_polyline_deviation_m']
        data, indices = self.mujoco.MjData(model), self.guard.indices
        jacobian = np.zeros((3, model.nv))
        tip_ids = [model.site(s+'_light').id for s in ('left', 'right')]
        self.report['maximum_command_tip_speed_m_s'] = 0.
        sample_index = 0

        def validate_command(full):
            nonlocal sample_index
            data.qpos[:] = full
            self.mujoco.mj_kinematics(model, data)
            t = min(sample_index/self.robot.policy.control_hz, (len(q)-1)*self.dt)
            expected = self.drawing_target(t)
            if expected is not None:
                actual = np.array([data.site(s+'_light').xpos for s in ('left', 'right')])
                error = float(np.linalg.norm(actual-expected, axis=1).max())
                self.report['resampled_shape_error_m'] = max(self.report['resampled_shape_error_m'], error)
                source_bound = error+source_sampling_error
                self.report['resampled_source_shape_error_bound_m'] = max(
                    self.report['resampled_source_shape_error_bound_m'], source_bound)
                if source_bound > .001:
                    raise RuntimeError(f'Resampling changes SVG by at most {source_bound:.6f} m, exceeding 1 mm')
            distance = self.checker.minimum_distance(data)
            if sample_index % 5000 == 0:
                print('SVG_RESAMPLE_CHECK', sample_index, flush=True)
            sample_index += 1
            return distance

        def validate_velocity(full_velocity):
            self.mujoco.mj_comPos(model, data)
            for site_id in tip_ids:
                self.mujoco.mj_jacSite(model, data, jacobian, None, site_id)
                speed = float(np.linalg.norm(jacobian@full_velocity))
                self.report['maximum_command_tip_speed_m_s'] = max(self.report['maximum_command_tip_speed_m_s'], speed)
                if speed > self.tip_speed+1e-6:
                    raise RuntimeError(f'Tool-tip velocity {speed:.6f} m/s exceeds the configured ceiling')
            if np.max(np.abs(full_velocity[indices])) > self.joint_speed+1e-6:
                raise RuntimeError('Resampled joint velocity exceeds the configured ceiling')

        print('SVG numerical plan passed; checking geometry once on each final SDK command', flush=True)
        plan_path, validation = export_model_trajectory(self.output/'sdk-plan.npz', model, q,
            dq*time_scale, self.dt, phase, control_hz=self.robot.policy.control_hz,
            validate_state=validate_command, provenance=dict(svg_sha256=self.report['svg_sha256'],
                model_revision=self.report['model_revision'], measured_initial_state=True,
                speed_scale=self.speed_scale, effective_time_scale=time_scale,
                joint_speed_limit_rad_s=self.joint_speed, tip_speed_limit_m_s=self.tip_speed,
                command_preview_s=self.robot.policy.command_preview_s), validate_velocity=validate_velocity)
        plan = Trajectory.from_npz(plan_path)
        self.report['command_validation'] = json.loads(validation.read_text())
        self.report['validation_strategy'] = dict(native_geometry_pass=False,
            command_geometry='every final command', control_hz=self.robot.policy.control_hz,
            minimum_clearance_m=.002, maximum_shape_error_m=.001,
            runtime_geometry_interval_s=.05, timing_method='feedrate')
        self.report['path_length_m'] = {s: parsed['arms'][s]['length_m'] for s in ('left', 'right')}
        self.report['feedrate_scope'] = 'Longest synchronized arm targets the configured cruise speed; shorter arm scales with path length; start/stop and joint limits reduce speed'
        self.checkpoint('resampling_and_validation')
        self.report['timings_s']['input_to_trajectory_ready'] = time.perf_counter()-self.started
        self.report.update(plan=str(plan_path), validation=str(validation), duration_s=float(plan.time[-1]))
        self.report['stages']['resampling'] = 'passed'
        self.save_report()
        if self.plan_only:
            self.report['status'] = 'planned'
            print('SVG plan ready; plan-only requested, no control or motion requested', flush=True)
            return self.report
        print(f'SVG execution starts: {plan.time[-1]:.1f}s; approach, draw, stop at drawing end', flush=True)
        self.handle = self.robot.execute(plan, state_guard=self.guard)
        started, last_progress = time.perf_counter(), 0.
        while not self.handle.done:
            elapsed = time.perf_counter()-started
            if elapsed-last_progress >= 5:
                print(f'SVG_EXECUTION {elapsed:.1f}/{plan.time[-1]:.1f}s', flush=True)
                last_progress = elapsed
            if elapsed > plan.time[-1]+10:
                self.handle.cancel()
                raise RuntimeError('SVG execution exceeded the duration/settling allowance')
            time.sleep(.1)
        execution = self.handle.wait()
        self.checkpoint('execution')
        self.report['execution'] = execution
        self.report['actual_sampled_minimum_clearance_m'] = self.guard.minimum_clearance
        if execution['status'] != 'completed':
            raise RuntimeError('SVG execution did not complete: '+str(execution.get('error')))
        self.report['stages']['execution'] = 'passed'
        self.audit_execution(execution['execution_id'], parsed)
        self.checkpoint('measured_drawing_audit')
        self.report['status'] = 'completed'
        return self.report

    def drawing_target(self, t):
        index = min(max(0, int(t/self.dt)), len(self.q)-1)
        if self.phase[index] != 'draw_both':
            return None
        nxt = min(index+1, len(self.q)-1)
        if self.phase[nxt] != 'draw_both':
            return self.desired[index]
        fraction = np.clip(t/self.dt-index, 0., 1.)
        return (1-fraction)*self.desired[index]+fraction*self.desired[nxt]

    def verify_kinematics(self):
        """Compare planner FK against the simulator's independent kinematic XML."""
        native_path = self.assets/'astribot_t1/model/mjcf/robot/t1_kinematic.xml'
        native = self.mujoco.MjModel.from_xml_path(str(native_path))
        mapping = [(self.model.jnt_qposadr[i], native.jnt_qposadr[native.joint(self.model.joint(i).name).id])
                   for i in range(self.model.njnt)]
        base = [i for i in range(native.njnt)
                if native.joint(i).name not in {self.model.joint(j).name for j in range(self.model.njnt)}]
        if len(base) != 3 or not np.allclose(native.jnt_axis[base], np.eye(3)):
            raise RuntimeError('Unsupported simulator chassis joint layout')
        self.native, self.native_mapping, self.native_base = native, mapping, native.jnt_qposadr[base]
        p, n = self.mujoco.MjData(self.model), self.mujoco.MjData(native)
        p.qpos[:] = self.initial
        n.qpos[:] = self.native_pose(self.initial)
        self.mujoco.mj_forward(self.model, p)
        self.mujoco.mj_forward(native, n)
        pb, nb = p.body('chassis_base'), n.body('chassis_base')
        self.frame_rotation = pb.xmat.reshape(3, 3)@nb.xmat.reshape(3, 3).T
        self.frame_translation = pb.xpos-self.frame_rotation@nb.xpos
        maximum = 0.
        rng = np.random.default_rng(10)
        for _ in range(8):
            q = self.initial.copy()
            q[self.guard.indices] += rng.uniform(-.05, .05, 18)
            p.qpos[:] = q
            n.qpos[:] = self.native_pose(q)
            self.mujoco.mj_forward(self.model, p)
            self.mujoco.mj_forward(native, n)
            for side in ('left', 'right'):
                b = n.body(f'astribot_arm_{side}_link_7')
                tip = b.xpos+b.xmat.reshape(3, 3)@np.array([0., -.23, 0.])
                error = np.linalg.norm(p.site(side+'_light').xpos-(self.frame_rotation@tip+self.frame_translation))
                maximum = max(maximum, float(error))
        self.report['planner_vs_simulator_fk_max_error_m'] = maximum
        if maximum > 1e-6:
            raise RuntimeError(f'Planner/simulator kinematics mismatch: {maximum:.6f} m')

    def native_pose(self, q):
        values = self.native.qpos0.copy()
        for src, dst in self.native_mapping:
            values[dst] = q[src]
        values[self.native_base] = self.auxiliary['astribot_chassis']['position']
        return values

    def audit_execution(self, execution_id, parsed):
        from PIL import Image, ImageDraw
        data = self.mujoco.MjData(self.native)
        rows, expected, actual, errors = [], [], [], []
        for line in (self.output/(execution_id+'.jsonl')).read_text(encoding='utf-8').splitlines():
            sample = json.loads(line)
            if sample['event'] != 'sample':
                continue
            target = self.drawing_target(sample['trajectory_time_s'])
            if target is None:
                continue
            q = self.initial.copy()
            q[self.guard.indices] = sample['measured']
            data.qpos[:] = self.native_pose(q)
            self.mujoco.mj_forward(self.native, data)
            tips = []
            for side in ('left', 'right'):
                body = data.body(f'astribot_arm_{side}_link_7')
                point = body.xpos+body.xmat.reshape(3, 3)@np.array([0., -.23, 0.])
                tips.append(self.frame_rotation@point+self.frame_translation)
            rows.append(sample['trajectory_time_s'])
            expected.append(target)
            actual.append(tips)
            errors.append(np.linalg.norm(np.array(tips)-target, axis=1))
        if not errors:
            raise RuntimeError('No measured draw-phase samples in SDK execution log')
        errors, actual, expected = np.array(errors), np.array(actual), np.array(expected)
        self.report['measured_drawing'] = dict(samples=len(rows), position_source='FK of received SDK joint feedback',
            max_error_m=errors.max(axis=0).tolist(), rms_error_m=np.sqrt(np.mean(errors**2, axis=0)).tolist(),
            threshold_m=.002)
        np.savez_compressed(self.output/'measured-drawing.npz', time=rows, actual=actual, desired=expected,
                            error=errors, sides=['left', 'right'])
        image = Image.new('RGB', (616, 560), 'white')
        draw = ImageDraw.Draw(image)
        for si, (side, color) in enumerate([('left', 'blue'), ('right', 'orange')]):
            draw.line([tuple(p) for p in parsed['arms'][side]['points_uv']], fill='#bbbbbb', width=5)
            uv = np.c_[(actual[:, si, 0]+.77)/.0025, (1.89-actual[:, si, 2])/.0025]
            draw.line([tuple(p) for p in uv], fill=color, width=2)
        image.save(self.output/'measured-vs-svg.png')
        if np.max(errors) > .002:
            raise RuntimeError(f'Measured SVG drawing exceeds 2 mm tracking gate: {np.max(errors)*1000:.3f} mm')
        self.report['stages']['measured_drawing'] = 'passed'
