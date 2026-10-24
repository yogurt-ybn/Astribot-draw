"""Receipt-timed feedback from the official astribot_simulation ROS wire."""
import time

from .contracts import JointState, T1_PARTS, T1_JOINTS, flatten_parts

AUXILIARY_PARTS = {'astribot_head': 2, 'astribot_gripper_left': 1,
                   'astribot_gripper_right': 1, 'astribot_chassis': 3}

class RosSimulationFeedback:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from astribot_msgs.msg import RobotJointState
        self.context = Context()
        self.node = self.executor = None
        self.parts, self.error, self.sequence = {}, None, 0
        self.auxiliary = {}
        self.subscriptions = []
        try:
            rclpy.init(context=self.context)
            self.node = rclpy.create_node('astribot_draw_sim_feedback', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            for part in T1_PARTS:
                self.subscriptions.append(self.node.create_subscription(
                    RobotJointState, f'/{part}/joint_space_states',
                    lambda msg, part=part: self.receive(part, msg),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
            for part in AUXILIARY_PARTS:
                self.subscriptions.append(self.node.create_subscription(
                    RobotJointState, f'/{part}/joint_space_states',
                    lambda msg, part=part: self.receive_auxiliary(part, msg),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
        except BaseException:
            self.close()
            raise

    def receive(self, part, msg):
        if msg.header.frame_id != 'simulation':
            self.error = 'ROS feedback is not tagged as simulation; refusing execution'
            return
        names = tuple(msg.name)
        expected = tuple(name for name in T1_JOINTS if name.startswith(part+'_joint_'))
        # HAL 1.3.6 RosJointChannel leaves name empty; its arrays follow the
        # per-part joint order in official config/astribot_t1/sim.yaml.
        if not names:
            names = expected
        if len(names) != len(expected) or set(names) != set(expected):
            self.error = f'unexpected joint names for {part}'
            return
        try:
            if len(msg.position) != len(expected) or len(msg.velocity) != len(expected):
                raise ValueError('expected the complete T1 part vector')
            indices = [names.index(name) for name in expected]
            q = [msg.position[i] for i in indices]
            dq = [msg.velocity[i] for i in indices]
            self.parts[part] = (q, dq, time.perf_counter())
            self.sequence += 1
        except (IndexError, TypeError, ValueError) as exc:
            self.error = f'invalid ROS joint arrays: {exc}'

    def read(self):
        # Spin on the SDK owner thread; bounded work and depth=1 keep old frames out.
        for _ in range(12):
            self.executor.spin_once(timeout_sec=0.)
        if self.error:
            raise RuntimeError(self.error)
        if not all(p in self.parts for p in T1_PARTS):
            return None
        states = [self.parts[p] for p in T1_PARTS]
        return JointState(flatten_parts([s[0] for s in states], 'ROS position'),
                          flatten_parts([s[1] for s in states], 'ROS velocity'),
                          min(s[2] for s in states), self.sequence)

    def receive_auxiliary(self, part, msg):
        if msg.header.frame_id != 'simulation' or len(msg.position) != AUXILIARY_PARTS[part]:
            self.error = f'invalid simulation auxiliary feedback: {part}'
            return
        self.auxiliary[part] = dict(position=list(msg.position), received_at=time.perf_counter())

    def extra_state(self):
        return {name: dict(position=list(state['position']), received_at=state['received_at'])
                for name, state in self.auxiliary.items()}

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.)
        if self.node is not None:
            self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
