"""Run a LeRobot policy (SmolVLA, ACT, pi0.5, ...) on the robot through ROS. Dry run unless --live.

    pixi run tools/lerobot.sh policy --policy lerobot/smolvla_base --task "put the red cube in the bin"
    pixi run tools/lerobot.sh policy --policy <trained model folder> --task "put the red cube in the bin" --live

The loop: take the newest observation (cameras and /joint_states, the same way record.py does), run the policy on
the Mac's GPU, get a chunk of actions (SmolVLA: 50, 1.7 s at 30 fps), and ask for the next one when --threshold of
the chunk has played.

Dry run (default): nothing goes to the arms. Each chunk is published on /lerobot_policy/planned
(trajectory_msgs/JointTrajectory, header stamped at the observation) and summarized here: inference time and how far
its first action is from the arm. Any policy runs, the base model included: when its inputs don't match the layout
(smolvla_base wants a 6-D state and cameras camera1-3) the cameras are fed in layout order and the state cut or
padded. That tests the plumbing and timing; the actions mean nothing.

--live: the policy must match the layout exactly, i.e. be trained on a dataset from record.py. It starts disengaged.
Press e (or call ~/engage, std_srvs/SetBool) to engage, space or esc to stop: the controllers drop the trajectory and
hold. It refuses to engage while something else (the leader arms) is commanding the arms. Each chunk goes to the arm
and gripper controllers as one JointTrajectory each, without the actions whose time passed during inference, clamped
to the joint limits, and cut at --max-horizon seconds so the arms stop that soon after this node dies. A chunk whose
first action is more than --max-jump rad from the arm disengages it.
"""
import argparse
import math
import threading
import time

import numpy as np
import rclpy

from .keys import KeyReader
from .robot_io import Layout, RosRobot, arm_joint_limits, init_ros, shutdown_ros


def parse_args(argv=None):
    p = argparse.ArgumentParser(prog="lerobot.sh policy", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--policy", default="lerobot/smolvla_base", help="Hub id or local pretrained_model folder")
    p.add_argument("--task", required=True, help="the prompt; use the sentence the dataset was recorded with")
    p.add_argument("--live", action="store_true", help="send the actions to the arms (after pressing e)")
    p.add_argument("--device", default="mps")
    p.add_argument("--threshold", type=float, default=0.5,
                   help="fraction of a chunk that plays before the next inference starts")
    p.add_argument("--actions-per-chunk", type=int, default=0, help="use only the first N actions (0: all)")
    p.add_argument("--max-horizon", type=float, default=1.5,
                   help="s; live trajectories reach at most this far ahead (the arms stop this soon if the node dies)")
    p.add_argument("--max-jump", type=float, default=0.35,
                   help="rad; a chunk starting further than this from the arm disengages")
    p.add_argument("--config", default=None, help="robot layout (default: the package's config/robot.yaml)")
    return p.parse_args(argv)


def load_policy(path, device):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.device = device
    policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=path,
                                         preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post


class PolicyRunner:
    def __init__(self, args):
        self.args = args
        self.layout = Layout.load(args.config)
        self.dt = 1.0 / self.layout.fps
        self.limits = arm_joint_limits()
        self.engaged = False
        # Engaging, stopping and sending a chunk happen under this lock, so a stop from the keyboard or the service
        # can't land between the main loop's "still engaged?" check and its publish.
        self.engage_lock = threading.RLock()
        self.quit = False
        self.wake = threading.Event()
        self.robot = None

    # -- features

    def match(self, policy, pre):
        """Does the policy take this layout's observation as is? A fine-tuned SmolVLA keeps the base model's camera
        names (camera1-3) and its config's 6-D state; its saved preprocessor renames our cameras to those, and its
        normalizer statistics have the real state size. Both are read from the preprocessor."""
        inputs, outputs = policy.config.input_features, policy.config.output_features
        rename, stats = {}, {}
        for step in pre.steps:
            rename.update(getattr(step, "rename_map", None) or {})
            stats.update(getattr(step, "stats", None) or {})
        theirs = sorted(k for k in inputs if k.startswith("observation.images."))
        ours = [self.layout.image_key(c) for c in self.layout.cameras]
        n = len(self.layout.joints)
        self.state_dim = inputs["observation.state"].shape[0] if "observation.state" in inputs else 0
        state_stats = stats.get("observation.state") or {}
        shapes = [np.shape(v) for k, v in state_stats.items() if k in ("mean", "std", "min", "max", "q01", "q99")]
        if shapes and shapes[0]:
            self.state_dim = int(shapes[0][0])
        self.action_dim = outputs["action"].shape[0]
        mapped = sorted(rename.get(k, k) for k in ours)
        self.exact = mapped == theirs and self.state_dim == n and self.action_dim == n
        self.rename = {} if self.exact else dict(zip(ours, theirs))
        print(f"policy {policy.config.type}: cameras {theirs}, state {self.state_dim}, action {self.action_dim}, "
              f"chunk {getattr(policy.config, 'chunk_size', '?')}"
              + (f", preprocessor renames {rename}" if rename else ""))
        if self.exact:
            print("matches the layout")
        else:
            print(f"does not match the layout (cameras {ours}, {n} joints): feeding "
                  f"{ {k.split('.')[-1]: v.split('.')[-1] for k, v in self.rename.items()} }, state cut/padded to "
                  f"{self.state_dim}. Plumbing and timing test only.")

    def adapt(self, obs):
        if self.exact:
            return obs
        out = {self.rename[k]: v for k, v in obs.items() if k in self.rename}
        if self.state_dim:
            s = obs["observation.state"]
            out["observation.state"] = np.pad(s, (0, max(0, self.state_dim - len(s))))[:self.state_dim]
        return out

    # -- engage / stop

    def set_engaged(self, on):
        if on and not self.args.live:
            return False, "dry run: start with --live to drive the arms"
        with self.engage_lock:
            if on and not self.engaged:
                _, problems = self.robot.action()
                if len(problems) < len(self.layout.joints):
                    return False, "something else is commanding the arms (leader arms engaged?), not engaging"
            if not on and self.engaged:
                self.robot.stop_motion()
            changed = on != self.engaged
            self.engaged = on
        if changed:
            self.wake.set()
        why = "ENGAGED, the policy drives the arms" if on else "disengaged, arms hold"
        print(f"\n>> {why}", flush=True)
        return True, why

    def on_key(self, key):
        if key == "e":
            ok, why = self.set_engaged(not self.engaged)
            if not ok:
                print(f"\n>> {why}", flush=True)
        elif key in ("space", "esc"):
            self.set_engaged(False)
        elif key == "q":
            self.set_engaged(False)
            self.quit = True
            self.wake.set()

    def on_engage_service(self, req, resp):
        resp.success, resp.message = self.set_engaged(req.data)
        return resp

    # -- loop

    def clamp(self, actions):
        out = actions.copy()
        for i, j in enumerate(self.layout.joints):
            if j in self.limits:
                lo, hi = self.limits[j]
                out[:, i] = np.clip(out[:, i], lo, hi)
        return out

    def publish_planned(self, actions, stamp):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        msg = JointTrajectory()
        msg.header.stamp = stamp
        msg.joint_names = (list(self.layout.joints) if actions.shape[1] == len(self.layout.joints)
                           else [f"action_{i}" for i in range(actions.shape[1])])
        for k, row in enumerate(actions):
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in row]
            t = (k + 1) * self.dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int(round((t - math.floor(t)) * 1e9))
            msg.points.append(pt)
        self.planned_pub.publish(msg)

    def run(self):
        import torch
        from lerobot.policies.utils import prepare_observation_for_inference
        from std_srvs.srv import SetBool
        from trajectory_msgs.msg import JointTrajectory

        a = self.args
        t0 = time.monotonic()
        policy, pre, post = load_policy(a.policy, a.device)
        print(f"loaded {a.policy} in {time.monotonic() - t0:.0f} s")
        self.match(policy, pre)
        if a.live and not self.exact:
            raise SystemExit("--live needs a policy trained on this layout (record.py); this one is a dry run only")

        init_ros()
        self.robot = RosRobot(self.layout, "lerobot_policy", publish_actions=a.live)
        self.planned_pub = self.robot.create_publisher(JointTrajectory, "/lerobot_policy/planned", 10)
        self.robot.create_service(SetBool, "/lerobot_policy/engage", self.on_engage_service)
        self.robot.spin_in_background()
        print("dry run: publishing chunks on /lerobot_policy/planned, nothing goes to the arms" if not a.live else
              "LIVE: disengaged. e = engage, space/esc = stop, q = quit")
        device = torch.device(a.device)
        policy.reset()
        chunk_start, chunk_len, count, last_problem = None, 0, 0, 0.0
        try:
            with KeyReader(self.on_key):
                while not self.quit and rclpy.ok():
                    if chunk_start is not None:   # wait until threshold of the current chunk has played
                        due = chunk_start + a.threshold * chunk_len * self.dt
                        self.wake.wait(max(0.0, due - time.monotonic()))
                        if self.wake.is_set():
                            self.wake.clear()
                            chunk_start = None    # engaged/disengaged: start a fresh chunk now
                            continue
                    obs, problems = self.robot.observation()
                    if problems:
                        if time.monotonic() - last_problem > 2.0:
                            last_problem = time.monotonic()
                            print(f"waiting: {'; '.join(problems[:3])}", flush=True)
                        chunk_start = None
                        time.sleep(0.1)
                        continue
                    t_obs, stamp = time.monotonic(), self.robot.get_clock().now().to_msg()
                    state = obs["observation.state"].copy()
                    batch = prepare_observation_for_inference(self.adapt(obs), device, a.task, "xlerobot_ros2")
                    with torch.inference_mode():
                        chunk = post(policy.predict_action_chunk(pre(batch)))
                    actions = chunk[0].float().cpu().numpy()
                    if a.actions_per_chunk:
                        actions = actions[:a.actions_per_chunk]
                    latency = time.monotonic() - t_obs
                    skip = min(int(math.ceil(latency / self.dt)), len(actions) - 1)
                    self.publish_planned(actions, stamp)
                    send = actions[skip:]
                    if a.live:
                        send = send[:max(1, int(a.max_horizon * self.layout.fps))]
                    jump = (float(np.max(np.abs(send[0] - state))) if send.shape[1] == len(state) else float("nan"))
                    count += 1
                    with self.engage_lock:
                        sent = self.engaged and jump <= a.max_jump
                        if sent:
                            send = self.clamp(send)
                            self.robot.send_chunk(send, self.dt)
                    gap = f"first action {jump:.2f} rad from the arm" if jump == jump else "actions are not joints"
                    print(f"chunk {count}: inference {latency * 1000:.0f} ms, {len(send)} of {len(actions)} actions "
                          f"({skip} played during inference), {gap}" + ("  [sent]" if sent else ""), flush=True)
                    if self.engaged and not sent:
                        print(f">> first action {jump:.2f} rad from the arm (> --max-jump {a.max_jump}), "
                              "disengaging", flush=True)
                        self.set_engaged(False)
                    chunk_start, chunk_len = time.monotonic(), len(send)
        except KeyboardInterrupt:
            pass
        finally:
            with self.engage_lock:
                if self.engaged:
                    self.robot.stop_motion()
                    self.engaged = False
                    time.sleep(0.2)
            self.robot.close()
            shutdown_ros()


def main(argv=None):
    PolicyRunner(parse_args(argv)).run()


if __name__ == "__main__":
    main()
