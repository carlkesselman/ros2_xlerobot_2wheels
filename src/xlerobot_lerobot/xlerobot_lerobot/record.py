"""Record leader-arm demonstrations from the ROS topics into a LeRobot dataset.

Needs, on the robot, bringup with_arms:=true cameras:=true sensor:=realsense, and here the leader arms streaming
(xlerobot_teleop teleop.launch.py, engaged). Then, on the operator station:

    pixi run tools/lerobot.sh record --repo-id carl/xle_cube_to_bin --task "put the red cube in the bin"

Each episode records --episode-time seconds of data, then leaves --reset-time seconds to put the scene back while it
saves. Keys in this terminal:
    right arrow, enter   end the episode now and keep it (during the reset: end the reset)
    left arrow, r        throw the episode away and record it again
    esc, q               keep the episode and stop
Ctrl-C stops and throws the unfinished episode away.

The dataset goes to ~/.cache/huggingface/lerobot/<repo-id> (--root to change). --resume adds episodes to an existing
one, --push uploads it to the Hugging Face Hub (private) at the end.

A frame is recorded only while every camera, the joint state and the leader commands are newer than the layout's
stale_s: a disengaged leader or a stalled camera pauses the episode instead of filling it with stale data. Episode time
counts recorded frames, so a pause does not use it up.
"""
import argparse
import subprocess
import sys
import time

import rclpy

from .keys import KeyReader
from .robot_io import Layout, RosRobot, init_ros, shutdown_ros


def parse_args(argv=None):
    p = argparse.ArgumentParser(prog="lerobot.sh record", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repo-id", required=True, help="<hf user>/<dataset name>, also the local folder name")
    p.add_argument("--task", required=True, help="what the demonstrations do, one sentence; the policy's prompt")
    p.add_argument("--num-episodes", type=int, default=10)
    p.add_argument("--episode-time", type=float, default=30.0, help="seconds of data per episode")
    p.add_argument("--reset-time", type=float, default=15.0, help="seconds between episodes")
    p.add_argument("--root", default=None, help="dataset folder (default ~/.cache/huggingface/lerobot/<repo-id>)")
    p.add_argument("--resume", action="store_true", help="add episodes to an existing dataset")
    p.add_argument("--push", action="store_true", help="upload to the Hugging Face Hub (private) when done")
    p.add_argument("--config", default=None, help="robot layout (default: the package's config/robot.yaml)")
    p.add_argument("--vcodec", default="libsvtav1",
                   help="video codec: libsvtav1 (lerobot's default), h264_videotoolbox (the Mac's hardware), h264, ...")
    p.add_argument("--streaming-encoding", action=argparse.BooleanOptionalAction, default=True,
                   help="encode video while recording (default); without it, saving takes about as long as the "
                        "episode")
    p.add_argument("--no-sound", action="store_true", help="don't speak the prompts (macOS say)")
    return p.parse_args(argv)


class Events:
    def __init__(self):
        self.clear()
        self.stop = False

    def clear(self):
        self.end = False
        self.rerecord = False

    def on_key(self, key):
        if key in ("right", "enter", "n"):
            self.end = True
        elif key in ("left", "r"):
            self.rerecord = self.end = True
        elif key in ("esc", "q"):
            self.stop = self.end = True


class Recorder:
    def __init__(self, args):
        self.args = args
        self.layout = Layout.load(args.config)
        self.fps = self.layout.fps
        self.events = Events()
        self.robot = None
        self.dataset = None

    def say(self, text):
        print(f"\n>> {text}", flush=True)
        if not self.args.no_sound and sys.platform == "darwin":
            subprocess.Popen(["say", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def open_dataset(self):
        from lerobot.configs.video import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        a, features = self.args, self.layout.dataset_features()
        threads = 4 * len(self.layout.cameras)
        encoder = RGBEncoderConfig(vcodec=a.vcodec)
        if a.resume:
            ds = LeRobotDataset.resume(a.repo_id, root=a.root, image_writer_threads=threads, rgb_encoder=encoder,
                                       streaming_encoding=a.streaming_encoding)
            if ds.fps != self.fps:
                raise SystemExit(f"{a.repo_id} was recorded at {ds.fps} fps, the layout says {self.fps}")
            for key, feat in features.items():
                have = ds.features.get(key)
                if have is None or tuple(have["shape"]) != tuple(feat["shape"]) or have.get("names") != feat["names"]:
                    raise SystemExit(f"{a.repo_id}: feature {key} differs from the layout ({have} vs {feat})")
            print(f"resuming {a.repo_id}: {ds.num_episodes} episodes so far, at {ds.root}")
        else:
            ds = LeRobotDataset.create(a.repo_id, self.fps, features=features, root=a.root,
                                       robot_type="xlerobot_ros2", use_videos=True, image_writer_threads=threads,
                                       rgb_encoder=encoder, streaming_encoding=a.streaming_encoding)
            print(f"new dataset {a.repo_id} at {ds.root}")
        return ds

    def wait_for_streams(self):
        """Until the cameras and the joint state arrive (the leader commands only start when engaged)."""
        last = 0.0
        while not self.events.stop:
            _, problems = self.robot.observation()
            if not problems:
                return True
            if time.monotonic() - last > 2.0:
                last = time.monotonic()
                print(f"waiting for the robot: {'; '.join(problems[:3])}", flush=True)
            time.sleep(0.1)
        return False

    def record_episode(self, index):
        """Fills the dataset's episode buffer. Returns (frames, longest gap in seconds)."""
        period, target = 1.0 / self.fps, int(round(self.args.episode_time * self.fps))
        frames, gap, gap_start, last_print = 0, 0.0, None, 0.0
        next_t = time.perf_counter()
        while frames < target and not self.events.end and rclpy.ok():
            obs, problems = self.robot.observation()
            action, more = self.robot.action()
            problems += more
            now = time.perf_counter()
            if problems:
                if gap_start is None:
                    gap_start = now
                hint = ", leader arms engaged?" if more and problems[0] in more else ""
                if now - last_print > 1.0:
                    last_print = now
                    print(f"\r  episode {index}: {frames / self.fps:5.1f}/{self.args.episode_time:.0f} s"
                          f"  PAUSED: {problems[0]}{hint}" + " " * 10, end="", flush=True)
            else:
                if gap_start is not None and frames > 0:
                    gap = max(gap, now - gap_start)
                gap_start = None
                self.dataset.add_frame({**obs, "action": action, "task": self.args.task})
                frames += 1
                if now - last_print > 0.5:
                    last_print = now
                    print(f"\r  episode {index}: {frames / self.fps:5.1f}/{self.args.episode_time:.0f} s"
                          + " " * 60, end="", flush=True)
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
        print(flush=True)
        return frames, gap

    def wait_reset(self, seconds):
        self.events.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.events.end:
            time.sleep(0.05)
        self.events.clear()

    def run(self):
        from lerobot.datasets.video_utils import VideoEncodingManager

        init_ros()
        self.robot = RosRobot(self.layout, "lerobot_recorder")
        self.robot.spin_in_background()
        self.dataset = self.open_dataset()
        saved = 0
        try:
            with KeyReader(self.events.on_key), VideoEncodingManager(self.dataset):
                if not self.wait_for_streams():
                    return
                print("keys: right/enter = keep and end episode, left/r = redo it, esc/q = keep and stop")
                while saved < self.args.num_episodes and not self.events.stop:
                    index = self.dataset.num_episodes
                    self.events.clear()
                    self.say(f"Recording episode {index}")
                    frames, gap = self.record_episode(index)
                    t0 = time.monotonic()
                    if self.events.rerecord:
                        self.dataset.clear_episode_buffer()
                        self.say("Discarded, it will be recorded again")
                    elif frames < self.fps:
                        self.dataset.clear_episode_buffer()
                        self.say("Under a second of data, discarded")
                    else:
                        if gap > 0.5:
                            print(f"  note: a {gap:.1f} s pause in this episode (stale stream); "
                                  "the motion jumps there")
                        self.dataset.save_episode()
                        saved += 1
                        print(f"  saved episode {index}: {frames} frames, in {time.monotonic() - t0:.1f} s")
                    if self.events.stop or saved >= self.args.num_episodes:
                        break
                    self.say("Reset the scene")
                    self.wait_reset(max(0.0, self.args.reset_time - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            # VideoEncodingManager has already dropped the unfinished episode and finalized the dataset.
            print("\ninterrupted, the unfinished episode is not saved")
        finally:
            self.dataset.finalize()
            self.robot.close()
            shutdown_ros()
        self.say(f"Stopped. {saved} episodes this session, {self.dataset.num_episodes} in the dataset")
        print(f"dataset: {self.dataset.root}")
        if self.args.push and saved:
            self.dataset.push_to_hub(private=True)
            print(f"pushed to https://huggingface.co/datasets/{self.args.repo_id} (private)")


def main(argv=None):
    Recorder(parse_args(argv)).run()


if __name__ == "__main__":
    main()
