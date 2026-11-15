# LeRobot: demonstrations and policies (SmolVLA) over ROS

`src/xlerobot_lerobot` connects LeRobot to this stack without putting it in the control path (ADR-0001). It is a client
of the topics: it records leader-arm demonstrations from them into a LeRobot dataset, and runs a trained policy that
sends its action chunks to the arm controllers. Everything here runs on the operator station (the Mac) on the LeRobot
venv's Python. elroy keeps running bringup.

| Where | What |
|---|---|
| elroy | bringup `with_arms:=true cameras:=true sensor:=realsense`, the controllers, the cameras |
| Mac | leader arms (`xlerobot_teleop`), `tools/lerobot.sh record`, `tools/lerobot.sh policy`, small training runs |
| rented NVIDIA GPU | SmolVLA or pi0.5 fine-tuning at a useful speed |

Why the Mac and not elroy: elroy is an 8 GB Orin Nano that already carries the stack. On the M5 Pro's GPU (MPS),
SmolVLA plans a 50-action chunk (1.7 s at 30 fps) in about 0.2 s and uses under 1 GB.

## Setup, once

```bash
tools/setup_venv.sh lerobot      # ~/venvs/xlerobot-lerobot: LeRobot 0.7 with SmolVLA, macOS only
pixi run build                   # includes xlerobot_lerobot
```

`tools/lerobot.sh` runs the modules on the venv's Python with rclpy from pixi, the venv's packages first on the path.

## The layout

`src/xlerobot_lerobot/config/robot.yaml` says which topics make an observation and an action:

- **cameras:** `head`, `left_wrist` and `right_wrist`, from their JPEG topics (`.../image_raw/compressed`, never
  `image_raw` over wifi). Three cameras at 30 fps are about 4 MB/s.
- **state:** the 12 arm joints from `/joint_states`, in radians (the model's angles, not lerobot's SO-101 degrees).
- **action:** the same 12 joints, as last commanded on the arm and gripper controllers' `joint_trajectory` topics. While
  recording those commands come from the leader arms; the policy sends to the same topics.

Everything is sampled by arrival time, newest message per topic. Header stamps are not used, because usb_cam 0.8.1
stamps the wrist frames 0-1 s early. A policy needs the layout its dataset was recorded with, so change the file
between datasets, not within one. Keep the head at the same pose while recording and running: the base and head are
not in the state.

## Record

On elroy, bringup with the arms and cameras. On the Mac, the leader arms (`teleop.launch.py`), engaged. Then:

```bash
pixi run tools/lerobot.sh record --repo-id carl/xle_cube_to_bin --task "put the red cube in the bin" --num-episodes 20
```

- Each episode records `--episode-time` seconds of data (default 30), then leaves `--reset-time` (15) to put the
  scene back. The prompts are spoken (macOS `say`; `--no-sound` to turn that off).
- **Keys in that terminal:** right arrow or Enter keeps the episode and ends it now (during a reset: ends the reset).
  Left arrow or r throws it away and records it again. Esc or q keeps it and stops. Ctrl-C stops and drops the
  unfinished episode.
- A frame is recorded only while every camera, the joint state and the leader commands are fresh (`stale_s`, 0.25 s).
  A disengaged leader or a stalled camera pauses the episode, shown as `PAUSED: ...`, and the pause doesn't count
  toward the episode time.
- The dataset goes to `~/.cache/huggingface/lerobot/<repo-id>`. `--resume` adds episodes to it. `--push` uploads it
  to the Hugging Face Hub as private (needs `hf auth login`).
- Video is encoded while recording (`--streaming-encoding`, the default), so saving takes a fraction of a second.
  Without it, saving takes about as long as the episode.

Use one sentence per task and the same wording in every episode, since it becomes the policy's prompt. About 20
episodes is a start for SmolVLA. Vary where the objects are.

## Train

**SmolVLA.** The base model names its cameras camera1-3, so map ours onto them. The fine-tuned model's preprocessor
keeps the mapping, and the policy node applies it.

```bash
~/venvs/xlerobot-lerobot/bin/lerobot-train \
  --policy.path=lerobot/smolvla_base --dataset.repo_id=carl/xle_cube_to_bin \
  --rename_map='{"observation.images.head": "observation.images.camera1", "observation.images.left_wrist": "observation.images.camera2", "observation.images.right_wrist": "observation.images.camera3"}' \
  --batch_size=64 --steps=20000 --policy.device=cuda --output_dir=outputs/smolvla_cube --policy.push_to_hub=false
```

On a rented GPU, push the dataset first (`--push`) and train from the Hub copy. On the Mac use `--policy.device=mps`
and a smaller batch: it works, but slowly.

**ACT,** as a check on the data, trains on the Mac:
`lerobot-train --policy.type=act --dataset.repo_id=... --policy.device=mps --output_dir=outputs/act_cube --policy.push_to_hub=false`.
The trained model is `<output_dir>/checkpoints/last/pretrained_model`.

Training reads the videos with torchcodec, which brings no FFmpeg of its own. On the Mac it loads Homebrew's
(`brew install ffmpeg`, FFmpeg 9; it looks in `/opt/homebrew/opt/ffmpeg/lib`). Without it LeRobot falls back to PyAV,
which decoded a three-camera sample in 3.9 ms against torchcodec's 0.9 ms (2026-11-14). That's no bottleneck on the
Mac, but it can be on a fast GPU, so install FFmpeg there too (`apt install ffmpeg`).

## Run a policy

```bash
pixi run tools/lerobot.sh policy --policy outputs/smolvla_cube/checkpoints/last/pretrained_model \
  --task "put the red cube in the bin"            # dry run
pixi run tools/lerobot.sh policy --policy ... --task "..." --live
```

**Dry run (the default).** Nothing goes to the arms. Each chunk is published on `/lerobot_policy/planned` and shown in
the Rerun bridge's **Policy** tab: the planned trajectory as points, the measured joints as lines, per arm. The terminal
shows the inference time and how far each chunk's first action is from the arm. A trained policy should start close to
where the arm is. The base model runs too, with its inputs fed in layout order: that tests plumbing and timing only.

**Live (`--live`).** The policy must match the layout: cameras (after its rename map) and 12 joints in and out.
- It starts disengaged. Press **e** (or call `/lerobot_policy/engage`, `std_srvs/SetBool`) to engage. **Space** or
  **Esc** stops: the controllers get an empty trajectory, drop the goal and hold. **q** quits.
- It refuses to engage while something else commands the arms. Disengage the leader arms first.
- Each chunk goes to the four controllers as one `JointTrajectory` each. The actions whose time passed during
  inference are dropped, the rest clamped to the joint limits and cut at `--max-horizon` (1.5 s). If the node dies,
  the arms stop that soon.
- A chunk whose first action is more than `--max-jump` (0.35 rad) from the arm disengages it.
- SIGTERM and Ctrl-C both stop the arms on the way out.

The next inference starts when `--threshold` (0.5) of the current chunk has played. The new chunk replaces the rest of
the old one, and with `interpolate_from_desired_state` the controller continues from the last command, so there is no
jump at the seam.

## Without the robot

`fake_robot` publishes the layout's cameras (a moving test pattern), `/joint_states` and leader commands, on sine waves.
It uses the real command topics, so it refuses ROS domain 0:

```bash
ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh fake_robot            # --no-leader: no commands, like a disengaged leader
ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh record --repo-id test/fake --task "wave" --num-episodes 2 --episode-time 5
ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh policy --policy lerobot/smolvla_base --task "wave"
```

## Notes

- The ROS side spins one executor thread. With a 4-thread `MultiThreadedExecutor` the callback threads competed with
  inference for the GIL, and a SmolVLA chunk took 1.8 s instead of 0.19 s.
- rclpy is started without its own signal handlers. With them, SIGTERM only shut ROS down and a live policy node kept
  running.
- pi0.5 trains from the same dataset (`--policy.path=lerobot/pi05_base`, on a rented GPU) and runs through the same
  node.
