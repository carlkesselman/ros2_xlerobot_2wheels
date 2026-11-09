# xlerobot_object_perception

On-demand object poses from the head camera (design in `docs/decisions/0005-object-perception.md`).
Nothing runs between requests: the camera is subscribed only while a call is being served, and the models stay loaded on the GPU.

```
ros2 launch xlerobot_object_perception object_perception.launch.py
ros2 service call /perception/detect xlerobot_object_perception/srv/DetectObjects "{prompt: 'a cube', label: cube}"
```

A call takes `frames` colour+depth frames (default 3) and, per frame, runs OWLv2 for boxes, EfficientViT-SAM for a mask on an
upscaled crop around each box, reads the table height from the depth around the object, and fits the silhouette of a cube of
the given size to the mask (`objperc/cube_fit.py`). Frames are merged by position. The reply is a
`vision_msgs/Detection3DArray` in `base_footprint` (also published on `/perception/detections`), the height of each top face from the
depth, status codes (`NO_DET`, `NO_MASK`, `FEW_DEPTH`, `SIZE_OOR`, `STALE_TF`, `BAD_PROMPT`, `NO_FRAMES`) and the latency. About 1.7 s for three frames on the Orin NX.

## Wrist cameras

`/perception/locate_in_wrist` (`srv/LocateInWrist.srv`) finds one cube-shaped object near a given position in a wrist
camera's view, for the last millimetres before touching or grasping (the `align` skill). Per frame: OWLv2's box nearest
the expected position, the mask, and a cube of known size fitted to the mask's outline (`objperc/wrist_fit.py`). The
claw hides part of what the camera sees; a fixed mask per camera marks it (`config/wrist_occluder_<arm>.png`, made with
`tools/wrist_occluder.py`; the ones here are the reference robot's printed claws), so the outline next to the claw is not
taken for the object's edge. The cube's size fixes its distance. The pose comes through the arm's kinematics, so the
offset to the claw tip does not depend on the arm's absolute accuracy. With `fix_height_yaw` only x and y are fitted, the height and yaw
given: enough when the claw hides much of the object, as with the claw tip right over it.

On the reference robot, with the object whole in view, repeated calls at a still pose agree to 0.3 mm horizontally and
2 mm in height; about 0.9 s for three frames. `wrist_debug_dir` (a parameter, settable at run time) saves every frame
with its mask and fitted outline.

## Setup

1. On a machine with an NVIDIA GPU and torch (`tools/setup_venv.sh models`, see [docs/tools.md](../../docs/tools.md#python-environment)):
   `python3 tools/perception/export_models.py` writes the ONNX files and the text embeddings of the prompts to
   `~/.cache/xlerobot/onnx`. Copy `owlv2_det.onnx` and `evit_l0_*.onnx` to `~/onnx/` on the robot and `prompts.npz` to
   `~/.cache/xlerobot/`. A new prompt means exporting again, since the text encoder does not run on the robot.
2. On the robot: `tools/perception/build_engines.sh` builds the TensorRT engines into `~/.cache/xlerobot/engines`.
3. The node runs with the tools venv's Python (`$XLEROBOT_VENV`, the repo's `.venv` by default, see `tools/setup_venv.sh`), which has `cuda-python`;
   TensorRT itself comes from JetPack. `python:=` on the launch file overrides it.

## Check against the placement sheet

`tools/perception_check.py` calls the service while looking at the sheet and prints the error to the nearest footprint.
`depth_scale` in `config/object_perception.yaml` corrects the depth camera's scale (it reads 0.8% too far); it is
measured with `tools/depth_scale.py`, see [docs/setup.md](../../docs/setup.md#9-head-depth-scale).
