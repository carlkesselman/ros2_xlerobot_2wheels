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

## Setup

1. Laptop: `~/venvs/xlerobot-perception/bin/python tools/perception/export_models.py` (needs torch) writes the ONNX files and the
   text embeddings of the prompts. Copy `owlv2_det.onnx` and the EfficientViT-SAM `evit_l0_*.onnx` to `~/onnx/` on the Jetson and
   `prompts.npz` to `~/.cache/xlerobot/`. A new prompt means exporting again, since the text encoder is not on the Jetson.
2. Jetson: `tools/perception/build_engines.sh` builds the TensorRT engines (`~/.cache/xlerobot/engines`).
3. Jetson: a venv with the system packages plus `cuda-python`: `python3 -m venv --system-site-packages ~/venvs/xlerobot-perception && ~/venvs/xlerobot-perception/bin/pip install cuda-python`.
   The launch file runs the node with it (`python:=` or `XLEROBOT_PERCEPTION_PYTHON` to change).

## Check against the placement sheet

`tools/perception_check.py` calls the service while looking at the sheet and prints the error to the nearest footprint.
`table_z_offset_m` in `config/object_perception.yaml` corrects the Gemini 2 depth reading the table about 5 mm low (found with the sheet at 0.57 m;
the effect is a shift along the viewing ray, so it needs checking at other distances).
