# ADR-0005: Object pose perception for manipulation

- Status: Proposed
- Date: 2026-10-25

## Context

Reaching and touching an object needs a pose that can be trusted to a few millimetres.
Today the head camera finds a yellow cube by colour and depth, good to about a centimetre, and
nothing finds other objects. The pieces that already exist:

- Head camera (Orbbec Gemini 2): 1280x720 colour at about 24 Hz, depth registered to colour at
  about 20 Hz. Calibrated against the arms to about 7 mm (ChArUco board,
  `tools/calibrate_arm_camera.py`). The table plane in the depth cloud is level to 0.02 degrees.
  A 28 mm cube at 0.5 m is about 37 px wide.
- Wrist cameras: 640x480 colour only, no depth. Calibrated mount and intrinsics.
- Everything on the robot runs on the Jetson Orin NX 16 GB (CPU is the constraint, the GPU is
  idle). A laptop with an RTX 4090 is reachable over Wi-Fi by explicit requests only (ROS
  discovery is local to each machine).
- The arms reach open loop to about 5 mm (servo backlash, sag), so the last centimetre is
  closed on the wrist camera. The head pose only has to seed that approach to about 10 mm.

## Decision

One pipeline from camera frames to object poses, each stage replaceable behind a fixed interface.

1. **Propose** candidate regions. Geometric first: blobs above the table plane, with the robot
   itself removed using the URDF. Open-vocabulary detection (a text prompt or class list) is a
   second source and also the labeller for a fine-tuned detector.
2. **Classify and segment** each candidate on a crop at native resolution (the table region is
   known from the plane). Mask models are prompted with a crop of about 128 px upscaled, never
   the full frame, because their masks are low resolution.
3. **Lift to 3D.** The height of the top face is the median depth of the mask. The horizontal
   position and the yaw come from the RGB contour (sub-pixel edge or line fit) with its rays
   intersected with the plane at that height, because depth edges smear and the stereo shadow is
   one-sided. Depth supplies height and validity only. Where depth is invalid (shiny or
   transparent objects), the known plane height is used and the result is flagged. Yaw is
   reported modulo the object's symmetry (90 degrees for a cube, 180 for a brick).
4. **Fuse and associate** over a short window with the head and base stationary. Data
   association is a gated nearest neighbour in `base_footprint`, assuming a static scene.
5. **Publish** `vision_msgs/Detection3DArray` in `base_footprint`, with the colour frame's stamp
   and a covariance. Failure reasons are enumerated codes in the result: `NO_DET`, `FEW_DEPTH`,
   `SIZE_OOR`, `TRUNCATED_AT_BORDER`, `OCCLUDED`, `YAW_AMBIGUOUS`, `STALE_TF`,
   `BACKEND_TIMEOUT`. TF carries a frame only for the currently selected target (TF cannot
   delete frames). Rerun shows mask overlay, cloud, pose axes and the error against ground truth.

### Model backends

Models sit behind a Python interface with two transports: in-process (the Jetson backend,
TensorRT, no image encoding on the CPU) and HTTP (FastAPI on the laptop, PyTorch on the GPU).
Requests carry the frame stamp and an id. Responses carry the model id and version, boxes,
scores, and masks as run-length encoding in crop coordinates. The perception manager (a ROS 2 node
on the Jetson) calls whichever backend is available and fast enough and can call both to compare.
Timeouts assume Wi-Fi tails of hundreds of milliseconds. Control loops never go through HTTP.

- LeRobot's async inference server is for policies (ACT, SmolVLA) and is used for those later.
- Candidates, to be benchmarked on this Jetson before choosing: YOLO-World (CNN, small-object
  friendly) as the first open-vocabulary detector, NanoOWL and OWLv2 for comparison, Grounding
  DINO on the laptop as an offline reference; NanoSAM or EfficientViT-SAM on the Jetson and SAM2
  on the laptop for masks. For a fixed set of objects, a small YOLO fine-tuned on our own images
  is the intended end state, labelled by the open-vocabulary pipeline and spot-checked.
- A colour-threshold backend stays as the baseline and as a fallback.
- Frame sync: the driver's colour and depth sync is enabled and pairs with a stamp difference
  above a threshold are rejected. Exposure and white balance are locked.

### Ground truth

A printed placement sheet (`tools/generate_placement_sheet.py`): markers around the edge give
the sheet's plane and pose to about 1 mm and 1 degree, and the footprints of the test objects
are drawn at known positions and yaws, so an object sitting on a footprint has a known pose.
The sheet's printed scale is measured with a ruler. A tag on top of the object is not used, since
it replaces the top face the detectors see. A side camera at table height is added when
available.

This measures the consistency of the camera's own estimate, not reach accuracy: the head's
placement and its measurement share the same calibration, so the 7 mm head-to-arm error cancels
out of this test.

### Exit criteria for M1

Over 300 placements of the cube and of 2x2 and 2x4 bricks, at 0.3 m to 0.8 m from the head, with
yaw covering a full turn:

- detection recall 95% or better, wrong-label rate under 2%, false positives under 0.1 per frame;
- horizontal position error under 4 mm at the 95th percentile, height error reported separately;
- yaw error under 3 degrees at the 95th percentile (modulo symmetry);
- latency from frame to pose under 200 ms at the 95th percentile on the Jetson backend, reported
  with and without the fusion window;
- each failure logged with its image and code.

Experiments in order: (1) the pose floor with no learned model (colour baseline plus the
contour method on the placement sheet; repeatability under 1 mm and 1 degree over 100 frames);
(2) offline recall of the detector candidates on about 200 labelled recorded frames, full frame
against table crop; (3) the Jetson port of the winner plus crop-based segmentation, poses within
1 mm of the laptop's on the same frames.

## Out of scope for this ADR

Grasp generation, the gripper's tool-centre-point calibration, arm motion, the wrist-camera
closed loop that provides the final millimetres, driving to the desk, and the task layer
(behaviour trees, planning by language models). The wrist cameras stay colour only for now.

## Consequences

- One interface for every model, so a better detector or a larger model on the laptop is a
  backend swap, not a rewrite.
- The perception manager must handle a backend being slow or absent (timeouts, fallback to the
  Jetson or the colour baseline).
- Small and flat objects (a Lego plate is 3.2 mm tall) are below the plane-removal threshold and
  need their own handling later. Colour-matched objects (green on the mat) defeat the colour
  baseline and rely on the learned detectors.
- Shiny or transparent objects (the SATA cage) will have depth holes and multipath. They are
  handled by the plane-height fallback and the flag, and are not claimed to work in M1.
