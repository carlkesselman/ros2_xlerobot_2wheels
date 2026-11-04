# Arm mount layouts on the top basket

A study. Layout B is now the model's layout (`shoulders: front_corners` in `robot_dimensions.yaml`, 2026-11-04);
the URDF files here are the study's snapshots, nothing in the launch files uses them. Generated 2026-11-04 by
`tools/arm_layout_reach.py` from `xlerobot.urdf.xacro with_arms:=true use_ros2_control:=false`, current
`robot_dimensions.yaml`. Only the two `*_arm_mount_joint` origins differ between the files.

| File | Layout | Pan axis (x fwd of axle, ±y) | Base bottom z |
|---|---|---|---|
| `A_current.urdf` | current: shoulders against the rear wall, facing forward | -0.239, 0.133 | 0.760 |
| `B_front.urdf` | front corners, facing forward | -0.058, 0.136 | 0.760 |
| `C_front_in30.urdf` | front corners, toed in 30° | -0.082, 0.108 | 0.760 |
| `D_front_in45.urdf` | front corners, toed in 45° | -0.089, 0.103 | 0.760 |
| `E_front_out20.urdf` | front corners, toed out 20° | -0.075, 0.131 | 0.760 |
| `F_front_low.urdf` | as C, base on the basket floor | -0.082, 0.108 | 0.695 |

Front layouts push the upstream `Base.stl` footprint into the basket's inside front corner with 5 mm clearance.
A redesigned shoulder piece with a smaller footprint would sit further into the corner.

## Results

Grasp point 8.5 cm from the wrist roll axis, top-down within 30° for surfaces, within 45° for basket drops,
2 cm bins. Table edge stops against the basket's front rim, counter edge is 2.5 cm ahead of the tyres.

| Layout | Past table edge | Past counter edge | Table area past edge, any / both arms (cm²) | Counter area (cm²) | Low table 0.45 m | Basket drops, any / both | Camera to farthest table point |
|---|---|---|---|---|---|---|---|
| A current | 0.14 m | none | 600 / 208 | 0 | none | 97% / 92% | 0.60 m |
| B front, forward | **0.32 m** | **0.13 m** | **1392** / 924 | **628** | none | 97% / 52% | 0.72 m |
| C front, in 30° | 0.30 m | 0.11 m | 1292 / **936** | 524 | none | 97% / 62% | 0.70 m |
| D front, in 45° | 0.30 m | 0.09 m | 1248 / 920 | 484 | none | 97% / 68% | 0.69 m |
| E front, out 20° | 0.30 m | 0.11 m | 1312 / 860 | 540 | none | 89% / 41% | 0.71 m |
| F as C, lowered | 0.28 m | none | 1228 / 860 | 0 | 0.10 m past tyres | 97% / 39% | 0.69 m |

No layout reaches the floor. Side reach at table height is about 0.46-0.50 m from the centreline in all of them.

`reach_maps.png`: top-down maps for A, B, C and F at table height, counter height and over the basket.

## Not modelled

Arm-to-arm and self collisions, the arm bases as obstacles, the neck box size (assumed 12 x 8 cm, TODO measure),
servo torque at full stretch (12 V STS3215s, about 30 kg·cm stall: a few hundred grams at 0.35 m is fine, check a heavy cup at full stretch), base flex on the
wire mesh, and orientation beyond the "roughly top-down" cone.
