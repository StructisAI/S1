# S1 Arm Description

URDF models of the Structis S1 7-DOF arm, right and left variants, for a
bimanual setup. Kinematics (link lengths, joint origins, axes, limits) are
CAD-derived and validated by replaying real recorded teleop episodes through
the models. Masses and inertias are CAD estimates (uniform effective density)
— good enough for visualization and stable simulation. Measured inertials
will follow.

## Files

```
s1_description/
├── urdf/
│   ├── s1_right.urdf    # right arm
│   └── s1_left.urdf     # left arm
├── meshes/visual/       # per-link OBJ + s1_materials.mtl (metres) — referenced by the URDFs
│                        # + per-link GLB with PBR materials (used by the web viewer)
└── meshes/collision/    # per-link convex hulls (optional; URDF ships with boxes)
```

Meshes are referenced by relative path, so keep the folder structure intact
and the URDFs load directly in pybullet, MuJoCo's URDF importer, yourdfpy,
trimesh, Isaac, etc.

The left arm is a true geometric mirror of the right (meshes, joint origins,
inertials, and revolute axes reflected across the local XZ plane), not a
yaw-rotated right arm. Both variants share the same mesh files; the left URDF
applies a -1 X scale.

## Joints (identical order in both arms)

| Joint | Type | Parent -> child | Limits (rad / m) |
| --- | --- | --- | --- |
| `base` | revolute | `base_link` -> `link1` | -1.590 .. 1.570 |
| `link1` | revolute | `link1` -> `link2` | -1.980 .. 1.970 |
| `link2` | revolute | `link2` -> `link3` | -1.180 .. 3.960 |
| `link3` | revolute | `link3` -> `link4` | -2.120 .. 1.950 |
| `link4` | revolute | `link4` -> `link5` | -0.650 .. 0.750 |
| `link5` | revolute | `link5` -> `gripper_base_link` | -3.170 .. 3.080 |
| `gripper` | prismatic | `gripper_base_link` -> `left_finger_link` | -0.045 .. 0.000 |

`gripper_right_mimic` mirrors `gripper` (one actuated gripper DOF total).
Gripper convention: `0.0` = fully open, `-0.045` = fully closed (45 mm travel
per finger). Axis directions are mirrored between the two arms where the
mirror requires it — drive each arm with its own URDF and the joint values
behave consistently.

Fixed frames on both arms: `camera_link` (wrist camera mount) and `tool0`
(TCP reference), both children of `gripper_base_link`.

## Quick load check

```python
from yourdfpy import URDF
import numpy as np

u = URDF.load("urdf/s1_right.urdf")
u.update_cfg(np.zeros(7))
print(u.get_transform("gripper_base_link", "base_link"))
u.show()  # interactive viewer
```

## Notes

- Units: meters / radians, Z-up, standard URDF conventions.
- Collision geometry: one simplified box per major link — fast and stable out
  of the box. Swap in convex-decomposed meshes if you need tight contact
  geometry.
- `link2` has a wide mechanical range (-1.18 .. 3.96) but at the zero posture
  only -1.18 .. 1.10 and 2.00 .. 3.96 are self-collision free; a URDF limit
  cannot encode a split range, so enforce this in your planner if you sweep
  the full range.
- `*.glb`: the full CAD tessellation (~944 k faces), welded, creased normals,
  PBR materials per part, Draco-compressed (2.2 MB total) — used by the browser
  viewer. `*.obj` + `s1_materials.mtl`: decimated copies for simulators. Any
  material whose name contains `clear` is rendered translucent by the viewer. `meshes/collision/*_hull.stl` are convex hulls if you want
  tighter contact than the default boxes.
- Interactive viewer: https://structisai.github.io/S1/viewer/ (drag the joints).
- Questions / rough edges: roberto@structis.ai
