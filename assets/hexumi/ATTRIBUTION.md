# Hexumi visual meshes

The visual meshes in this folder are copies of the Hexumi robot meshes by Maksymilian Firkowski.

- Source: https://github.com/firkowski/hexumi
- Original files: `src/hexumi_description/meshes/visual/*.stl`
- License: Creative Commons Attribution 4.0 International
- Changes: no mesh deformation or triangle reduction; `BODY.stl` is split into four face batches only to satisfy MuJoCo's per-mesh face limit. The original `0.001` URDF scale and assembly transforms are reproduced in `models/closed_loop.xml`.

These meshes are visual-only. The local MuJoCo primitive collision geometry, mass, joints, actuators, and gait controller remain authoritative for physics and control.
