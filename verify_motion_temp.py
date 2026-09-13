import math

import mujoco
import numpy as np

# 复现修复后的前进步态，检查速度、方向和姿态是否受控。
model = mujoco.MjModel.from_xml_path(
    "/mnt/d/Work/Projects/spider_sim_cpp/models/closed_loop.xml"
)
data = mujoco.MjData(model)
frame_duration = 0.0166666667
cycle_duration = 1.8
stride_angle = 0.12
lift_angle = 0.10
middle_stand = 0.294915656
distal_stand = -1.765373443
phase_offsets = (0.0, 0.5, 0.5, 0.0, 0.0, 0.5)
side_directions = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0)

data.qpos[2] = 0.120
for index in range(6):
    data.qpos[7 + index * 3:10 + index * 3] = (
        0.0,
        middle_stand,
        distal_stand,
    )
mujoco.mj_forward(model, data)

phase = 0.0
maximum_speed = 0.0
for frame in range(360):
    phase = math.fmod(phase + frame_duration / cycle_duration, 1.0)
    targets = []
    for index in range(6):
        leg_phase = math.fmod(phase + phase_offsets[index], 1.0)
        phase_angle = 2.0 * math.pi * leg_phase
        lift = lift_angle * math.sin(phase_angle) if leg_phase < 0.5 else 0.0
        swing = stride_angle * math.sin(phase_angle)
        targets.extend(
            (
                side_directions[index] * swing,
                middle_stand + max(0.0, lift),
                distal_stand - max(0.0, lift),
            )
        )

    frame_start = data.time
    while data.time - frame_start < frame_duration:
        for actuator, target in enumerate(targets):
            data.ctrl[actuator] = np.clip(
                4.0 * (target - data.qpos[7 + actuator])
                - 0.7 * data.qvel[6 + actuator],
                -0.8,
                0.8,
            )
        mujoco.mj_step(model, data)
        maximum_speed = max(maximum_speed, float(np.linalg.norm(data.qvel[:3])))

print("time", round(data.time, 3))
print("position", [round(float(value), 4) for value in data.qpos[:3]])
print("quaternion", [round(float(value), 4) for value in data.qpos[3:7]])
print("final_speed", round(float(np.linalg.norm(data.qvel[:3])), 4))
print("maximum_speed", round(maximum_speed, 4))
print("contacts", data.ncon)
