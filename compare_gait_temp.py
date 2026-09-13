import math

import mujoco
import numpy as np

# 比较根关节方向和抬腿时序，选择横向漂移最小的组合。
model = mujoco.MjModel.from_xml_path(
    "/mnt/d/Work/Projects/spider_sim_cpp/models/closed_loop.xml"
)


def simulate(side_directions, aligned_lift):
    data = mujoco.MjData(model)
    data.qpos[2] = 0.120
    for index in range(6):
        data.qpos[7 + index * 3:10 + index * 3] = (
            0.0,
            0.294915656,
            -1.765373443,
        )
    mujoco.mj_forward(model, data)

    phase = 0.0
    phase_offsets = (0.0, 0.5, 0.5, 0.0, 0.0, 0.5)
    maximum_speed = 0.0
    for _ in range(240):
        phase = math.fmod(phase + 0.0166666667 / 1.8, 1.0)
        targets = []
        for index in range(6):
            leg_phase = math.fmod(phase + phase_offsets[index], 1.0)
            phase_angle = 2.0 * math.pi * leg_phase
            if aligned_lift:
                lift = 0.10 * max(0.0, math.cos(phase_angle))
            else:
                lift = (
                    0.10 * math.sin(phase_angle)
                    if leg_phase < 0.5
                    else 0.0
                )
            swing = 0.12 * math.sin(phase_angle)
            targets.extend(
                (
                    side_directions[index] * swing,
                    0.294915656 + lift,
                    -1.765373443 - lift,
                )
            )

        frame_start = data.time
        while data.time - frame_start < 0.0166666667:
            for actuator, target in enumerate(targets):
                data.ctrl[actuator] = np.clip(
                    4.0 * (target - data.qpos[7 + actuator])
                    - 0.7 * data.qvel[6 + actuator],
                    -0.8,
                    0.8,
                )
            mujoco.mj_step(model, data)
            maximum_speed = max(
                maximum_speed,
                float(np.linalg.norm(data.qvel[:3])),
            )

    w, x, y, z = data.qpos[3:7]
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return data.qpos[:3].copy(), yaw, maximum_speed, data.ncon


old_signs = (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
new_signs = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
for label, signs, aligned in (
    ("old/current", old_signs, False),
    ("new/current", new_signs, False),
    ("old/aligned", old_signs, True),
    ("new/aligned", new_signs, True),
):
    position, yaw, maximum_speed, contacts = simulate(signs, aligned)
    print(
        label,
        "position",
        [round(float(value), 4) for value in position],
        "yaw",
        round(math.degrees(yaw), 2),
        "maximum_speed",
        round(maximum_speed, 3),
        "contacts",
        contacts,
    )
