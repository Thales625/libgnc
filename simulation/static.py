from embedded import EmbeddedBackend

from config import SimulationConfig, StaticSimulationConfig
from utils.quaternion import quaternion_to_euler

import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

if __name__ == "__main__":
    backend = EmbeddedBackend()

    t_arr = []
    cmd_arr = defaultdict(list)
    ctrl_input_arr = defaultdict(list)
    true_state_arr = defaultdict(list)
    est_state_arr = defaultdict(list)
    tgt_state_arr = defaultdict(list)

    next_time_plot = 0.0

    # simulate
    print("[simulation] starting...")

    for t in np.arange(0, StaticSimulationConfig.total_time, SimulationConfig.dt):
        backend.sim_loop(t)

        if t > next_time_plot:
            print(f"[simulation] {t:.2f}/{StaticSimulationConfig.total_time:.2f}s ({t/StaticSimulationConfig.total_time*100:.1f}%)")

            next_time_plot += StaticSimulationConfig.plot_dt

            # populate plot arrays
            for i, value in enumerate([r.control for r in backend.drone.rotors]):
                cmd_arr[i].append(value)

            for i, value in enumerate(backend.optimal_control):
                ctrl_input_arr[i].append(value)

            for i, value in enumerate(backend.drone.state):
                true_state_arr[i].append(value)

            for i, value in enumerate(backend.estimated_state):
                est_state_arr[i].append(value)

            for i, value in enumerate(backend.control.target_state):
                tgt_state_arr[i].append(value)

            t_arr.append(t)

    print("[simulation] done!")

    # plot
    fig = plt.figure(figsize=(17, 10))
    gs = fig.add_gridspec(2, 2)

    ignore_estimation = True

    # --- position plot ---
    ax_pos = fig.add_subplot(gs[0, 0])
    ax_pos.set_title("Position")
    ax_pos.set_ylabel("Position (m)")

    # target
    ax_pos.plot(t_arr, tgt_state_arr[0], color="red",   linestyle=":", alpha=0.6, label="Target X")
    ax_pos.plot(t_arr, tgt_state_arr[1], color="green", linestyle=":", alpha=0.6, label="Target Y")
    ax_pos.plot(t_arr, tgt_state_arr[2], color="blue",  linestyle=":", alpha=0.6, label="Target Z")

    # true
    ax_pos.plot(t_arr, true_state_arr[0], color="red",   linestyle="-", alpha=0.8, label="True X")
    ax_pos.plot(t_arr, true_state_arr[1], color="green", linestyle="-", alpha=0.8, label="True Y")
    ax_pos.plot(t_arr, true_state_arr[2], color="blue",  linestyle="-", alpha=0.8, label="True Z")

    # estimated
    if not ignore_estimation:
        ax_pos.plot(t_arr, est_state_arr[0], color="red",   linestyle="-.", linewidth=2, label="Est. X")
        ax_pos.plot(t_arr, est_state_arr[1], color="green", linestyle="-.", linewidth=2, label="Est. Y")
        ax_pos.plot(t_arr, est_state_arr[2], color="blue",  linestyle="-.", linewidth=2, label="Est. Z")
    ax_pos.grid(True, linestyle="--", alpha=0.5)
    ax_pos.legend(ncol=2 if ignore_estimation else 3, framealpha=0.9)

    # --- attitude plot ---
    ax_att = fig.add_subplot(gs[0, 1], sharex=ax_pos)
    ax_att.set_title("Attitude")
    ax_att.set_ylabel("Degrees (°)")

    # true
    pitch_arr, roll_arr, yaw_arr = [], [], []
    for i, _ in enumerate(t_arr):
        pitch, roll, yaw = quaternion_to_euler(np.array([true_state_arr[6][i], true_state_arr[7][i], true_state_arr[8][i], true_state_arr[9][i]]))
        pitch_arr.append(np.rad2deg(pitch))
        roll_arr.append(np.rad2deg(roll))
        yaw_arr.append(np.rad2deg(yaw))
    ax_att.plot(t_arr, pitch_arr, color="red",  linestyle="-", alpha=0.8, label=r"True $\phi$")
    ax_att.plot(t_arr, roll_arr, color="green", linestyle="-", alpha=0.8, label=r"True $\theta$")
    ax_att.plot(t_arr, yaw_arr, color="blue",   linestyle="-", alpha=0.8, label=r"True $\psi$")

    # estimated
    if not ignore_estimation:
        pitch_arr, roll_arr, yaw_arr = [], [], []
        for i, _ in enumerate(t_arr):
            pitch, roll, yaw = quaternion_to_euler(np.array([est_state_arr[6][i], est_state_arr[7][i], est_state_arr[8][i], est_state_arr[9][i]]))
            pitch_arr.append(np.rad2deg(pitch))
            roll_arr.append(np.rad2deg(roll))
            yaw_arr.append(np.rad2deg(yaw))
        ax_att.plot(t_arr, pitch_arr, color="red",   linestyle="-.", linewidth=2, label=r"Est. $\phi$")
        ax_att.plot(t_arr, roll_arr,  color="green", linestyle="-.", linewidth=2, label=r"Est. $\theta$")
        ax_att.plot(t_arr, yaw_arr,   color="blue",  linestyle="-.", linewidth=2, label=r"Est. $\psi$")
    ax_att.grid(True, linestyle="--", alpha=0.5)
    ax_att.legend(ncol=1 if ignore_estimation else 2, framealpha=0.9)

    # --- control input plot ---
    ax_ctrl_input = fig.add_subplot(gs[1, 0], sharex=ax_pos)
    ax_ctrl_input.set_title("Control Input")
    ax_ctrl_input.set_xlabel("Time (s)")
    ax_ctrl_input.set_ylabel("Force (N) | Torque (N·m)")
    ax_ctrl_input.plot(t_arr, ctrl_input_arr[0], "orange", label=r"$\text{Thrust}$")
    ax_ctrl_input.plot(t_arr, ctrl_input_arr[1], "red",    label=r"$\tau_{\text{roll}}$")
    ax_ctrl_input.plot(t_arr, ctrl_input_arr[2], "green",  label=r"$\tau_{\text{pitch}}$")
    ax_ctrl_input.plot(t_arr, ctrl_input_arr[3], "blue",   label=r"$\tau_{\text{yaw}}$")
    ax_ctrl_input.grid(True, linestyle="--", alpha=0.5)
    ax_ctrl_input.legend()

    # --- command plot ---
    ax_command = fig.add_subplot(gs[1, 1], sharex=ax_pos)
    ax_command.set_title("Rotor Command")
    ax_command.set_xlabel("Time (s)")
    ax_command.set_ylabel("Control Signal")
    ax_command.set_ylim(-0.05, 1.05)
    ax_command.plot(t_arr, cmd_arr[0], "red",    label="FR")
    ax_command.plot(t_arr, cmd_arr[1], "green",  label="FL")
    ax_command.plot(t_arr, cmd_arr[2], "blue",   label="BR")
    ax_command.plot(t_arr, cmd_arr[3], "orange", label="BL")
    ax_command.grid(True, linestyle="--", alpha=0.5)
    ax_command.legend(ncol=2)

    plt.tight_layout()
    plt.show()