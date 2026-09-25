import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

from config import SimulationConfig, LiveSimulationConfig
from utils.quaternion import rotate_vector

from core.vehicle import Vehicle

def run_live(vehicle: Vehicle, sim_loop: method):
    global t

    # control plot
    fig = plt.figure(figsize=(17, 9))
    gs = fig.add_gridspec(2, 2)

    # 3D plot
    ax = fig.add_subplot(gs[:, 0], projection="3d")
    ax.set_title("3D View")

    target_plot = ax.scatter(*np.zeros(3), c="r", s=50, label="target position")

    zoom = 5.
    axis_length = 0.1
    ax.set_xlim([-zoom, zoom])
    ax.set_ylim([-zoom, zoom])
    ax.set_zlim([-zoom, zoom])
    ax.set_xlabel('X')
    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_box_aspect([1, 1, 1])

    axis_x, = ax.plot([], [], [], "r-", lw=2, label="forward (x)")
    axis_y, = ax.plot([], [], [], "g-", lw=2, label="right (y)")
    axis_z, = ax.plot([], [], [], "b-", lw=2, label="up (z)")

    # position plot
    ax_pos = fig.add_subplot(gs[0, 1])
    ax_pos.set_title("Position Tracking")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.grid()

    line_pos_x, = ax_pos.plot([],[], 'r-', label="X")
    line_pos_y, = ax_pos.plot([],[], 'g-', label="Y")
    line_pos_z, = ax_pos.plot([],[], 'b-', label="Z")

    line_tgt_x, = ax_pos.plot([],[], 'r--', alpha=0.5, label="Target X")
    line_tgt_y, = ax_pos.plot([],[], 'g--', alpha=0.5, label="Target Y")
    line_tgt_z, = ax_pos.plot([],[], 'b--', alpha=0.5, label="Target Z")
    ax_pos.legend()

    pos_arr = [[], [], []]
    tgt_arr = [[], [],[]]

    # ESTIMATION
    est_axis_x, = ax.plot([], [], [], "m-", lw=3, label="estimation")
    est_axis_y, = ax.plot([], [], [], "m-", lw=3)
    est_axis_z, = ax.plot([], [], [], "m-", lw=3)

    rotor_vectors = []
    for rotor in vehicle.propulsions:
        line, = ax.plot([], [], [], "r-" if rotor.clockwise else "g-", lw=3, alpha=0.8)
        rotor_vectors.append(line)

    ax.legend()

    # control plot
    ax_control = fig.add_subplot(gs[1, 1])
    ax_control.set_title("Rotor Controls")
    ax_control.set_xlabel("Time (s)")
    ax_control.set_ylabel("Control Signal")
    ax_control.set_ylim(0., 1.)
    ax_control.grid()

    line_controls = []
    colors = ['r', 'g', 'b', "orange"]
    labels = ["front-right", "front-left", "back-right", "back-left"]
    for i in range(4):
        line_c, = ax_control.plot([], [], color=colors[i], label=labels[i])
        line_controls.append(line_c)

    ax_control.legend()

    t_arr = []
    ctrl_arr = [[], [], [], []]

    # loop
    camera_follow = False
    t = 0.

    def on_key(event):
        global camera_follow

        if event.key == 'c':
            camera_follow = not camera_follow

    def on_scroll(event):
        global zoom

        if not camera_follow:
            return

        if event.button == "up":
            zoom /= 1.1
        elif event.button == "down":
            zoom *= 1.1

    def update(frame):
        global t

        # execute N physics updates
        for _ in range(LiveSimulationConfig.steps_per_frame):
            sim_loop(t)

            t += SimulationConfig.dt

        pos = vehicle.position
        rot = vehicle.rotation
        target_state = vehicle.avionics.core.get_target_state()

        # plot target position
        target_plot._offsets3d = (
            [target_state[0]],
            [target_state[1]],
            [target_state[2]]
        )

        t_arr.append(t)

        for i in range(3):
            pos_arr[i].append(pos[i])
            tgt_arr[i].append(target_state[i])

        # update position plot
        line_pos_x.set_data(t_arr, pos_arr[0])
        line_pos_y.set_data(t_arr, pos_arr[1])
        line_pos_z.set_data(t_arr, pos_arr[2])

        # update target plot
        line_tgt_x.set_data(t_arr, tgt_arr[0])
        line_tgt_y.set_data(t_arr, tgt_arr[1])
        line_tgt_z.set_data(t_arr, tgt_arr[2])

        # rescale plot limits
        ax_pos.relim()
        ax_pos.autoscale_view()

        # vehicle axis
        vehicle_x_axis = rotate_vector(rot, np.array([axis_length, 0., 0.]))
        vehicle_y_axis = rotate_vector(rot, np.array([0., axis_length, 0.]))
        vehicle_z_axis = rotate_vector(rot, np.array([0., 0., axis_length]))

        axis_x.set_data([pos[0], pos[0] + vehicle_x_axis[0]], [pos[1], pos[1] + vehicle_x_axis[1]])
        axis_x.set_3d_properties([pos[2], pos[2] + vehicle_x_axis[2]])

        axis_y.set_data([pos[0], pos[0] + vehicle_y_axis[0]], [pos[1], pos[1] + vehicle_y_axis[1]])
        axis_y.set_3d_properties([pos[2], pos[2] + vehicle_y_axis[2]])

        axis_z.set_data([pos[0], pos[0] + vehicle_z_axis[0]], [pos[1], pos[1] + vehicle_z_axis[1]])
        axis_z.set_3d_properties([pos[2], pos[2] + vehicle_z_axis[2]])

        # ESTIMATION
        est_state = vehicle.avionics.core.estimated_state()
        est_pos = est_state[0:3]
        est_rot = est_state[6:10]

        est_vehicle_x_axis = rotate_vector(est_rot, np.array([axis_length, 0., 0.]))
        est_vehicle_y_axis = rotate_vector(est_rot, np.array([0., axis_length, 0.]))
        est_vehicle_z_axis = rotate_vector(est_rot, np.array([0., 0., axis_length]))

        est_axis_x.set_data([est_pos[0], est_pos[0] + est_vehicle_x_axis[0]], [est_pos[1], est_pos[1] + est_vehicle_x_axis[1]])
        est_axis_x.set_3d_properties([est_pos[2], est_pos[2] + est_vehicle_x_axis[2]])

        est_axis_y.set_data([est_pos[0], est_pos[0] + est_vehicle_y_axis[0]], [est_pos[1], est_pos[1] + est_vehicle_y_axis[1]])
        est_axis_y.set_3d_properties([est_pos[2], est_pos[2] + est_vehicle_y_axis[2]])

        est_axis_z.set_data([est_pos[0], est_pos[0] + est_vehicle_z_axis[0]], [est_pos[1], est_pos[1] + est_vehicle_z_axis[1]])
        est_axis_z.set_3d_properties([est_pos[2], est_pos[2] + est_vehicle_z_axis[2]])

        # rotors
        for i, rotor in enumerate(vehicle.propulsions):
            rotor_pos_world = pos + rotate_vector(rot, rotor.position_body)

            rotor_thrust_body = 0.2 * rotor.force
            rotor_thrust_world = rotate_vector(rot, rotor_thrust_body)

            rotor_vectors[i].set_data(
                [rotor_pos_world[0], rotor_pos_world[0] + rotor_thrust_world[0]],
                [rotor_pos_world[1], rotor_pos_world[1] + rotor_thrust_world[1]]
            )
            rotor_vectors[i].set_3d_properties(
                [rotor_pos_world[2], rotor_pos_world[2] + rotor_thrust_world[2]]
            )

        # follow vehicle
        if camera_follow:
            ax.set_xlim([pos[0] - zoom, pos[0] + zoom])
            ax.set_ylim([pos[1] - zoom, pos[1] + zoom])
            ax.set_zlim([pos[2] - zoom, pos[2] + zoom])

        # lock camera roll
        ax.view_init(elev=ax.elev, azim=ax.azim)

        # update control plot
        for i in range(4):
            ctrl_arr[i].append(vehicle.propulsions[i].throttle)
            line_controls[i].set_data(t_arr, ctrl_arr[i])

        ax_control.relim()
        ax_control.autoscale_view(scaley=False)

        return [axis_x, axis_y, axis_z] + [line_tgt_x, line_tgt_y, line_tgt_z] + [line_pos_x, line_pos_y, line_pos_z] + rotor_vectors + line_controls + [est_axis_x, est_axis_y, est_axis_z]

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    ani = FuncAnimation(fig, update, interval=1000.*SimulationConfig.dt*LiveSimulationConfig.steps_per_frame, blit=False, cache_frame_data=False)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    from .setup import vehicle, sim_loop

    run_live(vehicle, sim_loop)