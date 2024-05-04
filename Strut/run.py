import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np


# from blocklyTranslations import *
from types import SimpleNamespace

# from TimeHelper import TimeHelper  # TODO add to files downloaded
# import globals
import scipy.spatial
from scipy.optimize import linear_sum_assignment

import numpy as np

takeoff_height = 2
velocity = 0.7


total_time = 90  # Time in seconds
delta_t = 0.05
time_steps = (int)(total_time / delta_t)
drone_count = 8
dimensions = 3


drones = []  # ADD DRONES TO THIS LIST
button_pressed = np.zeros(11)  # True when a button has been pressed
button = "X"  # Updated with the most recently pressed button
max_vel = 0.8  #
_time_step = 0
fly_height = 2.1

Hz = 30
drone_id = 1


strut_matrix = np.zeros((drone_count, dimensions, 3))
v_matrix = np.zeros((drone_count, dimensions, 2))
intro_matrix = strut_matrix = np.zeros((drone_count, dimensions, 3))

left_side_strut_points = [[-2, 2, 2.1], [2, 2, 2.1]]
right_side_strut_points = [[2, 2, 2.1], [-2, 2, 2.1]]

v_mygoto = [[-1.9, -2, 2.1], [-1.6, -1, 2.1], [-1.3, 0, 2.1], [-1, 1, 2.1], [1.9, -2, 2.1], [1.6, -1, 2.1], [1.3, 0, 2.1], [1, 1, 2.1]]
v_mygoto2 = [[-1.9, -2, 1.99], [-1.6, -1.5, 1.66], [-1.3, -1, 1.33], [-1, -0.5, 1], [1.9, -2, 1.99], [1.6, -1.5, 1.66], [1.3, -1, 1.33], [1, -0.5, 1]]


left_side_intro_points = [[-2.75, 2, 2.1], [2.75, 2, 2.1]]
right_side_intro_points = [[2.75, 2, 2.1], [-2.75, 2, 2.1]]


# Write an np array of shape (8, 3 )
start_positions = np.array(
    [
        [-2.75, 1, fly_height],
        [-2.75, 0, fly_height],
        [-2.75, -1, fly_height],
        [-2.75, -2, fly_height],
        [2.75, 1, fly_height],
        [2.75, 0, fly_height],
        [2.75, -1, fly_height],
        [2.75, -2, fly_height],
    ]
)

default_destinations = np.array(
    [
        [-2, -2, fly_height],
        [-2, -1, fly_height],
        [-2, 0, fly_height],
        [-2, 1, fly_height],
        [2, -2, fly_height],
        [2, -1, fly_height],
        [2, 0, fly_height],
        [2, 1, fly_height],
    ]
)

drone_step_sequence = np.tile(start_positions[:, :, np.newaxis], (1, 1, time_steps))


# These two functions each initialize each half of the strut matrix
for drone_index in range(0, 4, 1):
    for point_index in range(3):
        if point_index == 0:
            target_point = left_side_strut_points[point_index]
        elif point_index == 1:
            target_point = left_side_strut_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
        strut_matrix[drone_index, :, point_index] = target_point

for drone_index in range(4, 8, 1):
    for point_index in range(3):
        if point_index == 0:
            target_point = right_side_strut_points[point_index]
        elif point_index == 1:
            target_point = right_side_strut_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
        strut_matrix[drone_index, :, point_index] = target_point


# These two functions initialize each side of the intro_matrix

for drone_index in range(0, 4, 1):
    for point_index in range(3):
        if point_index == 0:
            target_point = left_side_intro_points[point_index]
        elif point_index == 1:
            target_point = right_side_intro_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
        intro_matrix[drone_index, :, point_index] = target_point

for drone_index in range(4, 8, 1):
    for point_index in range(3):
        if point_index == 0:
            target_point = left_side_intro_points[point_index]
        elif point_index == 1:
            target_point = right_side_intro_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
        intro_matrix[drone_index, :, point_index] = target_point



def round_to_nearest_time_step(value):
    return np.round(value / delta_t) * delta_t


# This will copy the drone locations at the end_time to the remainder of the tensor
def hover_in_place(end_time):
    # hover_locations will be an 8 x 3 shaped array
    hover_locations = np.asarray(
        [drone_step_sequence[i, :, end_time] for i in range(drone_count)]
    )  # 8 x 3 array
    hover_locations = np.expand_dims(hover_locations, axis=2)
    num_steps_hovering = int(end_time / delta_t) - int(_time_step / delta_t)

    # We need to tile this hover location onto


    drone_step_sequence[
        :, :, (int)(_time_step / delta_t) : int(end_time / delta_t)
    ] = np.reshape(np.tile(
        hover_locations, (1, 1, num_steps_hovering)), (drone_count, dimensions, num_steps_hovering))

# Draws a straight line on the drone_step_sequence tensor for a drone
def straight_line(start_time, end_time, start_location, end_location, drone):
    trajectory_length = (int)((end_time - start_time) / delta_t)
    vel = (end_location - start_location) / (end_time - start_time)
    if vel[0] != 0:
        x_path = np.arange(start_location[0], end_location[0], vel[0] * delta_t)[
            0:trajectory_length
        ]
    else:
        x_path = np.full(trajectory_length, start_location[0])

    if vel[1] != 0:
        y_path = np.arange(start_location[1], end_location[1], vel[1] * delta_t)[
            0:trajectory_length
        ]
    else:
        y_path = np.full(trajectory_length, start_location[1])

    if vel[2] != 0:
        z_path = np.arange(start_location[2], end_location[2], vel[2] * delta_t)[
            0:trajectory_length
        ]
    else:
        z_path = np.full(trajectory_length, start_location[2])

    drone_step_sequence[
        drone, 0, (int)(start_time / delta_t) : (int)(end_time / delta_t)
    ] = x_path
    drone_step_sequence[
        drone, 1, (int)(start_time / delta_t) : (int)(end_time / delta_t)
    ] = y_path
    drone_step_sequence[
        drone, 2, (int)(start_time / delta_t) : (int)(end_time / delta_t)
    ] = z_path


# Input:
# target_locations - 8 x 3 np.array - where are our drones going?
# Return:
# end_time - float - returns the time ins second in which this timestep is done
def my_goto(target_locations):
    current_locations = np.asarray(
        [
            drone_step_sequence[i, :, (int)(_time_step / delta_t)]
            for i in range(drone_count)
        ]
    )  # 8 x 3 array
    max_duration = np.max(
        np.linalg.norm(target_locations - current_locations) / max_vel
    )
    end_time = _time_step + max_duration

    for i in range(drone_count):
        straight_line(
            _time_step, end_time, current_locations[i], target_locations[i], i
        )
    return end_time


# STRUT WILL FIRST SEND ALL DRONES TO THEIR DEFAULT POSITIONS, AND THEN INDEXING FROM strut_matrix,
# WILL RUN TWO LINES OF DRONES ACROSS FROM EACH OTHER
def strut():
    # Go to default locations
    # current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
    strut_begin_time = my_goto(default_destinations)
    for drone_index in range(drone_count):
        time_iterator = strut_begin_time
        for point in range(dimensions):
            target_location = strut_matrix[drone_index, :, point]
            cur_pos = None
            if point == 0:
                cur_pos = default_destinations[drone_index]
            else:
                cur_pos = strut_matrix[drone_index, :, point - 1]
            end_time = (
                round_to_nearest_time_step(
                    np.linalg.norm(target_location - cur_pos) / max_vel
                )
                # + time_step //not sure if this is needed?
            )
            straight_line(
                time_iterator, end_time, cur_pos, target_location, drone_index
            )
            time_iterator = end_time
    hover_in_place(time_iterator)


# TODO STRETCH GOAL: IMPLEMENT A FOLLOWER_MODE WITH A MOCAP SUBSRIBER
def follower_mode(mocap_coordinates):
    pass


def circles():

    k = 1
    layer_1_height = 1
    layer_2_height = 2.5

    circling_start_time = my_goto(default_destinations)

    phi_layer_1 = np.arange(0, 2 * 3.14159, 2 * 3.14159 / 4)
    phi_layer_2 = np.arange(3.14159 / 2, 2 * 3.14159 + 3.14159 / 2, 2 * 3.14159 / 4)

    assert len(phi_layer_1) == 4
    assert len(phi_layer_2) == 4

    num_steps_left = (int)((total_time - circling_start_time) / delta_t) + 1
    t = np.arange(start=circling_start_time, stop=total_time, step=delta_t)
    t = t[:, np.newaxis]

    layer1_cos_values = np.zeros((4, num_steps_left))
    layer1_sin_values = np.zeros((4, num_steps_left))
    layer2_cos_values = np.zeros((4, num_steps_left))
    layer2_sin_values = np.zeros((4, num_steps_left))
    for i in range(4):
        # Print the shape of k and t
        layer1_cos_values[i] = np.reshape(
            np.cos(k * t + phi_layer_1[i]), (num_steps_left,)
        )
        layer1_sin_values[i] = np.reshape(
            np.sin(k * t + phi_layer_1[i]), (num_steps_left,)
        )
        layer2_cos_values[i] = np.reshape(
            np.cos(k * t + phi_layer_2[i]), (num_steps_left,)
        )
        layer2_sin_values[i] = np.reshape(
            np.sin(k * t + phi_layer_2[i]), (num_steps_left,)
        )

    layer1_height_array = np.full((4, num_steps_left), layer_1_height)
    layer2_height_array = np.full((4, num_steps_left), layer_2_height)

    layer1_circling_paths = np.stack(
        (layer1_cos_values, layer1_sin_values, layer1_height_array), axis=-1
    ).reshape(4, 3, num_steps_left)
    layer2_circling_paths = np.stack(
        (layer2_cos_values, layer2_sin_values, layer2_height_array), axis=-1
    ).reshape(4, 3, num_steps_left)

    import scipy
    import scipy.spatial

    drone_step_sequence[
        0:4, :, (int)(circling_start_time / delta_t) : (int)(total_time / delta_t)
    ] = layer1_circling_paths
    drone_step_sequence[
        4:8, :, (int)(circling_start_time / delta_t) : (int)(total_time / delta_t)
    ] = layer2_circling_paths

    starting_values = drone_step_sequence[0:8, :, (int)(circling_start_time / delta_t)]
    
    dists_start = scipy.spatial.distance_matrix(
        starting_values,
        np.concatenate(layer1_circling_paths[:, :, 0], layer2_circling_paths[:, :, 0], axis=0),
    )
    assignments = linear_sum_assignment(dists_start)[1]

    for j in range(drone_count):
        i = assignments[j]
        if(j > 3):
            drone_step_sequence[i, :, -num_steps_left] = layer2_circling_paths[j - 4]
        else:
            drone_step_sequence[i, :, -num_steps_left] = layer1_circling_paths[j]


def train():
    pass


def intro_march():
    strut_begin_time = my_goto(start_positions)
    time_iterator = None
    for drone_index in range(drone_count):
        time_iterator = strut_begin_time
        for point in range(dimensions):
            target_location = intro_matrix[drone_index, :, point]
            cur_pos = None
            if point == 0:
                cur_pos = start_positions[drone_index]
            else:
                cur_pos = strut_matrix[drone_index, :, point - 1]
            end_time = (
                round_to_nearest_time_step(
                    np.linalg.norm(target_location - cur_pos) / max_vel
                )
            )
            straight_line(
                time_iterator, end_time, cur_pos, target_location, drone_index
            )
            time_iterator = end_time
    hover_in_place(time_iterator)


def disperse_to_default():
    for drone_index in range(drone_count):
        current_positions = drone_step_sequence[
            drone_index, :, (int)(_time_step / delta_t)
        ]
        end_time = (
            round_to_nearest_time_step(
                np.linalg.norm(default_destinations - current_positions) / max_vel
            )
            + _time_step
        )

        
        dists_start = scipy.spatial.distance_matrix(
            default_destinations,
            current_positions,
        )
        assignments = linear_sum_assignment(dists_start)[1]

        straight_line(
            _time_step, end_time, current_positions, default_destinations, drone_index
        )
        hover_in_place(end_time)
        drone_step_sequence = drone_step_sequence[assignments]


def v_formation():
    # Go to default locations
    # current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
    v_begin_time = my_goto(default_destinations)

    v_first_point_time = my_goto(v_mygoto, v_begin_time-delta_t)
    v_second_point_time = my_goto(v_mygoto2, v_first_point_time-delta_t)

    hover_in_place(v_second_point_time-delta_t)


    # Make a subscriber to some topic in the launch.py


def update_drone_step_sequence(button_index):
    print("Test")
    print(button_index)
    if button_index == 0:  # Follower
        print("Follower Mode: Green button")
        follower_mode("Placeholder")
    elif button_index == 2:  # Circles
        print("Circles! Blue button")
        circles()
    elif button_index == 3:  # Train
        print("Train! Yellow Button")
        train()
    elif button_index == 7:  # Intro March
        print("Intro March! Start button")
        intro_march()
    elif button_index == 6:  # Disperse
        print("Disperse! Back Button")
        disperse_to_default()
    elif button_index == 8:  # V-Formation
        print("V-form! Circle Button")
        v_formation()
    elif button_index == 10:  # Strut - right joy in
        print("Strut! right joystick in")
        strut()

def initialize():
    drone_step_sequence[:, :, 0] = start_positions
    hover_in_place(total_time)



def main():
    from crazyflie_py import Crazyswarm

    swarm = Crazyswarm(log_poses=False)
    allcfs = (
        swarm.allcfs
    )  # This is in order of x position(low x value means beginning of the list)
    drones = allcfs.crazyflies
    timeHelper = swarm.timeHelper

    # Takeoff
    allcfs.takeoff(2.1, 3)
    timeHelper.sleep(3.0)
    for time_step in np.arange(start=0, stop=total_time, step=delta_t):
        print(time_step)
        _time_step = time_step
        button_press = swarm.input.checkIfAnyButtonIsPressed()
        button_index = np.argmax(button_press)
        if np.count_nonzero(button_press) == 0:
            button_index = -1
        update_drone_step_sequence(button_index)
        for i, drone in enumerate(drones):
            coords = drone_step_sequence[i, :, int(time_step / delta_t)]
            drone.cmdPosition(coords)  # instead of goto
        timeHelper.sleepForRate(1 / delta_t)

    allcfs.land(0, 5.0)
    timeHelper.sleep(5.0)





if __name__ == "__main__":
    main()


# TODO:
# (3) Create the update codes for different moves
#       a. Strut - Draft Finished
#       b. Follower in one dimension(side to side of the performer)
#       c. Circles around the performer - Draft finished
#       d. Train Motion(Crazy lmfao)
#       e. Intro march in lines - Draft finished
#       f. Disperse
#       g. V-formation
# (7) ADJUST k VALUE IN circles() function
# (8) GETTING DRONES IN A POSITION TO BEGIN CIRCLING - DRONE_INIT_MATRIX
# (9) WHAT does groupState mean in the takeoff and land functions? How is it reltated to drone index?
# (10) ADD SCIPI OPTIMIZER/COLLISION AVOIDANCE MECHANISM SAME AS IN THE CIRCLES
