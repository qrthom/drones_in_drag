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
velocity = 1.2


total_time = 150  # Time in seconds
delta_t = 0.05
time_steps = (int)(total_time / delta_t)
drone_count = 8
dimensions = 3


drones = []  # ADD DRONES TO THIS LIST
button_pressed = np.zeros(11)  # True when a button has been pressed
button = "X"  # Updated with the most recently pressed button
max_vel = 1.3  #
_time_step = 0
fly_height = 2.1

Hz = 30
drone_id = 1


v_matrix = np.zeros((drone_count, dimensions, 2))
intro_matrix = np.zeros((drone_count, dimensions, 3))
strut_matrix = np.zeros((drone_count, dimensions, 3))


v_mygoto = [[-1.9, -2, 2.1], [-1.6, -1, 2.1], [-1.3, 0, 2.1], [-1, 1, 2.1], [1.9, -2, 2.1], [1.6, -1, 2.1], [1.3, 0, 2.1], [1, 1, 2.1]]
v_mygoto2 = [[-1.9, -2, 1.99], [-1.6, -1.5, 1.66], [-1.3, -1, 1.33], [-1, -0.5, 1], [1.9, -2, 1.99], [1.6, -1.5, 1.66], [1.3, -1, 1.33], [1, -0.5, 1]]
v_mygoto3 = [[-1.9, -1, 1.49], [-1.6, -0.5, 1.16], [-1.3, 0, .83], [-1, 0.5, .5], [1.9, -1, 1.49], [1.6, -0.5, 1.16], [1.3, -0, .83], [1, 0.5, .5]]
v_mygoto4 = [[-1.9, -2, 1.99], [-1.6, -1.5, 1.66], [-1.3, -1, 1.33], [-1, -0.5, 1], [1.9, -2, 1.99], [1.6, -1.5, 1.66], [1.3, -1, 1.33], [1, -0.5, 1]]
v_mygoto5 = [[-1.9, 2, 1.99], [-1.6, 1.5, 1.66], [-1.3, 1, 1.33], [-1, 0.5, 1], [1.9, 2, 1.99], [1.6, 1.5, 1.66], [1.3, 1, 1.33], [1, 0.5, 1]]
v_mygoto6 = [[-1.9, -2, 1.99], [-1.6, -1.5, 1.66], [-1.3, -1, 1.33], [-1, -0.5, 1], [1.9, -2, 1.99], [1.6, -1.5, 1.66], [1.3, -1, 1.33], [1, -0.5, 1]]



left_side_strut_points = [[-2, 2, 2.1], [2, 2, 2.1]]
right_side_strut_points = [[2, 2.4, 2.1], [-2, 2.4, 2.1]]

left_side_intro_points = [[-2.75, 2, 2.1], [-2, 2, 2.1]]
right_side_intro_points = [[2.75, 2, 2.1], [2, 2, 2.1]]


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

start_positions_fall1 = np.array(
    [
        [-2.75+.1, 1, fly_height-.4],
        [-2.75, 0, fly_height],
        [-2.75, -1, fly_height],
        [-2.75, -2, fly_height],
        [2.75, 1, fly_height],
        [2.75, 0, fly_height],  
        [2.75, -1, fly_height],
        [2.75, -2, fly_height],
    ]
)
start_positions_fall2 = np.array(
    [
        [-2.75+.1, 1, fly_height-.4],
        [-2.75, 0, fly_height],
        [-2.75+.1, -1, fly_height-.4],
        [-2.75, -2, fly_height],
        [2.75, 1, fly_height],
        [2.75, 0, fly_height],  
        [2.75, -1, fly_height],
        [2.75, -2, fly_height],
    ]
)

start_positions_fall3 = np.array(
    [
        [-2.75+.1, 1, fly_height-.4],
        [-2.75, 0, fly_height],
        [-2.75+.1, -1, fly_height-.4],
        [-2.75, -2, fly_height],
        [2.75-.1, 1, fly_height-.4],
        [2.75, 0, fly_height],  
        [2.75, -1, fly_height],
        [2.75, -2, fly_height],
    ]
)


start_positions_fall4 = np.array(
    [
        [-2.75+.1, 1, fly_height-.4],
        [-2.75, 0, fly_height],
        [-2.75+.1, -1, fly_height-.4],
        [-2.75, -2, fly_height],
        [2.75-.1, 1, fly_height-.4],
        [2.75, 0, fly_height],  
        [2.75-.1, -1, fly_height-.4],
        [2.75, -2, fly_height],
    ]
)

meet_in_the_middle_destinations = np.array(
    [
        [-1.5, -2, fly_height],
        [-1.5, -1, fly_height],
        [-1.5, 0, fly_height],
        [-1.5, 1, fly_height],
        [1.5, -2, fly_height],
        [1.5, -1, fly_height],
        [1.5, 0, fly_height],
        [1.5, 1, fly_height],
    ]
)

x1 = np.array(
    [
        [-2, -2, fly_height],
        [-1, -1, fly_height],
        [-1, 1, fly_height],
        [-2, 2, fly_height],
        [2, -2, fly_height],
        [1, -1, fly_height],
        [1, 1, fly_height],
        [2, 2, fly_height],
    ]
)

x2 = np.array(
    [
        [-2, -2, fly_height],
        [-1, -1, fly_height-.5],
        [-1, 1, fly_height-1],
        [-2, 2, fly_height-1.5],
        [2, -2, fly_height],
        [1, -1, fly_height-.5],
        [1, 1, fly_height-1],
        [2, 2, fly_height-1.5],
    ]
)

x3_land = np.array(
    [
        [-2, -1, 0],
        [-1, 0, 0],
        [-1, 1, 0],
        [-2, 2, 0],
        [2, -2, 0],
        [1, -1, 0],
        [1, 0, 0],
        [2, 1, 0],
    ]
)

step_foward_2 = np.array(
    [
        [-2, -2, fly_height],
        [-2, -1, fly_height],
        [-2, 0, fly_height],
        [-2, 1, fly_height],
        [2, -1, fly_height],
        [2, 0, fly_height],
        [2, 1, fly_height],
        [2, 2, fly_height],
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

bf_position1 = np.array(
    [
        [-1.75, -2, fly_height],
        [-2, -1, fly_height],
        [-1.75, 0, fly_height],
        [-2, 1, fly_height],
        [1.75, -2, fly_height],
        [2, -1, fly_height],
        [1.75, 0, fly_height],
        [2, 1, fly_height],
    ]
)

bf_position2 = np.array(
    [
        [-2, -2, fly_height],
        [-1.75, -1, fly_height],
        [-2, 0, fly_height],
        [-1.75, 1, fly_height],
        [2, -2, fly_height],
        [1.75, -1, fly_height],
        [2, 0, fly_height],
        [1.75, 1, fly_height],
    ]
)

bf_position3 = np.array(
    [
        [-2, -2, fly_height],
        [-1.5, -1, fly_height-1],
        [-2, 0, fly_height],
        [-1.5, 1, fly_height-1],
        [2, -2, fly_height],
        [1.5, -1, fly_height-1],
        [2, 0, fly_height],
        [1.5, 1, fly_height-1],
    ]
)

drone_step_sequence = np.tile(start_positions[:, :, np.newaxis], (1, 1, time_steps))


# These two functions each initialize each half of the strut matrix
for drone_index in range(0, 4, 1):
    for point_index in range(3):
        if point_index == 0 or point_index == 1:
            target_point = left_side_strut_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[7 - drone_index]
        strut_matrix[drone_index, :, point_index] = target_point

for drone_index in range(4, 8, 1):
    for point_index in range(3):
        if point_index == 0:
            target_point = right_side_strut_points[point_index]
        elif point_index == 1:
            target_point = right_side_strut_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[7 - drone_index]
        strut_matrix[drone_index, :, point_index] = target_point


# These two functions initialize each side of the intro_matrix

for drone_index in range(0, 4, 1):
    for point_index in range(3):
        if point_index == 0 or point_index == 1:
            target_point = left_side_intro_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
            
        intro_matrix[drone_index, :, point_index] = target_point


for drone_index in range(4, 8, 1):
    for point_index in range(3):
        if point_index == 0 or point_index == 1:
            target_point = right_side_intro_points[point_index]
        elif point_index == 2:
            target_point = default_destinations[drone_index]
        intro_matrix[drone_index, :, point_index] = target_point




def round_to_nearest_time_step(value):
    return np.round(value / delta_t) * delta_t


# This will copy the drone locations at the end_time to the remainder of the tensor
def hover_in_place(end_time):
    # hover_locations will be an 8 x 3 shaped array
    end_timestep = int(end_time / delta_t)
    if(end_timestep == int(total_time / delta_t)):
        end_timestep -= 1

    hover_locations = np.asarray(
        [drone_step_sequence[i, :, end_timestep] for i in range(drone_count)]
    )  # 8 x 3 array
    hover_locations = np.expand_dims(hover_locations, axis=2)
    start_hover_timestep = int(end_time / delta_t)
    stop_hover_timestep = int(total_time / delta_t)
    num_steps_hovering = stop_hover_timestep - start_hover_timestep

    # We need to tile this hover location onto
    drone_step_sequence[
        :, :, start_hover_timestep : stop_hover_timestep
    ] = np.reshape(np.tile(
        hover_locations, (1, 1, num_steps_hovering)), (drone_count, dimensions, num_steps_hovering))

# Draws a straight line on the drone_step_sequence tensor for a drone
def straight_line(start_time, end_time, start_location, end_location, drone):
    start_timestep = int(start_time / delta_t)
    end_timestep = int(end_time / delta_t)
    trajectory_length = end_timestep - start_timestep
    if(trajectory_length == 0):
        return 0
    vel = (end_location - start_location) / (end_time - start_time)
    if vel[0] != 0 and not np.isnan(vel[0]):
        x_path = np.arange(start_location[0], end_location[0], vel[0] * delta_t)[
            0:trajectory_length
        ]
    else:
        x_path = np.full(trajectory_length, start_location[0])

    if vel[1] != 0 and not np.isnan(vel[1]):
        y_path = np.arange(start_location[1], end_location[1], vel[1] * delta_t)[
            0:trajectory_length
        ]
    else:
        y_path = np.full(trajectory_length, start_location[1])

    if vel[2] != 0 and not np.isnan(vel[2]):
        z_path = np.arange(start_location[2], end_location[2], vel[2] * delta_t)[
            0:trajectory_length
        ]
    else:
        z_path = np.full(trajectory_length, start_location[2])
    drone_step_sequence[
        drone, 0, start_timestep : end_timestep
    ] = x_path
    drone_step_sequence[
        drone, 1, start_timestep : end_timestep
    ] = y_path
    drone_step_sequence[
        drone, 2, start_timestep : end_timestep
    ] = z_path


# Input:
# target_locations - 8 x 3 np.array - where are our drones going?
# Return:
# end_time - float - returns the time ins second in which this timestep is done
def my_goto(target_locations, start_time=None, current_locations=None):
    global _time_step
    if(start_time==None):
        start_time=_time_step
    if current_locations is None:
        current_locations = np.asarray(
            [
                drone_step_sequence[i, :, (int)(start_time / delta_t)]
                for i in range(drone_count)
            ]
        )  # 8 x 3 array

    max_duration = np.max(
        np.linalg.norm(target_locations - current_locations) / max_vel
    )


    end_time = start_time + max_duration+1


    for i in range(drone_count):
        straight_line(
            start_time, end_time, current_locations[i], target_locations[i], i
        )
    return end_time

def strut_index_helper():
    global drone_step_sequence
    new_indices = [7, 6, 5, 4, 3, 2, 1, 0]
    drone_step_sequence = drone_step_sequence[new_indices]
        



        

# STRUT WILL FIRST SEND ALL DRONES TO THEIR DEFAULT POSITIONS, AND THEN INDEXING FROM strut_matrix,
# WILL RUN TWO LINES OF DRONES ACROSS FROM EACH OTHER
def strut():
    # Go to default locations
    # current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
    strut_begin_time = my_goto(default_destinations)
    for drone_index in range(drone_count):
        time_iterator = strut_begin_time
        for point in range(3):
            
            target_location = strut_matrix[drone_index, :, point]
            cur_pos = None
            if point == 0:
                cur_pos = default_destinations[drone_index]
            else:
                cur_pos = strut_matrix[drone_index, :, point - 1]

            if drone_index > 3:
                alpha = 0.13
            else:
                alpha = 0

            end_time = (
                round_to_nearest_time_step(
                    np.linalg.norm(target_location - cur_pos) / (max_vel + alpha)
                ) + time_iterator
            )

            straight_line(
                time_iterator, end_time, cur_pos, target_location, drone_index
            )
            time_iterator = end_time
        

    hover_in_place(time_iterator - delta_t)
    strut_index_helper()


# TODO STRETCH GOAL: IMPLEMENT A FOLLOWER_MODE WITH A MOCAP SUBSRIBER
def follower_mode(mocap_coordinates):
    pass


def circles():
    global drone_step_sequence
    k = 1
    layer_1_height = 1
    layer_2_height = 2.2
    
    circling_start_time = my_goto(default_destinations)
    phi_layer_1 = np.arange(0, 2 * 3.14159, 2 * 3.14159 / 4)
    phi_layer_2 = np.arange(3.14159 / 4, 2 * 3.14159 + 3.14159 / 4, 2 * 3.14159 / 4)

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
    )
    layer1_circling_paths = np.swapaxes(layer1_circling_paths, 1, 2)


    layer2_circling_paths = np.stack(
        (layer2_cos_values, layer2_sin_values, layer2_height_array), axis=-1
    )
    layer2_circling_paths = np.swapaxes(layer2_circling_paths, 1, 2)
    assert(layer1_circling_paths.shape[0] == 4)
    assert(layer2_circling_paths.shape[0] == 4)
    assert(layer1_circling_paths.shape[1] == 3)
    assert(layer2_circling_paths.shape[1] == 3)


    import scipy
    import scipy.spatial

    drone_step_sequence[
        0:4, :, (int)(circling_start_time / delta_t) : int(total_time / delta_t)
    ] = layer1_circling_paths
    drone_step_sequence[
        4:8, :, (int)(circling_start_time / delta_t) : int(total_time / delta_t)
    ] = layer2_circling_paths

    starting_values = drone_step_sequence[0:8, :, int(circling_start_time / delta_t)]
    
    dists_start = scipy.spatial.distance_matrix(
        starting_values,
        np.concatenate((layer1_circling_paths[:, :, 0], layer2_circling_paths[:, :, 0]), axis=0),
    )
    assignments = linear_sum_assignment(dists_start)[0]

    start_goto_timestep = time_steps - num_steps_left
    start_goto = start_goto_timestep * delta_t
    assignments = [2, 0, 5, 7, 1, 4, 6, 3]

    drone_step_sequence = drone_step_sequence[assignments]


    beginning_positions = np.asarray([
            layer1_circling_paths[0, :, 0],
            layer1_circling_paths[1, :, 0],
            layer1_circling_paths[2, :, 0],
            layer1_circling_paths[3, :, 0],
            layer2_circling_paths[0, :, 0],
            layer2_circling_paths[1, :, 0],
            layer2_circling_paths[2, :, 0],
            layer2_circling_paths[3, :, 0]
    ])
    start_circling_time = my_goto(beginning_positions, start_time=start_goto - delta_t)
    
    start_circles_timestep = int(start_circling_time / delta_t)
    length = drone_step_sequence[i, :, start_circles_timestep:].shape[1]

  


    for j in range(drone_count):
        if(j < 4):
            drone_step_sequence[j, :, start_circles_timestep:] = layer1_circling_paths[j][:, 0:length]
        else:
            drone_step_sequence[j, :, start_circles_timestep:] = layer2_circling_paths[j - 4][:, 0:length]


def train():
    pass


def intro_march():
    march_begin_time = my_goto(start_positions)
    time_iterator = None
    for drone_index in range(drone_count):
        time_iterator = march_begin_time
        for point in range(dimensions):

            target_location = intro_matrix[drone_index, :, point]

            if point == 0:
                cur_pos = start_positions[drone_index]
            else:
                cur_pos = intro_matrix[drone_index, :, point - 1]
            end_time = (
                round_to_nearest_time_step(
                    np.linalg.norm(target_location - cur_pos) / max_vel
                ) + time_iterator
            )
            straight_line(
                time_iterator, end_time, cur_pos, target_location, drone_index
            )
            time_iterator = end_time
    hover_in_place(time_iterator - delta_t)


def disperse_to_default():
    global default_destinations
    global delta_t
    hover_in_place(my_goto(default_destinations) - delta_t)

def beginning_falls():
    fall_1_time = my_goto(start_positions)
    fall_2_time = my_goto(start_positions_fall1, fall_1_time-delta_t)
    fall_3_time = my_goto(start_positions, fall_2_time-delta_t)
    fall_4_time = my_goto(start_positions_fall2, fall_3_time-delta_t)
    fall_5_time = my_goto(start_positions, fall_4_time-delta_t)
    fall_6_time = my_goto(start_positions_fall3, fall_5_time-delta_t)
    fall_7_time = my_goto(start_positions, fall_6_time-delta_t)
    fall_8_time = my_goto(start_positions_fall4, fall_7_time-delta_t)
    fall_9_time = my_goto(start_positions, fall_8_time-delta_t)
   
    hover_in_place(fall_9_time-delta_t)

def back_and_forth():
    bf_1_time = my_goto(default_destinations)
    bf_2_time = my_goto(bf_position1, bf_1_time-delta_t)
    bf_3_time = my_goto(bf_position2, bf_2_time-delta_t)
    bf_4_time = my_goto(bf_position1, bf_3_time-delta_t)
    bf_5_time = my_goto(bf_position2, bf_4_time-delta_t)
    bf_6_time = my_goto(bf_position3, bf_5_time-delta_t)
    bf_7_time = my_goto(default_destinations, bf_6_time-delta_t)
 
    hover_in_place(bf_7_time-delta_t)

def meet_in_the_middle():
    middle1_time = my_goto(default_destinations)
    middle2_time = my_goto(meet_in_the_middle_destinations, middle1_time-delta_t)
    middle3_time = my_goto(default_destinations, middle2_time-delta_t)
    print(middle3_time)
    hover_in_place(middle3_time-delta_t)
    

def v_formation():
    # Go to default locations
    # current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
    v_begin_time = my_goto(default_destinations)

    v_first_point_time = my_goto(v_mygoto, v_begin_time-delta_t)
    v_second_point_time = my_goto(v_mygoto2, v_first_point_time-delta_t)
    v_third_point_time = my_goto(v_mygoto3, v_second_point_time-delta_t)
    v_fourth_point_time = my_goto(v_mygoto4, v_third_point_time-delta_t)
    v_fifth_point_time = my_goto(v_mygoto5, v_fourth_point_time-delta_t)
    v_sixth_point_time = my_goto(v_mygoto6, v_fifth_point_time-delta_t)
    print(v_sixth_point_time)
    hover_in_place(v_sixth_point_time-delta_t)


    # Make a subscriber to some topic in the launch.py
def go_to_floor():
    start_time = (x2)
    floor_time = my_goto(x3_land, start_time-delta_t)

def step_foward():
    step_begin_time = my_goto(default_destinations)
    step_time2 = my_goto(x1, step_begin_time-delta_t)
    step_time3 = my_goto(x2, step_time2-delta_t)
    hover_in_place(step_time3-delta_t)
# def v_formation():
#     # Go to default locations
#     # current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
#     v_begin_time = my_goto(default_destinations)

#     v_first_point_time = my_goto(v_mygoto, v_begin_time-delta_t)
#     v_second_point_time = my_goto(v_mygoto2, v_first_point_time-delta_t)
#     hover_in_place(v_second_point_time-delta_t)


#     # Make a subscriber to some topic in the launch.py

def update_drone_step_sequence(button_index):
    if button_index == 0:  # Follower
        print("Follower Mode: Green button")
        follower_mode("Placeholder")
    elif button_index == 1:  # Circles
        print("back and forth")
        back_and_forth()
    elif button_index == 2:  # Circles
        print("Circles! Blue button")
        circles()
    elif button_index == 3:  # Train
        print("Train! Yellow Button")
        train()
    elif button_index == 4:  # Train
        print("fall in the beginning")
        beginning_falls()
    elif button_index == 5:  # Train
        print("meet in the middle")
        meet_in_the_middle()
    elif button_index == 7:  # Intro March
        print("Intro March! Start button")
        intro_march()
    elif button_index == 6:  # Disperse
        print("Disperse! Back Button")
        disperse_to_default()
    elif button_index == 8:  # V-Formation
        print("V-form! Circle Button")
        v_formation()
    elif button_index == 9:  # V-Formation
        print("Step foward")
        step_foward()
    elif button_index == 10:  # Strut - right joy in
        print("Strut! right joystick in")
        strut()
# def update_drone_step_sequence(button_index):
#     if button_index == 0:  # Follower
#         print("Follower Mode: Green button")
#         follower_mode("Placeholder")
#     elif button_index == 2:  # Circles
#         print("Circles! Blue button")
#         circles()
#     elif button_index == 3:  # Train
#         print("Train! Yellow Button")
#         train()
#     elif button_index == 7:  # Intro March
#         print("Intro March! Start button")
#         intro_march()
#     elif button_index == 6:  # Disperse
#         print("Disperse! Back Button")
#         disperse_to_default()
#     elif button_index == 8:  # V-Formation
#         print("V-form! Circle Button")
#         v_formation()
#     elif button_index == 10:  # Strut - right joy in
#         print("Strut! right joystick in")
#         strut()

def initialize():
    drone_step_sequence[:, :, 0] = start_positions




def main():
    from crazyflie_py import Crazyswarm

    swarm = Crazyswarm(log_poses=False)
    allcfs = (
        swarm.allcfs
    )  # This is in order of x position(low x value means beginning of the list)
    drones = allcfs.crazyflies
    timeHelper = swarm.timeHelper

    # Takeoff
    global _time_step
    allcfs.takeoff(2.1, 3)
    timeHelper.sleep(3.0)
    for time_step in np.arange(start=0, stop=total_time, step=delta_t):
        if time_step == 0:
            update_drone_step_sequence(4) #falling sequence
            print(time_step)
        elif time_step == 15 :
            update_drone_step_sequence(7) #Intro march
            print(time_step)
        elif time_step == 23: 
            update_drone_step_sequence(1) # back and forth
        elif time_step == 40: 
            update_drone_step_sequence(10)  # Strut Formation
        #     print(time_step)
        # elif time_step == 48:
        #     update_drone_step_sequence(5) #meet in the middle
        elif time_step == 52:
             update_drone_step_sequence(6) # Disperse
        elif time_step == 54:
            update_drone_step_sequence(2) #Circles
        elif time_step == 76:
            update_drone_step_sequence(8) #V-formation
        elif time_step == 122: 
            update_drone_step_sequence(9) #step_foward

        # if(time_step == 12): # When the bass comes in intro_march, second 0:15 cumulative
        #     update_drone_step_sequence(7)
        # if(time_step == 37): # Strut Move, Second 0:40 cumulatively
        #     update_drone_step_sequence(10)
        # if(time_step == 50): # Circles, Second 0:40 cumulatively
        #         update_drone_step_sequence(2) 
        # if(time_step == 79): #  V-formation, Second 0:40 cumulatively
        #     update_drone_step_sequence(6)
        # if(time_step == 102): # Disperse, Second 0:40 cumulatively
        #     update_drone_step_sequence(8) #

        ##new
        


        # USE THIS CODE IF YOU WANT TO CONTROL MOVES WITH THE CONTROLLER
        # print(time_step)
        # _time_step = time_step
        # button_press = swarm.input.checkIfAnyButtonIsPressed()
        # button_index = np.argmax(button_press)
        # if np.count_nonzero(button_press) == 0:
        #     button_index = -1
        # update_drone_step_sequence(button_index)
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
