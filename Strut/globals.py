import numpy as np

takeoff_height = 2
velocity = 0.7


total_time = 90  # Time in seconds
delta_t = 0.05
time_steps = (int)(total_time / delta_t)
drone_count = 8
dimensions = 3
drone_step_sequence = np.zeros((drone_count, dimensions, time_steps))
drones = []  # ADD DRONES TO THIS LIST
button_pressed = np.zeros(11)  # True when a button has been pressed
button = "X"  # Updated with the most recently pressed button
max_vel = 0.8  #
time_step = 0


strut_matrix = np.zeros((drone_count, dimensions, 3))
v_matrix = np.zeros((drone_count, dimensions, 2))
intro_matrix = strut_matrix = np.zeros((drone_count, dimensions, 3))

left_side_strut_points = [[-2, 2, 2.1], [2, 2, 2.1]]
right_side_strut_points = [[2, 2, 2.1], [-2, 2, 2.1]]

left_side_intro_points = [[-2.75, 2, 2.1], [2.75, 2, 2.1]]
right_side_intro_points = [[2.75, 2, 2.1], [-2.75, 2, 2.1]]

right_side_v_1_points = [[.5, 1, 2.1], [.5, -.5, 1]]
right_side_v_2_points = [[1, 0, 2.1], [1, -1, 1.33]]
right_side_v_3_points = [[1.5, -1, 2.1], [1.5, -1.5, 1.66]]
right_side_v_4_points = [[2, -2, 2.1], [2, -2, 1.99]]

left_side_v_5_points = [[-.5, 1, 2.1], [-.5, -.5, 1]]
left_side_v_6_points = [[-1, 0, 2.1], [-1, -1, 1.33]]
left_side_v_7_points = [[-1.5, -1, 2.1], [-1.5, -1.5, 1.66]]
left_side_v_8_points = [[-2, -2, 2.1], [-2, -2, 1.99]]

default_destinations = [
    [2, 1, 2.1],
    [2, 0, 2.1],
    [2, -1, 2.1],
    [2, -2, 2.1],
    [-2, 1, 2.1],
    [-2, 0, 2.1],
    [-2, -1, 2.1],
    [-2, -2, 2.1],
]


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

# This function initializes the v matrix
for drone_index in range(0, 8, 1):
    for point_index in range(2):
        if drone_index == 0:
            target_point = right_side_v_1_points[point_index]
        if drone_index == 1:
            target_point = right_side_v_2_points[point_index]
        if drone_index == 2:
            target_point = right_side_v_3_points[point_index]
        if drone_index == 3:
            target_point = right_side_v_4_points[point_index]
        if drone_index == 4:
            target_point = left_side_v_5_points[point_index]
        if drone_index == 5:
            target_point = left_side_v_6_points[point_index]
        if drone_index == 6:
            target_point = left_side_v_7_points[point_index]
        if drone_index == 7:
            target_point = left_side_v_8_points[point_index]
        v_matrix[drone_index, :, point_index] = target_point


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
