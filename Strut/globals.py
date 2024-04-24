import numpy as np
takeoff_height = 2
velocity = .7


total_time = 90 # Time in seconds
delta_t = .05
time_steps = total_time/delta_t
drone_count = 8
dimensions = 3
drone_step_sequence = np.zeros((drone_count, dimensions, time_steps))
drones = [] #ADD DRONES TO THIS LIST
button_pressed = np.zeros(11) # True when a button has been pressed
button = 'X' # Updated with the most recently pressed button
timeHelper = TimeHelper() # IS THIS RIGHT?
max_vel = .8 #
time_step = 0


strut_matrix = np.zeros(drone_count, dimensions, 3)
intro_matrix = strut_matrix = np.zeros(drone_count, dimensions, 3)

left_side_strut_points = [[-2, 2, 2.1], [2, 2, 2.1]]
right_side_strut_points = [[2, 2, 2.1], [-2, 2, 2.1]]

left_side_intro_points = [[-2.75, 2, 2.1], [2.75, 2, 2.1]]
right_side_intro_points = [[2.75, 2, 2.1], [-2.75, 2, 2.1]]


default_destinations = [[2, 1, 2.1], [2, 0, 2.1], [2, -1, 2.1], [2, -2, 2.1], [-2, 1, 2.1], [-2, 0, 2.1], [-2, -1, 2.1], [-2, -2, 2.1] ]


#These two functions each initialize each half of the strut matrix
for drone_index in range(start=0, stop=4, step=1):
    for point_index in range(3):
        if(point_index == 0):
            target_point = left_side_strut_points[point_index]
        elif(point_index == 1):
            target_point = left_side_strut_points[point_index]
        elif(point_index == 2):
            target_point = default_destinations[drone_index]
        strut_matrix[drone_index,:,point_index] = target_point

for drone_index in range(start=4, stop=8, step=1):
    for point_index in range(3):
        if(point_index == 0):
            target_point = right_side_strut_points[point_index]
        elif(point_index == 1):
            target_point = right_side_strut_points[point_index]
        elif(point_index == 2):
            target_point = default_destinations[drone_index]
        strut_matrix[drone_index,:,point_index] = target_point


#These two functions initialize each side of the intro_matrix

for drone_index in range(start=0, stop=4, step=1):
    for point_index in range(3):
        if(point_index == 0):
            target_point = left_side_intro_points[point_index]
        elif(point_index == 1):
            target_point = right_side_intro_points[point_index]
        elif(point_index == 2):
            target_point = default_destinations[drone_index]
        intro_matrix[drone_index,:,point_index] = target_point

for drone_index in range(start=4, stop=8, step=1):
    for point_index in range(3):
        if(point_index == 0):
            target_point = left_side_intro_points[point_index]
        elif(point_index == 1):
            target_point = right_side_intro_points[point_index]
        elif(point_index == 2):
            target_point = default_destinations[drone_index]
        intro_matrix[drone_index,:,point_index] = target_point
