import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *
from types import SimpleNamespace
from TimeHelper import TimeHelper # TODO add to files downloaded
import globals
import scipi.spatial
from scipi.optimize import linear_sum_assignments

Hz = 30
drone_id = 1


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

#TODO SET THESE GLOBALS
default_positions = ... #NP ARRAY OF SHAPE (8, 3)
circle_start_pos = default_positions #NP ARRAY OF SHAPE (8, 3), ALSO NEEDS TO BE UPDATED
strut_matrix = globals.strut_matrix #NP ARRAY OF SHAPE (drone_count, dimensions, 3)
intro_matrix = globals.intro_matrix
takeoff_height = 2.1 #Max height of 2.25

def my_takeoff():
    for i in range(drone_count):
        takeoff(groupState=i, height=takeoff_height, duration=2)
def my_land():
    for i in range(drone_count):
        land(groupState=i, height=takeoff_height, duration=2)


def round_to_nearest_time_step(value):
    return np.round(value / delta_t) * delta_t


#This will copy the drone locations at the end_time to the remainder of the tensor 
def hover_in_place(end_time):
    #hover_locations will be an 8 x 3 shaped array
    hover_locations = np.asarray([drone_step_sequence[i,:,end_time] for i in range(drone_count)]) # 8 x 3 array
    #We need to tile this hover location onto 
    drone_step_sequence[:,:,(int)(end_time / delta_t):(int)(total_time / delta_t)] = np.tile(hover_locations, (1,1, (int)(end_time / delta_t) - (int)(total_time / delta_t)))



#Draws a straight line on the drone_step_sequence tensor for a drone
def straight_line(start_time, end_time, start_location, end_location, drone):
    vel = (end_location - start_location) / (end_time - start_time)
    x_path = np.arrange(start_location[0], end_location[0], vel[0])
    y_path = np.arrange(start_location[1], end_location[1], vel[1])
    z_path = np.arrange(start_location[2], end_location[2], vel[2])
    drone_step_sequence[drone, 0, start_time / delta_t:end_time / delta_t] = x_path
    drone_step_sequence[drone, 1, start_time / delta_t:end_time / delta_t] = y_path
    drone_step_sequence[drone, 2, start_time / delta_t:end_time / delta_t] = z_path

#Input:
#target_locations - 8 x 3 np.array - where are our drones going?
#Return:
#end_time - float - returns the time ins second in which this timestep is done
def my_goto(target_locations):
    current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)]) # 8 x 3 array
    np.reshape((drone_count, dimensions))
    max_duration = np.max(np.linalg.norm(target_locations - current_locations) / max_vel)
    end_time = time_step + max_duration
    for i in range(drone_count):
        straight_line(time_step, end_time, current_locations[i], target_locations[i])
    return end_time

#STRUT WILL FIRST SEND ALL DRONES TO THEIR DEFAULT POSITIONS, AND THEN INDEXING FROM strut_matrix,
#WILL RUN TWO LINES OF DRONES ACROSS FROM EACH OTHER
def strut():
    #Go to default locations
    #current_locations = np.asarray([drone_step_sequence[i,:,time_step] for i in range(drone_count)])
    strut_begin_time = my_goto(default_positions)
    for drone_index in range(drone_count):
        time_iterator = strut_begin_time
        for point in range(dimensions):
            target_location = strut_matrix[drone_index, :, point]
            cur_pos = None
            if point == 0:
                cur_pos = default_positions[drone_index]
            else:
                cur_pos = strut_matrix[drone_index, :, point - 1]
            end_time = round_to_nearest_time_step(np.linalg.norm(target_location - cur_pos) / max_vel) + time_step
            straight_line(time_iterator, end_time, cur_pos, target_location, drone_index)
            time_iterator = end_time
    hover_in_place(time_iterator)




#TODO STRETCH GOAL: IMPLEMENT A FOLLOWER_MODE WITH A MOCAP SUBSRIBER
def follower_mode(mocap_coordinates):
    pass
def circles():

    k = 1
    layer_1_height = 1
    layer_2_height = 2.5

    circling_start_time = my_goto(default_positions)

    phi_layer_1 = np.arange(start = 0, stop = 2*3.14159, step = 2*3.14159/4)
    phi_layer_2 = np.arange(start = 3.14159/2, stop = 2*3.14159 + 3.14159/2, step = 2*3.14159/4)

    assert(len(phi_layer_1) == 4)
    assert(len(phi_layer_2) == 4)

    num_steps_left = (int)((total_time - circling_start_time) / delta_t)
    t = np.arange(start=circling_start_time, stop=total_time, step=delta_t) #
    t = t[:, np.newaxis] 

    layer1_cos_values = np.cos(k * t + phi_layer_1[:, np.newaxis])
    layer1_sin_values = np.sin(k * t + phi_layer_1[:, np.newaxis])
    layer2_cos_values = np.cos(k * t + phi_layer_2[:, np.newaxis])
    layer2_sin_values = np.sin(k * t + phi_layer_2[:, np.newaxis])

    layer1_height_array = np.full((4, num_steps_left), layer_1_height)
    layer2_height_array = np.full((4, num_steps_left), layer_2_height)

    layer1_circling_paths = np.stack((layer1_cos_values, layer1_sin_values, layer1_height_array), axis=-1)
    layer2_circling_paths = np.stack((layer2_cos_values, layer2_sin_values, layer2_height_array), axis=-1)

    
    drone_step_sequence[0:4, :, (int)(circling_start_time / delta_t):(int)(total_time / delta_t)] = layer1_circling_paths
    drone_step_sequence[4:8, :, (int)(circling_start_time / delta_t):(int)(total_time / delta_t)] = layer2_circling_paths
    
    starting_values = drone_step_sequence[0:8, :, (int)(circling_start_time / delta_t)]
    
    dists_start = scipy.spatial.distance_matrix(starting_values, layer1_circling_paths)
    assignments = scipy.optimize.linear_sum_assignment(dists_start)[1]
    layer1_circling_paths = layer1_circling_paths[assignments]
    layer2_circling_paths = layer2_circling_paths[assignments]
    
def train():
    pass
def intro_march():
    strut_begin_time = my_goto(default_positions)
    time_iterator = None
    for drone_index in range(drone_count):
        time_iterator = strut_begin_time
        for point in range(dimensions):
            target_location = intro_matrix[drone_index, :, point]
            cur_pos = None
            if point == 0:
                cur_pos = default_positions[drone_index]
            else:
                cur_pos = strut_matrix[drone_index, :, point - 1]
            end_time = round_to_nearest_time_step(np.linalg.norm(target_location - cur_pos) / max_vel) + time_step
            straight_line(time_iterator, end_time, cur_pos, target_location, drone_index)
            time_iterator = end_time
    hover_in_place(time_iterator)

def disperse_to_default():
    for drone_index in range(drone_count):
        current_positions = drone_step_sequence[drone_index,:,(int)(time_step / delta_t)]
        end_time = round_to_nearest_time_step(np.linalg.norm(default_positions - current_positions) / max_vel) + time_step
        straight_line(time_step, end_time, current_positions, default_positions, drone_index)
        hover_in_place(end_time)
def v_formation():
    pass


    #Make a subscriber to some topic in the launch.py
def update_drone_step_sequence(button_pressed):
    button_index = np.argmax(button_pressed)
    if button_index == 0: #Follower
        print("Follower Mode: Green button")
        follower_mode("Placeholder")
    elif button_index == 2 : #Circles
        print("Circles! Blue button")
        circles()
    elif button_index == 3: #Train
        print("Train! Yellow Button")
        train()
    elif button_index == 7: #Intro March
        print("Intro March! Start button")
        intro_march()
    elif button_index == 6: #Disperse
        print("Disperse! Back Button")
        disperse_to_default()
    elif button_index == 8: #V-Formation
        print("V-form! Circle Button")
        v_formation()
    elif button_index == 10: #Strut - right joy in
        print("Strut! right joystick in")
        strut()

    


def main():
    from crazyflie_py import Crazyswarm
    swarm = Crazyswarm()
    allcfs = swarm.allcfs # This is in order of x position(low x value means beginning of the list)
    drones = allcfs.crazyflies

    # Takeoff
    my_takeoff()

    for time_step in range(start=0, stop=total_time, step=delta_t):
        button_press = swarm.input.checkifAnyButtonIsPressed()
        no_button_pressed = np.all(button_press == 0)
        button_pressed = not no_button_pressed
        if button_pressed:
            update_drone_step_sequence(button)
        for i, drone in enumerate(drones):
            coords = drone_step_sequence[i, :, int(time_step / delta_t)]
            drone.cmdPosition(coords) #instead of goto
        timeHelper.sleepForRate(1 / delta_t)
    my_land()
if __name__== "__main__":
    main()



#TODO:
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






