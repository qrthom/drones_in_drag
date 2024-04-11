import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *
from types import SimpleNamespace
from TimeHelper import TimeHelper # TODO add to files downloaded
Hz = 30

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        assert(isinstance(id, int))
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.timer = self.create_timer(1/Hz, self.timer_callback)
        self.timeHelper = TimeHelper(self)
        self.running = False
        self.done = False
    
    def compute_trajectories(self):
        """
        Inject Trajectory computation code here...
        """
        trajectories = []
        ### -----Insert Trajectories Here-------

        return trajectories

    def upload_trajectories(crazyflies, trajectories):
        '''
            Upload trajectories to crazyflies one by one
        '''

        for i, traj in enumerate(trajectories):
            for cf in crazyflies:
                cf.uploadTrajectory(traj, i, 0)

    def start(self):
        """
            Start execution of blocks
        """
        trajectories = self.compute_trajectories()
        self.upload_trajectories(trajectories)
        self.execute_blocks()

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def execute_blocks(self):
        """
        Must be injected into...

        Typical format should be:
        
        start_time = 0.0
        self.timeHelper.sleepUntil(start_time)
        takeoff(crazyflies, height=1.0, duration=2.0)

        start_time = 5.0
        self.wait_until(start_time)
        land(crazyflies, height=0.0, duration=2.0)
        
        ...

        Where a new start time is added for each block.
        """
        groupState = SimpleNamespace(crazyflies=self.crazyflies, timeHelper=self.timeHelper)
        ### ---------Insert Execution Code Here------------
        # Block Name: CF8 Movement
        start_time = 0.07999999999999996
        self.timeHelper.sleepUntil(start_time)
        takeoff(groupState, 1, 3)
        pos_array = globals.drone_1_pos
        goto_velocity(groupState, pos_array[0][0], pos_array[0][1], pos_array[0][2], pos_array[0][3], 1)
        goto_velocity(groupState, pos_array[1][0], pos_array[1][1], pos_array[1][2], pos_array[1][3], 1)
        goto_velocity(groupState, pos_array[2][0], pos_array[2][1], pos_array[2][2], pos_array[2][3], 1)
        goto_velocity(groupState, pos_array[3][0], pos_array[3][1], pos_array[3][2], pos_array[3][3], 1)
        land(groupState)

        
        self.done = True
    
    def timer_callback(self):
        if not self.running:
            self.start()
            self.running = True