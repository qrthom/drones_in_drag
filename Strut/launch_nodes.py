import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
import rclpy
import threading

# Inject Imports Here:

def launch(nodes):
    executor = rclpy.executors.MultiThreadedExecutor()

    for node in nodes:
        executor.add_node(node)
    
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    thread.join()
    

def main():
    # node_config_file = get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml'
    
    # Initialize swarm
    swarm = Crazyswarm()
    with open("cfs_ordering.yaml") as f:
        ordering = yaml.safe_load(f) #TODO!!!
        order = ordering['cfs']
    
    crazyflies = [swarm.allcfs.crazyfliesById[int(k)] for k in order]
    counter = 0
    nodes = []
    cfs = []

    #   -----------Insert Nodes Here----------- 
    cfs = []
    cfs.append(crazyflies[0])
    nodes.append(group1_node.worker_node(cfs, len(nodes), 2))


    cfs.append(crazyflies[1])
    nodes.append(group2_node.worker_node(cfs, len(nodes), 3))


    cfs = []
    cfs.append(crazyflies[2])
    nodes.append(group3_node.worker_node(cfs, len(nodes), 4))


    cfs = []
    cfs.append(crazyflies[3])
    nodes.append(group4_node.worker_node(cfs, len(nodes), 5))

    cfs = []
    cfs.append(crazyflies[4])
    nodes.append(group5_node.worker_node(cfs, len(nodes), 6))


    cfs = []
    cfs.append(crazyflies[5])
    nodes.append(group6_node.worker_node(cfs, len(nodes), 7))


    cfs = []
    cfs.append(crazyflies[6])
    nodes.append(group7_node.worker_node(cfs, len(nodes), 8))


    import group8_node
    cfs = []
    cfs.append(crazyflies[7])
    nodes.append(group8_node.worker_node(cfs, len(nodes), 9))



    
    # Launch all nodes
    return launch(nodes)

if __name__ == '__main__':
    main()
