from launch import LaunchDescription
from launch_ros.actions import Node


all_robots_ID = [1, 2, 3, 4]
# TODO: match this with yaml

IS_DISTRIBUTED_COMPUTATION = True

def generate_launch_description():
    ld = LaunchDescription()

    MRS_PKG = 'ccmrs_utu_course'
    MRS_NAMESPACE = 'mrs'

    # Simulator Node
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
             executable='ROS2_sim', name='sim' )
    )
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
             executable='ROS2_sensors', name='sens' )
    )
    
    # Visualizer
    ld.add_action(
        Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
               executable='ROS2_visualizer', name='viz')
    )

    if not IS_DISTRIBUTED_COMPUTATION:
        # Centralized Controller
        ld.add_action(
            Node(package=MRS_PKG, namespace=MRS_NAMESPACE,
                executable='ROS2_controller', name='controller')
        )

    else:
        # Distributed Controller
        for i in all_robots_ID:
            ld.add_action(
                Node(package=MRS_PKG,namespace=MRS_NAMESPACE,
                    executable='ROS2_dist_controller',
                    name='controller_' + str(i),
                    parameters=[ {'robot_ID': i} ]
                )
            )

    return ld