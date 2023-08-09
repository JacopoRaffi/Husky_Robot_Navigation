#!/usr/bin/env python
"""
This file is a state-machine prototype for the husky labyrinth.
It shows how to control the display of a series of screens using SetMaterialColorServiceState.
The screen will change color from blue to red in sequence so to guide the Braitenberg vehicle
out of the maze.

"""
import rospy
import smach
import smach_ros
from smach import StateMachine, CBState
from rosgraph_msgs.msg import Clock
from sm.smach_states import (RobotPoseMonitorState, SetMaterialColorServiceState)


FINISHED = 'FINISHED'
ERROR = 'ERROR'
PREEMPTED = 'PREEMPTED'

ROBOT_ID = 'husky_model'

hotspots = [lambda ud, p: not ((6.2 < p.position.x < 8.0) and (-4.0 < p.position.y < -1.3)),
            lambda ud, p: not ((5.8 < p.position.x < 8.0) and (4.0 < p.position.y < 6.0)),
            lambda ud, p: not ((-0.29 < p.position.x < 0.36879) and (3.15 < p.position.y < 6.0)),
            lambda ud, p: not ((-2 < p.position.x < 1) and (-0.44192 < p.position.y < 0.79615)),
            lambda ud, p: not ((-6.69517 < p.position.x < -5.09197) and (
                    -1.25467 < p.position.y < 0.67082)),
            lambda ud, p: not ((-7.05 < p.position.x < -4.26) and (
                    -7 < p.position.y < -5.5)),
            lambda ud, p: not (
                    (-1 < p.position.x < 1) and (-7 < p.position.y < -5))]

goal_hostspot = lambda ud, p: not ( (-1.2 < p.position.x < 1.2) and (-12 < p.position.y < -11.5) )


# Colors
RED = "Gazebo/RedGlow"
BLUE = "Gazebo/BlueGlow"


if __name__ == "__main__":
    rospy.init_node("screen_switcher_sm")

    sm = StateMachine(outcomes=[FINISHED, ERROR, PREEMPTED])

    with sm:

        link_name = "vr_screen::body"
        visual_name = "screen_glass"


        # Create State machine states
        for i, hotspot in enumerate(hotspots):

            rospy.loginfo(f"Adding spotlight_{i}")


            # Change the i-th screen color to RED
            StateMachine.add(
                f"set_screen_red_{i}",
                SetMaterialColorServiceState(f"virtual_screen_{i}",
                                             link_name,
                                             visual_name,
                                             RED),
                transitions={"succeeded": f"wait_for_husky_hotspot_{i}",
                             "aborted": FINISHED,
                             "preempted": PREEMPTED}
            )

            # Check the robot position in the world: i.e. check that the robot has entered a hotspot
            StateMachine.add(
                f"wait_for_husky_hotspot_{i}",
                RobotPoseMonitorState(ROBOT_ID, hotspot),
                transitions={'valid': f'wait_for_husky_hotspot_{i}',
                             'invalid': f'set_screen_blue_{i}',
                             'preempted': PREEMPTED}
            )

            if i >= len(hotspots) - 1:

                # Change the i-th screen color to BLUE
                StateMachine.add(
                    f"set_screen_blue_{i}",
                    SetMaterialColorServiceState(f"virtual_screen_{i}",
                                                 link_name,
                                                 visual_name,
                                                 BLUE),
                    transitions={"succeeded": "set_screen_red_end",
                                 "aborted": FINISHED,
                                 "preempted": PREEMPTED}
                )

                StateMachine.add(
                    "set_screen_red_end",
                    SetMaterialColorServiceState(f'virtual_screen_{i + 1}',
                                                 link_name,
                                                 visual_name,
                                                 RED),
                    transitions={'succeeded': 'wait_for_husky_hotspot_end',
                                 'aborted': ERROR,
                                 'preempted': PREEMPTED}
                )

                StateMachine.add(
                    "wait_for_husky_hotspot_end",
                    RobotPoseMonitorState(ROBOT_ID, goal_hostspot),
                    transitions={'valid': 'wait_for_husky_hotspot_end',
                                 'invalid': FINISHED,
                                 'preempted': PREEMPTED}
                )
                
            else:
                StateMachine.add(
                    f"set_screen_blue_{i}",
                    SetMaterialColorServiceState(f"virtual_screen_{i}",
                                                 link_name,
                                                 visual_name,
                                                 BLUE),
                    transitions={'succeeded': f"set_screen_red_{i + 1}",
                                 'aborted': ERROR,
                                 'preempted': PREEMPTED}
                )

    # start SM
    breakpoint()
    sm.execute()

    rospy.spin()


    # We need to exit, request SM preemption.    
    sm.request_preempt()

