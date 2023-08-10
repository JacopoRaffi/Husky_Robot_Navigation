#!/usr/bin/python3
import rospy
from robot_vision.msg import Vision
import numpy as np
import time

Kp = 1
Kd = 1
Ki = 1

errors = np.array([0])
times = np.array([0])
areas = np.array([0])

def pid(vision):
    if (vision.target_found):
        diff_x = vision.center_x - vision.target_x
        t = time.time() * 1000
        np.append(areas, diff_x * (t - times[-1]))
        
        proportional = Kp * diff_x
        derivative = Kd * (diff_x - errors[-1]) / (t - times[-1])
        integral = Ki * sum(areas)
        
        np.append(errors, diff_x)
        np.append(times, t)

        result = proportional + derivative + integral


    
def wheel_controller():
    rospy.init_node("r_controller")
    rospy.Subscriber("/vision", Vision, pid, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    wheel_controller()