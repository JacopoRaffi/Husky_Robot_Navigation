#!/usr/bin/python3
import rospy
from robot_vision.msg import Vision
import numpy as np
import time
from husky_gazebo_plugins.srv import GetHuskyOdometryRequest
from husky_gazebo_plugins.srv import SetHuskyWheelSpeeds
from husky_gazebo_plugins.msg import WheelSpeeds

import numpy as np

Kp = 2
Kd = 0.0001
Ki = 0.1
threshold = 2
K_speed = 1

wheel_srv = rospy.ServiceProxy('/husky_model/husky/wheel_speeds_service', SetHuskyWheelSpeeds)

errors = np.array([0])
times = np.array([0])
areas = np.array([0])

def pid(vision):
    global errors, areas, times
    if (vision.target_found):
        diff_x = vision.center_x - vision.target_x
        t = time.time()
        areas = np.append(areas, diff_x * (t - times[-1]))
        
        proportional = Kp * diff_x
        derivative = Kd * (diff_x - errors[-1]) / (t - times[-1])
        integral = Ki * sum(areas[2:])
        
        errors = np.append(errors, diff_x)
        times = np.append(times, t)
        result = proportional + derivative + integral
        abs_result = abs(result)

        #back_left, back_right, front_left, front_right
        w_speed = WheelSpeeds(0, 0, 0, 0)
        if (result >= threshold): #vai a sinistra
            w_speed.back_right_wheel = K_speed * abs_result
            w_speed.front_right_wheel = K_speed * abs_result
        elif (result <= -threshold): #vai a destra
            w_speed.back_left_wheel = K_speed * abs_result
            w_speed.front_left_wheel = K_speed * abs_result
        else: #vai dritto
            w_speed.back_left_wheel = 1
            w_speed.back_right_wheel = 1
            
        wheel_srv(w_speed)


    
def wheel_controller():
    rospy.init_node("r_controller")
    rospy.Subscriber("/vision", Vision, pid, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    wheel_controller()