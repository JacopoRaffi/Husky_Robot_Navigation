#!/usr/bin/python3
import rospy
from robot_vision.msg import Vision
import numpy as np
import time
from husky_gazebo_plugins.srv import SetHuskyWheelSpeeds
from husky_gazebo_plugins.msg import WheelSpeeds

import numpy as np

Kp = 2
Kd = 0.001
Ki = 0.05
threshold = 3
speed_threshold = 50
K_speed = 0.1

wheel_srv = rospy.ServiceProxy('/husky_model/husky/wheel_speeds_service', SetHuskyWheelSpeeds)

is_reset = False
flip = True

areas = np.array([0])
prev_time = 0
prev_err = 0

def reset():
    global areas, prev_time, prev_err, is_reset
    prev_time = prev_err = 0
    areas = np.array([0])
    is_reset = True

def pid(vision):
    global areas, prev_time, prev_err, is_reset, flip
    if (vision.target_found):
        is_reset = False
        diff_x = vision.center_x - vision.target_x
        t = time.time()
        areas = np.append(areas, diff_x * (t - prev_time))
        
        proportional = Kp * diff_x
        derivative = Kd * (diff_x - prev_err) / (t - prev_time)
        integral = Ki * sum(areas[2:])
        
        prev_err = diff_x
        prev_time = t
        result = proportional + derivative + integral
        abs_result = abs(result)
        final_speed = abs_result
        
        if (abs_result > speed_threshold):
            final_speed = speed_threshold

        #back_left, back_right, front_left, front_right
        w_speed = WheelSpeeds(0, 0, 0, 0)
        if (result >= threshold): #vai a sinistra
            w_speed.back_right_wheel = K_speed * final_speed
            w_speed.front_right_wheel = K_speed * final_speed
        elif (result <= -threshold): #vai a destra
            w_speed.back_left_wheel = K_speed * final_speed
            w_speed.front_left_wheel = K_speed * final_speed
        else: #vai dritto
            w_speed.back_left_wheel = 5
            w_speed.back_right_wheel = 5
            w_speed.front_left_wheel = 5
            w_speed.front_right_wheel = 5
            
        wheel_srv(w_speed)
    else:
        if (not is_reset):
            reset()
        
        w_speed = WheelSpeeds(0, 0, 0, 0)
        
        if flip:
            w_speed.back_right_wheel = 3
            w_speed.front_right_wheel = 3
        else:
            w_speed.back_left_wheel = -0.5
            w_speed.back_right_wheel = -0.5
            w_speed.front_left_wheel = -0.5
            w_speed.front_right_wheel = -0.5

        wheel_srv(w_speed)
        flip = not flip


    
def wheel_controller():
    rospy.init_node("r_controller")
    rospy.Subscriber("/vision", Vision, pid, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    wheel_controller()