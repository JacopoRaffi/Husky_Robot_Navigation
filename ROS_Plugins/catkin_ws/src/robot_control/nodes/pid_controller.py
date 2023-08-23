#!/usr/bin/python3
import rospy
from robot_vision.msg import Vision
import numpy as np
import time
from husky_gazebo_plugins.srv import SetHuskyWheelSpeeds
from husky_gazebo_plugins.msg import WheelSpeeds

# ROS node that implements a synchronous PID controller for robot motion 
# Our robot behaviour is organized in two core states:
#   1- If a human is found -> move towards it
#   2- else -> move in order to rotate and explore the 360 degree field of view in order to find the next target human

# PID controller parameters
Kp = 1.5
Kd = 0.001
Ki = 0.025

threshold = 3.5      # acceptable error range for the robot to continue in a straight direction
speed_threshold = 8  # maximum amount of speed generated the correction of the cruise
K_speed = 0.1        # attenuation parameter for the speed imposed to wheels

wheel_srv = rospy.ServiceProxy('/husky_model/husky/wheel_speeds_service', SetHuskyWheelSpeeds)

is_reset = False
flip = True

areas = np.array([0]) # array containing the rectangular areas subtended by the error function for calculating the integral (sum of areas)
prev_time = 0         # last (previous) moment at which a correction was computed
prev_err = 0          # last (previous) error on which a correction was computed

# method to reset the state of the PID controller when the target is changed
def reset():
    global areas, prev_time, prev_err, is_reset
    prev_time = prev_err = 0
    areas = np.array([0])
    is_reset = True

# method containing the PID controller algorithm
def pid(vision):
    global areas, prev_time, prev_err, is_reset, flip

    if (vision.target_found): # case: a human is recongized by Yolov8 in the camera field of view
        is_reset = False

        # the PID controller base its correction on the difference between the center of the camera field of view and the center of the identified human
        # in this way the robot aims directly to the identified human
        diff_x = vision.center_x - vision.target_x 
        t = time.time()
        areas = np.append(areas, diff_x * (t - prev_time))
        
        proportional = Kp * diff_x
        derivative = Kd * (diff_x - prev_err) / (t - prev_time) # application of the derivative formula to this specific instant
        integral = Ki * sum(areas[2:]) # calculated as the sum of the areas subtended by the error function
        
        prev_err = diff_x
        prev_time = t
        result = proportional + derivative + integral
        abs_result = abs(result)
        final_speed = abs_result
        
        # the maximum velocity is limited by a threshold so to better apply in the specific case of this project
        if (abs_result > speed_threshold):
            final_speed = speed_threshold

        # as the correction has been computed a speed is applied to the wheels thanks to the specific ROS service    
        w_speed = WheelSpeeds(0, 0, 0, 0) # back_left, back_right, front_left, front_right
        if (result >= threshold): # turn left
            w_speed.back_right_wheel = K_speed * final_speed
            w_speed.front_right_wheel = K_speed * final_speed

        elif (result <= -threshold): # turn right
            w_speed.back_left_wheel = K_speed * final_speed
            w_speed.front_left_wheel = K_speed * final_speed

        else: # go straight
            w_speed.back_left_wheel = 5
            w_speed.back_right_wheel = 5
            w_speed.front_left_wheel = 5
            w_speed.front_right_wheel = 5
            
        wheel_srv(w_speed)

    else:
        if (not is_reset):
            reset()
        
        w_speed = WheelSpeeds(0, 0, 0, 0)
        
        # when Yolov8 can't perceive a human in the camera field of view, the robot will move in order to rotate anti-clockwise until a new human is found
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

# ROS node initialization method 
# register and listen on the "/vision" topic to receive information about the target (Vision ROS msg) from the "v_talker" ROS node
def wheel_controller():
    rospy.init_node("r_controller")
    rospy.Subscriber("/vision", Vision, pid, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    wheel_controller()