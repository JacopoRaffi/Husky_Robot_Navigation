# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains classes to simplify the usage of robot monitors
"""
# pylint: disable=redefined-builtin
from builtins import zip

import smach_ros

from gazebo_msgs.msg import ModelStates, LinkStates

__author__ = 'Georg Hinkel'


class RobotPoseMonitorState(smach_ros.MonitorState):
    """
    This state monitors the pose of a robot
    """
    def __init__(self, robot_id, position_cb):
        """
        Creates a new state that monitors the robots position

        :param robot_id: The name of the robot to monitor
        :param position_cb: A callback that should be executed when a new pose is available
         for the robot
        """
        super(RobotPoseMonitorState, self).__init__('/gazebo/model_states', ModelStates,
                                                    self.__callback)
        self.__position_cb = position_cb
        self.__robot_id = robot_id

    def __callback(self, user_data, states):
        """
        The callback when a new model state message arrives

        :param user_data: The state machine user data
        :param states: The model states
        """
        for name, pose in zip(states.name, states.pose):
            if name == self.__robot_id:
                return self.__position_cb(user_data, pose)

        return True


class RobotTwistMonitorState(smach_ros.MonitorState):
    """
    This state monitors the twist of a robot
    """
    def __init__(self, robot_id, twist_cb):
        """
        Creates a new state that monitors the robots twist

        :param robot_id: The name of the robot to monitor
        :param twist_cb: A callback that should be executed when a new twist is available
         for the robot
        """
        super(RobotTwistMonitorState, self).__init__('/gazebo/model_states', ModelStates,
                                                     self.__callback)
        self.__twist_cb = twist_cb
        self.__robot_id = robot_id

    def __callback(self, user_data, states):
        """
        The callback when a new model state message arrives

        :param user_data: The state machine user data
        :param states: The model states
        """
        for name, twist in zip(states.name, states.twist):
            if name == self.__robot_id:
                return self.__twist_cb(user_data, twist)

        return True


class LinkPoseMonitorState(smach_ros.MonitorState):
    """
    This state monitors the pose of a robot's link.
    """
    def __init__(self, position_cb, link_name):
        """
        Creates a new state that monitors the link position

        :param position_cb: A callback that should be executed when a new pose is available
         for the link
        :param link_name: The link name prefixed by model name (e.g. pr2::base_link)
        """
        super(LinkPoseMonitorState, self).__init__('/gazebo/link_states', LinkStates,
                                                   self.__callback)
        self.__link_name = link_name
        self.__position_cb = position_cb

    def __callback(self, user_data, states):
        """
        The callback when a new model state message arrives

        :param user_data: The state machine user data
        :param states: The link states
        """
        for name, pose in zip(states.name, states.pose):
            if name == self.__link_name:
                return self.__position_cb(user_data, pose)

        return True
