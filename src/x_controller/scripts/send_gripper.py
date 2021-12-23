#!/usr/bin/python3

# Copyright (C) 2021 Filippo Rossi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import argparse

import rospy
import actionlib
import control_msgs.msg


def gripper_client(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1.0  # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.2",
                            help="Value betwewen 0.0 (open) and 0.8 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)
        print(result)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")