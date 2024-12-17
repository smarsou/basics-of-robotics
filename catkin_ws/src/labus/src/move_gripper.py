#!/usr/bin/env python

import roslib
import rospy

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


rospy.init_node('Robotiq3FGripperSimpleController')

pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)

while not rospy.is_shutdown():

    # activating the gripper
    command = Robotiq3FGripperRobotOutput()
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    command.rATR = 0
    command.rMOD = 0
    command.rPRA = 0

    pub.publish(command)
    rospy.sleep(0.1)




#     break

# if __name__ == '__main__':
