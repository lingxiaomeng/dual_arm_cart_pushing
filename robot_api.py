import sys
import rospy
import time
import math
from kinova_msgs.srv import *
from kinova_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import actionlib



class Robot_Api:
    def __init__(self, robot_name):
        try:
            rospy.init_node('arm_control')
            # Get node params
            self.robot_name = rospy.get_param('~robot_name', robot_name)
            action_address = self.robot_name+'_driver/pose_action/tool_pose'
            self.client = actionlib.SimpleActionClient(
                action_address, kinova_msgs.msg.ArmPoseAction)
            self.client.wait_for_server()
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
    
    def homeRobot(self):
        service_address = '/' + self.robot_name + '_driver/in/home_arm'
        rospy.wait_for_service(service_address)
        try:
            home = rospy.ServiceProxy(service_address, HomeArm)
            home()
            return None
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def get_pose(self):
        feedback = rospy.wait_for_message(
            "/" + self.robot_name + "_driver/out/tool_pose", PoseStamped)
            
        return feedback.pose.position, feedback.pose.orientation

    def send_pose(self, position, orientation):
        """Send a cartesian goal to the action server."""
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(
            frame_id=(self.robot_name+'_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.client.send_goal(goal)


if __name__ == "__main__":
    left_arm = Robot_Api("left_arm")
    right_arm = Robot_Api("right_arm")
    pose, oriention = left_arm.get_pose()
    # left_arm.homeRobot()
    # right_arm.homeRobot()
    # print(pose)

    res = left_arm.send_pose((0.47722487688, -0.205575549603, 0.397170269489), (0.640116751194,
                    0.33752951026, 0.428907394409, 0.54070597887))
    res = right_arm.send_pose((0.47722487688, -0.205575549603, 0.397170269489), (0.640116751194,
                    0.33752951026, 0.428907394409, 0.54070597887))
    
    # print(res) 