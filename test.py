from robot_api import Robot_Api
import rospy
import time
from geometry_msgs.msg import Twist


class Main:
    def __init__(self):
        rospy.init_node('cart_pushing', anonymous=True)
        print("start load arm")

        self.left_arm = Robot_Api("left_arm")
        self.right_arm = Robot_Api("right_arm")
        print("arm load succeed")

        self.cmd_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/cmd_vel', Twist, queue_size=1)


    def get_dual_arm_action_state(self):
        left_state = (self.left_arm.client.get_state() == 3)
        right_state = (self.right_arm.client.get_state() == 3)
        return left_state, right_state

    def home_dual_arm(self):
        self.left_arm.homeRobot()
        self.right_arm.homeRobot()

    def cmd_speed(self, linear_speed, angle_speed):
        speed = Twist()
        speed.linear.x = linear_speed
        speed.angular.z = angle_speed
        self.cmd_pub.publish(speed)

    def main(self):
        state = 0
        left_end = True
        right_end = True
        rate = rospy.Rate(100)
        linear_speed = 0
        angle_speed = 0
        print("start main")
        n = 0
        while not rospy.is_shutdown():
            if left_end and right_end:
                if state == 0:
                    left_orientation = [0, 0.7071068, -0.7071068, 0]
                    right_orientation = [0, 0.7071068, -0.7071068, 0]

                    left_pose = [0.36, -0.64, 0.16]
                    right_pose = [-0.36, -0.64-0.05, 0.16]
                    # y-- move forward
                    self.left_arm.send_pose(left_pose, left_orientation)
                    self.right_arm.send_pose(right_pose, right_orientation)

                    state = 1
                    linear_speed = 0.05
                    angle_speed = 0.05
                elif state == 1:
                    n+=1
                    if n>300:
                        state = 2
                        linear_speed = 0
                        angle_speed = 0
                elif state ==2:
                    break

                    # break
            self.cmd_speed(linear_speed, angle_speed)
            left_end, right_end = self.get_dual_arm_action_state()
            # print left_end
            # print right_end
            rate.sleep()


if __name__ == "__main__":
    main = Main()
    # main.home_dual_arm()
    main.main()
