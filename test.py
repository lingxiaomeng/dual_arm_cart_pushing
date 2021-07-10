from robot_api import Robot_Api
import rospy
import time


class Main:
    def __init__(self):
        self.left_arm = Robot_Api("left_arm")
        self.right_arm = Robot_Api("right_arm")

    def get_dual_arm_action_state(self):
        left_state = (self.left_arm.client.get_state() == 3)
        right_state = (self.right_arm.client.get_state() == 3)
        return left_state, right_state

    def home_dual_arm(self):
        self.left_arm.homeRobot()
        self.right_arm.homeRobot()

    def main(self):
        state = 0
        left_end = True
        right_end = True
        while not rospy.is_shutdown():
            if left_end and right_end:
                if state == 0:
                    self.left_arm.send_pose((0.47722487688, -0.205575549603, 0.397170269489), (0.640116751194,
                                                                                          0.33752951026, 0.428907394409, 0.54070597887))
                    self.right_arm.send_pose((0.47722487688, -0.205575549603, 0.397170269489), (0.640116751194,
                                                                                           0.33752951026, 0.428907394409, 0.54070597887))
                    state = 1
                elif state == 1:
                    break

            left_end, right_end = self.get_dual_arm_action_state()
            time.sleep(0.01)


if __name__ == "__main__":
    main = Main()
    main.home_dual_arm()
    main.main()
