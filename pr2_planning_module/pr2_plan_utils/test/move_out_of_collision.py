import roslib; roslib.load_manifest('pr2_plan_utils')
import rospy

from pr2_plan_utils.arm_mover import ArmMover

rospy.init_node('test_move_out_of_collision', anonymous=True)

arm_mover = ArmMover('right_arm')
arm_mover.move_out_of_collision(0.3)
