import roslib; roslib.load_manifest('pr2_plan_utils')
import rospy
from pr2_plan_utils import transform_listener

rospy.init_node('test_transform_listener', anonymous=True)

print transform_listener.get_transform('/base_link', '/r_wrist_roll_link')

t = rospy.Time(rospy.Time.now().to_sec() + 2.0)
print transform_listener.get_transform('/base_link', '/r_wrist_roll_link', t)
