import roslib; roslib.load_manifest('pr2_python')
import rospy

from pr2_python.base import Base, _yaw
from pr2_python import transform_listener
from pr2_python import conversions

rospy.init_node('check_base_pose', anonymous=True)

H = transform_listener.get_transform('/map', '/base_footprint')
pose = conversions.mat_to_pose(H)
print (pose.position.x, pose.position.y,  _yaw(pose.orientation))

