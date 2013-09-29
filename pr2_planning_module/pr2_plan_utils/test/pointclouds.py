import roslib; roslib.load_manifest('pr2_plan_utils')
import numpy as np
from sensor_msgs.msg import PointCloud2

from pr2_plan_utils import pointclouds

points = np.random.random((10, 3))
cloud_msg = pointclouds.xyz_array_to_pointcloud2(points)
print len(cloud_msg.data)

points_new = pointclouds.pointcloud2_to_xyz_array(cloud_msg)
print points - points_new
print points
print points_new
