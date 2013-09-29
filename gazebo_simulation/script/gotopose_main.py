#!/usr/bin/env python

import roslib; 
roslib.load_manifest('gazebo_simulation')
roslib.load_manifest('pr2_plan_utils')

# for tucking the left arm
#roslib.load_manifest('pr2_tuckarm')
#from pr2_tuck_arms_action import tuck_arms_main
#from pr2_common_action_msgs.msg import *
# end for tucking the left arm

import numpy
import rospy
import tf
import actionlib
import gazebo_simulation.msg

from geometry_msgs.msg import PoseStamped
from pr2_plan_utils.arm_mover import ArmMover
#from pr2_tasks.tasks import Tasks
from pr2_plan_utils.arm_tasks import ArmTasks
from pr2_plan_utils import head
from pr2_plan_utils import base
from pr2_plan_utils import torso


#arm_name = 'right_arm' #arm to use for doing awesome stuff

class BeAPoserAction(object):
	#result msg
	_result   = gazebo_simulation.msg.GoToPoseResult()


	def __init__(self, name):
		self._arm_mover = ArmMover()
		#self._tasks = Tasks()
		self._arm_tasks = ArmTasks()
		self._base = base.Base()
		self._head = head.Head()
		self._torso = torso.Torso()
		self._tf = tf.TransformListener( True, rospy.Duration(100) )
		self._tfBr = tf.TransformBroadcaster()

		#################### MOVE TORSO TO MAX HEIGHT ####################
		self._torso.up()
		rospy.loginfo('[gtp] Torso lifted!')
		
		#################### TUCK LEFT ARM #######################
		#self._tasks.move_arms_to_side()
		self._arm_tasks.move_arm_to_side('left_arm')
		rospy.loginfo('[gtp] Left arm moved to the side!')
		
		#action_name = 'tuck_arms'
		#tuck_arms_action_server = tuck_arms_main.TuckArmsActionServer(action_name)
		#tuck_arm_client = actionlib.SimpleActionClient(action_name, TuckArmsAction)

		#rospy.logdebug('Waiting for tuckarm action server to start')
		#tuck_arm_client.wait_for_server(rospy.Duration(10.0))

		#rospy.logdebug('Tucking left arm...')
		#goal = TuckArmsGoal()
		#goal.tuck_left = True
		#goal.tuck_right = False
		#tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

		#################### MOVE RIGHT ARM TO INITAL POSE #######################
		#self._arm_tasks.move_arm_to_side('right_arm')
		#rospy.loginfo('[gtp] Right arm moved to the side!')		
		self._init_arm_pose = PoseStamped()
		self._init_arm_pose.pose.position.x = 0.282
		self._init_arm_pose.pose.position.y = -0.485 
		self._init_arm_pose.pose.position.z = 0.303
		self._init_arm_pose.pose.orientation.x = 0.030
		self._init_arm_pose.pose.orientation.y = 0.342
		self._init_arm_pose.pose.orientation.z = -0.057 
		self._init_arm_pose.pose.orientation.w = 0.937
		self._init_arm_pose.header.frame_id = 'torso_lift_link'
		
		for t in range(0,3):
			handle = self._arm_mover.move_to_goal( 'right_arm', self._init_arm_pose, 
									collision_aware_goal=True, planner_timeout=10.,
									bounds=(0.01, 0.01, 0.01, 0.1, 0.1, 0.1),
									try_hard=False )
			if handle.reached_goal():
				break
					
		if handle.reached_goal():
			rospy.loginfo('[gtp] Right arm init pose reached!')
		else:
			rospy.logerr('[gtp] Moving right arm to init pose failed!')
			rospy.logerr( handle.get_errors() )

		#################### STRAIGHTEN HEAD #################
		self._head.look_at_relative_point(0.8, 0.0, 0.5)
		rospy.loginfo('[gtp] Head straightened!')

		rospy.loginfo('[gtp] Attempting to reach initial pose...')
		init_goal = self._get_init_goal()
		success = self._go_to_pose( init_goal )
		if success:
			rospy.loginfo('[gtp] Initial pose successfully reached!')
		else:
			rospy.logwarn('[gtp] Initial pose was not reached :(')

		## Initialize action server
		self._action_name = name
		self._as = actionlib.SimpleActionServer( self._action_name, gazebo_simulation.msg.GoToPoseAction, execute_cb=self.execute_cb, auto_start = False )
		self._as.start()
		rospy.loginfo('[gtp] Action Server Initialized!')



	#
	#	_get_init_goal
	#
	def _get_init_goal(self):
		#tabOrigin = numpy.array([0.98, 0.0, 0.55])
		tabOrigin = numpy.array([1.475, 0.175, 0.85])
		sensorStart = tabOrigin + numpy.array([-1.0, -0.4, 0.5])
		offsets = numpy.array([0.0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, -0.4, 0.4, -0.5, 0.5])


		dirVec = tabOrigin - sensorStart
		dirVec = dirVec / numpy.linalg.norm(dirVec)
		yaw_pitch_roll = numpy.array([numpy.arctan2(dirVec[1], dirVec[0]), 
												-numpy.arctan2(dirVec[2], numpy.hypot(dirVec[0], dirVec[1])), 0.0 ])
		
		# angle2quat
		cyaw = numpy.cos(yaw_pitch_roll[0]/2)
		cpitch = numpy.cos(yaw_pitch_roll[1]/2)
		croll = numpy.cos(yaw_pitch_roll[2]/2)
	
		syaw = numpy.sin(yaw_pitch_roll[0]/2)
		spitch = numpy.sin(yaw_pitch_roll[1]/2)
		sroll = numpy.sin(yaw_pitch_roll[2]/2)

		sensorOrient = numpy.array([ cyaw*cpitch*sroll - syaw*spitch*croll, 
					cyaw*spitch*croll + syaw*cpitch*sroll,
					syaw*cpitch*croll - cyaw*spitch*sroll,
					cyaw*cpitch*croll + syaw*spitch*sroll ])
		# end angle2quat

		init_goal = gazebo_simulation.msg.GoToPoseGoal()
		for i in range(len(offsets)):
			init_goal.goal_pose_arr.append(PoseStamped())
			init_goal.goal_pose_arr[i].pose.position.x = sensorStart[0] + offsets[i]*dirVec[0]
			init_goal.goal_pose_arr[i].pose.position.y = sensorStart[1] + offsets[i]*dirVec[1]
			init_goal.goal_pose_arr[i].pose.position.z = sensorStart[2] + offsets[i]*dirVec[2]
			init_goal.goal_pose_arr[i].pose.orientation.x = sensorOrient[0]
			init_goal.goal_pose_arr[i].pose.orientation.y = sensorOrient[1]
			init_goal.goal_pose_arr[i].pose.orientation.z = sensorOrient[2]
			init_goal.goal_pose_arr[i].pose.orientation.w = sensorOrient[3]
			init_goal.goal_pose_arr[i].header.frame_id = "map"
			init_goal.goal_pose_arr[i].header.stamp = rospy.Time.now()

		return init_goal
		

	
	#
	#	execute_cb
	#
	def execute_cb(self, goal):
		rospy.loginfo('[gtp] Starting action execution!')

		success = self._go_to_pose(goal)
		if success:
			rospy.loginfo('%s: Succeeded!' % (self._action_name) )
		else:
			rospy.loginfo('%s: Failed!' % (self._action_name) )
		self._result.success = success
		self._as.set_succeeded(self._result)
	



	#
	#	_go_to_pose
	#
	def _go_to_pose(self, goal):
		success = False
		bnds = (0.02, 0.02, 0.02, 0.1, 0.1, 0.1)
		num_arm_try = 3
		
		# Compute the camera to wrist transform
		while True:
			try:		
				(o_t_w, o_q_w) = self._tf.lookupTransform('/arm_kinect_depth_optical_frame', '/r_wrist_roll_link', rospy.Time(0))
				k_t_w = (o_t_w[2], -o_t_w[0], -o_t_w[1])
				k_q_w = self._quatmult([-0.5, 0.5, -0.5, 0.5],o_q_w)
				break
			except:
				rospy.logerr('[gtp] CANNOT LOOKUP camera to wrist transform')
				
		#################################################################
		########## Compute goal poses in /r_wrist_roll_link #############
		wrist_pose_arr = []
		for i in range(len(goal.goal_pose_arr)):
			map_t_kinect = numpy.array([goal.goal_pose_arr[i].pose.position.x, 
				                         goal.goal_pose_arr[i].pose.position.y,
				                         goal.goal_pose_arr[i].pose.position.z ])
			map_R_kinect = self._quat2rot( goal.goal_pose_arr[i].pose.orientation.x,
							 goal.goal_pose_arr[i].pose.orientation.y,
							 goal.goal_pose_arr[i].pose.orientation.z,
							 goal.goal_pose_arr[i].pose.orientation.w )
										 
			map_t_wrist = numpy.dot(map_R_kinect, k_t_w) + map_t_kinect		
			map_q_wrist = self._quatmult([goal.goal_pose_arr[i].pose.orientation.x,
							goal.goal_pose_arr[i].pose.orientation.y,
							goal.goal_pose_arr[i].pose.orientation.z,
							goal.goal_pose_arr[i].pose.orientation.w]
							, k_q_w)
			
			r_wrist_pose = goal.goal_pose_arr[i]
			r_wrist_pose.pose.position.x = map_t_wrist[0]
			r_wrist_pose.pose.position.y = map_t_wrist[1]
			r_wrist_pose.pose.position.z = map_t_wrist[2]
			r_wrist_pose.pose.orientation.x = map_q_wrist[0]
			r_wrist_pose.pose.orientation.y = map_q_wrist[1]
			r_wrist_pose.pose.orientation.z = map_q_wrist[2]
			r_wrist_pose.pose.orientation.w = map_q_wrist[3]
			r_wrist_pose.header.frame_id = 'map'
			r_wrist_pose.header.stamp = rospy.Time.now()
			wrist_pose_arr.append(r_wrist_pose)
		
		rospy.loginfo('[gtp] Done determining the wrist poses')
		
		#################################################################
		## Try each of the poses with the arm first
		needToMoveBase = True
		for i in range(len(wrist_pose_arr)):
			rospy.loginfo('[gtp] Trying pose %d of %d\n' % (i+1, len(wrist_pose_arr)) )
			# Broadcast for visualization
			self._broadcast_gpose( goal.goal_pose_arr[i] )
			rospy.loginfo('[gtp] Move end-effector to map: (%f, %f, %f | %f, %f, %f, %f)' % 
						(wrist_pose_arr[i].pose.position.x, 
				 		 wrist_pose_arr[i].pose.position.y, 
				 		 wrist_pose_arr[i].pose.position.z, 
				 		 wrist_pose_arr[i].pose.orientation.x, 
				 		 wrist_pose_arr[i].pose.orientation.y, 
				 		 wrist_pose_arr[i].pose.orientation.z, 
				 		 wrist_pose_arr[i].pose.orientation.w) )
			try:
				for t in range(0,num_arm_try):
					handle = self._arm_mover.move_to_goal( 'right_arm', wrist_pose_arr[i], 
				  										collision_aware_goal=True, planner_timeout=10.,
				  										bounds=bnds, try_hard=False )
				  	if handle.reached_goal():
				  		break
				  		
				if handle.reached_goal():
					needToMoveBase = False
					success = True
					break
			except:
				rospy.loginfo('Arm mover exception for goal %d of %d\n' % 
										(i+1, len(wrist_pose_arr)) )
		
		#################################################################
		## If the goal was not reached, we need to move the base
		if needToMoveBase:
			rospy.loginfo('[gtp] Moving arm from current position failed, need to move base.')
			# Raise arm first not to hit anything
			for t in range(0,num_arm_try):
				handle = self._arm_mover.move_to_goal('right_arm', self._init_arm_pose,
													collision_aware_goal=True, planner_timeout=10.,
													bounds=bnds, try_hard=False )
				if handle.reached_goal():
					break
									
			for i in range(len(wrist_pose_arr)):
				# Broadcast for visualization
				self._broadcast_gpose( goal.goal_pose_arr[i] )
				try:
					self._base.move_manipulable_pose( wrist_pose_arr[i].pose.position.x,
															 wrist_pose_arr[i].pose.position.y, 
															 wrist_pose_arr[i].pose.position.z, 
															 'torso', False, 
															 wrist_pose_arr[i].pose.orientation.x, 
															 wrist_pose_arr[i].pose.orientation.y, 
															 wrist_pose_arr[i].pose.orientation.z, 
															 wrist_pose_arr[i].pose.orientation.w )
				except:
					rospy.logerr('EXCEPTION DUDE: Move base failed!!!!\n')
					continue
				
				# Base pose was reached try to move the arm now
				rospy.loginfo('LALALBUBUBUB: DESIRED BASE POSE REACHED!');
				try:
					for t in range(0,num_arm_try):
						handle = self._arm_mover.move_to_goal( 'right_arm', wrist_pose_arr[i], 
				  										collision_aware_goal=True, planner_timeout=10.,
				  										bounds=bnds, try_hard=False )
				  		if handle.reached_goal():
				  			break
				  		
				  	if handle.reached_goal():
				  		success = True
				  		break
				except:
					rospy.loginfo('Arm mover exception for goal %d of %d\n' %
											(i+1, len(wrist_pose_arr)) )
					
		return success
				
	#
	# Broadcasts a PoseStamped
	#
	def _broadcast_gpose(self, gpose):
		self._tfBr.sendTransform( (gpose.pose.position.x, 
											gpose.pose.position.y, 
											gpose.pose.position.z),
										  (gpose.pose.orientation.x, gpose.pose.orientation.y,
											gpose.pose.orientation.z, gpose.pose.orientation.w),
										  rospy.Time.now(),
										  "kinect_goal_pose", "/map" )
										  
	def _quat2rot(self, q0, q1, q2, q3):
		R = numpy.array([[1.0-2.0*(q1*q1+q2*q2), 
		  		2.0*q0*q1-2.0*q3*q2,
		  		2.0*q3*q1+2.0*q0*q2],
    	  	  [2.0*q0*q1+2.0*q3*q2,
		  		1.0-2.0*(q0*q0+q2*q2),
		  		2.0*q1*q2-2.0*q3*q0],
    	  	  [2.0*q0*q2-2.0*q3*q1,
		  		2.0*q1*q2+2.0*q3*q0,
		  		1.0-2.0*(q0*q0+q1*q1)]])
		return R

	def _quatmult(self, q1, q2):
		x1, y1, z1, w1 = q1
		x2, y2, z2, w2 = q2
		x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
		y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
		z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
		w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
		return x, y, z, w

if __name__ == '__main__':
	rospy.init_node('go_to_pose_act')
	BeAPoserAction(rospy.get_name())
	rospy.spin()
