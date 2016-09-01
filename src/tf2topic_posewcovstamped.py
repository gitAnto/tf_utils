#!/usr/bin/env python
#*********************************************************************
#*
#*  Copyright (c) 2016.
#*  All rights reserved.
#*
#*  Software License Agreement (BSD License)
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#* 		 the distribution.
#*   * The names of its contributors may not be used to endorse or promote 
#*     products derived from this software without specific prior
#*     written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#*  Author: Antonio Petitti on May, 2015
#*********************************************************************/
PKG = 'tf_utils'
NAME = 'tf2topic_posewcovstamped'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from common.tf2topic import tf2topic
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np

import numpy.linalg as la
from math import sqrt, pow, cos, sin, exp, atan2


class tf2topic_posewcovstamped(tf2topic):
		
	def __init__(self):
		tf2topic.__init__(self)

		#node parameters
		self.frame_ref = rospy.get_param('~frame_ref', 'world')
        	self.frame_id  = rospy.get_param('~frame_id', 'robot')
        	self.rate = float(rospy.get_param('~rate', '1.0'))
        	self.topic = rospy.get_param('~topic', 'topic')

		#define the topic to publish on
		self.pub = rospy.Publisher(self.topic, PoseWithCovarianceStamped)

	def listenToTF(self):

		#define message to publish
		robot_pose = PoseWithCovarianceStamped()
		#declare a tf listener
		listener = tf.TransformListener()

		#set the publish rate
		r = rospy.Rate(self.rate) 

		while not rospy.is_shutdown():
			
			#listen from the tf
			try:
				(trans,rot) = listener.lookupTransform(self.frame_ref, self.frame_id, rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				
				continue

			#set the topic message 
			robot_pose.header.frame_id = self.frame_ref;
			robot_pose.header.stamp = rospy.get_rostime();

			robot_pose.pose.pose.position.x = trans[0];
			robot_pose.pose.pose.position.y = trans[1];
			robot_pose.pose.pose.position.z = trans[2];

			robot_pose.pose.pose.orientation.x = rot[0];
			robot_pose.pose.pose.orientation.y = rot[1];
			robot_pose.pose.pose.orientation.z = rot[2];
			robot_pose.pose.pose.orientation.w = rot[3];

			#robot_pose.pose.covariance = [0.01, 	0.0, 	0.0,         0.0,         0.0,         0.0, \
			#   														0.0, 0.01, 0.0,         0.0,         0.0,         0.0, \
			 #  														0.0, 0.0, 0.01,         0.0,         0.0,         0.0, \
				#	   												0.0,         0.0,         0.0, 0.01,         0.0,         0.0, \
			#		   												0.0,         0.0,         0.0,          0.0, 0.01,         0.0, \
				#	   												0.0,         0.0,         0.0,          0.0,         0.0,    0.01 ]

			#covariance definition
			x = robot_pose.pose.pose.position.x
			y = robot_pose.pose.pose.position.y
			z = robot_pose.pose.pose.position.z

			d = sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2) )
			theta = atan2( y , x )
			phi   = atan2( z , sqrt( pow(x,2) + pow(y,2) ) )

			a = 0.0092
			b = 0.3258
			k_theta = 0.1
			k_phi = 0.1
			r_sensing= 10

			sigma_d     = a*exp(b*d)
			sigma_theta = k_theta*d/r_sensing
			sigma_phi = k_phi*d/r_sensing	

			T = np.matrix([[sin(theta)*cos(phi), cos(theta)*cos(phi), -sin(theta)*sin(phi)], [sin(theta)*sin(phi), cos(theta)*sin(phi), sin(theta)*cos(phi)], [cos(theta), -sin(theta),0.0]])
			B = np.matrix([[sigma_d, 0.0, 0.0],[0.0, sigma_theta, 0.0],[sigma_phi, 0.0, 0.0]])

			Rxyz = T*B*T.T

			robot_pose.pose.covariance = [Rxyz[0,0], Rxyz[0,1], Rxyz[0,2],         0.0,         0.0,         0.0, \
			   														Rxyz[1,0], Rxyz[1,1], Rxyz[1,2],         0.0,         0.0,         0.0, \
			      														Rxyz[2,0], Rxyz[2,1], Rxyz[2,2],         0.0,         0.0,         0.0, \
			   																   0.0,         0.0,         0.0, sigma_theta,         0.0,         0.0, \
				   															   0.0,         0.0,         0.0,          0.0, sigma_theta,         0.0, \
			   																   0.0,         0.0,         0.0,          0.0,         0.0,    sigma_phi ]

			#publish the message
			self.pub.publish(robot_pose)

			r.sleep()


def tfpub_main(argv=None):
	rospy.init_node(NAME, anonymous=False)

	posePublisher_ = tf2topic_posewcovstamped()
	rospy.loginfo("Listening to tf the pose of " + posePublisher_.frame_id + " w.r.t. " + posePublisher_.frame_ref)
	rospy.loginfo("Publishing on the topic:" + posePublisher_.topic)
	posePublisher_.listenToTF()

if __name__ == '__main__':
	tfpub_main()
