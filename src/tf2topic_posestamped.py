#!/usr/bin/env python
#*********************************************************************
#*
#*  Copyright (c) 2014, CNR-ISSIA | Mobile Robotics Laboratory [MRL].
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
#*   * Neither the name of the CNR-ISSIA nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
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
PKG = 'issia_tf_utils'
NAME = 'issia_tf2topic_posestamped'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from common.issia_tf2topic import issia_tf2topic
from geometry_msgs.msg import PoseStamped


class issia_tf2topic_posestamped(issia_tf2topic):
		
	def __init__(self):
		issia_tf2topic.__init__(self)

		#node parameters
		self.frame_ref = rospy.get_param('~frame_ref', 'world')
        	self.frame_id  = rospy.get_param('~frame_id', 'robot')
        	self.rate = float(rospy.get_param('~rate', '1.0'))
        	self.topic = rospy.get_param('~topic', 'topic')

		#define the topic to publish on
		self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=10)

	def listenToTF(self):

		#define message to publish
		robot_pose = PoseStamped()
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

			robot_pose.pose.position.x = trans[0];
			robot_pose.pose.position.y = trans[1];
			robot_pose.pose.position.z = trans[2];

			robot_pose.pose.orientation.x = rot[0];
			robot_pose.pose.orientation.y = rot[1];
			robot_pose.pose.orientation.z = rot[2];
			robot_pose.pose.orientation.w = rot[3];

			#publish the message
			self.pub.publish(robot_pose)

			r.sleep()


def tfpub_main(argv=None):
	rospy.init_node(NAME, anonymous=False)

	posePublisher_ = issia_tf2topic_posestamped()
	rospy.loginfo("Listening to tf the pose of " + posePublisher_.frame_id + " w.r.t. " + posePublisher_.frame_ref)
	rospy.loginfo("Publishing on the topic:" + posePublisher_.topic)
	posePublisher_.listenToTF()

if __name__ == '__main__':
	tfpub_main()
