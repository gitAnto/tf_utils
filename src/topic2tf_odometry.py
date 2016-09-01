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
PKG = 'topic2tf'
NAME = 'topic2tf_odometry'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from common.topic2tf import topic2tf
from nav_msgs.msg import Odometry


class topic2tf_odometry(topic2tf):
		
	def __init__(self):
		topic2tf.__init__(self)

		#node parameters
		self.frame_ref = rospy.get_param('~frame_ref', 'world')
        	self.frame_id  = rospy.get_param('~frame_id', 'robot')
        	self.topic = rospy.get_param('~topic', 'topic')
        	
		#define the topic to publish on
		self.sub = rospy.Subscriber(self.topic, Odometry, self.callback);

	def callback(self, msg):

		#declare a tf broadcaster
		broadcaster = tf.TransformBroadcaster()

		#broadcast the transformation
		broadcaster.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),                  #translation vector
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), #quaternion orientation,
                     rospy.Time.now(),
                     self.frame_id,
                     self.frame_ref)



def odombroad_main(argv=None):
	rospy.init_node(NAME, anonymous=False)

	odomBroadcaster_ = topic2tf_odometry()
	rospy.loginfo("Listening to topic: " + odomBroadcaster_.topic)
	rospy.loginfo("Broadcasting the pose of " + odomBroadcaster_.frame_id + " w.r.t. " + odomBroadcaster_.frame_ref)
	rospy.spin()

if __name__ == '__main__':
	odombroad_main()
