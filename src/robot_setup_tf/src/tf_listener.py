#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import PointStamped
import tf


"""
Note:
To check that the tf frames are set-up properly:
rosrun tf tf_echo frm_prnt frm_chld
"""

if __name__ == "__main__":

    rospy.init_node("robot_tf_listener")
    listener = tf.TransformListener()

    rate = rospy.Rate(10)

    # a dummy point that was captured by the laser scanner (hence is a stamped point shown wrt the base_laser frame)
    laser_point = PointStamped()
    laser_point.header.frame_id = "base_laser"

    # user the most recent available transform
    laser_point.header.stamp = rospy.Time(0)

    # some dummy values for the point (this could be a topic that is subscribed to, or that of a point cloud)
    laser_point.point.x = 1.0
    laser_point.point.y = 3.3
    laser_point.point.z = 0.0

    while not rospy.is_shutdown():
        
        try:
            
            # give the listener's buffer time to populate
            listener.waitForTransform("base_link", "base_laser", rospy.Time(0),rospy.Duration(3))
  
        except:
            rospy.logerr("Transform does not exist."+\
                         "\nHave you tried waiting for the listener's buffer to fill first?")

        # transform the location of the point from being wrt the base_laser frame to being wrt the base_link frame
        base_point = listener.transformPoint("base_link",laser_point)

        rospy.loginfo(\
            "base_laser: ({}, {}, {}) -----> base_link: ({}, {}, {})".format(\
            laser_point.point.x, laser_point.point.y, laser_point.point.z,\
            base_point.point.x, base_point.point.y, base_point.point.z)\
            )

        rate.sleep()