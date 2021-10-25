#!/usr/bin/env python2.7

import rospy
import tf


prnt_frm = "base_link"
chld_frm = "base_laser"

# transformation between form the parent frame to the child frame
trans = (0.1, 0.0, 0.2)
rot   = (0.0, 0.0, 0.0, 1.0)

if __name__ == "__main__":

    # initialize the rosnode and the transformation broadcaster (i.e. publishes tf)
    rospy.init_node("robot_tf_broadcaster")
    broadcaster = tf.TransformBroadcaster()

    # rate at which tf will be updated
    rate = rospy.Rate(100)

    # publish the tf!
    while not rospy.is_shutdown():
        broadcaster.sendTransform(trans, rot, rospy.Time.now(), chld_frm, prnt_frm)
        rate.sleep()
