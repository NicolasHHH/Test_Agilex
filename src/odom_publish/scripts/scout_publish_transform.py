#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

if __name__ == "__main__":
    rospy.init_node("scout_publish_transform")
    rospy.loginfo("scout_publish_transform node started")

    br = TransformBroadcaster()

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.child_frame_id = "base_link"

    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    model = GetModelStateRequest()
    model.model_name = "scout/"

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        result = get_model_state(model)
        odom.pose.pose.position.x = - result.pose.position.y
        odom.pose.pose.position.y = result.pose.position.x
        odom.pose.pose.position.z = result.pose.position.z
        odom.twist.twist = result.twist

        odom_pub.publish(odom)

        # publish a tf transform from world to base_link
        br.sendTransform((result.pose.position.x, result.pose.position.y, result.pose.position.z),
                         (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z,
                          result.pose.orientation.w),
                         rospy.Time.now(),
                         "base_link",
                         "world")

        rate.sleep()
