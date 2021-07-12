#! /usr/bin/env python
#this node takes information about the robot being published in gazebo and publishes an odometry and tf message for use in RVIZ
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher ('/gazebo_odom', Odometry, queue_size=10)

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='robot'

r = rospy.Rate(15)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = 'gazebo_dingo_frame'
    t.transform.translation.x = result.pose.position.x
    t.transform.translation.y = result.pose.position.y
    t.transform.translation.z = result.pose.position.z
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = result.pose.orientation.x
    t.transform.rotation.y = result.pose.orientation.y
    t.transform.rotation.z = result.pose.orientation.z
    t.transform.rotation.w = result.pose.orientation.w

    br.sendTransform(t)
    

    r.sleep()
