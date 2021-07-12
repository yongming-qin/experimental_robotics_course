#!/usr/bin/env python
'''
    Receive a nav_msgs/Path and only leave the ones near the objects.
    Yongming Qin
    2021/03/23
'''
import copy
import time
import rospy


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Planning():
    def __init__(self):
        rospy.init_node('path_node')
        self.n1 = rospy.get_param("~n1", 0)
        self.n2 = rospy.get_param("~n2", 10)
        self.path_sub = rospy.Subscriber('/path/viz', Path, self.path_callback)
        self.path_pub = rospy.Publisher('/yq_path', Path, queue_size=10)

    def plan(self):
        time.sleep(2)
        self.pub_manual_path()
        print("published all the path")
        time.sleep(4)
        rate = rospy.Rate(2)
        self.n_manual = 0
        while not rospy.is_shutdown():
            self.pub_manual_path_sequentially()
            print("Looping")
            rate.sleep()

    def path_callback(self, msg):
        self.msg = copy.deepcopy(msg)
        print("Finish callback.")

    def pub_manual_path(self):
        path = Path()
        path.header = self.msg.header
        self.ids = []
        for seg in [[0,600], [870, 980], [1360, 1400], [1410, 1450]]:
            for i in range(seg[0],seg[1]):
                self.ids.append(i)
        for i in self.ids:
            pose_stamped_new = PoseStamped()
            pose_stamped_ori = self.msg.poses[i]
            pose_stamped_new.pose.position.x = pose_stamped_ori.pose.position.x
            pose_stamped_new.pose.position.y = pose_stamped_ori.pose.position.y
            pose_stamped_new.pose.position.z = 0
            pose_stamped_new.pose.orientation.x = pose_stamped_ori.pose.orientation.x
            pose_stamped_new.pose.orientation.y = pose_stamped_ori.pose.orientation.y
            pose_stamped_new.pose.orientation.z = pose_stamped_ori.pose.orientation.z
            pose_stamped_new.pose.orientation.w = pose_stamped_ori.pose.orientation.w
            path.poses.append(pose_stamped_new)
        self.path_pub.publish(path)

    def pub_manual_path_sequentially(self):
        path = Path()
        path.header = self.msg.header
        if self.n_manual + 10 <= len(self.ids):
            print("first: ", self.ids[self.n_manual], "last: ", self.ids[self.n_manual+10])
            for i in range(self.n_manual, self.n_manual + 10):
                id = self.ids[i]
                pose_stamped_new = PoseStamped()
                pose_stamped_ori = self.msg.poses[id]
                pose_stamped_new.pose.position.x = pose_stamped_ori.pose.position.x
                pose_stamped_new.pose.position.y = pose_stamped_ori.pose.position.y
                pose_stamped_new.pose.position.z = 0
                pose_stamped_new.pose.orientation.x = pose_stamped_ori.pose.orientation.x
                pose_stamped_new.pose.orientation.y = pose_stamped_ori.pose.orientation.y
                pose_stamped_new.pose.orientation.z = pose_stamped_ori.pose.orientation.z
                pose_stamped_new.pose.orientation.w = pose_stamped_ori.pose.orientation.w
                path.poses.append(pose_stamped_new)
            self.path_pub.publish(path)
            self.n_manual += 10
            

    def pub_all_path_sequentially(self):
        print("pub_path: ", self.n1, self.n2)
        path = Path()
        path.header = self.msg.header
        for i in range(self.n1,self.n2):
            pose_stamped_new = PoseStamped()
            pose_stamped_ori = self.msg.poses[i]
            pose_stamped_new.pose.position.x = pose_stamped_ori.pose.position.x
            pose_stamped_new.pose.position.y = pose_stamped_ori.pose.position.y
            pose_stamped_new.pose.position.z = 0
            pose_stamped_new.pose.orientation.x = pose_stamped_ori.pose.orientation.x
            pose_stamped_new.pose.orientation.y = pose_stamped_ori.pose.orientation.y
            pose_stamped_new.pose.orientation.z = pose_stamped_ori.pose.orientation.z
            pose_stamped_new.pose.orientation.w = pose_stamped_ori.pose.orientation.w
            path.poses.append(pose_stamped_new)

        self.n1 = self.n1 + 10; self.n2 = self.n2 + 10
        self.path_pub.publish(path)

if __name__ == '__main__':
    planning = Planning()
    planning.plan()

    