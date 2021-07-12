"""
    Call the move_base/make_plan service for path planning so that the planned path
      is not executed by the move_base.
    Yongming Qin
    2020/11/17



"""


import sys
import rospy


from nav_msgs/GetPlan.srv import GetPlan


if __name__ == "__main__":
    
