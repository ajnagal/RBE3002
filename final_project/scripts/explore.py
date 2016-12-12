#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
import rospy, tf, numpy, math

#Main handler of the project
def run():
    global pub
    global pubpath
    global pubway

    rospy.init_node('lab3')
    rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", "0.15")
    
   # sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/move_base/NavfnROS/plan", Path, queue_size=1)
    #goal_sub = rospy.Subscriber('/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    #start_sub = rospy.Subscriber('/lab4_pose', Point, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)  
        print("Complete")

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass