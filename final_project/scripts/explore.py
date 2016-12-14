#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
import rospy, tf, numpy, math
from find_frontiers import find_frontiers

def mapCallBack(data):
    global mapData
    global pubgoal
    global xPosition
    global yPosition


    myMap = []
    myMap = list(mapData)
    myStart = (xPosition, yPosition)

    goal = find_frontiers(myStart, myMap, 20, 5,1)
    if(goal == False):
        print "done"
    else :
        pubgoal(goal)


#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global theta

    initialTheta = theta
    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentTheta = math.fabs((theta-initialTheta))
        if(currentTheta > 180):
            currentTheta = 360 - currentTheta
        print "angle: ", theta, " curangle: ", currentTheta, " targetangle: ", angle
        if(currentTheta > math.fabs(angle)):
            atTarget = True
            publishTwist(0,0)
        else:
            if(angle > 0):
                publishTwist(0,0.5)
            else:
                publishTwist(0,-0.5)
            rospy.sleep(0.15)

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global xPosition
    global yPosition
    global theta


    odom_list.waitForTransform('/odom', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('/odom','base_footprint', rospy.Time(0)) 

    xPosition = position[0]
    yPosition = position[1]
    quat = orientation
    q = [quat[0], quat[1], quat[2], quat[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    theta = math.degrees(yaw)

def publishTwist(linearVelocity, angularVelocity):
    global pub

    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)

#Main handler of the project
def run():
    global xPosition
    global yPosition
    global theta
    global pub
    global pubgoal
    global pubpath
    global pubway


    xPosition = 0
    yPosition = 0
    theta = 0

    rospy.init_node('lab3')
    rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", "0.15")
    
    pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size = 1) # Publisher for commanding robot motion 
    pubgoal = rospy.Publisher("/clicked_point", Point, queue_size=1) # you can use other types if desired
    #start_sub = rospy.Subscriber('/lab4_pose', Point, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)


    rospy.Timer(rospy.Duration(.1), timerCallback)
    rotate(360);
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)  
        print("Complete")

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass