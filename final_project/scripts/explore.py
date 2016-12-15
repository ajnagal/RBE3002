#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from kobuki_msgs.msg import BumperEvent
import rospy, tf, numpy, math
from find_frontiers import find_frontiers
from tf.transformations import euler_from_quaternion

def updateGrid(dataPack):
    global mapData
    global width
    global height
    global pubtest
    global info

    x = dataPack.x
    y = dataPack.y
    upWid = dataPack.width
    upHei = dataPack.height
    temp = list(mapData)
    for j in range(0, upWid):
        for k in range(0, upHei):
            bi = (j+y)*width+(k+x)
            ui = j*upWid + k
            if (bi >= len(temp)):
                #print("bi too big by ", bi - len(temp))
                continue
                #bi = len(temp)-1
            if (ui >= len(dataPack.data)):
                #print("ui too big by ", ui - len(dataPack.data))
                continue
                #ui = len(dataPack.data)-1
            temp[bi] = dataPack.data[ui]

    mapData = tuple(temp)
    tempwhee = OccupancyGrid()
    tempwhee.info = info
    tempwhee.data = mapData
    pubtest.publish(tempwhee)

    print ("Updated!")
    #print "Len is: " + str(len(mapData))
    #for whee in mapData:
    #   if not whee == -1:
    #       print "FOUND!: " + str(whee)
    #print "None found!!!!!! :("

# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global info
    global offsetX
    global offsetY

    mapgrid = data
    info = data.info
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    #print "Len is: " + str(len(mapData))
    #for whee in mapData:
    #    if not whee == -1:
    #        print "FOUND!: " + str(whee)
    #print "None found!!!!!! :("


    #print data.info

def explore():
    global theta
    global mapData
    global pubgoal
    global pubdisp
    global xPosition
    global yPosition
    global width
    global height
    global done
    global pubtest
    global offsetX
    global offsetY

    myMap = list(mapData)
    myMap = numpy.reshape(myMap, (height, width))

    startAX = (int)((xPosition - offsetX - (.5 * resolution)) / resolution)
    startAY = (int)((yPosition - offsetY - (.5 * resolution)) / resolution)
    myStart = (startAX, startAY)

    if(myMap[myStart[0]][myStart[1]] > 20):
        radius = 3;
        for j in range(0, 2*radius):
            for k in range(0, 2*radius):
                myMap[myStart[0]+j-radius][myStart[1]+k-radius] = 0

    goal = find_frontiers(myStart, myMap, 20, 5, 1)
    if(goal == False):
        print "done"
        done = True;
    else :
        output = PoseStamped()
        output.header.frame_id = 'map'
        output.pose.position.x = (goal[0] * resolution) + offsetX + (.5 * resolution)
        output.pose.position.y = (goal[1] * resolution) + offsetY + (.5 * resolution)
        output.pose.orientation.w = 1
        print "publishing goal"
        pubgoal.publish(output)


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
            print "done"
            atTarget = True
            publishTwist(0,0)
        else:
            print "spin"
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
    global odom_list


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
    global pubdisp
    global pubpath
    global pubway
    global pubtest
    global odom_list
    global done

    done = False;
    xPosition = 0
    yPosition = 0
    theta = 0

    rospy.init_node('lab3')
    #rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", "0.15")

    # Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()

    pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size = 1) # Publisher for commanding robot motion 
    pubgoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pubdisp = rospy.Publisher("/goal", PoseStamped, queue_size=1)
    pubtest = rospy.Publisher("/testmap", OccupancyGrid, queue_size=1)
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    subUp = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, updateGrid)
    #start_sub = rospy.Subscriber('/lab4_pose', Point, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)


    rospy.Timer(rospy.Duration(.1), timerCallback)
    rotate(90)
    rotate(90)
    rotate(90)
    rotate(90)

    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)
        if done:
            print "Complete"
        else:
            print "Running"
            explore()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass