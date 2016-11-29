#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion



#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global ddist
    global da
    dx = goal.point.x
    dy = goal.point.y
    print "dx ", dx
    print "dy ", dy

    da = math.degrees(math.atan2(dy,dx))


    ddist = math.sqrt(dx**2+dy**2)


def continousNav():
    global ddist
    global da

    if(abs(da) > 3.1415/8):
        print "spin!" , da
        if da > 0:
            publishTwist(0,.2)
        else:
            publishTwist(0,-.2)
    elif(ddist > 0):
        publishTwist(.2,0)
        print "move!", ddist
    else:
        publishTwist(0,0)


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.6,.25)
    rotate(-90)
    driveStraight(.45,.25)
    rotate(135)


def publishTwist(linearVelocity, angularVelocity):
    global pub

    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = (u1 + u2)/2
    ang_vel = (u1 - u2)/.23
    twist_msg = Twist()
    stop_msg = Twist()

    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    now = rospy.Time.now().secs
    while(rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(distance, speed):
    global xPosition
    global yPosition
    global b_stop

    initialX = xPosition
    initialY = yPosition
    atTarget = False
    while (not atTarget and not rospy.is_shutdown() and not b_stop):
        currentX = xPosition
        currentY = yPosition
        currentDistance = math.sqrt((currentX-initialX)**2 + (currentY-initialY)**2)
        print "distance: ", currentDistance, " target: ", distance
        if(currentDistance > distance):
            atTarget = True
            publishTwist(0,0)
        else:
            if(currentDistance < 0.1):
                publishTwist(speed*(currentDistance*7.5+.25),0)
            elif((distance - currentDistance) < 0.1 and (distance - currentDistance) > 0):
                publishTwist(speed*((distance - currentDistance)*7.5+.25),0)
            else: 
                publishTwist(speed,0)
                rospy.sleep(0.15)



    
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



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global theta

    initialTheta = theta
    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentTheta = math.fabs((theta-initialTheta))
        if(currentTheta > 180):
            currentTheta = 360 - currentTheta
        print "angle: ", theta, " curangle: ", currentTheta
        if(currentTheta > math.fabs(angle)):
            atTarget = True
            publishTwist(0,0)
        else:
            if(angle > 0):
                publishTwist(0,0.5)
            else:
                publishTwist(0,-0.5)
            rospy.sleep(0.15)




#Bumper Event Callback function
def readBumper(msg):
    global b_stop
    if (msg.state == 1):
        b_stop = 1
        print "Ouch!"
    else:
        b_stop = 0
    #if (msg.state == 1):
        #spinWheels(-.1,-.25,2)
        # navToPose([0,.2,0])
        #rospy.sleep(1)
        #navToPose([.2,.2])
        #rospy.sleep(1)
        # executeTrajectory()
        #print "x: ", xPosition, " y: ", yPosition , " theta: ", theta
        



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
    #print [xPosition, yPosition, theta]


def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)

def odomCallback(data):
    global xPosition
    global yPosition
    global theta


    xPosition = data.pose.pose.position.x
    yPosition = data.pose.position.y
    quat = data.pose.oriention
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    theta = math.degrees(yaw)
    curPoint = Point()
    curPoint.x = xPosition
    curPoint.y = yPosition
    pubcur(curPoint)



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('lab2')
    
    global pub
    global xPosition
    global yPosition
    global theta
    global odom_tf
    global odom_list
    global b_stop
    global da
    global ddist

    b_stop = 0
    da = 0
    ddist = 0
    xPosition = 0
    yPosition = 0
    theta = 0
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size = 1) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    pubcur = rospy.Publisher('/lab4_cur', PointStamped, queue_size=1)

    position_sub = rospy.Subscriber('/lab4_goal', PointStamped, navToPose)
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 2"

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(.01), timerCallback)
    while (1 and not rospy.is_shutdown()):
        continousNav()
        rospy.sleep(.1) 
        print("Complete")

