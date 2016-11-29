import rospy


def timerCallback(event):
    global pubpose

    Point robotPos;
    odom_list.waitForTransform('/odom', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('/odom','base_footprint', rospy.Time(0)) 

    xPosition = position[0]
    yPosition = position[1]
    quat = orientation
    q = [quat[0], quat[1], quat[2], quat[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    theta = math.degrees(yaw)
    robotPos.x = xPosition
    robotPos.y = yPosition
    pubpose.publish(robotPos)

#Main handler of the project
def run():

    global pubpose

    pubpose = rospy.Publisher("/lab4_pose", Point)


    # wait a second for publisher, subscribers, and TF
    rospy.Timer(rospy.Duration(.01), timerCallback)
    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2) 
        print("Complete")

# This is the program's main function
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass