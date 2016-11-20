import rospy

def pointCallBack(path):
    global nextPoint
    print path.poses
    nextPoint = path.poses[0].pose.position

def curCallBack(point):
    global curPoint
    curPoint = point

#Main handler of the project
def run():
    global nextPoint
    global pubgoal
    global pubpose
    global curPoint

    subcur = rospy.Subscriber("/lab4_cur", Point, curCallBack)
    subway = rospy.Subscriber("/waypoints", Path, pointCallBack)
    pubpose = rospy.Publisher("/lab4_pose", Point)


    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        pubpose.publish(curPoint) #publishing map data every 2 seconds
        rospy.sleep(2) 
        print("Complete")

# This is the program's main function
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass