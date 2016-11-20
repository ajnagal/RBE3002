import rospy

def pointCallBack(path):
    global nextPoint
    print path.poses
    nextPoint = path.poses[0]

#Main handler of the project
def run():
    global nextPoint


    subway = rospy.Subscriber("/waypoints", Path, pointCallBack)
    pubgoal = rospy.Publisher("/lab4_goal", Point_Stamped)
    pubpose = rospy.Publisher("/lab4_pose", Point_Stamped)

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        pubpose.publish(nextPoint) #publishing map data every 2 seconds
        rospy.sleep(1) 
        pubgoal.publish(nextPoint)
        print("Complete")

# This is the program's main function
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass