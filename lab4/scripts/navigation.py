import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from tf.transformations import euler_from_quaternion



def timerCallback(event):
    global pubpose
    global odom_list

    robotPos = Point()
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
    print robotPos
    pubpose.publish(robotPos)
    print "wheeeeee"

def sendPoint(path):
    global pubnext
    next = PointStamped()
    if(len(path.poses) == 0):
        print "no path"
        next.header.frame_id = 'map'
        next.point.x = 0
        next.point.y = 0
    else:
        next.header.frame_id = 'map'
        next.point.x = path.poses[(int)(len(path.poses)*3/4)-1].pose.position.x
        next.point.y = path.poses[(int)(len(path.poses)*3/4)-1].pose.position.y
    print "sent"
    pubnext.publish(next)

#Main handler of the project
def run():

    global pubpose
    global pubnext
    global pathsub

    pubpose = rospy.Publisher('/lab4_pose', Point, queue_size=1)
    pubnext = rospy.Publisher('/next_goal', PointStamped, queue_size=1)
    pathsub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, sendPoint)

    # wait a second for publisher, subscribers, and TF
    rospy.Timer(rospy.Duration(1), timerCallback)
    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)
        print("Complete")

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('Group1')
    global odom_list
    odom_list = tf.TransformListener()
    try:
        run()
    except rospy.ROSInterruptException:
        pass