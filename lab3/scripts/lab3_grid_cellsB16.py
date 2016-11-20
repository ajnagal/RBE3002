#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
from rdp import rdp
import tf
import numpy
import math 
import rospy, tf, numpy, math
from a_star import astar



# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
    global startPosX
    global goalX
    global goalY
    global goalAX
    global goalAY
    global offsetX
    global offsetY
    global resolution
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    goalAX = (int)((goalX - offsetX - (.5 * resolution)) / resolution)
    goalAY = (int)((goalY - offsetY - (.5 * resolution)) / resolution)
    print "Goal: "
    print goal.pose
    if 'startPosX' in globals():
        star()


def readStart(startPoint):

    global goalX
    global startPosX
    global startPosY
    global startAX
    global startAY
    global offsetX
    global offsetY
    global resolution
    startPosX = startPoint.point.x
    startPosY = startPoint.point.y
    startAX = (int)((startPosX - offsetX - (.5 * resolution))/resolution)
    startAY = (int)((startPosY - offsetY - (.5 * resolution))/resolution)

    print "Start: "
    print startPoint.point
    if 'goalX' in globals():
        star()

def star():
    global mapData
    global startPosX
    global startPosY
    global goalX
    global goalY
    global width
    global height
    # create a new instance of the map
    print "In Star..."
    myMap = []
    myMap = list(mapData)
    print "Map Len: " + str(len(myMap))
    #print temp
    #for j in range(0, height):
    #    temp1 = []
    #    for i in range(0, width):
    #        temp1.append(temp.pop(0))
    #    myMap.append(temp1)
    #    print "Line: " + str(j) + " of " + str(height)
    myMap = numpy.reshape(myMap, (height, width))


    #readGoal(goal)
    #readStart(start)
    myStart = (startAX, startAY)
    myGoal = (goalAX, goalAY)

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py
    print "starting Astar..."
    tup_path = astar(myMap, myStart, myGoal, 20, 1)
    point_path = []
    way_path= []
    if(tup_path):
        # for each node in the path, process the nodes to generate GridCells and Path messages
        for i in range(0, len(tup_path)):
            temp = Point()
            temp.x = tup_path[i][0]
            temp.y = tup_path[i][1]
            temp.z = 0
            point_path.append(temp)


        tup_path = rdp(tup_path, epsilon=0.5)
        for i in range(0, len(tup_path)):
            temp = Point()
            temp.x = tup_path[i][0]
            temp.y = tup_path[i][1]
            temp.z = 0
            way_path.append(temp)
    else:
        print "WARNING: No path found"
    print point_path
    # Publish points
    publishPath(point_path)

    print way_path
    publishWaypoints(way_path)
    print "Astar Complete!"

#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to height of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x = (j * resolution) + offsetX + (.5 * resolution)
                point.y = (i * resolution) + offsetY + (.5 * resolution)
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)           


def publishPath(points):
    global pubpath
    print "Publishing Path"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0,len(points)):
        point=Point()
        point.x = (points[i].x * resolution) + offsetX + (.5 * resolution)
        point.y = (points[i].y * resolution) + offsetY + (.5 * resolution)
        point.z=0
        cells.cells.append(point)
    pubpath.publish(cells)

def publishWaypoints(points):
    global pubway
    print "Publishing Waypoints"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    waypoints = Path()
    waypoints.header.frame_id = 'map'

    for i in range(0,len(points)):
        point=Point()
        point.x = (points[i].x * resolution) + offsetX + (.5 * resolution)
        point.y = (points[i].y * resolution) + offsetY + (.5 * resolution)
        point.z=0
        cells.cells.append(point)
        temp = PoseStamped()
        temp.pose.position.x = point.x
        temp.pose.position.y = point.y
        #temp.pose.orientation.z = 0
        waypoints.poses.append(temp)

    pubway.publish(waypoints)



#Main handler of the project
def run():
    global pub
    global pubpath
    global pubway

    rospy.init_node('lab3')
    rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", "0.15")
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", Path, queue_size=1)
    goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('/lab4_pose', PointStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
