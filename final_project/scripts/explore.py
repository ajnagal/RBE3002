#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
import rospy, tf, numpy, math
from heapq import *

def mapCallBack(data):
    global pubpath
    global pubgoal
    goal = find_frontiers()
    if(goal == False):
        print "done"
    else :
        pubgoal(goal)


def findFrontiers(start, grid, wall_value, unknown_value):
    print("Start is:", start)
    grid = numpy.swapaxes(grid,0,1)
    adj = [(0,step),(step, step),(0,-step),(step, -step),(step,0),(-step, step), (-step,0),(-step, -step)]
    gscore = {start: 0}
    pile = []
    visited = set()
    cameFrom = {}
    heappush(pile, (0, start))
    
    while pile: #While nodes exist in the heap (grid not fully explored
        curNode = heappop(pile)[1]

        if grid[curNode[0], curNode[1]] >= unknown_value : #If the current node is the goal...
            return curNode

        visited.add(curNode)
        for i, j in adj:
            neighbor = curNode[0] + i, curNode[1] + j
            AproxGscore = gscore[curNode] + dist(curNode, neighbor)
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    #print(array[neighbor[0]][neighbor[1]])
                    #if array[neighbor[0]][neighbor[1]] == wall_value:
                    if grid[neighbor[0]][neighbor[1]] >= wall_value or grid[neighbor[0]][neighbor[1]] == -1:
                        continue
                else:
                    # Escape if currNode is at a y wall
                    continue
            else:
                # Escape if the currNode is at an x wall
                continue
                
            if neighbor in visited:
                continue
                
            if AproxGscore < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in pile]:
                #If not calculate the neighbors values and add to heap
                cameFrom[neighbor] = curNode
                gscore[neighbor] = AproxGscore
                fscore[neighbor] = AproxGscore
                heappush(pile, (fscore[neighbor], neighbor))
                
    return False #If no path could be found (this can take a while)



#Main handler of the project
def run():
    global pub
    global pubgoal
    global pubpath
    global pubway

    rospy.init_node('lab3')
    rospy.set_param("/move_base/global_costmap/inflation_layer/inflation_radius", "0.15")
    
    sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubgoal = rospy.Publisher("/clicked_point", Point, queue_size=1) # you can use other types if desired
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