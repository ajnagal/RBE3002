
import numpy, math
from heapq import *
from geometry_msgs.msg import Twist, Point, PointStamped

def dist(cur, end):
    dx = cur[0] - end[0]
    dy = cur[1] - end[1]
    return math.sqrt(dx**2 + dy**2)

def find_frontiers(start, grid, wall_value, unknown_value, step):
    grid = numpy.swapaxes(grid,0,1)
    print "start is: ", start
    print "start val: ", grid[start[0]][start[1]]
    adj = [(0,step),(step, step),(0,-step),(step, -step),(step,0),(-step, step), (-step,0),(-step, -step)]
    gscore = {start: 0}
    pile = []
    visited = set()
    cameFrom = {}
    heappush(pile, (0, start))
    
    while pile: #While nodes exist in the heap (grid not fully explored
        curNode = heappop(pile)[1]



        visited.add(curNode)
        for i, j in adj:
            neighbor = curNode[0] + i, curNode[1] + j
            AproxGscore = gscore[curNode] + dist(curNode, neighbor)
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    #print(array[neighbor[0]][neighbor[1]])
                    #if array[neighbor[0]][neighbor[1]] == wall_value:
                    if grid[neighbor[0]][neighbor[1]] >= wall_value:
                        continue
                    elif grid[neighbor[0]][neighbor[1]] == -1:
                        print "goal is: ", curNode
                        print "neigh val: ", grid[neighbor[0]][neighbor[1]]
                        return curNode
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
                heappush(pile, (gscore[neighbor], neighbor))
                
    return False #If no path could be found (this can take a while)