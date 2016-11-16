#Originally written for RBE 2002 by Marek Travnikar and Toby Maculoso

import numpy, math
from heapq import *
from scipy.spatial import distance

#Takes the grid (map), start location, goal location,
# values to treat as walls (anything above that value)
# step size (to skip over some adj spaces for faster calc
def astar(grid, start, goal, wall_value, step):
    print("Start is:", start, "goal is", goal)
    grid = numpy.swapaxes(grid,0,1)
    adj = [(0,step),(0,-step),(step,0),(-step,0)]
    fscore = {start:distance(start, goal)}
    gscore = {start: 0}
    pile = []
    visited = set()
    cameFrom = {}
    heappush(pile, (fscore[start], start))
    
    while pile: #While nodes exist in the heap (grid not fully explored
        curNode = heappop(pile)[1]

        if curNode == goal: #If the current node is the goal...
            path = []
            while curNode in cameFrom: #extract the path that lead to it
                path.append(current)
                current = cameFrom[current]
            return path

        visited.add(curNode)
        for i, j in adj:
            neighbor = curNode[0] + i, curNode[1] + j
            AproxGscore = gscore[curNode] + distance(curNode, neighbor)
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
                
            if neighbor in visited and AproxGscore >= gscore.get(neighbor, 0):
                continue #if the neighbor has been visited and the current node costs more to get to
                
            if AproxGscore < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in pile]:
                #If not calculate the neighbors values and add to heap
                visited[neighbor] = curNode
                gscore[neighbor] = AproxGscore
                fscore[neighbor] = AproxGscore + distance(neighbor, goal)
                heappush(pile, (fscore[neighbor], neighbor))
                
    return False #If no path could be found (this can take a while)
