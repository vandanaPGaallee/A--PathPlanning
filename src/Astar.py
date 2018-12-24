#!/usr/bin/env python
import numpy as np 
import math
import operator 
import rospy


r = 20
c = 18
s_x, s_y = -2, -8
g_x, g_y = 0,0
source = [r/2  - math.floor(s_x) - 1, math.floor(s_y) + c/2 - 1]
goal = []
open_list = []
hgrid = np.zeros((r,c))
adj_list = [(1,0),(-1,0),(1,1),(-1,-1),(0,1),(0,-1),(1,-1),(-1,1)]
grid = [[0 for j in range(c)]for i in range(r)]
results = []
closed_list = []
def readFile():
    global grid
    f = open("/home/first/catkin_ws/src/lab5/src/map.txt", "r")
    lines = f.readlines()
    for i, line in enumerate(lines):
        k = 0
        for ch in line:
            if(ch.isdigit()):
                grid[i][k] = ch
                k += 1
    # print(grid)
    # print(len(grid), type(grid[0][0]))

def createDict(p, s, g, h):
    dic = {
        "point":p,
        "parent":s,
        "hcost":h,
        "gcost":g,
        "fcost":g + h
    }
    return dic

def find_min():
    global open_list
    seq = [x['fcost'] for x in open_list]
    m = np.argmin(seq)
    # print("*************")
    # # print("fcost = ",seq)
    # print("min of cost = " ,np.min(seq))
    # print("min point =" ,open_list[m]["point"])
    return open_list[m], m

def find_hcost():
    global r,c, hgrid, goal
    for i in range(r):
        for j in range(c):
            hgrid[i,j] = math.sqrt((i - goal[0])**2 + (j - goal[1])**2)

def FindIfVisited(point, gc, src):
    global open_list
    for dic in open_list:
        if point == dic["point"]:
            if dic["gcost"] <= gc:
                return True
            else:
                dic["gcost"] = gc
                dic["fcost"] = gc + dic["hcost"]
                dic["parent"] = src
                return True
    return False

def CheckClosedList(pt):
    global closed_list
    for dic in closed_list:
        if dic["point"] == pt :
            return False
    return True

def AddToOpenList(src):
    global hgrid, adj_list, c, r, grid, goal
    x,y = src["point"][0],src["point"][1]
    # print(x,y)
    for point in adj_list:
        # print(point)
        if(0 <= (x + point[0]) < r and 0 <= y + point[1] < c):
            if((grid[int(x + point[0])][int(y + point[1])] != "1" and CheckClosedList([x + point[0], y + point[1]])) or ([x + point[0], y + point[1]] == goal)):
                gc = src["gcost"] + math.sqrt((point[0])**2 + (point[1])**2)
                checker = FindIfVisited([x + point[0], y + point[1]], gc, src)
                if(not checker):
                    # print([x + point[0], y + point[1]])
                    open_list.append(createDict([x + point[0], y + point[1]], src, hgrid[int(x + point[0]), int(y + point[1])], gc))
    # print("*****************")

def TracePath(target):
    global source, results, grid
    grid[int(target["point"][0])][int(target["point"][1])] = "T"
    while(target["point"] != source):
        # print(r/2 - target["point"][0], -c/2 + target["point"][1] + 1)
        results = [(-c/2 + target["point"][1] + 1, r/2 - target["point"][0] - 1)] + results
        target = target["parent"]
        grid[int(target["point"][0])][int(target["point"][1])] = "X"
    grid[int(target["point"][0])][int(target["point"][1])] = "S"
    # results = [(-c/2 + target["point"][1] + 1, r/2 - target["point"][0] - 1)] + results
    # print(r/2 - target["point"][0], -c/2 + target["point"][1] + 1)
    
def A_algo():
    global open_list,goal, closed_list
    open_list.append(createDict(source, None, 0, 0))
    # while(len(open_list) < 2):
    k = 0
    while(k<60):
        k += 1
        node_reached,min_index = find_min()
        # print(open_list[min_index])
        # print(min_index)
        closed_list.append(open_list[min_index])
        del open_list[min_index]
        # print(len(open_list))
        AddToOpenList(node_reached)
        # print(len(open_list))
        # print("----------------------------------")
        if(node_reached["point"] == goal):
            print("Goal Reached")
            TracePath(node_reached)
            break

def AstarMain():
    readFile()
    find_hcost()
    A_algo()
    print("Shortest Path from Source to Destination")
    print(results)
    print("\n")
    print("Path Trace in Grid - S denotes source, T denotes target, X is the trail")
    print("\n")
    for row in grid:
        print(row)
    return results

def init(x,y):
    global g_x,g_y,goal,r,c,source
    g_x, g_y = y,x
    goal = [r/2 - math.floor(g_x) - 1, math.floor(g_y) + c/2 - 1]
    print(source,goal)

if __name__ == "__main__":
    rospy.init_node('lab5', anonymous=True)
    readFile()
    find_hcost()
    A_algo()
    print(results)
    for row in grid:
        print(row)
    rospy.spin()