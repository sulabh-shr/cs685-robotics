import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math

class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)

def generateKNeighbors(x, y, k, arrayX, arrayY):
    # x, y = starting point
    # k = number of neighbors
    neighbors = []
    distances = []
    dist = 0
    length = len(arrayX)
    for i in range(length):
        dist = distance(x, y, arrayX[i], arrayY[i])
        distances.append((arrayX[i], arrayY[i], dist))
    distances.sort(key=lambda tup: tup[2])
    for i in range(1, k+1):
        neighbors.append((distances[i][0], distances[i][1]))
    return neighbors

def distance(x1, y1, x2, y2):
    return math.sqrt(((x1-x2)**2)+((y1-y2)**2))

def generate_road_map(x, y, rr, img):
    # x, y = arrays of sample points
    # rr = robot radius (5)
    # generating the roadmap proceeds by going through all samples 
    # for each sample finding k - nearest neighbours for each edge between a sample and it’s neighbours 
    # check whether that edges is in free space if is is append the edge to the graph

    #1000 indices: 1000 lists with each indice attached to a list 
    roadmap = dict()
    neighbors = []
    newNum = len(x)
    check = False
    counter = 0
    # print(newNum)
    for i in range(newNum):
        edge_id = []
        neighbors = generateKNeighbors(x[i], y[i], 10, x, y)
        counter += 1
        # print(x[i], y[i], neighbors, counter)
        #check if theres a collision between the neighbors
        for j in neighbors:
            if isCollision(x[i], y[i], j[0], j[1], 5, img):
                plt.plot([x[i], j[0]], [y[i], j[1]], 'r-')
            else:
                plt.plot([x[i], j[0]], [y[i], j[1]], 'g-')
                edge_id.append((j[0], j[1]))
        roadmap[(x[i], y[i])] = edge_id
    return roadmap

def isCollision(x1, y1, x2, y2, rr, img):

    # r is clearance parameters
    '''
    your code goes here

    check if the two points can be connected by searching the line
    The function should return True if there is collision 
    False if the edge is in the free space. 
    
    '''
    def eqn_line_y(x1, y1, x2, y2):
        m = (y2-y1)/(x2-x1)
        return lambda x: m * (x-x1) + y1
    
    def eqn_line_x(x1, y1, x2, y2):
        m_inv = (x2-x1)/(y2-y1)
        return lambda y: m_inv * (y-y1) + x1

    d = 1
    if abs(x2-x1)>abs(y2-y1):
        if x2 < x1: d = -1
        line_eqn = eqn_line_y(x1, y1, x2, y2)
        for x in range(x1, x2, d):
            y = round(line_eqn(x))
            if img[y,x] == 0:
                return True
    else:
        if y2 < y1: d = -1
        line_eqn = eqn_line_x(x1, y1, x2, y2)       
        for y in range(y1, y2, d):
            x = round(line_eqn(y))
            if img[y, x] == 0:
                return True

    return False

def dijkstra(sx, sy, gx, gy, road_map):
    rx = []
    ry = []
    open_set, closed_set = dict(), dict() # open_set is the unvisited set, closed_set is the visited set
    path_found = True
    start_node = Node(sx, sy, 0.0, (-1, -1))
    goal_node = Node(gx, gy, 0.0, (-1, -1))
    open_set[(sx, sy)] = start_node
    path_found = True
    while True:
        # print(open_set)
        # print("Goal: ", gx, gy)
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        if c_id == (gx, gy):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break
        
        for i in road_map[(current.x, current.y)]:
            tempnode = Node(i[0], i[1], distance(current.x, current.y, i[0], i[1]), (current.x, current.y))
            if (i[0], i[1]) in closed_set:
                continue
            if (i[0], i[1]) not in open_set.keys():
                open_set[(i[0], i[1])] = tempnode
            elif tempnode.cost < open_set[(i[0], i[1])].cost:
                open_set[(i[0], i[1])].cost = tempnode.cost
                open_set[(i[0], i[1])].parent_index = c_id
        del open_set[c_id]
        closed_set[c_id] = current
    
    rx, ry = [], []
    if path_found == False:
        return [], []

    parent_index = goal_node.parent_index
    while parent_index != (-1, -1):
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index
    return rx, ry



def main():


    img = mpimg.imread('uvalda_05.png')
    height, width = img.shape;
    # number of samples
    num = 1000
    x = list((np.random.rand(1, num) * width)[0].astype(int))
    y = list((np.random.rand(1, num) * height)[0].astype(int))
    plt.figure(1)
    plt.imshow(img)

    filteredX = []
    filteredY = []
    newNum = 0

    # find the samples that are in free space 
    for i in range(num):
        if img[y[i]][x[i]] != 0:
            newNum += 1
            filteredX.append(x[i])
            filteredY.append(y[i])
    for i in range(newNum):
        plt.plot(filteredX[i], filteredY[i], 'b.')

    sx = 100
    sy = 50
    gx = 80
    gy = 350
    filteredX.extend([sx, gx])
    filteredY.extend([sy, gy])
    
    # select a start and goal     
    # sx = filteredX[0]
    # sy = filteredY[0]
    # gx = filteredX[len(filteredX) - 1]
    # gy = filteredY[len(filteredY) - 1] 

    plt.plot(sx, sy,  'r*')
    plt.plot(gx, gy,  'r*')
    # plt.show()

    plt.figure(2)
    plt.imshow(img)
    roadmap = generate_road_map(filteredX, filteredY, 5, img)
    rx, ry = dijkstra(sx, sy, gx, gy, roadmap)
    print(rx)
    print(ry)
    for i in range(1, len(rx)):
        plt.plot([rx[i-1], rx[i]], [ry[i-1], ry[i]], 'b-')
    plt.show()
    
if __name__ == "__main__":
    main()
