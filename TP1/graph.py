import numpy as np
import copy
import time
from queue import Queue

def read_graph():
    return np.loadtxt("montreal", dtype='i', delimiter=',')

class Solution:
    def __init__(self, places, graph):
        """
        places: a list containing the indices of attractions to visit
        p1 = places[0]
        pm = places[-1]
        """
        self.g = 0 # current cost
        self.graph = graph
        self.visited = [places[0]] # list of already visited attractions
        self.not_visited = copy.deepcopy(places[1:]) # list of attractions not yet visited

    def add(self, idx):
        """
        Adds the point in position idx of not_visited list to the solution
        """
        self.g=self.g+graph[self.visited[-1],self.not_visited[idx]]
        self.visited.append(self.not_visited[idx])
        self.not_visited.remove(self.not_visited[idx])



def bfs(graph, places):
    """
    Returns the best solution which spans over all attractions indicated in 'places'
    """
    #

    queue=[Solution(places,graph)]
    #sol=queue.pop(0)

    #while sol.not_visited:
    #    for  not_visited : sol.not_visited sauf sol.not_visited[-1] # sauf dernier elem de sol.not_visited
    #        queue.append(sol.add(not_visited))
    #print(queue)

    coutFinal = 1000
    solutionFinale = []

    while queue:
        sol=queue.pop(0)
        i=0
        while i<len(sol.not_visited):
            sol1=copy.deepcopy(sol)
            sol1.add(i)
            queue.append(sol1)
            i=i+1

        cout = sol.g
        if len(sol.not_visited)==0 and sol.visited[-1]==places[-1]:
            if sol.g<coutFinal:
                coutFinal=sol.g
                solutionFinale=sol

    return solutionFinale


def fastest_path_estimation(sol):
    """
    Returns the time spent on the fastest path between
    the current vertex c and the ending vertex pm
    """
    c = sol.visited[-1]
    pm = sol.not_visited[-1]

    d = {0:0}
    print (d)
    for i in sol.not_visited:
        d[i]=sol.graph[c,i]

    while sol.not_visited:
        cheapestCost = 1000000
        cheapestNode = None
        for i in sol.not_visited:
            if d[i]<cheapestCost and i!=pm and len(sol.not_visited)>1:
                cheapestCost = d[i]
                cheapestNode = i

            if d[i]<cheapestCost and i==pm and len(sol.not_visited)==1:
                cheapestCost = d[i]
                cheapestNode = i
        print (d)
        print(cheapestNode)
        sol.visited.append(cheapestNode)
        sol.not_visited.remove(cheapestNode)

        for i in sol.not_visited:
           if d[i]>sol.graph[cheapestNode,i]+d[cheapestNode]:
            d[i]=sol.graph[cheapestNode,i]+d[cheapestNode]


    print (d)


graph = read_graph()
places=[0, 5, 13, 16, 6, 9, 4]
sol = Solution(places,graph)
fastest_path_estimation(sol)


#Debut A* algorithme

import heapq

def A_star(graph, places):
    """
    Performs the A* algorithm
    """

    # blank solution
    root = Solution(graph=graph, places=places)
    print(root)
    # search tree T
    T = []
    heapq.heapify(T)
    heapq.heappush(T, root)

    for i in T:
        #current_price =   graph(i, i+ 1) # g(s)
        #estimation = fastest_path_estimation(root)

print('=======================')
A_star(graph, places)

#test 1  --------------  OPT. SOL. = 27
#start_time = time.time()
#places=[0, 5, 13, 16, 6, 9, 4]
#sol = bfs(graph=graph, places=places)
#print(sol.g)
#print(sol.visited)
#print("--- %s seconds ---" % (time.time() - start_time))


#test 2 -------------- OPT. SOL. = 30
#start_time = time.time()
#places=[0, 1, 4, 9, 20, 18, 16, 5, 13, 19]
#sol = bfs(graph=graph, places=places)
#print(sol.g)
#print("--- %s seconds ---" % (time.time() - start_time))


#test 3 -------------- OPT. SOL. = 26
#start_time = time.time()
#places=[0, 2, 7, 13, 11, 16, 15, 7, 9, 8, 4]
#sol = bfs(graph=graph, places=places)
#print(sol.g)
#print("--- %s seconds ---" % (time.time() - start_time))
