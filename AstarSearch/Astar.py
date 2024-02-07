#--------------------------------------------------------
# A* Path Planning Algorithm Implemntation
#
# Author - Anubhav Singh
#--------------------------------------------------------
import csv
from collections import defaultdict
import numpy as np

class Astar:

    def __init__(self):
        self.graph = defaultdict(list)
        self.heuristic_cost_to_go = defaultdict(list)
        self.v = 0    # number of nodes/vertices
        self.end = -1 # known from 'nodes.csv'
    
    def addEdge(self, u, v, w):
        """Add edges to the graph.
        :param u: Node ID i
        :param w: Node ID j
        :param w: the cost of traversing that edge (in either direction)"""
        self.graph[u].append((v, w))
        self.graph[v].append((u, w))

    def readNodesCSV(self, filename):
        """Read nodes.csv file and finds number of nodes, end node ID 
           and fills heuristic_cost_to_go.
        :param filename: path to the nodes.csv file"""
        with open(filename, mode='r') as file:
            csvFile = csv.reader(file)
            for lines in csvFile:
                if str.__contains__(lines[0], '#') == False:
                    self.v = self.v + 1
                    self.heuristic_cost_to_go[int(lines[0])].append(float(lines[3]))
                    
                    # find the end node
                    if float(lines[3]) == 0.0:
                        self.end = int(lines[0])

    def readEdgesCSV(self, filename):
        """Read edges.csv file and add edges to the graph
        :param filename: path to the edges.csv file"""
        with open(filename, mode='r') as file:
            csvFile = csv.reader(file)
            for lines in csvFile:
                if str.__contains__(lines[0], '#') == False:
                    self.addEdge(int(lines[0]), int(lines[1]), float(lines[2]))
    
    def writeToCSV(self, filename, path):
        """Write A* solution path to 'path.csv'
        :param filename: path to the path.csv
        :param path: solution path from start node ID to end node ID (list)"""
        with open(filename, mode='w') as file:
            csvFile = csv.writer(file)
            csvFile.writerow(path)
    
    def callAstar(self, start):
        """A* Path Planning Alogithm
        :param start: start node ID"""

        past_cost = [float('inf')]*self.v
        past_cost[start - 1] = 0

        parent = [-1]*self.v

        est_total_cost = [float('inf')]*self.v
        est_total_cost[start - 1] = past_cost[start - 1] + self.heuristic_cost_to_go[start][0]

        OPEN = []
        # second elem is important since we have to sort OPEN based on est_total_cost
        # In case, sorted list was not required we could have appended only nodes ID, 
        # not est_total_cost
        OPEN.append((start, est_total_cost[start - 1])) 
        #OPEN.append(start)

        CLOSED = []
       
        while len(OPEN) != 0:
            current = OPEN.pop(0)
            CLOSED.append(current[0])
            
            if current[0] == self.end:
                print("SHORTEST PATH FOUND")
                # "path from start -> end"
                return self.printShortestPath(parent, self.end)

            for nbr in self.graph[current[0]]:
                #print(nbr)
                if nbr[0] not in CLOSED:
                    tentative_past_cost = past_cost[current[0] - 1] + nbr[1]
                    if tentative_past_cost < past_cost[nbr[0] - 1]:
                        past_cost[nbr[0] - 1] = tentative_past_cost
                        parent[nbr[0] - 1] = current[0]
                        est_total_cost[nbr[0] - 1] = past_cost[nbr[0] - 1] + self.heuristic_cost_to_go[nbr[0]][0]
                        OPEN.append((nbr[0], est_total_cost[nbr[0] - 1])) 
            
            OPEN = sorted(OPEN, key=lambda x: x[1]) 
        return False

    def printShortestPath(self, parent, goalID):
        """Print shortest path from start node to goal node (goal node is not 
           necessarily the end node, it can be an intermediate node as well)
        :param parent: list of parents of all nodes
        :param goalID: goal node ID
        """
        if parent[goalID - 1] == -1:
            return str(goalID)
        return self.printShortestPath(parent, parent[goalID - 1]) + "," + str(goalID) 

    def printGraph(self, key):
        """Print data from nodes.csv and edges.csv
        :param key: 'nodes' | 'edges'
        """
        if key == 'nodes':
            for elem in self.heuristic_cost_to_go:
                print(elem, self.heuristic_cost_to_go[elem][0])
        else: # 'edges'
            for elem in self.graph:
                print(elem, self.graph[elem])
                #for subelem in self.graph[elem]:
                #    print("["+str(elem) + " - " + str(subelem[0]) + "] ==> " + str(subelem[1]))
            

if __name__ == '__main__':
    astar = Astar()
    astar.readNodesCSV('data/nodes.csv')
    astar.readEdgesCSV('data/edges.csv')
    #astar.printGraph('nodes')
    #astar.printGraph('edges')
    #print(50*'#')

    start = 1 # start node
    path = astar.callAstar(start)
    list_of_nodes_IDs_in_path = list(map(int, path.split(",")))
    print(list_of_nodes_IDs_in_path)

    astar.writeToCSV("output/path.csv", list_of_nodes_IDs_in_path)


