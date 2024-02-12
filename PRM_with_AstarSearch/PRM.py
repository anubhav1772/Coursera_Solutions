# -*- coding: utf-8 -*-
import csv
import numpy as np
import random
from collections import defaultdict
import matplotlib.pyplot as plt

"""The Probabilistic Roadmap (PRM) Sampling-Based Planning
"""

plt.xlim( -0.8 , 0.8 )
plt.ylim( -0.8 , 0.8 )

class Obstacle:
	""" Obstacle class to define (x,y) coordinates and radius of each obstacle
	"""
	def __init__(self, x, y, radius):
		self.x = x
		self.y = y
		self.radius = radius

class Graph:
	""" Graph class to define edge between two nodes/vertices
		The graph is undirected and weighted.
	"""
	def __init__(self):
		self.node = defaultdict(list)

	def addEdge(self, u, v, w):
		"""Add edges to the undirected graph.
		:param u: Node ID i
		:param w: Node ID j
		:param w: the cost of traversing that edge (in either direction)"""
		self.node[u].append((v, w))
		self.node[v].append((u, w))

class Astar:
	""" A* Path Planning Algorithm Implementation
	"""
	def __init__(self):
		self.roadmap = Graph()
		self.heuristic_cost_to_go = defaultdict(list)
		self.v = 0    # number of nodes/vertices

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

	def readEdgesCSV(self, filename):
		"""Read edges.csv file and add edges to the graph
		:param filename: path to the edges.csv file"""
		with open(filename, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					self.roadmap.addEdge(int(lines[0]), int(lines[1]), float(lines[2]))
	
	def writeToCSV(self, filename, path):
		"""Write A* solution path to 'path.csv'
		:param filename: path to the path.csv
		:param path: solution path from start node ID to end node ID (list)"""
		with open(filename, mode='w') as file:
			csvFile = csv.writer(file)
			csvFile.writerow(path)
	
	def callAstar(self, start, goal):
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
			
			if current[0] == goal:
				print("SHORTEST PATH FOUND")
				# "path from start -> goal"
				return self.printShortestPath(parent, goal)

			for nbr in self.roadmap.node[current[0]]:
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

class PRM:
	"""PRM Sample-Based Planner Algorithm Implementation
	"""
	def __init__(self, radius):
		self.obstacles = []
		self.robot_radius = radius
		self.samples = []
		self.start = (-0.5, -0.5) 	# START NODE
		self.goal = (0.5, 0.5)    	# END NODE
		self.COORDS_TO_ID = {}		# COORDS -> NODE ID (edges.csv)
		#self.roadmap = Graph()

	def readObstaclesCSV(self, filepath="../Input/obstacles.csv"):
		"""Read obstacles' position and diameter from'obstacles.csv'
		:param filename: path to the obstacles.csv"""
		with open(filepath, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					self.obstacles.append(Obstacle(float(lines[0]), float(lines[1]), float(lines[2])/2.0 + self.robot_radius))
					self.addCircleToGrid(float(lines[0]), float(lines[1]), float(lines[2])/2.0)
					plt.text(float(lines[0]), float(lines[1]), str((float(lines[0]), float(lines[1]), float(lines[2])/2.0)))

	def addCircleToGrid(self, x, y, r, color='black'):
		"""Plot a circle using Matplotlib
		"""
		circle = plt.Circle((x, y), r, color=color, alpha=0.5)
		plt.gca().add_patch(circle)

	def addStartAndGoalStateToGrid(self):
		"""Add start and goal nodes on pyplot grid (radius=0.01)
		"""
		self.addCircleToGrid(float(self.start[0]), float(self.start[1]), 0.001)
		plt.text(float(self.start[0]), float(self.start[1]), 'Start')

		self.addCircleToGrid(float(self.goal[0]), float(self.goal[1]), 0.001, color='orange')
		plt.text(float(self.goal[0]), float(self.goal[1]), 'Goal')

	def IsInFreeSpace(self, x, y):
		"""Check if a random sampled node lies in free space or not
		"""
		for obstacle in self.obstacles:
			x1 = obstacle.x
			y1 = obstacle.y
			r = obstacle.radius
			cond = (x - x1)*(x - x1) + (y - y1)*(y - y1) - r*r
			if(cond <= 0): # condition for a point to lie inside|on the circle (intersecting with obstacle)
				return False
		return True

	def generate_random_samples(self, N):
		""" Generate N random samples within the square [-0.5, 0.5] x [-0.5, 0.5]
		"""
		i = 0
		nodes = [] 		   # content of nodes.csv

		# START NODE
		nodes.append({'ID': '1', 'x': str(self.start[0]), 'y': str(self.start[1]), 'heuristic-cost-to-go': str(self.euclideanDistance(self.start, self.goal))})
		self.COORDS_TO_ID[self.start] = 1

		# Generating N nodes in free configuration space
		while(i<N):
			x = round(random.uniform(-0.5, 0.5), 1)
			y = round(random.uniform(-0.5, 0.5), 1)
			if(((x, y) not in self.samples) and (self.IsInFreeSpace(x,y))):
				self.samples.append((x, y))
				# ID of START node is 1 and GOAL node is N+2
				nodes.append({'ID': str(i + 2), 'x': str(x), 'y': str(y), 'heuristic-cost-to-go': str(self.euclideanDistance((x, y), self.goal))})
				self.COORDS_TO_ID[(x, y)] = i + 2
				self.addCircleToGrid(x, y, 0.001, color='r')
				i += 1

		# GOAL NODE
		nodes.append({'ID': str(N + 2), 'x': str(self.goal[0]), 'y': str(self.goal[1]), 'heuristic-cost-to-go': '0'}) 
		self.COORDS_TO_ID[self.goal] = N + 2
		# print(nodes)
		self.writeNodesToCSV(nodes)
		# Add start and Goal state to the grid as well
		self.addStartAndGoalStateToGrid()
		
	def writeNodesToCSV(self, nodes, filepath='../Outputs/nodes.csv'):
		"""Write nodes' details to 'nodes.csv'
		:param nodes: list of nodes(ID, x, y, heuristic-cost-to-go)
		:param filename: path to the nodes.csv"""
		fields = ['ID', 'x', 'y', 'heuristic-cost-to-go']
		with open(filepath, 'w', newline="") as csvfile:
			writer = csv.DictWriter(csvfile, fieldnames=fields)
			writer.writerows(nodes)

	def writeEdgesToCSV(self, edges, filepath='../Outputs/edges.csv'):
		"""Write edges' details to 'edges.csv'
		:param edges: list of edges(ID1, ID2, cost)
		:param filename: path to the edges.csv"""
		fields = ['ID1', 'ID2', 'cost']
		with open(filepath, 'w', newline="") as csvfile:
			writer = csv.DictWriter(csvfile, fieldnames=fields)
			writer.writerows(edges)

	def euclideanDistance(self, pt1, pt2):
		"""Calculate Euclidean Distance between two nodes/vertices
		   and round it to 5 decimal places
		"""
		return round(np.linalg.norm(np.array(pt1) - np.array(pt2)), 5)

	def findKNearestNeigbors(self, pt, K):
		"""Find K Nearest Neighbors
		:param pt: Node's (x,y) coordinate whose nearest neighbors are to be found
		:param K:  Number of nearest neighbors
		:return list of K nearest neighbors [(x1, y1), (x2, y2), (x3, y3), ...]
		"""
		# self.COORDS_TO_ID[pt] != self.COORDS_TO_ID[point] for ignoring pt == point since 
		# 
		neighbors = [(point, self.euclideanDistance(pt, point)) for point in self.samples if self.COORDS_TO_ID[pt] != self.COORDS_TO_ID[point]]
		neighbors.sort(key = lambda x: x[1]) # sort neighbors list based on euclidean distances 
		return [nbr[0] for nbr in neighbors[:K]]

	def isIntersecting(self, pt1, pt2):
		"""Function to check if the straight path between pt1 and pt2 
		   is in collision with any of the obstacles
		:param pt1: (x, y) of point 1
		:param pt2: (x, y) of point 2
		:return True (in case of collision) | False (No collision)
		"""
		# Intersection of a line(not line segment yet) with the circle
		if (pt2[0] != pt1[0]):
			m = (pt2[1] - pt1[1])/(pt2[0] - pt1[0])
			const = pt1[1] - m*pt1[0] # y = mx + const => const = (y-mx) at pt1||pt2 
		else:
			m = np.inf
			const = np.nan

		for obstacle in self.obstacles:
			x1 = obstacle.x
			y1 = obstacle.y
			r = obstacle.radius
			a = (1 + m*m)
			b = (2*m*const - 2*x1 - 2*y1*m)
			c = (x1*x1 + const*const + y1*y1 - 2*y1*const - r*r)

			discriminant = b*b - 4*a*c

			#print("a = "+str(a)+",b = "+str(b)+",c = "+str(c)+", discriminant = "+str(discriminant))

			# Check if discriminant is non-negative
			if discriminant>=0:
				# intersection points
				x1 = (-b + np.sqrt(discriminant))/float(2*a)
				y1 = m*x1 + const

				x2 = (-b - np.sqrt(discriminant))/float(2*a)
				y2 = m*x2 + const

				# Check if the intersection points lie are within the line segment(not line)
				if min(pt1[0], pt2[0]) <= x1 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y1 <= max(pt1[1], pt2[1]):
					return True

				if min(pt1[0], pt2[0]) <= x2 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y2 <= max(pt1[1], pt2[1]):
					return True
		return False

	def runPRM(self, N, K):
		# STEP 2.1 : GENERATE SAMPLES FROM FREE CONFIGURATION SPACE
		self.generate_random_samples(N)
		edges = []
		# STEP 2.1 : LOOP THROUGH THE GENERATED SAMPLE NODES
		for pt in self.samples:
			pt_ID = self.COORDS_TO_ID[pt]	# (x, y) -> NODE ID
			# STEP 2.2 : FIND K NEAREST NEIGHBORS OF THE NODE
			neighbors = self.findKNearestNeigbors(pt, K)
			#print(str(pt)+" "+str(neighbors))

			# STEP 2.3 : CHECK FOR COLLISION-FREE PATH
			for nbr in neighbors:
				if not self.isIntersecting(pt, nbr):
					#self.roadmap.addEdge(pt_ID, self.COORDS_TO_ID[nbr]))
					#print(str(pt), " ", str(pt_ID), " ==> ", str(nbr), " ", str(self.COORDS_TO_ID[nbr]))
					edges.append({'ID1': str(pt_ID), 'ID2': str(self.COORDS_TO_ID[nbr]), 'cost': str(self.euclideanDistance(pt, nbr))})
					plt.plot([pt[0], nbr[0]], [pt[1], nbr[1]], color='blue')
				else:
					print(str(pt), " ", str(pt_ID), " ==> ", str(nbr), " ", str(self.COORDS_TO_ID[nbr]))
					# PATH FROM pt AND nbr IS COLLIDING WITH ONE OF THE OBSTACLES 
					plt.plot([pt[0], nbr[0]], [pt[1], nbr[1]], color='red')
					#plt.text((pt[0] + nbr[0])/2.0, (pt[1] + nbr[1])/2.0, "co")
					plt.text(pt[0], pt[1], str(pt))
					plt.text(nbr[0], nbr[1], str(nbr))

		# STEP 2.4 : ADD START NODE TO THE GRAPH (CONNECTING IT TO THE ROADMAP)
		start_nbrs = self.findKNearestNeigbors(self.start, N)
		for nbr in start_nbrs:
			if not self.isIntersecting(self.start, nbr):
				# non-collision path found from start node to one of the 
				# samples (sorted based on its distance from start node)
				# Add the edge to the Roadmap
				edges.append({'ID1': '1', 'ID2': str(self.COORDS_TO_ID[nbr]), 'cost': str(self.euclideanDistance(self.start, nbr))})
				plt.plot([self.start[0], nbr[0]], [self.start[1], nbr[1]], color='black')
				break
			
		# STEP 2.5 : ADD GOAL NODE TO THE GRAPH (CONNECTING IT TO THE ROADMAP)
		goal_nbrs = self.findKNearestNeigbors(self.goal, N)
		for nbr in goal_nbrs:
			if not self.isIntersecting(self.goal, nbr):
				# non-collision path found from end node to one of the 
				# samples (sorted based on its distance from goal node)
				# Add the edge to the Roadmap
				edges.append({'ID1': str(self.COORDS_TO_ID[self.goal]), 'ID2': str(self.COORDS_TO_ID[nbr]), 'cost': str(self.euclideanDistance(self.goal, nbr))})
				plt.plot([self.goal[0], nbr[0]], [self.goal[1], nbr[1]], color='black')
				break

		# STEP 2.6 :WRITE COLLISION-FREE EDGES TO THE "EDGES.CSV"
		self.writeEdgesToCSV(edges)
		return (1, self.COORDS_TO_ID[self.goal])

	def printObstacles(self):
		print("Obstacles:")
		for obst in self.obstacles:
			print(obst.x, obst.y, obst.radius)

if __name__ == '__main__':
	radius = 0.01
	prm = PRM(radius)
	# STEP 1 : READ "OBSTACLES.CSV"
	prm.readObstaclesCSV()
	#prm.isIntersecting((0.3, -0.4), (0.4, -0.3))
	#plt.show()

	# STEP 2 : CALL PRM FUNCTION (N = NUMBER OF NODES, K = NUMBER OF NEAREST NEIGHBORS TO CONSIDER)
	N = 20
	K = 5
	plt.text(-0.2, 0.7, '(N: '+str(N)+", K: "+str(K)+")", fontsize=15, color='orange')	
	plt.text(-0.25, 0.65, 'Including paths with collision', fontsize=13, color='red')
	startID, goalID = prm.runPRM(N, K)

	# STEP 3 : RUN A* SEARCH ALGORITHM TO FIND SHORTEST PATH
	astar = Astar()
	astar.readNodesCSV('../Outputs/nodes.csv')
	astar.readEdgesCSV('../Outputs/edges.csv')
	
	path = astar.callAstar(startID, goalID)
	if path == False:
		print("NO PATH FOUND")
	else:
		list_of_nodes_IDs_in_path = list(map(int, path.split(",")))
		#print(list_of_nodes_IDs_in_path)
		astar.writeToCSV("../Outputs/path.csv", list_of_nodes_IDs_in_path)

		key_list = list(prm.COORDS_TO_ID.keys())
		val_list = list(prm.COORDS_TO_ID.values())
		for i in range(len(list_of_nodes_IDs_in_path) - 1):
			x1 = key_list[val_list.index(list_of_nodes_IDs_in_path[i])][0]
			y1 = key_list[val_list.index(list_of_nodes_IDs_in_path[i])][1]

			x2 = key_list[val_list.index(list_of_nodes_IDs_in_path[i + 1])][0]
			y2 = key_list[val_list.index(list_of_nodes_IDs_in_path[i + 1])][1]
			plt.plot([x1, x2], [y1, y2], color='green')

	leg = plt.legend(['Shortest Path', 'Valid Edges', 'Edges in collision with obstacle'], fontsize=10)
	colors=['green', 'blue', 'red']

	for i, j in enumerate(leg.legendHandles):
		j.set_color(colors[i])
	plt.show()
