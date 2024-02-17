import csv
import numpy as np
import random
from collections import defaultdict
import matplotlib.pyplot as plt
import math

plt.xlim( -0.8 , 0.8 )
plt.ylim( -0.8 , 0.8 )

class Obstacle:

	def __init__(self, x, y, radius):
		self.x = x
		self.y = y
		self.radius = radius

class PRM:

	def __init__(self):
		self.obstacles = []
		self.samples = []
		self.start = (-0.5, -0.5) # Start Position
		self.goal = (0.5, 0.5)    # Goal Position
		self.thresh_radius = 0.05 
		self.edges = defaultdict(list)

	def readObstaclesCSV(self, filepath="Input/obstacles.csv"):
		with open(filepath, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False: # Ignore line start with '#'
					self.obstacles.append(Obstacle(float(lines[0]), float(lines[1]), float(lines[2])/2.0))
					self.addCircleToGrid(float(lines[0]), float(lines[1]), float(lines[2])/2.0)

	def addCircleToGrid(self, x, y, r, color='b'):
		circle = plt.Circle((x, y), r, color=color, alpha=0.5)
		#plt.text(x, y, str((x, y)))
		plt.gca().add_patch(circle)

	def addStartAndGoalStateToGrid(self):
		self.addCircleToGrid(float(self.start[0]), float(self.start[1]), 0.015, color='orange')
		plt.text(float(self.start[0]), float(self.start[1]), 'Start')

		self.addCircleToGrid(float(self.goal[0]), float(self.goal[1]), 0.015, color='green')
		plt.text(float(self.goal[0]), float(self.goal[1]), 'Goal')

	def IsInFreeSpace(self, x, y):
		for obstacle in self.obstacles:
			x1 = obstacle.x
			y1 = obstacle.y
			r = obstacle.radius + self.thresh_radius

			cond = (x - x1)*(x - x1) + (y - y1)*(y - y1) - r*r

			if(cond <= 0): # condition for a point to lie inside|on the circle (intersecting with obstacle)
				return False
		return True

	def generate_random_samples(self, N):
		# Generate N random samples within the square [-0.5, 0.5] x [-0.5, 0.5]
		i = 0
		while(i<N):
			x = round(random.uniform(-0.5, 0.5), 1)
			y = round(random.uniform(-0.5, 0.5), 1)
			if(((x, y) not in self.samples) and (self.IsInFreeSpace(x,y))):
				self.samples.append((x, y))
				#self.edges[(x,y)].append()
				self.addCircleToGrid(x, y, 0.01, color='black')
				i += 1
		# Add start and Goal state to the grid as well
		self.addStartAndGoalStateToGrid()

	def euclideanDistance(self, pt1, pt2):
		return round(np.linalg.norm(np.array(pt1) - np.array(pt2)), 5)

	def findKNearestNeigbors(self, pt, K):
		neighbors = [(point, self.euclideanDistance(pt, point)) for point in self.samples if pt != point]
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
	
			for obstacle in self.obstacles:
				x1 = obstacle.x
				y1 = obstacle.y
				r  = obstacle.radius + self.thresh_radius
				
				a = (1 + m*m)
				b = (2*m*const - 2*x1 - 2*y1*m)
				c = (x1*x1 + const*const + y1*y1 - 2*y1*const - r*r)

				discriminant = b*b - 4*a*c

				#print("a = "+str(a)+",b = "+str(b)+",c = "+str(c)+", discriminant = "+str(discriminant))

				if discriminant < 0:
					return False
				
				else:
					# intersection points
					x1 = (-b + np.sqrt(discriminant))/float(2*a)
					y1 = m*x1 + const

					# Check if the intersection points lie are within the line segment(not line)
					if min(pt1[0], pt2[0]) <= x1 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y1 <= max(pt1[1], pt2[1]):
						return True

					x2 = (-b - np.sqrt(discriminant))/float(2*a)
					y2 = m*x2 + const

					if min(pt1[0], pt2[0]) <= x2 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y2 <= max(pt1[1], pt2[1]):
						return True
		
		else:	# VERTICAL PATH
			# m = infinity
			# eqn of line : x = c (c is the common x coordinate)
			for obstacle in self.obstacles:
				x1 = obstacle.x
				y1 = obstacle.y
				r  = obstacle.radius + self.thresh_radius
				
				a = 1
				b = -2*y1
				c = y1*y1 + (pt1[0] - x1)*(pt1[0] - x1) - r*r

				discriminant = b*b - 4*a*c

				#print("a = "+str(a)+",b = "+str(b)+",c = "+str(c)+", discriminant = "+str(discriminant))

				if discriminant < 0:
					return False
				
				else:
					# intersection points
					x1 = pt2[0]
					y1 = (-b + np.sqrt(discriminant))/float(2*a)

					# Check if the intersection points lie are within the line segment(not line)
					if min(pt1[0], pt2[0]) <= x1 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y1 <= max(pt1[1], pt2[1]):
						return True

					x2 = pt2[0]
					y2 = (-b - np.sqrt(discriminant))/float(2*a)

					if min(pt1[0], pt2[0]) <= x2 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y2 <= max(pt1[1], pt2[1]):
						return True
		return False

	def runPRM(self, N, K):
		# STEP 2.1 : GENERATE SAMPLES FROM FREE CONFIGURATION SPACE
		self.generate_random_samples(N)

		# STEP 2.1 : ADD START NODE TO THE GRAPH (CONNECTING IT TO THE ROADMAP)
		start_nbrs = self.findKNearestNeigbors(self.start, K)
		for nbr in start_nbrs:
			if (nbr in self.edges[self.start]) or (self.start in self.edges[nbr]):
				# edge already exists
				continue
			else:
				if not self.isIntersecting(self.start, nbr):
					self.edges[self.start].append(nbr)
					plt.plot([self.start[0], nbr[0]], [self.start[1], nbr[1]], color='black')

		# STEP 2.2 : LOOP THROUGH THE GENERATED SAMPLE NODES
		for pt in self.samples:
			# STEP 2.2.1 : FIND K NEAREST NEIGHBORS OF THE NODE
			neighbors = self.findKNearestNeigbors(pt, K)
			# STEP 2.2.2 : CHECK FOR COLLISION-FREE PATH
			for nbr in neighbors:
				# STEP 2.2.2.1 : IF COLLISION-FREE PATH => APPEND IT TO EDGES LIST
				if (nbr in self.edges[pt]) or (pt in self.edges[nbr]):
					# edge already exists
					continue
				else: # (pt, nbr) pair doesn't exist in the edges dictionary
					if not self.isIntersecting(pt, nbr):
						self.edges[pt].append(nbr) # adding (pt, nbr) pair into the edges dictionary
						#plt.text((pt[0] + nbr[0])/2.0, (pt[1] + nbr[1])/2.0, str(""))
						plt.plot([pt[0], nbr[0]], [pt[1], nbr[1]], color='blue')
					else: # STEP 2.2.2.2 : PATH IS COLLIDING WITH ONE OF THE OBSTACLES => DON'T ADD IT TO THE LIST
						plt.plot([pt[0], nbr[0]], [pt[1], nbr[1]], color='red')

		# STEP 2.3 : ADD GOAL NODE TO THE GRAPH (CONNECTING IT TO THE ROADMAP)
		goal_nbrs = self.findKNearestNeigbors(self.goal, K)
		for nbr in goal_nbrs:
			if (nbr in self.edges[self.goal]) or (self.goal in self.edges[nbr]):
				# edge already exists
				continue
			else:
				if not self.isIntersecting(self.goal, nbr):
					self.edges[self.goal].append(nbr)
					plt.plot([self.goal[0], nbr[0]], [self.goal[1], nbr[1]], color='black')

		print(len(self.edges))

if __name__ == '__main__':
	prm = PRM()
	prm.readObstaclesCSV()
	#prm.printObstacles()
	# Phase 1: Sampling
	#prm.generate_random_samples(10)
	
	#print(prm.samples)
	# K nearest neighbors
	# TESTING : K NEAREST NEIGHBORS
	# print(prm.findKNearestNeigbors(prm.start, 5))
	# Phase 2: Creating edges

	N = 50
	K = 3
	prm.runPRM(N, K)
	plt.show()
	# TESTING : INTERSECTION OF A LINE WITH OBSTACLE
	# print(prm.isIntersecting((1, 0), (0, 1)))
	# print(prm.isIntersecting((2, 0), (0, 2)))