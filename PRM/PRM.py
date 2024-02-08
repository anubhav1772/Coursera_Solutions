import csv
import numpy as np
import random
from collections import defaultdict

class Obstacle:

	def __init__(self, x, y, radius):
		self.x = x
		self.y = y
		self.radius = radius

class PRM:

	def __init__(self):
		self.obstacles = []
		self.samples = []
		self.start = (-0.5, -0.5)
		self.goal = (0.5, 0.5)
		self.edges = defaultdict()

	def euclideanDistance(self, pt1, pt2):
		return np.linalg.norm(np.array(pt1) - np.array(pt2))

	def readObstaclesCSV(self, filepath="obstacles.csv"):
		with open(filepath, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					self.obstacles.append(Obstacle(float(lines[0]), float(lines[1]), float(lines[2])/2.0))

	def isIntersecting(self, pt1, pt2):
		#self.obstacles.append(Obstacle(0, 0, 1.0)) => for testing with one obstacle
		m = (pt2[1] - pt1[1])/(pt2[0] - pt1[0] + 0.0000000001)
		const = pt1[1] - m*pt1[0] # y = mx + const => const = (y-mx) at pt1|pt2 
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

				if min(pt1[0], pt2[0]) <= x1 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y1 <= max(pt1[1], pt2[1]):
					return True

				if min(pt1[0], pt2[0]) <= x2 <= max(pt1[0], pt2[0]) and min(pt1[1], pt2[1]) <= y2 <= max(pt1[1], pt2[1]):
					return True

		return False

	def findKNearestNeigbors(self, pt, K):
		neighbors = [(point, self.euclideanDistance(pt, point)) for point in self.samples]
		neighbors.sort(key = lambda x: x[1]) # sort neighbors list based on euclidean distances 
		return [nbr[0] for nbr in neighbors[:K]]

	def runPRM(self):
		N = 10
		self.generate_random_samples(N)

		for pt in self.samples:
			neighbors = self.findKNearestNeigbors(pt, 5)
			for nbr in neighbors:
				if not self.isIntersecting(pt, nbr):
					 #self.edges[].append()
					 print(str(pt) + "->" + str(nbr))

	def generate_random_samples(self, N):
		# Generate N random samples within the square [-0.5, 0.5] x [-0.5, 0.5]
		i = 0
		while(i<N):
			x = round(random.uniform(-0.5, 0.5), 1)
			y = round(random.uniform(-0.5, 0.5), 1)
			if((x, y) not in self.samples):
				self.samples.append((x, y))
				i += 1

	def printObstacles(self):
		print("Obstacles:")
		for obst in self.obstacles:
			print(obst.x, obst.y, obst.radius)

		print("##############")


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
	# Phase 3: A* Search

	prm.runPRM()
	# TESTING : INTERSECTION OF A LINE WITH OBSTACLE
	# print(prm.isIntersecting((1, 0), (0, 1)))
	# print(prm.isIntersecting((2, 0), (0, 2)))