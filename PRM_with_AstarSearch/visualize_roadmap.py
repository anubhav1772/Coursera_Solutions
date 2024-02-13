# -*- coding: utf-8 -*-
import csv
import matplotlib.pyplot as plt

plt.xlim( -0.8 , 0.8 )
plt.ylim( -0.8 , 0.8 )

class Visualize:
	def __init__(self):
		self.COORDS_TO_ID = {}
	
	def addCircleToGrid(self, x, y, r, color='black'):
		"""Plot a circle using Matplotlib
		"""
		circle = plt.Circle((x, y), r, color=color, alpha=0.5)
		plt.gca().add_patch(circle)
	
	def showObstacles(self, filepath="../results/obstacles.csv"):
		with open(filepath, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					self.addCircleToGrid(float(lines[0]), float(lines[1]), float(lines[2])/2.0)
					plt.text(float(lines[0]), float(lines[1]), str((float(lines[0]), float(lines[1]), float(lines[2])/2.0)))

	def showNodes(self, filename):
		with open(filename, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					self.COORDS_TO_ID[(float(lines[1]), float(lines[2]))] = int(lines[0])
					self.addCircleToGrid(float(lines[1]), float(lines[2]), 0.001, color='r')
					plt.text(float(lines[1]), float(lines[2]), lines[0], fontsize=12, color='red')
	
	def showShortestPath(self, filename):
		key_list = list(self.COORDS_TO_ID.keys())
		val_list = list(self.COORDS_TO_ID.values())
		with open(filename, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				list_of_nodes_IDs_in_path = list(map(int, lines))
			
				for i in range(len(list_of_nodes_IDs_in_path) - 1):
					x1 = key_list[val_list.index(list_of_nodes_IDs_in_path[i])][0]
					y1 = key_list[val_list.index(list_of_nodes_IDs_in_path[i])][1]

					x2 = key_list[val_list.index(list_of_nodes_IDs_in_path[i + 1])][0]
					y2 = key_list[val_list.index(list_of_nodes_IDs_in_path[i + 1])][1]
				
					plt.plot([x1, x2], [y1, y2], color='green')
					
	def showEdges(self, filename):
		key_list = list(self.COORDS_TO_ID.keys())
		val_list = list(self.COORDS_TO_ID.values())
		with open(filename, mode='r') as file:
			csvFile = csv.reader(file)
			for lines in csvFile:
				if str.__contains__(lines[0], '#') == False:
					x1 = key_list[val_list.index(int(lines[0]))][0]
					y1 = key_list[val_list.index(int(lines[0]))][1]
					x2 = key_list[val_list.index(int(lines[1]))][0]
					y2 = key_list[val_list.index(int(lines[1]))][1]
					
					plt.plot([x1, x2], [y1, y2], color='blue')


if __name__ == '__main__':
	visualize = Visualize()
	visualize.showNodes('../results/nodes.csv')
	visualize.showEdges('../results/edges.csv')
	visualize.showObstacles()

	visualize.showShortestPath('../results/path.csv')

	N = 20
	K = 5
	plt.text(-0.2, 0.7, '(N: '+str(N)+", K: "+str(K)+")", fontsize=15, color='orange')	
	plt.text(-0.5, 0.6, 'Not including paths with collision', fontsize=13, color='red')	
	plt.show()