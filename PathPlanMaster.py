"""

Runs path planners and compares results

"""
import matplotlib.pyplot as plt, matplotlib.image as mpimg
from PIL import Image
import numpy as np
import cv2
import yaml
import rrt_star

class Master(object):
	def __init__(self,map_data):
		"""
		map_data (string): path to map information
		"""

		# load map data
		map_info = open(map_data)
		map_info = yaml.load(map_info)
		self.x0,self.y0,self.z0 = map_info['origin']
		self.resolution = map_info['resolution']
		self.start = (-50,-30)

		# create occupancy grid and dilate
		kernel = np.ones((5,5), np.uint8)
		self.permissible_region = np.loadtxt("omap.csv", delimiter=",")
		self.permissible_region = cv2.erode(self.permissible_region, kernel, iterations=1)
		
		self.map_width = self.permissible_region.shape[0]*self.resolution
		self.map_height = self.permissible_region.shape[1]*self.resolution

		# display map and start location
		self.fig, self.ax = plt.subplots(1,1)
		self.plot_map()

		# path planner classes
		self.RRT = rrt_star.PathPlanner(self.permissible_region, self.resolution, (self.x0,self.y0), self.start)

		# runs main event handling function
		self.main()

	def onclick(self, event):

		""" 
		gets clicked point and sets it as goal:

		event: mouse click event
		"""

		if isinstance(event.xdata, float) and isinstance(event.ydata, float):
			# check if point on map has been clicked
			self.goal = (event.xdata, event.ydata)
			print "New goal: ", self.goal

			# plan paths
			print "planning path"
			self.plot_map(self.goal)
			path1 = self.RRT.plan_path(self.goal)
			self.plot_map(self.goal,path1)
		
		else:
			print "Must select a point on the map"


	def plot_map(self, goal = None, path = None):
		"""
		plots the map and goal/path if they are passed as inputs
		"""
		self.ax.cla()
		x1,y1 = self.start
		self.ax.imshow(self.permissible_region, cmap='gray', 
			extent=[self.x0, self.x0 - self.map_width, self.y0 - self.map_height, self.y0])
		self.ax.scatter(x1,y1, c='g')

		# checks if goal exists
		if goal is not None:
			x2,y2 = goal
			print goal
			self.ax.scatter(x2,y2,c='r')
			plt.pause(.01)

		# checks if path exists
		if path is not None:
			for i in range(len(path)):
				if i < len(path)-1:
					x0,y0 = path[i]
					x1,y1 = path[i+1]
					self.ax.plot([x0,x1],[y0,y1],'b')
					plt.pause(.01)

		self.ax.draw
		
	def main(self):
		# main function which listens for user input and finds a path to the input point
		cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
		plt.show()
		


Master = Master("building_31.yaml")