
import numpy as np
import threading, time, collections, heapq, itertools, math, os
import matplotlib.pyplot as plt

class Node(object):
	def __init__(self,x,y):
		self.x = x
		self.y = y
		self.parent = None
		self.cost = 0.0

	def distance(self, random_point):
		distance = np.sqrt((self.x-random_point[0])**2 + (self.y - random_point[1])**2)
		return distance

class PathPlanner(object):
	""" Abstract class for path planning """
	def __init__(self, map_grid, resolution, map_origin, start):

		# map info
		self.map_width = map_grid.shape[1]*resolution
		self.map_height = map_grid.shape[0]*resolution
		self.x0, self.y0 = map_origin
		self.resolution = resolution
		self.shape_map = map_grid.shape
		self.permissible_region = map_grid

		# start/goal info
		self.start = start
		self.goal = None
		
		# distance to goal location to end RRT search
		self.epsilon_goal = 0.5

		# step size to expand tree
		self.step = .5

		# width of path 
		self.path_width = 0.04

		# node stuff
		self.MAX_ITERS = 2000
		self.nodes = []


	def nearest_node(self, random_point):
		""" 
		Finds the index of the nearest node in self.nodes to random_point 
		"""

		# index of nearest node
		nn_idx = 0

		# loops through all nodes and finds the closest one
		for i in range(len(self.nodes)):
			node = self.nodes[i]
			nearest_node = self.nodes[nn_idx]

			if node.distance(random_point) < nearest_node.distance(random_point):
				nn_idx = i

		return nn_idx

	def steer_to_node(self, x_near, rnd):
		""" 
		Steers from x_near to a point on the line between rnd and x_near 
		"""

		theta = math.atan2(rnd[1] - x_near.y, rnd[0] - x_near.x)
		new_x = x_near.x + self.step*math.cos(theta)
		new_y = x_near.y + self.step*math.sin(theta)

		x_new = Node(new_x, new_y)
		x_new.cost += self.step
		
		return x_new, theta

	def obstacle_free(self, nearest_node, new_node, theta, function_call = "normal"):
		""" 
		Checks if the path from x_near to x_new is obstacle free 
		"""

		dx = math.sin(theta)*self.path_width
		dy = math.cos(theta)*self.path_width

		if (nearest_node.x < new_node.x):
			bound1 = ((nearest_node.x+dx, nearest_node.y-dy), (new_node.x+dx, new_node.y-dy))
			bound2 = ((nearest_node.x-dx, nearest_node.y+dy), (new_node.x-dx, new_node.y+dy))
		else:
			bound1 = ((new_node.x+dx, new_node.y-dy), (nearest_node.x+dx, nearest_node.y-dy))
			bound2 = ((new_node.x-dx, new_node.y+dy), (nearest_node.x-dx, nearest_node.y+dy))


		if (self.line_collision(bound1, function_call) or self.line_collision(bound2, function_call)):
			return False
		else:
			return True

	def line_collision(self, line,function_call):
		""" 
		Checks if line collides with obstacles in the map
		"""

		# discretize values of x and y along line according to map using Bresemham's alg
		
		x_ind = np.arange(np.round(-line[1][0]/self.resolution + self.x0/self.resolution), np.round(-line[0][0]/self.resolution + self.x0/self.resolution),)
		y_ind = []
		dx = max(1,np.round(line[1][0]/self.resolution) - np.round(line[0][0]/self.resolution))
		dy = np.round(line[1][1]/self.resolution) - np.round(line[0][1]/self.resolution)
		deltaerr = abs(dy/dx)
		error = .0

		y = int(np.round(-line[1][1]/self.resolution + self.y0/self.resolution))
		for x in x_ind:
			y_ind.append(y)
			error = error + deltaerr
			if error >= 0.5:
				y += -np.sign(dy)*1
				error += -1

		y_ind = np.array(y_ind[::-1])
		
		# check if each cell along line contains obstacle
		for i in range(len(x_ind)):

			row = min([int(y_ind[i]), self.shape_map[0] - 1])
			column = min([int(x_ind[i]), self.shape_map[1]-1])
			
			if self.permissible_region[row,column] == 0.:
				plt.plot([line[0][0],line[1][0]], [line[0][1],line[1][1]], "r")
				plt.pause(.01)
				return True

		if x_ind == []:
			# checks if line rasteriztion worked
			return True

		plt.plot([line[0][0],line[1][0]], [line[0][1],line[1][1]], "b")
		plt.pause(.01)
		return False

	def get_last_idx(self):
		dlist = []
		for node in self.nodes:
			d = node.distance((self.goal[0], self.goal[1]))
			dlist.append(d)
		goal_idxs = [dlist.index(i) for i in dlist if i <= self.step]

		if len(goal_idxs) == 0:
			return None

		mincost = min([self.nodes[i].cost for i in goal_idxs])

		for i in goal_idxs:
			if self.nodes[i].cost == mincost:
				return i

		return None

	def get_path(self, last_idx):
		"""
		returns path
		"""
		path = [(self.goal[0], self.goal[1])]
		while self.nodes[last_idx].parent is not None:
			# checks that there are no loops
			node = self.nodes[last_idx]
			path.append((node.x, node.y))
			last_idx = node.parent
		path.append((self.start[0], self.start[1]))
		path.reverse()
		return path

	def plan_path(self, goal):
		self.nodes = []
		self.goal = goal
		x,y = self.start
		start_node = Node(x,y)
		self.nodes.append(start_node)
		j = 0
		path_found = False
		while not(path_found) and (j < self.MAX_ITERS):

			# random sample
			if j%25 == 0:
				# sample goal location every 25 iterations
				rnd = [goal[0], goal[1]]
			else:
				rnd = [self.x0 - self.map_width*np.random.uniform(), self.y0 - self.map_height*np.random.uniform()]
				
			# find the nearest node
			nn_idx = self.nearest_node(rnd)
			x_near = self.nodes[nn_idx]

			# expand the tree
			x_new, theta = self.steer_to_node(x_near,rnd)
			x_new.parent = nn_idx

			# check for obstacle collisions
			if self.obstacle_free(x_near, x_new, theta):
			
				self.nodes.append(x_new)
			
				# check if sample point is in goal region
				dx = x_new.x - self.goal[0]
				dy = x_new.y - self.goal[1]
				d = np.sqrt(dx**2 + dy**2)
				if d <= self.epsilon_goal:
					# construct path
					last_idx = self.get_last_idx()
					if last_idx is None:
						return None
					path = self.get_path(last_idx)
					return path
			j += 1
		print "Couldn't find path"
		return None