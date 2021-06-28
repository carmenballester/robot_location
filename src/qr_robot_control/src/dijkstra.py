#!/usr/bin/env python

""" ENCONTRAR EL CAMINO MAS CORTO ENTRE DOS NODOS USNDO DIJKSTRA

Carmen Ballester Bernabeu

Grupo Aurova
Universidad de Alicante

"""

import collections
import heapq
import sys

# Libraries for ROS
import rospy
# Personalized services
from qr_robot_control.srv import Dijkstra, DijkstraResponse

# Define global variables
path = None
cont = 0

def shortest_path(edges, source, sink):
	""" Dijkstra algorithm for finding the shortest path between two nodes.
	
	Parameters
	----------
	edges:
		list of the edges between every node of the graph, which represents paths that the robot can follow.
	source: str
		Initial node. 
	sink: str
		Final node. 	  

	Credits
	------- 
	This implementation has been done by hangfan, and its avaliable in his github. 
	"""	

	# Create a weighted DAG - {node:[(cost,neighbour), ...]}
	graph = collections.defaultdict(list)
	for l, r, c in edges:
		graph[l].append((c,r))

	# Create a priority queue and hash set to store visited nodes
	queue, visited = [(0, source, [])], set()
	heapq.heapify(queue)

	# Traverse graph with BFS
	while queue:
		(cost, node, path) = heapq.heappop(queue)
		# Visit the node if it was not visited before
		if node not in visited:
			visited.add(node)
			path = path + [node]
			# Hit the sink
			if node == sink:
				return (cost, path)
			# Visit neighbours
			for c, neighbour in graph[node]:
				if neighbour not in visited:
					heapq.heappush(queue, (cost+c, neighbour, path))

	return float("inf")

def handle_dijkstra(req):
	""" Callback executed everytime the dijkstra service is called. 

	Parameters
	----------
	req : none
	
	"""
	
	# Define global variables that are used
	global path
	global cont
	
	# Check if the path is finished and send the next node or the ending order
	if cont == len(path):
		node = "-1"
		cont = 0
		print("Global goal sended. End of the process.")

	else: 
		node = path[cont]
		cont = cont+1		
		print("Sending new goal to controller: {}...".format(node))
	
	return DijkstraResponse(node)

def main():
	""" Main function where ROS parameters are initialized and the path is defined

	"""

	# Check the parameters	
	if len(sys.argv) != 3:
		print("Initial and goal node required.")	
		sys.exit()	
	
	else:
		# Define the edges of the grpah
		edges = [
			("O", "A", 2.0),
			("O", "C", 3.35),
			("O", "D", 1.5),
			("A", "B", 1.80),
			("A", "D", 2.5),
			("D", "E", 2.24),
			("D", "F", 2.12),
			("D", "G", 2.55),
			("D", "H", 4.5),
			("F", "G", 2.24),
			("H", "I", 3.5),
			("I", "J", 1.58),
		]
		
		# Define global variables
		global path
		global cost
		
		init = sys.argv[1]
		goal = sys.argv[2]
	
		# Find the shortest path between two nodes
		print("Finding the shortest path with Dijkstra: {}->{}...".format(init, goal))
		dijkstra = shortest_path(edges, init, goal)
		
		# Get the path 
		cost = dijkstra[0]
		path = dijkstra[1]
#		path = ['O', 'D', 'F', 'G']
		print("The cost is: {} and the path is: {}".format(cost, path))
	
		# Init ROS params
		rospy.init_node('dijkstra_server')
		s = rospy.Service('dijkstra', Dijkstra, handle_dijkstra)
		
		print("Ready to send path to controller")
		
		while not rospy.is_shutdown(): 
			pass	

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
