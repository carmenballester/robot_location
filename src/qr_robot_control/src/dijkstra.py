import collections
import heapq
import sys

import rospy
from qr_robot_control.srv import Dijkstra, DijkstraResponse

path = None
cont = 0

def shortest_path(edges, source, sink):
	# create a weighted DAG - {node:[(cost,neighbour), ...]}
	graph = collections.defaultdict(list)
	for l, r, c in edges:
		graph[l].append((c,r))
	# create a priority queue and hash set to store visited nodes
	queue, visited = [(0, source, [])], set()
	heapq.heapify(queue)
	# traverse graph with BFS
	while queue:
		(cost, node, path) = heapq.heappop(queue)
		# visit the node if it was not visited before
		if node not in visited:
			visited.add(node)
			path = path + [node]
			# hit the sink
			if node == sink:
				return (cost, path)
			# visit neighbours
			for c, neighbour in graph[node]:
				if neighbour not in visited:
					heapq.heappush(queue, (cost+c, neighbour, path))
	return float("inf")


def handle_dijkstra(req):
	global path
	global cont
	
	if cont == len(path):
		node = "-1"
		cont = 0
		print("Global goal sended. End of the process.")

	else: 
		node = path[cont]
		cont = cont+1		
		print("Sending new goal to controller: {}...".format(node))
	
	return DijkstraResponse(node)


if __name__ == "__main__":
	edges = [
		("O", "A", 2.0),
		("O", "C", 3.35),
		("O", "D", 1.5),
		("A", "B", 1.80),
		("A", "D", 2.5),
	]
		
	
	print("Finding the shortest path with Dijkstra: O->C...")
	dijkstra = shortest_path(edges, "O", "C")
	
	cost = dijkstra[0]
	path = dijkstra[1]
	print("The cost is: {} and the path is: {}".format(cost, path))

	rospy.init_node('dijkstra_server')
	s = rospy.Service('dijkstra', Dijkstra, handle_dijkstra)
	
	print("Ready to send path to controller")
	
	while not rospy.is_shutdown(): 
		pass	
