#!/usr/bin/env python3
import rclpy
import os
from collections import deque
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_msgs.msg import String
import random

INFINITY = float("inf")

class GraphSearchNode(Node):
    def __init__(self, filename):
    
    	#ros related#
        super().__init__("dijkstra")
        self.temperature_publisher_ = self.create_publisher(Int64, "test", 10)
        self.temperature_timer_ = self.create_timer(2.0, self.publish_temperature)
        
        self.subscription = self.create_subscription(String,'start_end',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        #read file & create graph object#
        graph_edges = []
        with open(filename) as fhandle:
            for line in fhandle:
                edge_from, edge_to, cost, *_ = line.strip().split(" ")
                graph_edges.append((edge_from, edge_to, float(cost)))

        self.nodes = set()
        for edge in graph_edges:
            self.nodes.update([edge[0], edge[1]])

        self.adjacency_list = {node: set() for node in self.nodes}
        for edge in graph_edges:
            self.adjacency_list[edge[0]].add((edge[1], edge[2]))
        
        
    def shortest_path(self, start_node, end_node):
        """Uses Dijkstra's algorithm to determine the shortest path from
        start_node to end_node. Returns (path, distance).
        """

        unvisited_nodes = self.nodes.copy()  # All nodes are initially unvisited.

        # Create a dictionary of each node's distance from start_node. We will
        # update each node's distance whenever we find a shorter path.
        distance_from_start = {
            node: (0 if node == start_node else INFINITY) for node in self.nodes
        }

        # Initialize previous_node, the dictionary that maps each node to the
        # node it was visited from when the the shortest path to it was found.
        previous_node = {node: None for node in self.nodes}

        while unvisited_nodes:
            # Set current_node to the unvisited node with shortest distance
            # calculated so far.
            current_node = min(
                unvisited_nodes, key=lambda node: distance_from_start[node]
            )
            unvisited_nodes.remove(current_node)

            # If current_node's distance is INFINITY, the remaining unvisited
            # nodes are not connected to start_node, so we're done.
            if distance_from_start[current_node] == INFINITY:
                break

            # For each neighbor of current_node, check whether the total distance
            # to the neighbor via current_node is shorter than the distance we
            # currently have for that node. If it is, update the neighbor's values
            # for distance_from_start and previous_node.
            for neighbor, distance in self.adjacency_list[current_node]:
                new_path = distance_from_start[current_node] + distance
                if new_path < distance_from_start[neighbor]:
                    distance_from_start[neighbor] = new_path
                    previous_node[neighbor] = current_node

            if current_node == end_node:
                break # we've visited the destination node, so we're done

        # To build the path to be returned, we iterate through the nodes from
        # end_node back to start_node. Note the use of a deque, which can
        # appendleft with O(1) performance.
        path = deque()
        current_node = end_node
        while previous_node[current_node] is not None:
            path.appendleft(current_node)
            current_node = previous_node[current_node]
        path.appendleft(start_node)

        return path, distance_from_start[end_node]
                
    def publish_temperature(self):
        temperature = random.randint(20, 30)
        msg = Int64()
        msg.data = temperature
        self.temperature_publisher_.publish(msg)
        
    def listener_callback(self, msg):
    	start_end = msg.data
    	start, end = start_end.strip().split(",")
    	returned_path, returned_distance = self.shortest_path(start, end)
    	self.get_logger().info('I heard: "%s"' % msg.data)
    	print('      start/end nodes: {0} -> {1}'.format(start, end))
    	print('      shortest path: {0}'.format(returned_path))
    	print('      total distance: {0}'.format(returned_distance))

        
def main(args=None):
    rclpy.init(args=args)
    path = os.path.join(os.path.expanduser('~'), 'turtlebot3_ws', 'src', 'graph_search', 'graph_search', 'simple_graph.txt')
    print(path)
    node = GraphSearchNode(path)
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
