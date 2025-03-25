#!/usr/bin/env python
import heapq

class Graph:
    def __init__(self):
        print("Creating new Graph")
        self.edges = {}
    
    def add_edge(self, from_node, to_node, weight):
        if from_node not in self.edges:
            print(f"Adding new node to graph: {from_node}")
            self.edges[from_node] = {}
        print(f"Adding edge: {from_node} -> {to_node} with weight {weight}")
        self.edges[from_node][to_node] = weight
    
    def get_neighbors(self, node):
        if node in self.edges:
            neighbors = self.edges[node]
            print(f"Node {node} has {len(neighbors)} neighbors: {list(neighbors.keys())}")
            return neighbors
        print(f"Node {node} has no neighbors")
        return {}

def dijkstra_shortest_path(graph, start):
    print(f"Starting Dijkstra's algorithm from node {start}")
    distances = {start: 0}
    previous = {}
    pq = [(0, start)]
    visited_count = 0
    
    print(f"Initial priority queue: {pq}")
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        visited_count += 1
        
        print(f"Visiting node {current_node} with distance {current_distance}")
        
        if current_distance > distances.get(current_node, float('infinity')):
            print(f"Skipping node {current_node} as we found a better path already")
            continue
        
        neighbors = graph.get_neighbors(current_node)
        print(f"Processing {len(neighbors)} neighbors of {current_node}")
        
        for neighbor, weight in neighbors.items():
            distance = current_distance + weight
            
            if distance < distances.get(neighbor, float('infinity')):
                print(f"Found better path to {neighbor} with distance {distance}")
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
                print(f"Updated priority queue, size: {len(pq)}")
            else:
                print(f"Path to {neighbor} with distance {distance} is not better than current {distances.get(neighbor, float('infinity'))}")
    
    print(f"Dijkstra's algorithm completed after visiting {visited_count} nodes")
    print(f"Found paths to {len(distances)} nodes")
    print(f"Previous nodes map has {len(previous)} entries")
    
    return distances, previous

