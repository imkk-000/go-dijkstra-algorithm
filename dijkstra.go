// source from https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/

package dijkstra

import (
	"fmt"
	"math"
)

// Number of vertices in the graph
const V int = 9

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
func MinDistance(dist [V]int, sptSet [V]bool) int {
	// Initialize min value
	var min = math.MaxInt32
	var minIndex int
	for v := 0; v < V; v++ {
		if sptSet[v] == false && dist[v] <= min {
			min = dist[v]
			minIndex = v
		}
	}
	return minIndex
}

// A utility function to print the constructed distance array
func printSolution(dist [V]int, n int) {
	fmt.Printf("Vertex   Distance from Source\n")
	for i := 0; i < V; i++ {
		fmt.Printf("%d tt %d\n", i, dist[i])
	}
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
func Dijkstra(graph [V][V]int, src int) {
	var dist [V]int // The output array.  dist[i] will hold the shortest
	// distance from src to i
	var sptSet [V]bool // sptSet[i] will be true if vertex i is included in shortest
	// path tree or shortest distance from src to i is finalized
	// Initialize all distances as INFINITE and stpSet[] as false
	for i := 0; i < V; i++ {
		dist[i] = math.MaxInt32
		sptSet[i] = false
	}

	// Distance of source vertex from itself is always 0
	dist[src] = 0

	// Find shortest path for all vertices
	for count := 0; count < V-1; count++ {
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in the first iteration.
		u := MinDistance(dist, sptSet)

		// Mark the picked vertex as processed
		sptSet[u] = true

		// Update dist value of the adjacent vertices of the picked vertex.
		for v := 0; v < V; v++ {
			// Update dist[v] only if is not in sptSet, there is an edge from
			// u to v, and total weight of path from src to  v through u is
			// smaller than current value of dist[v]
			if !sptSet[v] {

			}
			if !sptSet[v] && (graph[u][v] != 0) &&
				(dist[u] != math.MaxInt32) &&
				(dist[u]+graph[u][v] < dist[v]) {
				dist[v] = dist[u] + graph[u][v]
			}
		}
	}

	// print the constructed distance array
	printSolution(dist, V)
}
