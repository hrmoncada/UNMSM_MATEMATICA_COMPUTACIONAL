/* https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/
C++ program for Dijkstra's single source shortest path algorithm. The program is for adjacency matrix representation of the graph
*/
#include <bits/stdc++.h>
using namespace std;

// Define INF as a large value to represent infinity
#define INF 0x3f3f3f3f

// iPair ==> Integer Pair
typedef pair<int, int> iPair;

// Class representing a graph using adjacency list representation
class Graph {
    int V; // Number of vertices
    list<iPair> *adj; // Adjacency list

public:
    Graph(int V); // Constructor

    void addEdge(int u, int v, int w); // Function to add an edge
    void shortestPath(int s); // Function to print shortest path from source
};

// Constructor to allocate memory for the adjacency list
Graph::Graph(int V) {
    this->V = V;
    adj = new list<iPair>[V];
}

// Function to add an edge to the graph
void Graph::addEdge(int u, int v, int w) {
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w)); // Since the graph is undirected
}

// Function to print shortest paths from source
void Graph::shortestPath(int src) {
    // Create a priority queue to store vertices being processed
    // Priority queue sorted by the first element of the pair (distance)
    priority_queue<iPair, vector<iPair>, greater<iPair>> pq;

    // Create a vector to store distances and initialize all distances as INF
    vector<int> dist(V, INF);

    // Insert source into priority queue and initialize its distance as 0
    pq.push(make_pair(0, src));
    dist[src] = 0;

    // Process the priority queue
    while (!pq.empty()) {
        // Get the vertex with the minimum distance
        int u = pq.top().second;
        pq.pop();

        // Iterate through all adjacent vertices of the current vertex
        for (auto &neighbor : adj[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            // If a shorter path to v is found
            if (dist[v] > dist[u] + weight) {
                // Update distance and push new distance to the priority queue
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }
        }
    }

    // Print the shortest distances
    cout << "Vertex Distance from Source" << endl;
    for (int i = 0; i < V; ++i)
        cout << i << " \t\t " << dist[i] << endl;
}

// Driver's code
int main() {
    int V = 9; // Number of vertices
    Graph g(V);

    // Add edges to the graph
    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    // Call the shortestPath function
    g.shortestPath(0);

    return 0;
}

