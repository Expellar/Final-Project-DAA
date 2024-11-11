#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <utility>

using namespace std;

// Define INF as a large value to represent infinity
constexpr int INF = 0x3f3f3f3f;

// iPair ==> Integer Pair
using iPair = pair<int, int>;

// Class representing a graph using adjacency list representation
class Graph {
    int V; // Number of vertices
    vector<list<iPair>> adj; // Adjacency list

public:
    explicit Graph(int V); // Constructor

    void addEdge(int u, int v, int w); // Function to add an edge
    void shortestPath(int src) const; // Function to print shortest path from source
};

// Constructor to allocate memory for the adjacency list
Graph::Graph(int V) : V(V), adj(V) {}

// Function to add an edge to the graph
void Graph::addEdge(int u, int v, int w) {
    adj[u].emplace_back(v, w);
    adj[v].emplace_back(u, w); // Since the graph is undirected
}

// Function to print shortest paths from source
void Graph::shortestPath(int src) const {
    // Priority queue to store vertices being processed
    priority_queue<iPair, vector<iPair>, greater<>> pq;

    // Vector to store distances and initialize all distances as INF
    vector<int> dist(V, INF);

    // Insert source into priority queue and initialize its distance as 0
    pq.emplace(0, src);
    dist[src] = 0;

    // Process the priority queue
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        // Iterate through all adjacent vertices of the current vertex
        for (const auto& [v, weight] : adj[u]) {
            // If a shorter path to v is found
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.emplace(dist[v], v);
            }
        }
    }

    // Print the shortest distances
    cout << "Vertex Distance from Source\n";
    for (int i = 0; i < V; ++i)
        cout << i << " \t\t " << dist[i] << '\n';
}

// Driver code
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
