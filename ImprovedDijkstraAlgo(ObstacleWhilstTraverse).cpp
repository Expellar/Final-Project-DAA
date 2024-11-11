#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <utility>
#include <unordered_set>

using namespace std;

constexpr int INF = 0x3f3f3f3f;
using iPair = pair<int, int>;

class Graph {
    int V;
    vector<list<iPair>> adj;
    unordered_set<int> blockedNodes; // Set to keep track of blocked nodes

public:
    explicit Graph(int V);
    void addEdge(int u, int v, int w);
    void blockNode(int node); // Mark a node as blocked
    void unblockNode(int node); // Unblock a node if the obstacle is removed
    void shortestPath(int src, int target); // Path recalibration from src to target
};

Graph::Graph(int V) : V(V), adj(V) {}

void Graph::addEdge(int u, int v, int w) {
    adj[u].emplace_back(v, w);
    adj[v].emplace_back(u, w); // Undirected graph
}

void Graph::blockNode(int node) {
    blockedNodes.insert(node);
}

void Graph::unblockNode(int node) {
    blockedNodes.erase(node);
}

void Graph::shortestPath(int src, int target) {
    priority_queue<iPair, vector<iPair>, greater<>> pq;
    vector<int> dist(V, INF);
    vector<int> parent(V, -1); // To keep track of the path

    dist[src] = 0;
    pq.emplace(0, src);

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        // Skip processing if this node is blocked
        if (blockedNodes.find(u) != blockedNodes.end()) continue;

        for (const auto& [v, weight] : adj[u]) {
            if (blockedNodes.find(v) != blockedNodes.end()) continue;

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.emplace(dist[v], v);
            }
        }
    }

    // If target is unreachable due to obstacles
    if (dist[target] == INF) {
        cout << "Target " << target << " is unreachable from source " << src << " due to obstacles.\n";
        return;
    }

    // Output the path
    cout << "Shortest path from " << src << " to " << target << ": ";
    int current = target;
    vector<int> path;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        cout << *it << (it + 1 != path.rend() ? " -> " : "\n");
    }
    cout << "Distance: " << dist[target] << '\n';
}

int main() {
    int V = 9;
    Graph g(V);

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

    int source = 0;
    int target = 8;
    g.shortestPath(source, target);

    cout << "\nEncountering obstacle between nodes 3 and 4...\n";
    g.blockNode(4); // Block node 4 due to obstacle
    g.shortestPath(source, target);

    return 0;
}
