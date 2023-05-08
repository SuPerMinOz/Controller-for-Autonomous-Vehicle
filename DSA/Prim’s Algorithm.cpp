#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <utility>

// Prim’s Algorithm use Greedy approach to find the minimum spanning tree. 
// In Prim’s Algorithm we grow the spanning tree from a starting position. Unlike an edge in Kruskal's, we add vertex to the growing spanning tree in Prim's.

// Algorithm Steps:
// 1. Maintain two disjoint sets of vertices. One containing vertices that are in the growing spanning tree and other that are not in the growing spanning tree.
// 2. Select the cheapest vertex that is connected to the growing spanning tree and is not in the growing spanning tree and add it into the growing spanning tree. 
// This can be done using Priority Queues. Insert the vertices, that are connected to growing spanning tree, into the Priority Queue.
// 3. Check for cycles. To do that, mark the nodes which have been already selected and insert only those nodes in the Priority Queue that are not marked.

using namespace std;

const int nodes = 5, edges = 7;
typedef pair<int, int> PII;
vector<bool> marked(nodes, false);
vector<vector<PII>> adj(nodes);

int prim(int x) {
  priority_queue<PII, vector<PII>, greater<PII>> Q;
  int y;
  int minimumCost = 0;
  PII p;
  Q.push(make_pair(0, x)); // cost of x equals zero
  while (!Q.empty()) {
    // Select the edge with minimum weight
    p = Q.top();
    Q.pop();
    x = p.second;
    // Checking for cycle
    if (marked[x] == true) {
      continue;
    }
    minimumCost += p.first;
    cout << "Current node info: " << p.second + 1 << " cost is " << p.first << endl;
    marked[x] = true; // into growing spanning tree
    for (int i = 0; i < adj[x].size(); ++i) {
      y = adj[x][i].second;
      if (marked[y] == false) {
        Q.push(adj[x][i]);
      }
    }
  }
  return minimumCost;
}

int main()
{
  int minimumCost;
  adj[0].push_back({1, 1});
  adj[0].push_back({7, 2});
  adj[1].push_back({1, 0});
  adj[1].push_back({5, 2});
  adj[1].push_back({4, 3});
  adj[1].push_back({3, 4});
  adj[2].push_back({7, 0});
  adj[2].push_back({5, 1});
  adj[2].push_back({6, 4});
  adj[3].push_back({4, 1});
  adj[3].push_back({2, 4});
  adj[4].push_back({3, 1});
  adj[4].push_back({6, 2});
  adj[4].push_back({2, 3});
  // Selecting 1 as the starting node
  minimumCost = prim(0);
  cout << "Final cost: " << minimumCost << endl;
  return 0;
}