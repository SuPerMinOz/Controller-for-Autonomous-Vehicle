#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>

// Kruskal’s Algorithm builds the spanning tree by adding edges one by one into a growing spanning tree. 
// Kruskal's algorithm follows greedy approach as in each iteration it finds an edge which has least weight and add it to the growing spanning tree.

// Algorithm Steps:
// 1. Sort the graph edges with respect to their weights.
// 2. Start adding edges to the MST(Minimum Spanning Tree) from the edge with the smallest weight until the edge of the largest weight.
// 3. Only add edges which doesn't form a cycle , edges which connect only disconnected components.
// So now the question is how to check if vertices are connected or not ?
// This could be done using DFS which starts from the first vertex, then check if the second vertex is visited or not. 
// But DFS will make time complexity large as it has an order of where is the number of vertices, is the number of edges. 
// So the best solution is "Disjoint Sets": 并查集
// Disjoint sets are sets whose intersection is the empty set so it means that they don't have any element in common.

// Time Complexity:
// In Kruskal’s algorithm, most time consuming operation is sorting because the total complexity of the Disjoint-Set operations will be O(ElogV), 
// which is the overall Time Complexity of the algorithm.

using namespace std;

const int nodes = 5, edges = 7;
vector<int> id(edges);
vector<pair<int, pair<int, int>>> p(edges);

void initialize() {
  for (int i = 0; i < edges; ++i) {
    id[i] = i;
  }
}

int root(int x) {
  while (id[x] != x) {
    id[x] = id[id[x]];
    x = id[x];
  }
  return x;
}

void join(int x, int y) {
  int p = root(x);
  int q = root(y);
  id[p] = id[q];
}

int kruskal(vector<pair<int, pair<int, int>>> &p) {
  int x, y;
  int cost, minimumCost = 0;
  for (int i = 0; i < edges; ++i) {
    // Selecting edges one by one in increasing order from the beginning
    x = p[i].second.first;
    y = p[i].second.second;
    cost = p[i].first;
    // Check if the selected edge is creating a cycle or not 并查集
    if (root(x) != root(y)) {
      minimumCost += cost;
      cout << "Current edge info: from " << x + 1 << " to " << y + 1 << " cost is " << cost << endl;
      join(x, y);
    }
  }
  return minimumCost;
}

int main() {
  int minimumCost;
  initialize();
  p[0] = {1, {0, 1}};
  p[1] = {7, {0, 2}};
  p[2] = {5, {1, 2}};
  p[3] = {4, {1, 3}};
  p[4] = {3, {1, 4}};
  p[5] = {6, {2, 4}};
  p[6] = {2, {3, 4}};
  // Sort the edges in the ascending order
  sort(p.begin(), p.end(), [](pair<int, pair<int, int>> &a, pair<int, pair<int, int>> &b) {
    return a.first < b.first;
  });
  minimumCost = kruskal(p);
  cout << "Final cost: " << minimumCost << endl;
  return 0;
}