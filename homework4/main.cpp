#include "graph.h"
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stack>
#include <vector>
using namespace std;
const int INF = std::numeric_limits<int>::max();
// Function to print the shortest path from source to a destination
// Function to print the shortest path from source to a destination
void print_shortest_path(int source, int destination,
                         const vector<int> &parent) {
  // If the destination is unreachable (parent[destination] == -1 and it's not
  // the source), or if the source and destination are the same
  if (destination == source) {
    cout << source;
    return;
  }
  if (parent[destination] == -1) {
    cout << "Path not found (unreachable)";
    return;
  }

  // Use a stack to reverse the path so it prints from source to destination
  stack<int> path;
  int current = destination;

  // Trace back from destination to source using the parent array
  while (current != -1) {
    path.push(current);
    current = parent[current];
  }

  // Print the path from the stack
  while (!path.empty()) {
    cout << path.top();
    path.pop();
    if (!path.empty()) {
      cout << " -> ";
    }
  }
}

// // Function to find the shortest paths and record predecessors
// void dijkstra(const Graph &adj, int V, int source) {
//   vector<int> dist(V, INF);
//   vector<int> parent(V, -1);
//   set<pair<int, int>> st;

//   dist[source] = 0;
//   parent[source] = -1;
//   st.insert({0, source});

//   // --- TABLE GENERATION ---
//   cout << "---------------------------------------------------------" <<
//   endl; cout << "Dijkstra's Algorithm Trace (Source Node: " << source + 1 <<
//   ")"
//        << endl;
//   cout << "---------------------------------------------------------" <<
//   endl;

//   // Print the header for the table
//   cout << "Iter | Picked |";
//   for (int i = 0; i < V; ++i) {
//     cout << " Node " << i + 1 << " (Dist, Path) |";
//   }
//   cout << endl;
//   cout << "---------------------------------------------------------" <<
//   endl;

//   // Print Initialization (Iter 0)
//   cout << " 0   |   -    |";
//   for (int i = 0; i < V; ++i) {
//     if (i == source) {
//       cout << "  0, -         |";
//     } else {
//       cout << "  INF, -       |";
//     }
//   }
//   cout << endl;

//   int iteration = 1;
//   // -------------------------

//   while (!st.empty()) {
//     pair<int, int> current = *st.begin();
//     st.erase(st.begin());

//     int u_dist = current.first;
//     int u = current.second;

//     if (u_dist > dist[u]) {
//       continue;
//     }

//     // --- TABLE ROW START ---
//     // Print the current iteration and the picked node
//     cout << setw(3) << iteration << " | " << setw(5) << u + 1 << "  |";

//     // Relaxation step
//     for (const auto &edge : adj[u]) {
//       int v = edge.first;
//       int weight = edge.second;

//       if (dist[u] + weight < dist[v]) {
//         if (dist[v] != INF) {
//           st.erase({dist[v], v});
//         }

//         // Update distance and predecessor
//         dist[v] = dist[u] + weight;
//         parent[v] = u;

//         st.insert({dist[v], v});
//       }
//     }

//     // Print the updated distance array for this iteration
//     for (int i = 0; i < V; ++i) {
//       string dist_str;
//       if (dist[i] == INF) {
//         dist_str = "INF";
//       } else {
//         dist_str = to_string(dist[i]);
//       }

//       string parent_str = (parent[i] == -1) ? "-" : to_string(parent[i] + 1);

//       // Format: Distance, Path
//       cout << " " << setw(4) << dist_str << ", " << setw(2) << parent_str
//            << "         |";
//     }
//     cout << endl;
//     // --- TABLE ROW END ---

//     iteration++;
//   }

//   // You can also add the final shortest path results here if needed.
// }

// Function to find the shortest paths and record predecessors
void dijkstra(const Graph &adj, int V, int source) {
  vector<int> dist(V, INF);

  // Parent vector to store the predecessor of each node on the shortest path.
  // Initialized to -1 to indicate no predecessor/unvisited.
  vector<int> parent(V, -1);

  set<pair<int, int>> st;

  // Initialization
  dist[source] = 0;
  parent[source] = -1; // Source has no parent
  st.insert({0, source});

  while (!st.empty()) {
    pair<int, int> current = *st.begin();
    st.erase(st.begin());

    int u_dist = current.first;
    int u = current.second;

    if (u_dist > dist[u]) {
      continue;
    }

    // Relaxation step
    for (const auto &edge : adj[u]) {
      int v = edge.first;
      int weight = edge.second;

      if (dist[u] + weight < dist[v]) {
        if (dist[v] != INF) {
          st.erase({dist[v], v});
        }

        // *** CRITICAL MODIFICATION: RECORD THE PREDECESSOR ***
        dist[v] = dist[u] + weight;
        parent[v] = u; // The shortest path to 'v' now comes from 'u'

        st.insert({dist[v], v});
      }
    }
  }

  // Output the results, including the path
  cout << "Shortest path results from source node " << source << ":" << endl;
  cout << "-------------------------------------------" << endl;
  for (int i = 0; i < V; ++i) {
    cout << "Node " << i << ": ";
    if (dist[i] == INF) {
      cout << "Unreachable" << endl;
    } else {
      cout << "Distance = " << dist[i] << ", Path: ";
      print_shortest_path(source, i, parent);
      cout << endl;
    }
  }
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " </your/filepath.csv> <optional: starting node number>"
              << std::endl;
    return 1;
  }

  int source = 0;
  if (argc > 2) {
    try {
      source = std::stoi(argv[2]);
    } catch (const std::invalid_argument &e) {
      std::cerr << "Error: The first argument is not a valid number: "
                << argv[2] << std::endl;
      return 1;
    } catch (const std::out_of_range &e) {
      std::cerr << "Error: The number is out of range: " << argv[2]
                << std::endl;
      return 1;
    }
  }

  Graph graph;
  std::string filename = argv[1];
  int num_vertices = skimCsvForNumVertices(filename);
  if (num_vertices == -2) {
    std::cerr << "Couldn't determine max node number" << std::endl;
    return 1;
  }
  if (num_vertices == -1) {
    return 1; // couldn't open the file
  }
  if (source < 0 || source > num_vertices) {
    std::cerr << "Invalid source node " << std::to_string(source)
              << "... must be between 0 and " << std::to_string(num_vertices)
              << std::endl;
    return 1;
  }
  num_vertices = num_vertices + 1; // 0 index based

  if (readGraphFromCsv(filename, graph, num_vertices)) {
    return 1;
  }

  if (graph.empty()) {
    std::cerr << "Graph was not loaded. Exiting.\n";
    return 1;
  }

  // Optional: Print the graph to verify it was loaded correctly
  printGraph(graph);

  dijkstra(graph, num_vertices, source);

  // std::cout << "\nShortest distances from source " << source << ":\n";
  // for (int i = 0; i < num_vertices; ++i) {
  //   if (distances[i] == INF) {
  //     std::cout << "To vertex " << i << ": INF\n";
  //   } else {
  //     std::cout << "To vertex " << i << ": " << distances[i] << "\n";
  //     print_shortest_path(source, i, parent)
  //   }
  // }

  return 0;
}
