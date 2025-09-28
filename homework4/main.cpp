#include "graph.h"
#include <iostream>
#include <limits>
#include <set>
#include <vector>

const int INF = std::numeric_limits<int>::max();

void dijkstra(const Graph &graph, int source, std::vector<int> &distances) {
  int num_vertices = graph.size();
  distances.assign(num_vertices, INF);

  // A set stores pairs of (distance, vertex)
  // and automatically keeps them sorted by distance
  std::set<std::pair<int, int>> active_vertices;

  distances[source] = 0;
  active_vertices.insert({0, source});

  while (!active_vertices.empty()) {
    // Extract the vertex with the minimum distance
    int u = active_vertices.begin()->second;
    active_vertices.erase(active_vertices.begin());

    // Check neighbors
    for (const auto &edge : graph[u]) {
      int v = edge.first;
      int weight = edge.second;

      // Relaxation step: a shorter path was found
      if (distances[u] != INF && distances[u] + weight < distances[v]) {
        // If v is already in the set, remove its old entry
        if (distances[v] != INF) {
          active_vertices.erase({distances[v], v});
        }

        // Update distance and insert the new, improved entry
        distances[v] = distances[u] + weight;
        active_vertices.insert({distances[v], v});
      }
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

  std::vector<int> distances;
  dijkstra(graph, source, distances);

  std::cout << "\nShortest distances from source " << source << ":\n";
  for (int i = 0; i < num_vertices; ++i) {
    if (distances[i] == INF) {
      std::cout << "To vertex " << i << ": INF\n";
    } else {
      std::cout << "To vertex " << i << ": " << distances[i] << "\n";
    }
  }

  return 0;
}
