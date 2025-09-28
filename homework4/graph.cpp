#include "graph.h"

int skimCsvForNumVertices(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return -1;
  }
  std::string line;
  int max_num = -2;
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string segment;
    std::vector<int> edge_data;

    while (std::getline(ss, segment, ',')) {
      edge_data.push_back(std::stoi(segment));
    }

    if (edge_data.size() == 3) {
      int u = edge_data[0];
      // int v = edge_data[1];
      // int weight = edge_data[2];
      max_num = std::max(u, max_num);
    }
  }
  return max_num;
}
// Function to read graph edges from a CSV file
int readGraphFromCsv(const std::string &filename, Graph &graph,
                     int num_vertices) {
  graph.resize(num_vertices);
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return 1;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty())
      continue;

    std::stringstream ss(line);
    std::string segment;
    std::vector<int> edge_data;

    while (std::getline(ss, segment, ',')) {
      edge_data.push_back(std::stoi(segment));
    }

    if (edge_data.size() == 3) {
      int u = edge_data[0];
      int v = edge_data[1];
      int weight = edge_data[2];

      // Check for valid vertex indices
      if (u >= 0 && u < num_vertices && v >= 0 && v < num_vertices) {
        graph[u].push_back({v, weight});
      } else {
        std::cerr << "Warning: Skipping invalid edge (" << u << ", " << v
                  << ", " << weight << ")\n";
      }
    }
  }

  file.close();
  return 0;
}

// Function to print the graph for verification
void printGraph(const Graph &graph) {
  for (size_t i = 0; i < graph.size(); ++i) {
    std::cout << "Vertex " << i << ":";
    for (const auto &edge : graph[i]) {
      std::cout << " -> (" << edge.first << ", " << edge.second << ")";
    }
    std::cout << "\n";
  }
}