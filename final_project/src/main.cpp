#include "csv.hpp"
#include "modified_algos.hpp"
#include "paper_algos.hpp"
#include <iostream>

void printPath(FlightPath f) {
  for (const auto &n : f) {
    std::cout << n.name << "->";
  }
  std::cout << std::endl;
}
void printNodes(std::vector<Node> V) {
  for (const auto &n : V) {
    n.print();
  }
  std::cout << std::endl;
}
void printMatrix(DistanceMatrix m) {
  for (const auto &row : m) {
    std::cout << row.first.first.name << "->" << row.first.second.name << ": "
              << row.second << std::endl;
  }
}

int main(int argc, char *argv[]) {
  // TODO: pull in the constants from a file to modify battery amounts,
  // efficiency etc

  std::string program = argv[1];
  int risk = 999;
  std::string node_filename;
  std::string edges_filename;
  std::string constants_filename;
  FlightPath f0;
  if (program == "original") {
    node_filename = argv[2];
    edges_filename = argv[3];
    constants_filename = argv[4];
    std::vector<Node> V = readNodesFromCSV(node_filename);
    DistanceMatrix m = readEdgesFromCSV(edges_filename, V);
    Find_Plan(V, m);
  } else {
    risk = std::atoi(argv[2]);
    node_filename = argv[3];
    edges_filename = argv[4];
    constants_filename = argv[5];
    std::vector<Node> V = readNodesFromCSV(node_filename);
    DistanceMatrix m = readEdgesFromCSV(edges_filename, V);
    // TODO: update this to the risk model
    Find_Plan_Risk(V, m, risk);
  }

  // Runs the whole paper's algorithms

  printPath(f0);
  return 0;
}