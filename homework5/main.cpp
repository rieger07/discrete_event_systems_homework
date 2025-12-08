#include "Eigen/Dense"
#include <iomanip>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Need to at least provide the path to the node csv, and the "
                 "number of nodes";
    return -1;
  }
  std::string filename = argv[1];
  int n_nodes = std::stoi(argv[2]);
  int start_node = std::stoi(argv[3]);

  // Adjacency Matrix
  MatrixXd adj_mat(n_nodes, n_nodes);

  // Fastener Matrix
  MatrixXd f_mat(n_nodes, n_nodes);

  // populate adjacency matrix

  // populate fastener matrix

  return 0;
}