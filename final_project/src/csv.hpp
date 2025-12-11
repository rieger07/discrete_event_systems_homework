#include "paper_algos.hpp"
#include "vectormath.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

bool parseBool(std::string &in) { return (in == "true"); }

std::vector<Node> readNodesFromCSV(const std::string &filename) {
  std::vector<Node> nodes;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return nodes;
  }

  std::string line;

  // Always skip the first line (header)
  if (!std::getline(file, line)) {
    return nodes; // empty file
  }

  // Process remaining lines
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    std::string name;
    int id;
    bool charging;
    bool target;
    int threat;

    // Parse name
    if (!std::getline(ss, name, ','))
      continue;

    // Parse id
    if (!std::getline(ss, token, ','))
      continue;
    id = std::stoi(token);
    // Parse charging
    if (!std::getline(ss, token, ','))
      continue;
    charging = parseBool(token);
    // Parse target
    if (!std::getline(ss, token, ','))
      continue;
    target = parseBool(token);
    // Parse threat
    if (!std::getline(ss, token, ','))
      continue;
    threat = std::stoi(token);

    nodes.push_back(Node(name, id, charging, target, threat));
  }

  return nodes;
}

DistanceMatrix readEdgesFromCSV(std::string filename,
                                const std::vector<Node> &nodes) {
  DistanceMatrix matrix;
  std::map<std::string, Node> m;
  for (const auto &n : nodes) {
    m[n.name] = n;
  }

  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return matrix;
  }

  std::string line;

  // Always skip the first line (header)
  if (!std::getline(file, line)) {
    return matrix; // empty file
  }

  // Process remaining lines
  int i = 0;
  while (std::getline(file, line)) {
    i++;
    std::stringstream ss(line);
    std::string token;
    std::string from;
    std::string to;
    int cost;

    // Parse name
    if (!std::getline(ss, from, ','))
      continue;

    // Parse id
    if (!std::getline(ss, to, ','))
      continue;

    // Parse charging
    if (!std::getline(ss, token, ','))
      continue;
    cost = std::stoi(token);
    matrix[{m[from], m[to]}] = cost;
    matrix[{m[to], m[from]}] = cost;

    // std::cout << "iteration " << i << " " << m[from].name << m[to].name
    //           << std::to_string(cost) << " " << matrix.size() << std::endl;
  }

  return matrix;
}