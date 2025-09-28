#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility> // For std::pair
#include <vector>

// Adjacency list representation of the graph
using Graph = std::vector<std::vector<std::pair<int, int>>>;
int skimCsvForNumVertices(const std::string &filename);
int readGraphFromCsv(const std::string &filename, Graph &graph,
                     int num_vertices);

void printGraph(const Graph &graph);