#include "vectormath.hpp"
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

// Represents distances (matrix or map)
using DistanceMatrix = std::map<std::pair<Node, Node>, double>;

/**
 * Calculates the energy consumption factor c_f(u, v)
 * based on the steady-state power consumption model (Equation 5)
 * divided by the assumed constant speed.
 * This factor models asymmetry due to wind direction.
 */
double Get_Energy_Factor(const Node &u, const Node &v, double Constant_Speed,
                         const Regression_Coefficients &Beta,
                         double Payload_Mass) {

  // --- 1. Define vectors and fixed parameters based on constraints ---

  // Horizontal speed magnitude (constant assumption)
  double v_xy_mag = Constant_Speed;

  // Horizontal and vertical acceleration are zero for steady flight
  double a_xy_mag = 0.0;
  // Vertical speed and acceleration are ignored for long-haul trips (per source
  // assumption)
  double v_z_mag = 0.0;
  double a_z_mag = 0.0;

  // --- 2. Determine Wind Interaction (Asymmetry Factor) ---

  // Calculate the direction vector of flight u -> v (Direction_uv)
  // This requires external location data for u and v (not provided in source
  // context, assumed retrievable)
  // Vector2D Direction_uv = u.location.direction(v.location);

  // Obtain the external steady wind vector (W_xy)
  // The wind profile (w) might be uncertain (w in W) requiring max c_f [4]
  // Here, we assume a single expected wind vector W_xy
  // Vector2D W_xy = Get_Current_Wind_Vector(u, v);

  // Calculate the dot product: v_xy . w_xy
  // This captures how much the wind aids or opposes the movement (wind effect
  // term in Eq. 5)
  // double v_xy_dot_w_xy = (Direction_uv * v_xy_mag).dot(W_xy);
  double v_xy_dot_w_xy = 0;

  // --- 3. Calculate Steady-State Power (P̂) using linear regression model
  // (Eq. 5) ---

  // Term 1: Horizontal Motion (simplified: only magnitude term remains
  // since acceleration is 0)
  double P_horizontal = Beta.beta1 * v_xy_mag;

  // Term 2: Vertical Motion (simplified: all terms are zero for horizontal
  // flight)
  double P_vertical = 0.0;

  // Term 3: Mass and Environmental Factors
  double P_factors = (Beta.beta7 * Payload_Mass) +
                     (Beta.beta8 * v_xy_dot_w_xy) +
                     (Beta.beta9 * 1.0); // Constant offset term

  // Estimated total power consumption P̂
  double P_hat_steady_state = P_horizontal + P_vertical + P_factors;

  // Ensure power is non-negative
  if (P_hat_steady_state <= 0.0) {
    P_hat_steady_state = 0.0;
  }

  // --- 4. Calculate Energy Factor c_f (Power / Speed) ---

  // c_f(u, v) = P̂ / V_const
  double c_f_uv = P_hat_steady_state / Constant_Speed;

  return c_f_uv;
}

// Definition of an edge weight based on energy consumption E(u, v)
double Calculate_Energy_Cost(const Node &u, const Node &v,
                             const DistanceMatrix &d_input, double ETA_D) {
  // This is an abstraction of the full energy model (Section IV) and the
  // linear model assumption (Equations 6 & 7 in Source [3, 7]).
  double Constant_Speed = 1.0;
  const Regression_Coefficients Beta;
  double Payload_Mass;
  // Step 1: Get raw distance d(u, v) from d_input.
  double raw_distance = d_input.at({u, v});

  // Step 2: Determine the energy consumption coefficient c_f(u, v).
  // This coefficient models environmental factors like wind direction/speed,
  // making the cost asymmetric (c_f(u, v) != c_f(v, u)) [1, 3].
  double c_f_uv =
      Get_Energy_Factor(u, v, Constant_Speed, Beta,
                        Payload_Mass); // Requires a complex lookup/calculation
                                       // based on weather/payload

  // Step 3: Calculate energy consumption E(u, v) = c_f(u, v) * d(u, v) [3].
  double E_uv = c_f_uv * raw_distance;

  // Step 4: Calculate the total energy drained (d-hat, or Delta) [8, 9].
  double d_hat_uv = ETA_D * E_uv;

  return d_hat_uv; // This energy cost is the "distance" in G0 [2].
}

// Algorithm implementation using Dijkstra's (run for all source nodes)
DistanceMatrix Compute_Shortest_Distances(const std::vector<Node> &V,
                                          const DistanceMatrix &d_input,
                                          double ETA_D, double B_MAX,
                                          double B_MIN) {

  DistanceMatrix d_hat; // Stores the resulting shortest energy costs d̂(u, v)

  // Iterate through every node in V, treating it as the source (s)
  for (const auto &s : V) {
    // std::cout << s.name << std::endl;
    // 1. Initialization for Dijkstra's [10]
    std::map<Node, double> distance; // Array d[v] of best estimates [11]
    std::set<Node> permanent_set; // Set S: Vertices whose shortest paths have
                                  // been determined [11]

    // Priority Queue (to efficiently find the node with the smallest distance
    // value) Stores pairs: {distance, node}
    auto comparator = [](const std::pair<double, Node> &a,
                         const std::pair<double, Node> &b) {
      return a.first > b.first;
    };
    std::priority_queue<std::pair<double, Node>,
                        std::vector<std::pair<double, Node>>,
                        decltype(comparator)>
        temporary_set(comparator); // Represents V-S (remainder) [11]

    // Initialize distances: d[s] = 0, d[v] = infinity for v != s [10]
    for (const auto &v : V) {
      distance[v] =
          (v.id == s.id) ? 0.0 : std::numeric_limits<double>::infinity();
    }
    temporary_set.push({0.0, s});

    // 2. Iteration (While there are still vertices in V-S) [12]
    while (!temporary_set.empty()) {
      // Get u, the closest vertex in V-S (smallest distance value) [12, 13]
      Node u = temporary_set.top().second;
      temporary_set.pop();

      // If already processed, skip
      if (permanent_set.count(u)) {
        continue;
      }

      // Add u to S (Set status label to Permanent) [12-14]
      permanent_set.insert(u);

      // Relax all adjacent vertices v still in V-S connected to u [12]
      for (const auto &v : V) {
        if (u.id == v.id || permanent_set.count(v)) {
          continue; // Skip self and already determined paths
        }
        // If there is no route
        if (d_input.find({u, v}) == d_input.end()) {
          continue;
        }

        // Calculate the edge cost c[u, v] (energy consumption)
        double cost_uv = Calculate_Energy_Cost(u, v, d_input, ETA_D);
        // std::cout << "cost: " << u.name << "->" << v.name << ":" << cost_uv
        //           << std::endl;
        // Relaxation process: if d[v] > d[u] + c[u,v] then update d[v] [15]
        if (distance[u] != std::numeric_limits<double>::infinity() &&
            distance[u] + cost_uv < distance[v]) {

          distance[v] = distance[u] + cost_uv; // Update the estimate to v [15]
          temporary_set.push(
              {distance[v], v}); // Reinsert or update in priority queue
        }
      }
    }

    // Store the computed shortest distances (energy costs) from source s
    // std::cout << "storing computed shortest distances..." << std::endl;
    for (const auto &v : V) {
      d_hat[{s, v}] = distance[v];

      //   std::cout << s.name << "->" << v.name << ":" << distance[v] <<
      //   std::endl;
    }
  }

  return d_hat;
}

class Hungarian {
public:
  Hungarian(const std::vector<Node> &nodes,
            const std::map<std::pair<Node, Node>, double> &costLookup)
      : nodes(nodes) {
    int n = nodes.size();
    cost.assign(n, std::vector<double>(n, 0.0));

    // Build cost matrix from lookup
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        auto it = costLookup.find({nodes[i], nodes[j]});
        if (it != costLookup.end()) {
          cost[i][j] = it->second;
        } else {
          cost[i][j] = 1e9; // large penalty if missing
        }
      }
    }
    N = n;
  }

  // Return vector<Node> showing assignment path
  std::vector<Node> solve(double &optimalCost) {
    std::vector<double> u(N + 1), v(N + 1);
    std::vector<int> p(N + 1), way(N + 1);

    for (int i = 1; i <= N; i++) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(N + 1, std::numeric_limits<double>::infinity());
      std::vector<bool> used(N + 1, false);
      do {
        used[j0] = true;
        int i0 = p[j0], j1 = 0;
        double delta = std::numeric_limits<double>::infinity();
        for (int j = 1; j <= N; j++) {
          if (!used[j]) {
            double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
            if (cur < minv[j]) {
              minv[j] = cur;
              way[j] = j0;
            }
            if (minv[j] < delta) {
              delta = minv[j];
              j1 = j;
            }
          }
        }
        for (int j = 0; j <= N; j++) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0);
    }

    // Build assignment as vector<Node>
    std::vector<Node> assignment(N);
    for (int j = 1; j <= N; j++) {
      if (p[j] != 0) {
        assignment[p[j] - 1] = nodes[j - 1];
      }
    }

    optimalCost = -v[0];
    return assignment;
  }

private:
  int N;
  std::vector<Node> nodes;
  std::vector<std::vector<double>> cost;
};