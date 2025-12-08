#pragma once
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

static const double MAX_BATTERY_CAPACITY = 10.0;
static const double MIN_RESIDUAL_SOC = 1;
static const double CHARGING_EFFICIENCY = .8;
static const double DISCHARGING_EFFICIENCY = .9;

struct Node {
  int id;
  bool is_charging_station; // true if in C
  bool is_target_site;      // true if in S
  int threat_level;         // 0-10 level of threat
};

/**
 * Vector2D:
 * A structure representing a two-dimensional vector, typically used for
 * horizontal (XY plane) movement or forces in the drone's environment.
 * Examples include horizontal speed vector (v_xy) and wind dynamics (w_xy).
 */
struct Vector2D {
  // Component along the first axis (e.g., North/South or standard X)
  double x;

  // Component along the second axis (e.g., East/West or standard Y)
  double y;

  // Optional: Include common vector operations (Dot_Product, magnitude, etc.)
  // as member functions or external helper functions.
};

/**
 * Regression_Coefficients:
 * A structure to hold the nine regression coefficients (beta1 through beta9)
 * used in the multi-variate linear model for estimating drone battery power
 * consumption (P̂). These coefficients account for horizontal movement, vertical
 * movement, and environmental factors (payload weight, wind dynamics).
 */
struct Regression_Coefficients {
  // Coefficients for Horizontal Motion (typically related to speed and
  // acceleration)
  double beta1; // Coefficient for horizontal speed magnitude (||v_xy||)
  double beta2; // Coefficient for horizontal acceleration magnitude (||a_xy||)
  double beta3; // Coefficient for product of horizontal speed and acceleration
                // (||v_xy||*||a_xy||)

  // Coefficients for Vertical Motion (typically related to speed and
  // acceleration in the z-direction)
  double beta4; // Coefficient for vertical speed magnitude (||v_z||)
  double beta5; // Coefficient for vertical acceleration magnitude (||a_z||)
  double beta6; // Coefficient for product of vertical speed and acceleration
                // (||v_z||*||a_z||)

  // Coefficients for Mass and Environmental Factors
  double beta7; // Coefficient for payload weight (m)
  double beta8; // Coefficient for wind dynamics (v_xy . w_xy), responsible for
                // asymmetry
  double beta9; // Coefficient for the constant offset term (1)
};

// Represents a flight path (sequence of Nodes)
using FlightPath = std::vector<Node>;

// Represents distances (matrix or map)
using DistanceMatrix = std::map<std::pair<Node, Node>, double>;

// Charging strategy: Node -> amount of energy recharged (b(u))
using ChargingStrategy = std::map<Node, double>;

// Global constants derived from physical model and constraints [9, 10]
const double B_MAX = MAX_BATTERY_CAPACITY;   // B
const double B_MIN = MIN_RESIDUAL_SOC;       // B
const double ETA_C = CHARGING_EFFICIENCY;    // ηc (<= 1) [11]
const double ETA_D = DISCHARGING_EFFICIENCY; // ηd (>= 1) [11]

// U: Maximum distance span based on battery capacity [10]
const double U_MAX = (B_MAX - B_MIN) / ETA_D;

/**
 * Calculates the energy consumption factor c_f(u, v)
 * based on the steady-state power consumption model (Equation 5)
 * divided by the assumed constant speed.
 * This factor models asymmetry due to wind direction.
 */
double Get_Energy_factor(const Node &u, const Node &v, double Constant_Speed,
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
  Vector2D Direction_uv = Get_Flight_Direction_Vector(u, v);

  // Obtain the external steady wind vector (W_xy)
  // The wind profile (w) might be uncertain (w in W) requiring max c_f [4]
  // Here, we assume a single expected wind vector W_xy
  Vector2D W_xy = Get_Current_Wind_Vector();

  // Calculate the dot product: v_xy . w_xy
  // This captures how much the wind aids or opposes the movement (wind effect
  // term in Eq. 5)
  double v_xy_dot_w_xy = Dot_Product(Direction_uv * v_xy_mag, W_xy);

  // --- 3. Calculate Steady-State Power (P̂) using linear regression model (Eq.
  // 5) ---

  // Term 1: Horizontal Motion (simplified: only magnitude term remains since
  // acceleration is 0)
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

  // Step 1: Get raw distance d(u, v) from d_input.
  double raw_distance = d_input.at({u, v});

  // Step 2: Determine the energy consumption coefficient c_f(u, v).
  // This coefficient models environmental factors like wind direction/speed,
  // making the cost asymmetric (c_f(u, v) != c_f(v, u)) [1, 3].
  double c_f_uv = Get_Energy_Factor(
      u, v); // Requires a complex lookup/calculation based on weather/payload

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

        // Calculate the edge cost c[u, v] (energy consumption)
        double cost_uv = Calculate_Energy_Cost(u, v, d_input, ETA_D);

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
    for (const auto &v : V) {
      d_hat[{s, v}] = distance[v];
    }
  }

  return d_hat;
}

// Algorithm 1 Find-plan [ V , d ] [5]
FlightPath Find_Plan(const std::vector<Node> &V,
                     const DistanceMatrix &d_input) {

  // Step 1: Compute pairwise shortest feasible distances d̂(u, v) on G0. [5, 12]
  // G0 is a weighted directed complete graph where edge lengths are based on
  // energy cost [12]. This function must account for battery constraints for
  // direct flights u -> v.
  DistanceMatrix d_hat =
      Compute_Shortest_Distances(V, d_input, ETA_D, B_MAX, B_MIN);

  // Prepare inputs for Init-distances, requiring nearest charging station
  // distances [13]
  std::map<Node, double> d_prime; // d'(u) distance to nearest CS [13]
  std::map<Node, double>
      d_double_prime;           // d''(u) shortest distance from a CS to u [13]
  std::map<Node, Node> s_prime; // s'(u) nearest CS from u [13]
  std::map<Node, Node> s_double_prime; // s''(u) nearest CS to u [13]

  Calculate_Nearest_Charging_Stations(V, d_hat, d_prime, s_prime,
                                      d_double_prime, s_double_prime);

  // Step 2-3: Compute minimum possible distance d̃(u, v) and path P(u, v) [6,
  // 14]
  DistanceMatrix d_tilde;
  std::map<std::pair<Node, Node>, FlightPath> P;

  for (const auto &u : V) {
    for (const auto &v : V) {
      std::pair<double, FlightPath> result =
          Init_Distances(V, d_hat, u, v, d_prime, s_prime, d_double_prime,
                         s_double_prime, U_MAX);
      d_tilde[{u, v}] = result.first;
      P[{u, v}] = result.second;
    }
  }

  // Step 4: Consider the weighted directed graph G based on d̃ [5]
  // (Implicitly used by the Hungarian Algorithm call)

  // Step 5: F0 ← find a tour using the Hungarian algorithm on G [5]
  // This finds the tour based on the shortest feasible path distances d̃.
  FlightPath F0 = Hungarian_Algorithm(V, d_tilde);

  // Step 6: F ← Fix-plan[G, F0] (Converts F0 to a feasible tour for SDFP) [5,
  // 7]
  FlightPath F = Fix_Plan(F0, P, s_prime, s_double_prime);

  // Step 7: b′(·)← Fix-charge[F, b(·)] (Finds minimal recharging requirements)
  // [5, 7]
  ChargingStrategy b_prime = Fix_Charge(F, d_hat, d_input);

  // Return the resulting flight plan F and minimal charge strategy b_prime
  // Note: Pseudocode returns (F, b'(·)). We return F and assume b_prime is
  // managed externally.
  return F;
}

// Algorithm 2 Init-distances [ V , d̂, u, v ] [6]
std::pair<double, FlightPath>
Init_Distances(const std::vector<Node> &V, const DistanceMatrix &d_hat,
               const Node &u, const Node &v,
               const std::map<Node, double> &d_prime,
               const std::map<Node, Node> &s_prime,
               const std::map<Node, double> &d_double_prime,
               const std::map<Node, Node> &s_double_prime, double U) {

  // Check if direct flight is feasible [14]
  // Requires sufficient energy at u to reach v AND sufficient residual charge
  // at v to reach its nearest charging station (s'(v)) [14, 15]
  if (d_hat.at({u, v}) <= U - d_double_prime.at(u) - d_prime.at(v)) {
    // Line 2: d̃(u, v) ← d̂(u, v), P(u, v) ← {(u, v)} [6]
    FlightPath P_uv = {u, v};
    return {d_hat.at({u, v}), P_uv};
  } else {
    // Line 5: Construct a weighted directed graph G using only charging
    // stations (C) plus u and v, focusing on paths that are feasible between
    // charging stops [6, 14]. Edges between charging stations (z, z') must
    // satisfy d̂(z, z') <= U [6].

    // This simulates finding the shortest route that utilizes intermediate
    // refueling stops to maintain feasibility [14].

    // Simplified pseudocode abstraction for complex graph construction and
    // shortest path calculation:
    FlightPath shortest_path;
    double shortest_distance = Find_Shortest_Path_Through_Charging_Stations(
        V, d_hat, u, v, U, d_double_prime.at(u), d_prime.at(v), shortest_path);

    // Line 7: d̃(u, v) ← length of P(u, v) [16]
    // Line 8: return (d̃(u, v), P(u, v)) [16]
    return {shortest_distance, shortest_path};
  }
}

// Algorithm 3 Fix-plan [ G, F0 ] [7]
FlightPath Fix_Plan(const FlightPath &F0,
                    const std::map<std::pair<Node, Node>, FlightPath> &P,
                    const std::map<Node, Node> &s_prime,
                    const std::map<Node, Node> &s_double_prime) {

  FlightPath F;

  // Line 2-3: Replace edges (u, v) in F0 with the corresponding shortest
  // feasible path P(u, v) [7, 17] Assuming F0 is represented as a sequence of
  // nodes (F1, F2, ..., F|F|)
  for (size_t k = 0; k < F0.size() - 1; ++k) {
    Node u = F0[k];
    Node v = F0[k + 1];
    FlightPath path_segment = P.at({u, v});
    F.insert(F.end(), path_segment.begin(), path_segment.end());
  }

  // Ensure F ends at v0 if F0 was a tour [9] (Handling sequence overlap and
  // endpoints omitted for brevity)

  // Line 4: Add to F a set of sub-tours (round trips to closest charging
  // stations) [7, 17] These ensure that every required site visit (u) is
  // immediately followed by a refueling option.
  FlightPath initial_F = F;

  for (const auto &u : F0) {        // Iterate over major sites/stops in F0
    Node s_u_prime = s_prime.at(u); // Nearest CS from u [13]
    Node s_u_double_prime = s_double_prime.at(u); // Nearest CS to u [13]

    // Sub-tour: {(u, s′u), (s′u, s′′u), (s′′u, u)} [7]
    // (Simplified replacement/insertion logic for readability)
    Insert_Sub_Tour_At_Node(F, u, s_u_double_prime, s_u_prime);
  }

  // Line 5-7: Greedily drop added charging stations while feasibility is
  // maintained [7, 17] The feasibility check implicitly relies on the ability
  // of the resulting tour to meet SoC constraints B <= xk <= B [9, 12].

  // This requires iterating through all inserted sub-tours and attempting
  // removal.
  Remove_Redundant_Sub_Tours(F);

  return F;
}

// Algorithm 4 Fix-charge [F, b(·)] [7]
ChargingStrategy Fix_Charge(const FlightPath &F, const DistanceMatrix &d_hat,
                            const DistanceMatrix &d_original) {

  // Identify charging stations in sequence along tour F [7]
  std::vector<Node> charging_stops;
  for (const auto &node : F) {
    // Assuming charging stations are pre-identified
    if (node.is_charging_station) {
      charging_stops.push_back(node);
    }
  }
  size_t r = charging_stops.size(); // Number of charging stops [7]

  // Calculate energy consumption (distance D_j) between charging points (or
  // base/endpoint) [7] Note: The energy consumption D_j is derived from
  // distances d and cost factors c_f (Source 18, 36) D_j = ηd *
  // sum_{k=ij}^{ij+1 - 1} c_f(Fk, Fk+1) * d(Fk, Fk+1) [7]

  std::vector<double> D(r + 1); // D0 to Dr
  // Calculation of D_j segments (omitted detailed segment integration for
  // brevity)
  Calculate_Energy_Consumption_Segments(F, d_original, D);

  // B_j is used in Lemma 2/Proof structure (related to total charged amount)
  // [18, 19] Since we are finding the minimal charge, we often use a simplified
  // approach based on cumulative consumption and minimum SoC requirement.

  ChargingStrategy b_prime; // Initialize minimal charge amounts to zero

  // Line 6: Iterate backwards to find minimum charge required at each station
  // [18] This greedy approach ensures the drone can successfully complete all
  // subsequent legs while maintaining SoC >= B_MIN.

  double required_residual_charge = 0.0;

  for (int j = r; j >= 1; --j) {
    Node current_charge_stop = charging_stops[j - 1];

    // Add energy required for the segment immediately following this charge
    // stop
    required_residual_charge +=
        D[j]; // D[j] is the consumption for the segment F_{i_j} to F_{i_{j+1}}

    // Total capacity required is required_residual_charge, adjusted by B_MIN
    // offset. The maximum charge needed is constrained by the initial charge
    // (x0=B) and subsequent segments.

    // Calculate needed charge amount [18] (Simplified representation of line 7
    // logic)
    double total_needed_energy = (required_residual_charge + B_MIN) - B_MAX;

    // Ensure b'(Fij) is non-negative [18]
    double minimal_charge_needed = std::max(0.0, total_needed_energy / ETA_C);

    b_prime[current_charge_stop] = minimal_charge_needed;

    // Update required residual charge for the preceding segment
    // (Subtract charge added here, and incorporate energy consumed in D[j-1])
    required_residual_charge -= minimal_charge_needed * ETA_C;
    required_residual_charge +=
        D[j - 1]; // Consumption D[j-1] is for segment F_{i_{j-1}} to F_{i_j}

    // Constraint check: current accumulated residual charge cannot exceed B_MAX
    required_residual_charge = std::min(required_residual_charge, B_MAX);
  }

  // Line 10: return b′(·) [18]
  return b_prime;
}