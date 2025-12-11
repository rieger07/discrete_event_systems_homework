#pragma once
#include "config.hpp"
#include "distance.hpp"
#include "vectormath.hpp"

#include <algorithm>
#include <assert.h>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

// Represents a flight path (sequence of Nodes)
using FlightPath = std::vector<Node>;

// Charging strategy: Node->CS -> amount of energy needed recharged (b(u))
using ChargingStrategy = std::vector<std::pair<Node, double>>;

void printFlightWithChargingStrat(
    std::pair<FlightPath, ChargingStrategy> results, DistanceMatrix &d_hat) {
  auto config = get_battery_config();
  double battery_level = config.MAX_BATTERY_CAPACITY;
  FlightPath F = results.first;
  ChargingStrategy b_prime = results.second;
  std::cout << std::endl
            << "Drone starts out at maximum battery charge of: "
            << config.MAX_BATTERY_CAPACITY << std::endl;
  std::cout << std::endl << "Displaying final tour:" << std::endl;
  int cs = 0;
  for (auto it = F.begin(); it != F.end(); it++) {
    if (std::next(it) == F.end()) {
      std::cout << "Completed tour." << std::endl;
      break;
    }
    double cost = d_hat.at({*it, *(it + 1)});
    battery_level -= cost;
    std::cout << "Traveling from " << it->name << " to " << (it + 1)->name
              << " using " << cost << " battery units."
              << " " << battery_level << " battery units remain." << std::endl;

    if ((it + 1)->is_charging_station) {
      auto charging = b_prime.at(cs);
      battery_level = charging.second;
      std::cout << "Drone is now at charging station " << charging.first.name
                << ". Charging to " << charging.second
                << " battery units according to charging strategy. Current "
                   "battery level: "
                << battery_level << std::endl;
      cs++;
    }
  }
};

FlightPath::iterator
Insert_Sub_Tour_At_Node(FlightPath &F, FlightPath::iterator &it, const Node &u,
                        Node &s_u_double_prime, Node &s_u_prime) {
  // find the node in the vector
  // auto it = std::find(F.begin(), F.end(), u);
  if (it == F.end()) {
    throw("Node wasn't part of F");
  } else {
    // travel to charging station
    it = F.insert(it + 1, s_u_prime); // u -> s_uprime
    // travel back to node
    it = F.insert(it + 1, u); // u -> next node
  }
  return it + 1;
}

bool checkFeasible(FlightPath &F, const DistanceMatrix &d_tilde) {
  auto config = get_battery_config();
  double currentBattery = config.B_MAX;
  // double maxCost = U_MAX;
  for (FlightPath::iterator currentNode = F.begin(); currentNode != F.end();
       currentNode++) {
    std::cout << "Starting Battery @ " << currentNode->name << ": "
              << currentBattery << std::endl;
    if (std::next(currentNode) == F.end()) {
      // // next stop is the starting node
      // double usage = d_tilde.at({*currentNode, *F.begin()});
      // currentBattery -= d_tilde.at({*currentNode, *F.begin()});
      // std::cout << "Battery Used (" << currentNode->name << "->"
      //           << F.begin()->name << "): " << usage << std::endl;
      // std::cout << "Current Battery (" << currentNode->name << "->"
      //           << F.begin()->name << "): " << currentBattery << std::endl;
      // if (currentBattery < B_MIN) {
      //   return false;
      // }

      std::cout << "Done" << std::endl;
      break;
    } else {
      double usage = d_tilde.at({*currentNode, *(currentNode + 1)});
      currentBattery -= usage;
      std::cout << "Battery Used (" << currentNode->name << "->"
                << (currentNode + 1)->name << "): " << usage << std::endl;
      // std::cout << "Current Battery (" << (currentNode + 1)->name
      //           << "): " << currentBattery << std::endl;
      if (currentBattery < config.B_MIN) {
        return false;
      }
      if ((currentNode + 1)->is_charging_station) {
        currentBattery = config.B_MAX;
        std::cout << "Charged up @ " << (currentNode + 1)->name << " to "
                  << config.B_MAX << std::endl;
      }
    }
  }
  return true;
}

int numUniqueSites(FlightPath &F) {
  std::set<Node> sites;
  for (auto n : F) {
    sites.emplace(n);
  }
  return sites.size();
}

bool checkValid(FlightPath &attempt, FlightPath &original) {

  int all_sites = numUniqueSites(original);
  int new_sites = numUniqueSites(attempt);
  return all_sites == new_sites;
};

void Remove_Redundant_Sub_Tours(FlightPath &F, const DistanceMatrix &d_tilde) {
  bool feasible = checkFeasible(F, d_tilde);
  std::cout << std::endl
            << "Feasible with subroutes: " << feasible << std::endl;
  if (!feasible) {
    throw("Tour not feasible even with charging subtours");
  }
  // greedily remove charging loops
  // pattern like Node -> CS -> Node -> Next Node -> CS -> Node -> Next Node...
  FlightPath f_new = F;
  // get the num sites
  // size_t num_sites = std::count_if(f_new.begin(), f_new.end(),
  //                                  [](Node &n) { return n.is_target_site; });
  size_t i = 0;
  std::vector<bool> feasibles;
  while (i < f_new.size() - 1) {
    FlightPath temp = f_new;
    temp.erase(temp.begin() + i + 1);
    temp.erase(temp.begin() + i + 1);
    std::cout << std::endl << "Trying to remove subtour..." << std::endl;
    feasible = checkFeasible(temp, d_tilde);
    feasibles.push_back(feasible);
    if (feasible) {
      bool valid = checkValid(temp, F);
      if (!valid) {
        std::cout << "missing required sites" << std::endl;
        i = i + 3;
        continue;
      }
      i++;
      f_new = temp;
      continue;
    } else {
      std::cout << "not feasible" << std::endl;
      i = i + 3;
    }
  }
  F = f_new;
  if (!std::any_of(feasibles.begin(), feasibles.end(),
                   [](bool i) { return i; })) {
    throw("No feasible plans");
  }
};

void Calculate_Energy_Consumption_Segments(const FlightPath &F,
                                           const DistanceMatrix &d_original,
                                           std::vector<double> &D) {

  for (auto it = F.cbegin(); it != F.cend();) {
    // we aren't doing complex energy calculations, just what the edge was from
    // the graph
    if (std::next(it) == F.end()) {
      break;
    }
    Node u = *it;
    Node v = *(it + 1);
    D.push_back(d_original.at({u, v}));
    std::advance(it, 2);
  }
}

// Algorithm 4 Fix-charge [F, b(·)] [7]
ChargingStrategy Fix_Charge(const FlightPath &F, const DistanceMatrix &d_hat,
                            const DistanceMatrix &d_original) {
  auto config = get_battery_config();
  ChargingStrategy b_prime;
  // Identify charging costs up to charging stations along tour F
  std::vector<double> tours;
  std::vector<size_t> tour_v_index;
  size_t tour_num = 0;
  size_t idx = 1;
  for (auto it = F.cbegin(); it != F.cend(); it++) {
    Node u = *it;
    if (std::next(it) == F.cend()) {
      break;
    }
    Node v = *(it + 1);
    if (tours.size() != tour_num + 1) {
      tours.push_back(0);
    }
    tours[tour_num] += d_hat.at({u, v});
    // Assuming charging stations are pre-identified
    if (v.is_charging_station) {
      tour_num++;
      tour_v_index.push_back(idx);
    }
    idx++;
  }

  // Using this information, calculate the charging strategy that requires the
  // minimum amount of charge along the flight path
  int cs = 0;

  for (Node n : F) {
    if (n.is_charging_station) {
      if (cs == 0) {
        // double current_charge = config.B_MAX - tours[cs];
        double needed_charge = tours[cs + 1] + config.MIN_RESIDUAL_SOC;
        assert(needed_charge <= config.U_MAX);
        b_prime.push_back({n, needed_charge});
        cs++;
      } else {
        // double current_charge = b_prime[cs - 1].second - tours[cs];
        double needed_charge = tours[cs + 1] + config.MIN_RESIDUAL_SOC;
        assert(needed_charge <= config.U_MAX);
        b_prime.push_back({n, needed_charge});
        cs++;
      }
    }
  }

  // size_t r = charging_stops.size(); // Number of charging stops [7]

  // // Calculate energy consumption (distance D_j) between charging points (or
  // // base/endpoint) [7] Note: The energy consumption D_j is derived from
  // // distances d and cost factors c_f (Source 18, 36) D_j = ηd *
  // // sum_{k=ij}^{ij+1 - 1} c_f(Fk, Fk+1) * d(Fk, Fk+1) [7]

  // std::vector<double> D(r + 1); // D0 to Dr
  // // Calculation of D_j segments (omitted detailed segment integration for
  // // brevity)
  // Calculate_Energy_Consumption_Segments(F, d_original, D);

  // // B_j is used in Lemma 2/Proof structure (related to total charged amount)
  // // [18, 19] Since we are finding the minimal charge, we often use a
  // simplified
  // // approach based on cumulative consumption and minimum SoC requirement.

  // // Line 6: Iterate backwards to find minimum charge required at each
  // station
  // // [18] This greedy approach ensures the drone can successfully complete
  // all
  // // subsequent legs while maintaining SoC >= B_MIN.

  // double required_residual_charge = 0.0;

  // for (int j = r; j >= 1; --j) {
  //   Node current_charge_stop = charging_stops[j - 1];

  //   // Add energy required for the segment immediately following this charge
  //   // stop
  //   required_residual_charge +=
  //       D[j]; // D[j] is the consumption for the segment F_{i_j} to
  //       F_{i_{j+1}}

  //   // Total capacity required is required_residual_charge, adjusted by B_MIN
  //   // offset. The maximum charge needed is constrained by the initial charge
  //   // (x0=B) and subsequent segments.

  //   // Calculate needed charge amount [18] (Simplified representation of line
  //   7
  //   // logic)
  //   double total_needed_energy = (required_residual_charge + B_MIN) - B_MAX;

  //   // Ensure b'(Fij) is non-negative [18]
  //   double minimal_charge_needed = std::max(0.0, total_needed_energy /
  //   ETA_C);

  //   b_prime[current_charge_stop] = minimal_charge_needed;

  //   // Update required residual charge for the preceding segment
  //   // (Subtract charge added here, and incorporate energy consumed in
  //   D[j-1]) required_residual_charge -= minimal_charge_needed * ETA_C;
  //   required_residual_charge +=
  //       D[j - 1]; // Consumption D[j-1] is for segment F_{i_{j-1}} to F_{i_j}

  //   // Constraint check: current accumulated residual charge cannot exceed
  //   B_MAX required_residual_charge = std::min(required_residual_charge,
  //   B_MAX);
  // }

  // Line 10: return b′(·) [18]
  return b_prime;
}

// Algorithm 3 Fix-plan [ G, F0 ] [7]
FlightPath Fix_Plan(const FlightPath &F0,
                    const std::map<std::pair<Node, Node>, FlightPath> &P,
                    const std::map<Node, Node> &s_prime,
                    const std::map<Node, Node> &s_double_prime,
                    const DistanceMatrix &d_tilde) {

  FlightPath F = F0;

  // Line 2-3: Replace edges (u, v) in F0 with the corresponding shortest
  // feasible path P(u, v) [7, 17] Assuming F0 is represented as a sequence of
  // nodes (F1, F2, ..., F|F|)
  // for (size_t k = 0; k < F0.size() - 1; ++k) {
  //   Node u = F0[k];
  //   Node v = F0[k + 1];
  //   FlightPath path_segment = P.at({u, v});
  //   F.insert(F.end(), path_segment.begin(), path_segment.end());
  // }

  // Ensure F ends at v0 if F0 was a tour [9] (Handling sequence overlap and
  // endpoints omitted for brevity)

  // Line 4: Add to F a set of sub-tours (round trips to closest charging
  // stations) [7, 17] These ensure that every required site visit (u) is
  // immediately followed by a refueling option.
  FlightPath initial_F = F0;
  bool feasible = checkFeasible(initial_F, d_tilde);
  std::cout << "Feasible with no subroutes: " << feasible << std::endl;

  FlightPath::iterator it = F.begin();
  size_t last_node_idx = initial_F.size();
  size_t count = 0;
  for (const auto &u : F0) { // Iterate over major sites/stops in F0
    count++;
    if (count == last_node_idx) {
      break;
    }
    Node s_u_prime = s_prime.at(u);               // Nearest CS from u [13]
    Node s_u_double_prime = s_double_prime.at(u); // Nearest CS to u [13]

    // Sub-tour: {(u, s′u), (s′u, s′′u), (s′′u, u)} [7]
    // (Simplified replacement/insertion logic for readability)
    it = Insert_Sub_Tour_At_Node(F, it, u, s_u_double_prime, s_u_prime);
  }

  // Line 5-7: Greedily drop added charging stations while feasibility is
  // maintained [7, 17] The feasibility check implicitly relies on the ability
  // of the resulting tour to meet SoC constraints B <= xk <= B [9, 12].

  // This requires iterating through all inserted sub-tours and attempting
  // removal.
  Remove_Redundant_Sub_Tours(F, d_tilde);

  return F;
}

/**
 *
 */
void Calculate_Nearest_Charging_Stations(const std::vector<Node> &V,
                                         DistanceMatrix d_hat,
                                         std::map<Node, double> &d_prime,
                                         std::map<Node, Node> &s_prime,
                                         std::map<Node, double> &d_double_prime,
                                         std::map<Node, Node> &s_double_prime) {
  // Identify charging stations in sequence along tour F [7]
  std::vector<Node> charging_stops;
  std::vector<Node> destinations;
  for (const auto &node : V) {
    // Assuming charging stations are pre-identified
    if (node.is_charging_station) {
      charging_stops.push_back(node);
    } else {
      destinations.push_back(node);
    }
  }

  // d_prime and d_double prime are the same since we removed asymmetric travel
  // same goes for s_prime and s_double_prime
  for (const auto &node : destinations) {
    int min_dist = 999;
    for (const auto &c : charging_stops) {
      // if there is no path to the charging stop
      if (d_hat.find({node, c}) == d_hat.end()) {
        continue;
      }
      int dist = d_hat.at({node, c});
      if (dist < min_dist) {
        d_prime[node] = dist;
        d_double_prime[node] = dist;

        s_prime[node] = c;
        s_double_prime[node] = c;
        min_dist = dist;
      }
    }
  }
  for (const auto &node : charging_stops) {
    d_prime[node] = 0;
    d_double_prime[node] = 0;
    s_prime[node] = node;
    s_double_prime[node] = node;
  }
};

double Find_Shortest_Path_Through_Charging_Stations(
    const std::vector<Node> &V, const DistanceMatrix &d_hat, const Node &u,
    const Node &v, double U, double shortest_from_cs_to_u,
    double distance_to_nearest_cs, const Node &s_u_prime,
    FlightPath &shortest_path) {
  // TODO: implement
  // Cost to go from u -> CS -> v
  double cost = shortest_from_cs_to_u + d_hat.at({s_u_prime, v});
  // std::cout << u.name << "->" << s_u_prime.name << "->" << v.name << ":" <<
  // cost
  //           << std::endl;
  if (cost > U) {
    std::cout << "shit" << std::endl;
  }
  shortest_path.push_back(u);
  shortest_path.push_back(s_u_prime);
  shortest_path.push_back(v);
  return cost;
};
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
  int dist = d_hat.at({u, v});
  int battery_min_cost = U - d_double_prime.at(u) - d_prime.at(v);
  // std::cout << dist << ":" << battery_min_cost << std::endl;
  if (dist <= (U - d_double_prime.at(u) - d_prime.at(v))) {
    // std::cout << "I can make it on my own thx" << std::endl;
    // Line 2: d̃(u, v) ← d̂(u, v), P(u, v) ← {(u, v)} [6]
    FlightPath P_uv = {u, v};
    return {d_hat.at({u, v}), P_uv};
  } else {
    std::cout << "Making a subroute" << std::endl;
    // Line 5: Construct a weighted directed graph G using only charging
    // stations (C) plus u and v, focusing on paths that are feasible between
    // charging stops [6, 14]. Edges between charging stations (z, z') must
    // satisfy d̂(z, z') <= U [6].

    // This simulates finding the shortest route that utilizes intermediate
    // refueling stops to maintain feasibility [14].

    // Instead of going directly to v, go to nearest charging station directly,
    // then try to go to v
    FlightPath shortest_path;
    double shortest_distance = Find_Shortest_Path_Through_Charging_Stations(
        V, d_hat, u, v, U, d_double_prime.at(u), d_prime.at(u), s_prime.at(u),
        shortest_path);

    // Line 7: d̃(u, v) ← length of P(u, v) [16]
    // Line 8: return (d̃(u, v), P(u, v)) [16]
    return {shortest_distance, shortest_path};
  }
}

// Algorithm 1 Find-plan [ V , d ] [5]
std::pair<FlightPath, ChargingStrategy>
Find_Plan(const std::vector<Node> &V, const DistanceMatrix &d_input) {
  auto config = get_battery_config();
  // Step 1: Compute pairwise shortest feasible distances d̂(u, v) on G0. [5, 12]
  // G0 is a weighted directed complete graph where edge lengths are based on
  // energy cost [12]. This function must account for battery constraints for
  // direct flights u -> v.
  DistanceMatrix d_hat = Compute_Shortest_Distances(V, d_input, config.ETA_D,
                                                    config.B_MAX, config.B_MIN);

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
                         s_double_prime, config.U_MAX);
      d_tilde[{u, v}] = result.first;
      P[{u, v}] = result.second;
    }
  }
  // print out d_hat
  for (const auto &temp : d_tilde) {
    std::cout << temp.first.first.name << "->" << temp.first.second.name << ": "
              << temp.second << std::endl;
  }
  // Step 4: Consider the weighted directed graph G based on d̃ [5]
  // (Implicitly used by the Hungarian Algorithm call)

  // Step 5: F0 ← find a tour using the Hungarian algorithm on G [5]
  // This finds the tour based on the shortest feasible path distances d̃.
  Hungarian h(V, d_tilde);
  double cost;
  FlightPath F0 = h.solve(cost);
  // Remove Charging Stations from Initial Flight Path
  FlightPath::iterator it = F0.begin();
  while (it != F0.end()) {
    it = F0.erase(std::remove_if(F0.begin(), F0.end(),
                                 [](Node i) { return i.is_charging_station; }));
  }
  F0.push_back(*F0.begin()); // makes it a tour
  // Step 6: F ← Fix-plan[G, F0] (Converts F0 to a feasible tour for SDFP)
  // [5, 7]
  FlightPath F = Fix_Plan(F0, P, s_prime, s_double_prime, d_tilde);

  // Step 7: b′(·)← Fix-charge[F, b(·)] (Finds minimal recharging requirements)
  // [5, 7]
  ChargingStrategy b_prime = Fix_Charge(F, d_hat, d_input);
  std::cout << "printing flight path: ";
  for (const auto &n : F) {
    std::cout << n.name << "->";
  }
  std::cout << std::endl;

  std::cout << "printing charging strategy: ";
  for (const auto &n : b_prime) {
    std::cout << n.first.name << ":" << n.second << std::endl;
  }
  printFlightWithChargingStrat({F, b_prime}, d_hat);
  return {F, b_prime};
}
