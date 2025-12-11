#pragma once
#include "paper_algos.hpp"

void Remove_Risky_Sub_Tours(FlightPath &F, const DistanceMatrix &d_tilde,
                            double risk_tolerance) {
  std::cout << std::endl << "Looking for risky subtours..." << std::endl;
  FlightPath f_new = F;
  size_t i = 0;
  std::vector<bool> feasibles;
  while (i < f_new.size() - 1) {
    FlightPath temp = f_new;
    bool risky_charge = std::any_of(
        temp.begin() + i + 1, temp.begin() + i + 2,
        [&](const Node &i) { return i.threat_level > risk_tolerance; });
    if (risky_charge) {
      temp.erase(temp.begin() + i + 1);
      temp.erase(temp.begin() + i + 1);
      std::cout << std::endl << "Removing risky subtour..." << std::endl;
      f_new = temp;
      i++;
    } else {
      i += 3;
    }
  }
  F = f_new;
};

// Algorithm 5 Fix-plan [ G, F0 ] [7]
FlightPath Fix_Plan_Risk(const FlightPath &F0,
                         const std::map<std::pair<Node, Node>, FlightPath> &P,
                         const std::map<Node, Node> &s_prime,
                         const std::map<Node, Node> &s_double_prime,
                         const DistanceMatrix &d_tilde, double risk_tolerance) {

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

  Remove_Risky_Sub_Tours(F, d_tilde, risk_tolerance);

  // Line 5-7: Greedily drop added charging stations while feasibility is
  // maintained [7, 17] The feasibility check implicitly relies on the ability
  // of the resulting tour to meet SoC constraints B <= xk <= B [9, 12].

  // This requires iterating through all inserted sub-tours and attempting
  // removal.
  Remove_Redundant_Sub_Tours(F, d_tilde);

  return F;
}

// Algorithm 1 Find-plan [ V , d ] [5]
std::pair<FlightPath, ChargingStrategy>
Find_Plan_Risk(const std::vector<Node> &V, const DistanceMatrix &d_input,
               double risk_tolerance) {
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
  FlightPath F =
      Fix_Plan_Risk(F0, P, s_prime, s_double_prime, d_tilde, risk_tolerance);

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