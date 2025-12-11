#pragma once
#include <iostream>
#include <math.h>
#include <string>
/**
 * Vector2D:
 * A structure representing a two-dimensional vector, typically used for
 * horizontal (XY plane) movement or forces in the drone's environment.
 * Examples include horizontal speed vector (v_xy) and wind dynamics (w_xy).
 */
struct Vector2D {
  Vector2D() : x(0), y(0) {}
  Vector2D(double x, double y) : x(x), y(y) {}
  // Component along the first axis (e.g., North/South or standard X)
  double x;

  // Component along the second axis (e.g., East/West or standard Y)
  double y;

  // Optional: Include common vector operations (Dot_Product, magnitude, etc.)
  // as member functions or external helper functions.
  double dot(const Vector2D &other) const { return x * other.x + y * other.y; }

  // mult
  Vector2D operator*(double mult) const { return Vector2D(x * mult, y * mult); }
  // sub
  Vector2D operator-(const Vector2D &other) const {
    return {x - other.x, y - other.y};
  }

  Vector2D direction(const Vector2D &other) const {
    return Vector2D(x - other.x, y - other.y);
  }

  double magnitude() const { return sqrt(x * x + y * y); };

  Vector2D normalize() const {
    double len = magnitude();
    if (len == 0)
      return {0, 0};
    return {x / len, y / len};
  }
};

struct Node {
  Node()
      : name("undefined"), id(0), is_charging_station(false),
        is_target_site(false), threat_level(0){};
  Node(std::string name, int id, bool charging, bool target, int threat)
      : name(name), id(id), is_charging_station(charging),
        is_target_site(target), threat_level(threat){};
  std::string name;
  int id;
  bool is_charging_station; // true if in C
  bool is_target_site;      // true if in S
  int threat_level;         // 0-10 level of threat
  //   Vector2D location;
  // TODO: make smarter if needed
  bool operator==(const Node &other) { return id == other.id; };
  bool operator<(const Node &other) const { return id < other.id; }
  void print() const {
    std::cout << std::boolalpha << name << ", " << std::to_string(id) << ", "
              << is_charging_station << ", " << is_target_site << ", "
              << std::to_string(threat_level) << std::endl;
  }
};

/**
 * Regression_Coefficients:
 * A structure to hold the nine regression coefficients (beta1 through beta9)
 * used in the multi-variate linear model for estimating drone battery power
 * consumption (P̂). These coefficients account for horizontal movement,
 * vertical movement, and environmental factors (payload weight, wind
 * dynamics).
 */
struct Regression_Coefficients {
  Regression_Coefficients()
      : beta1(1), beta2(1), beta3(1), beta4(0), beta5(0), beta6(0), beta7(1),
        beta8(0), beta9(0){};
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
  double beta8; // Coefficient for wind dynamics (v_xy . w_xy), responsible
                // for asymmetry
  double beta9; // Coefficient for the constant offset term (1)
};

// TODO: implement
Vector2D Get_Flight_Direction_Vector(const Node &u, const Node &v) {
  return Vector2D();
}

// TODO: implement
Vector2D Get_Current_Wind_Vector(const Node &u, const Node &v) {
  return Vector2D(); // will probably provide a random wind along this vector,
                     // or lookup from file
}
