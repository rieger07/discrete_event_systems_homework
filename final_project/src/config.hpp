#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

// 1. Define a struct to hold all configuration constants.
struct BatteryConfig {
  const double MAX_BATTERY_CAPACITY;
  const double MIN_RESIDUAL_SOC;
  const double CHARGING_EFFICIENCY;
  const double DISCHARGING_EFFICIENCY;
  const double B_MAX;
  const double B_MIN;
  const double ETA_C;
  const double ETA_D;
  const double U_MAX;
};

// Helper function to trim whitespace from a string
std::string trim(const std::string &str) {
  size_t first = str.find_first_not_of(' ');
  if (std::string::npos == first) {
    return str;
  }
  size_t last = str.find_last_not_of(' ');
  return str.substr(first, (last - first + 1));
}

// 2. A function that calculates or loads the configuration from a file.
BatteryConfig initialize_config_runtime() {
  std::cout << "Attempting to load configuration from config.ini..."
            << std::endl;
  std::ifstream config_file("config.ini");
  if (!config_file.is_open()) {
    // Handle error if file is missing/cannot be read
    throw std::runtime_error("Error: config.ini file not found!");
  }

  std::string line;
  std::map<std::string, double> values;

  while (std::getline(config_file, line)) {
    // Remove whitespace and skip comments/empty lines
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    if (line.empty() || line[0] == '#') {
      continue;
    }

    size_t delimiter_pos = line.find('=');
    if (delimiter_pos != std::string::npos) {
      std::string key = line.substr(0, delimiter_pos);
      std::string value_str = line.substr(delimiter_pos + 1);
      // Use stringstream to convert the value string to a double
      std::istringstream iss(value_str);
      double value_double;
      if (iss >> value_double) {
        values[key] = value_double;
      }
    }
  }
  config_file.close();

  //   // Ensure all required values were read
  //   if (values.find("MAX_BATTERY_CAPACITY") == values.end()) {
  //     throw std::runtime_error(
  //         "Error: Missing required configuration keys in config.txt!");
  //   }

  // Return the struct with all fields initialized as const
  return {/* MAX_BATTERY_CAPACITY */ values["MAX_BATTERY_CAPACITY"],
          /* MIN_RESIDUAL_SOC     */ values["MIN_RESIDUAL_SOC"],
          /* CHARGING_EFFICIENCY  */ values["CHARGING_EFFICIENCY"],
          /* DISCHARGING_EFFICIENCY */ values["DISCHARGING_EFFICIENCY"],
          /* B_MAX                */ values["MAX_BATTERY_CAPACITY"],
          /* B_MIN                */ values["MIN_RESIDUAL_SOC"],
          /* ETA_C                */ values["CHARGING_EFFICIENCY"],
          /* ETA_D                */ values["DISCHARGING_EFFICIENCY"],
          /* U_MAX                */
          (values["MAX_BATTERY_CAPACITY"] - values["MIN_RESIDUAL_SOC"]) /
              values["DISCHARGING_EFFICIENCY"]};
}

// 3. Use the function-local static pattern to ensure safe, thread-safe,
//    single-time initialization of the constant configuration struct.
const BatteryConfig &get_battery_config() {
  // This variable is initialized exactly once, the first time this function is
  // called.
  static const BatteryConfig config = initialize_config_runtime();
  return config;
}

// --- Example Usage ---

// int main() {
//   try {
//     // The first call to get_battery_config() triggers the file reading
//     logic. const auto &cfg = get_battery_config();

//     std::cout << std::fixed << std::setprecision(2);
//     std::cout << "\n--- System Configuration Loaded (Read-Only) ---"
//               << std::endl;
//     std::cout << "Max Battery Capacity:   " << cfg.MAX_BATTERY_CAPACITY
//               << std::endl;
//     std::cout << "Min Residual SOC:       " << cfg.MIN_RESIDUAL_SOC
//               << std::endl;
//     std::cout << "U_MAX:                  " << cfg.U_MAX << std::endl;
//     // Access all other cfg.B_MAX, cfg.ETA_C, etc.
//   } catch (const std::runtime_error &e) {
//     std::cerr << e.what() << std::endl;
//     return 1; // Exit with error code
//   }

//   return 0;
// }
