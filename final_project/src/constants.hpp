static const double MAX_BATTERY_CAPACITY = 14;
static const double MIN_RESIDUAL_SOC = 1;
static const double CHARGING_EFFICIENCY = 1;
static const double DISCHARGING_EFFICIENCY = 1;

// Global constants derived from physical model and constraints [9, 10]
const double B_MAX = MAX_BATTERY_CAPACITY;   // B
const double B_MIN = MIN_RESIDUAL_SOC;       // B
const double ETA_C = CHARGING_EFFICIENCY;    // ηc (<= 1) [11]
const double ETA_D = DISCHARGING_EFFICIENCY; // ηd (>= 1) [11]

// U: Maximum distance span based on battery capacity [10]
const double U_MAX = (B_MAX - B_MIN) / ETA_D;