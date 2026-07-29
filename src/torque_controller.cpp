/**
 * VLA Torque Sentinel — Real-Time Joint Torque Safety Controller
 * Implements ISO 10218-1 compliant torque limiting with predictive
 * collision detection for Vision-Language-Action robotic manipulators.
 *
 * Safety-critical: All torque commands pass through this controller
 * before reaching joint actuators.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <array>

constexpr int MAX_JOINTS = 7;        // 7-DOF manipulator
constexpr double SAFETY_FACTOR = 0.8; // 80% of rated torque
constexpr double COLLISION_THRESHOLD_NM = 5.0;
constexpr int HISTORY_WINDOW = 100;

struct JointState {
    double position_rad;
    double velocity_rads;
    double torque_nm;
    double temperature_c;
    double current_a;
};

struct TorqueLimit {
    double max_nm;
    double rate_limit_nm_per_s;
    double thermal_derating;  // 0.0 - 1.0
};

class CollisionDetector {
    std::array<std::vector<double>, MAX_JOINTS> torque_history;
    std::array<double, MAX_JOINTS> baseline_torque;
    bool calibrated = false;

public:
    void calibrate(const std::array<JointState, MAX_JOINTS>& states) {
        for (int j = 0; j < MAX_JOINTS; j++) {
            baseline_torque[j] = states[j].torque_nm;
            torque_history[j].clear();
        }
        calibrated = true;
    }

    bool detect_collision(int joint_id, double current_torque) {
        if (!calibrated || joint_id >= MAX_JOINTS) return false;

        torque_history[joint_id].push_back(current_torque);
        if (torque_history[joint_id].size() > HISTORY_WINDOW) {
            torque_history[joint_id].erase(torque_history[joint_id].begin());
        }

        // External torque estimation: current - gravity compensation baseline
        double external_torque = std::abs(current_torque - baseline_torque[joint_id]);

        // Derivative-based collision detection
        if (torque_history[joint_id].size() >= 3) {
            size_t n = torque_history[joint_id].size();
            double dt1 = torque_history[joint_id][n-1] - torque_history[joint_id][n-2];
            double dt2 = torque_history[joint_id][n-2] - torque_history[joint_id][n-3];
            double jerk = std::abs(dt1 - dt2);
            if (jerk > COLLISION_THRESHOLD_NM * 0.5) return true;
        }

        return external_torque > COLLISION_THRESHOLD_NM;
    }
};

class TorqueSafetyController {
    std::array<TorqueLimit, MAX_JOINTS> limits;
    std::array<double, MAX_JOINTS> last_commanded;
    CollisionDetector collision_detector;
    int safety_stops = 0;
    int total_commands = 0;

public:
    TorqueSafetyController() {
        // Default limits for a 7-DOF collaborative arm (e.g., Franka-class)
        double rated_torques[] = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
        for (int j = 0; j < MAX_JOINTS; j++) {
            limits[j] = {
                rated_torques[j] * SAFETY_FACTOR,
                rated_torques[j] * 10.0,  // Rate limit: 10x rated per second
                1.0                         // No thermal derating initially
            };
            last_commanded[j] = 0.0;
        }
    }

    void set_thermal_derating(int joint_id, double temp_c) {
        if (joint_id >= MAX_JOINTS) return;
        // Linear derating above 60°C, full shutdown at 85°C
        if (temp_c < 60.0) {
            limits[joint_id].thermal_derating = 1.0;
        } else if (temp_c < 85.0) {
            limits[joint_id].thermal_derating = 1.0 - (temp_c - 60.0) / 25.0;
        } else {
            limits[joint_id].thermal_derating = 0.0;
        }
    }

    struct SafetyResult {
        double commanded_torque;
        bool collision_detected;
        bool torque_limited;
        bool thermal_shutdown;
    };

    SafetyResult apply(int joint_id, double desired_torque, double measured_torque, double dt_s) {
        total_commands++;
        SafetyResult result = {desired_torque, false, false, false};

        if (joint_id >= MAX_JOINTS) {
            result.commanded_torque = 0.0;
            return result;
        }

        // Collision detection
        if (collision_detector.detect_collision(joint_id, measured_torque)) {
            result.collision_detected = true;
            result.commanded_torque = 0.0;
            safety_stops++;
            return result;
        }

        // Thermal derating
        double effective_max = limits[joint_id].max_nm * limits[joint_id].thermal_derating;
        if (effective_max <= 0.0) {
            result.thermal_shutdown = true;
            result.commanded_torque = 0.0;
            safety_stops++;
            return result;
        }

        // Torque magnitude limiting
        double clamped = std::clamp(desired_torque, -effective_max, effective_max);

        // Rate limiting
        double max_delta = limits[joint_id].rate_limit_nm_per_s * dt_s;
        double delta = clamped - last_commanded[joint_id];
        if (std::abs(delta) > max_delta) {
            clamped = last_commanded[joint_id] + std::copysign(max_delta, delta);
            result.torque_limited = true;
        }

        last_commanded[joint_id] = clamped;
        result.commanded_torque = clamped;
        return result;
    }

    void calibrate(const std::array<JointState, MAX_JOINTS>& states) {
        collision_detector.calibrate(states);
    }

    int get_safety_stops() const { return safety_stops; }
    int get_total_commands() const { return total_commands; }
    double safety_ratio() const {
        return total_commands > 0 ? (double)safety_stops / total_commands : 0.0;
    }
};

// Standalone test runner
int main() {
    TorqueSafetyController controller;

    // Test torque limiting
    auto result = controller.apply(0, 100.0, 10.0, 0.001);
    std::cout << "[TorqueSentinel] Joint 0: desired=100Nm, output="
              << result.commanded_torque << "Nm, limited="
              << result.torque_limited << std::endl;

    // Test thermal derating
    controller.set_thermal_derating(0, 75.0);
    result = controller.apply(0, 50.0, 5.0, 0.001);
    std::cout << "[TorqueSentinel] Joint 0 @75C: desired=50Nm, output="
              << result.commanded_torque << "Nm, thermal_shutdown="
              << result.thermal_shutdown << std::endl;

    std::cout << "[TorqueSentinel] Safety stops: " << controller.get_safety_stops()
              << "/" << controller.get_total_commands() << std::endl;

    return 0;
}
