#include "driveCurve.hpp"


// Constants for drive curve adjustments
constexpr int32_t DRIVE_CURVE_THRESHOLD = 127;
constexpr int32_t DRIVE_CURVE_MULTIPLIER = 21;
constexpr int32_t DRIVE_CURVE_DIVISOR = 1000;
constexpr int32_t OUTPUT_LIMIT_WHEN_RESTRICTED = 98;
constexpr int32_t FIRST_INPUT_LIMIT = 114;
constexpr int32_t SECOND_INPUT_LIMIT = 114;


std::pair<int32_t, int32_t> driveCurve(const std::pair<int32_t, int32_t>& input) {
    int32_t first = input.first;
    int32_t second = input.second;

    // Compute absolute value of the angular input to optimize performance.
    int32_t absSecond = std::abs(second);

    // Apply an exponential drive curve for better sensitivity at lower speeds.
    second = std::exp(((absSecond - DRIVE_CURVE_THRESHOLD) * DRIVE_CURVE_MULTIPLIER) /
                      DRIVE_CURVE_DIVISOR) * second;

    // Limit the output if both lateral and angular inputs exceed thresholds.
    if (std::abs(first) > FIRST_INPUT_LIMIT && absSecond > SECOND_INPUT_LIMIT) {
        second = (second < 0) ? -OUTPUT_LIMIT_WHEN_RESTRICTED
                              : OUTPUT_LIMIT_WHEN_RESTRICTED;
    }

    return {first, second};
}