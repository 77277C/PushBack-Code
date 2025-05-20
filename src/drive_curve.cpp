#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>



// Constants for drive curve adjustments
constexpr int32_t DRIVE_CURVE_THRESHOLD = 127;
constexpr int32_t DRIVE_CURVE_MULTIPLIER = 21;
constexpr int32_t DRIVE_CURVE_DIVISOR = 1000;
constexpr int32_t OUTPUT_LIMIT_WHEN_RESTRICTED = 98;
constexpr int32_t FIRST_INPUT_LIMIT = 114;
constexpr int32_t SECOND_INPUT_LIMIT = 114;

/**
 * @brief Applies a drive curve transformation to input values.
 *
 * This function takes a pair of control inputs (lateral and angular) and
 * applies a drive curve function for smoother and more responsive control.
 *
 * @param input A pair of values: first = lateral input, second = angular input.
 * @return A transformed pair of values for smoother control.
 */
std::pair<int32_t, int32_t> drive_curve(const std::pair<int32_t, int32_t>& input) {
    int32_t first = input.first;
    int32_t second = input.second;

    // Compute absolute value of the angular input to optimize performance.
    int32_t abs_second = std::abs(second);

    // Apply an exponential drive curve for better sensitivity at lower speeds.
    second = std::exp(((abs_second - DRIVE_CURVE_THRESHOLD) * DRIVE_CURVE_MULTIPLIER) /
                      DRIVE_CURVE_DIVISOR) * second;

    // Limit the output if both lateral and angular inputs exceed thresholds.
    if (std::abs(first) > FIRST_INPUT_LIMIT && abs_second > SECOND_INPUT_LIMIT) {
        second = (second < 0) ? -OUTPUT_LIMIT_WHEN_RESTRICTED
                              : OUTPUT_LIMIT_WHEN_RESTRICTED;
    }

    // Scale controller inputs [-127, 127] to motor voltage range [-12000, 12000] mV.
    return {first * (12000.0 / 127.0), second * (12000.0 / 127.0)};
}