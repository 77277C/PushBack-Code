#include "driveCurve.hpp"


// Constants for drive curve adjustments
constexpr int32_t DRIVE_CURVE_THRESHOLD = 140;
constexpr int32_t DRIVE_CURVE_MULTIPLIER = 7;
constexpr int32_t DRIVE_CURVE_DIVISOR = 1000;


std::pair<int32_t, int32_t> driveCurveTony(const std::pair<int32_t, int32_t>& input) {
    int32_t first = input.first;
    int32_t second = input.second;

    // Compute absolute value of the angular input to optimize performance.
    int32_t absSecond = std::abs(second);

    // Apply an exponential drive curve for better sensitivity at lower speeds.
    second = std::exp(((absSecond - DRIVE_CURVE_THRESHOLD) * DRIVE_CURVE_MULTIPLIER) /
                      DRIVE_CURVE_DIVISOR) * second;


    return {first, second};
}

constexpr int32_t T = 2.5;
constexpr float DRIVE_CURVE_SCALE = 1;

std::pair<int32_t, int32_t> driveCurveGeorge(const std::pair<int32_t, int32_t>& input) {
    int32_t first = input.first;
    float second = input.second;

    second = (
            std::exp(-(T/10)) +
            std::exp((std::abs(second) - 127) / 10) *
            (1 - std::exp(-(T/10)))
        ) * second * DRIVE_CURVE_SCALE;

    return {first, second};
}

