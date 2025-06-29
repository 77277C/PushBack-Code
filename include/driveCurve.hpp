#pragma once

#include <cmath>
#include <cstdint>
#include <utility>


/**
 * @brief Applies a drive curve transformation to input values.
 *
 * This function takes a pair of control inputs (lateral and angular) and
 * applies a drive curve function for smoother and more responsive control.
 *
 * @param input A pair of values: first = lateral input, second = angular input.
 * @return A transformed pair of values for smoother control.
 */
std::pair<int32_t, int32_t> driveCurve(const std::pair<int32_t, int32_t>& input);
