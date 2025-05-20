#pragma once

#include <Eigen/Core>
#include <pros/distance.hpp>


constexpr double MM_TO_IN = 1 / 25.4;
constexpr double THRESHOLD = 393.0;


class Distance {
public:
    Distance(pros::Distance sensor, Eigen::Vector3f offset) :
        sensor(std::move(sensor)), offset(std::move(offset)) {}

    void update() {
        distance = sensor.get() * MM_TO_IN;
        stddev = 0.2 * distance / sqrt(sensor.get_confidence() / 64.0);
    }

    [[nodiscard]] std::optional<double> get_probability(const Eigen::Vector3f& X) const {
        if (distance > THRESHOLD) {
            return std::nullopt;
        }
    }

private:
    Eigen::Vector3f offset;
    pros::Distance sensor;

    double distance;
    double stddev;
};
