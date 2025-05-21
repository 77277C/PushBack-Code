#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pros/distance.hpp>


constexpr double MM_TO_IN = 1 / 25.4;


class Distance {
public:
    Distance(pros::Distance sensor, Eigen::Vector3f offset) :
        sensor(std::move(sensor)), offset(std::move(offset)) {}

    void update() {
        distance = sensor.get() * MM_TO_IN;
        stddev = 0.2 * distance / sqrt(sensor.get_confidence() / 64.0);
    }

    [[nodiscard]] std::optional<double> get_probability(const Eigen::Vector3f& X) const {
        if (distance > 9999 * MM_TO_IN) {
            return std::nullopt;
        }

        // Get the current absolute angle of the sensor (robot angle + offset angle)
        auto angle = X.z() + offset.z();

        // Get the current absolute position of the sensor (robot position + offset relative to field)
        Eigen::Vector2f x = X.head<2>() + Eigen::Rotation2Df(X.z()) * offset.head<2>();
    }

private:
    Eigen::Vector3f offset;
    pros::Distance sensor;

    double distance;
    double stddev;
};
