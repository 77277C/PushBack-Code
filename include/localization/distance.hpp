#pragma once

#include <cmath>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pros/distance.hpp>
#include "constants.hpp"


inline double angle_diff(double a, double b) {
    return std::remainder(b - a, 2 * M_PI);
}


// Cheap approximation of normpdf (normal distribution PDF)
double normpdf(double x, double mu = 0.0, double sigma = 1.0) {
    constexpr double inv_sqrt_2pi = 0.3989422804014327; // 1 / sqrt(2 * pi)
    double z = (x - mu) / sigma;
    return (inv_sqrt_2pi / sigma) * std::exp(-0.5 * z * z);
}


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

        // Predict the value of the distance sensor by finding the closest wall to it
        // then dividing the distance from the wall by cos(theta) to find the hypotenuse
        // which represents the predicted distance sensor reading
        double predicted = 50.0;

        if (const double theta = std::abs(angle_diff(0, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (WALL_0_X - x.x()) / std::cos(theta));
        }

        if (const double theta = std::abs(angle_diff(M_PI_2, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (WALL_1_Y - x.y()) / std::cos(theta));
        }

        if (const double theta = std::abs(angle_diff(M_PI, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (x.x() - WALL_2_X) / std::cos(theta));
        }

        if (const double theta = std::abs(angle_diff(3 * M_PI_2, angle)); theta < M_PI_2) {
            predicted = std::min(predicted, (x.y() - WALL_3_Y) / std::cos(theta));
        }

        return normpdf(distance, predicted, stddev);
    }

protected:
    Eigen::Vector3f offset;
    pros::Distance sensor;

    double distance;
    double stddev;
};
