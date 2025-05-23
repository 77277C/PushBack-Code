#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pros/distance.hpp>


constexpr double MM_TO_IN = 1 / 25.4;
constexpr double M_TO_IN = 39.3701;


// Values from alexDickhans/echo on GitHub
constexpr float WALL_0_X = 1.78308 * M_TO_IN;
constexpr float WALL_1_Y = 1.78308 * M_TO_IN;
constexpr float WALL_2_X = -1.78308 * M_TO_IN;
constexpr float WALL_3_Y = -1.78308 * M_TO_IN;


inline double angle_diff(double a, double b) {
    return std::remainder(b - a, 2 * M_PI);
}


// Approximate normcdf using Abramowitz and Stegun formula 7.1.26
double normcdf(double z) {
    // Constants
    constexpr double b1 =  0.319381530;
    constexpr double b2 = -0.356563782;
    constexpr double b3 =  1.781477937;
    constexpr double b4 = -1.821255978;
    constexpr double b5 =  1.330274429;
    constexpr double p  =  0.2316419;
    constexpr double c  =  0.39894228; // 1/sqrt(2*pi)

    if (z >= 0.0) {
        double t = 1.0 / (1.0 + p * z);
        return 1.0 - c * std::exp(-z * z / 2.0) * t *
            (b1 + t * (b2 + t * (b3 + t * (b4 + t * b5))));
    } else {
        return 1.0 - normcdf(-z); // use symmetry
    }
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

        return normpdf((measured - predicted) / stddev);
    }

private:
    Eigen::Vector3f offset;
    pros::Distance sensor;

    double distance;
    double stddev;
};
