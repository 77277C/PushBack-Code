#pragma once

#include <memory>
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include "particle_filter.hpp"


class ParticleFilterChassis: public lemlib::Chassis {
public:
    ParticleFilterChassis(lemlib::Drivetrain drivetrain, lemlib::ControllerSettings linearSettings, lemlib::ControllerSettings angularSettings,
                lemlib::OdomSensors odomSensors, lemlib::DriveCurve* throttleCurve = &lemlib::defaultDriveCurve,
                lemlib::DriveCurve* steerCurve = &lemlib::defaultDriveCurve, std::vector<std::unique_ptr<Distance>>& pfSensors)
                    : Chassis(drivetrain, linearSettings, angularSettings, odomSensors, throttleCurve,
                        steerCurve) {
        pf = std::make_unique<ParticleFilter<500>>(sensors, [this]() {
            return this->getPose(true, true).theta;
        });
    }

    void setPose(float x, float y, float theta, bool radians = false) {
        Chassis::setPose(x, y, theta, radians);
        pf->init_norm_dist({x, y});
    }

    void odomUpdate() {
        lemlib::Pose before = getPose();
        lemlib::update();
        lemlib::Pose after = getPose();
        lemlib::Pose change = after - before;

        std::uniform_real_distribution xDistribution(change.x - DRIVE_NOISE * change.x,
                                                    change.x + DRIVE_NOISE * change.x);
        std::uniform_real_distribution yDistribution(change.y - DRIVE_NOISE * change.y,
                                                    change.y + DRIVE_NOISE * change.y);
        std::uniform_real_distribution angleDistribution(
            pf->get_angle() - 3 * M_PI / 180,
            pf->get_angle() + 3 * M_PI / 180);

        auto random_gen = pf->get_random_gen();

        pf->update([&]() {
            const auto noisyX = xDistribution(random_gen);
            const auto noisyY = yDistribution(random_gen);
            const auto angle = angleDistribution(random_gen);

            // Create a vector from noisyX and noisyY, then rotate by angle
            return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisyX, noisyY});
        });
    }

protected:
    std::unique_ptr<ParticleFilter<500>> pf;
};



