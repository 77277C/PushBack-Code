#pragma once

#include <memory>

#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include "particle_filter.hpp"


template<size_t N>
class ParticleFilterChassis: public lemlib::Chassis {
public:
    ParticleFilterChassis(lemlib::Drivetrain drivetrain, lemlib::ControllerSettings linearSettings, lemlib::ControllerSettings angularSettings,
                lemlib::OdomSensors odomSensors, lemlib::DriveCurve* throttleCurve = &lemlib::defaultDriveCurve,
                lemlib::DriveCurve* steerCurve = &lemlib::defaultDriveCurve, std::vector<std::unique_ptr<Distance>>& pfSensors)
                    : Chassis(drivetrain, linearSettings, angularSettings, odomSensors, throttleCurve,
                        steerCurve) {
        pf = std::make_unique<ParticleFilter<N>>(sensors, [this]() {
            return this->getPose(true, true).theta;
        });
    }

    void setPose(float x, float y, float theta, bool radians = false, bool resetParticles = true) {
        Chassis::setPose(x, y, theta, radians);
        if (resetParticles) {
            pf->init_norm_dist({x, y});
        }   
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

        // Angle distribution doesn't factor in current theta because odom change is already rotated
        // to be in (x, y) format
        static std::uniform_real_distribution angleDistribution(-ANGLE_NOISE, ANGLE_NOISE);

        pf->update([&]() {
            static auto random_gen = pf->get_random_gen();
            const auto noisyX = xDistribution(random_gen);
            const auto noisyY = yDistribution(random_gen);
            const auto noisyAngleDelta = angleDistribution(random_gen);

            // Create a vector from noisyX and noisyY and rotate it by possible angular noise
            return Eigen::Rotation2Df(noisyAngleDelta) * Eigen::Vector2f({noisyX, noisyY});
        });

        auto prediction = pf->get_prediction();
        setPose(prediction.x(), prediction.y(), after.theta, after.radians, true);
    }

protected:
    std::unique_ptr<ParticleFilter<N>> pf;
};



