#pragma once

#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include "particle_filter.hpp"


template<size_t N>
class ParticleFilterChassis: public lemlib::Chassis {
public:
    explicit ParticleFilterChassis(
        lemlib::Drivetrain drivetrain,
        lemlib::ControllerSettings linearSettings,
        lemlib::ControllerSettings angularSettings,
        lemlib::OdomSensors odomSensors,
        const std::vector<Distance*>& pfSensors, lemlib::DriveCurve* throttleCurve = &lemlib::defaultDriveCurve,
        lemlib::DriveCurve* steerCurve = &lemlib::defaultDriveCurve
    ) : Chassis(drivetrain, linearSettings, angularSettings, odomSensors, throttleCurve,
                        steerCurve), pf(ParticleFilter<N>(pfSensors)) {}

    void setPose(float x, float y, float theta, bool radians = false, bool resetParticles = true) {
        Chassis::setPose(x, y, theta, radians);
        if (resetParticles) {
            pf.initNormDist({x, y, getPose(true, true).theta});
        }   
    }

    void odomUpdate() {
        unsigned long long start = pros::micros();

        // Get the change in movement
        const lemlib::Pose before = getPose(true, true);
        lemlib::update();
        const lemlib::Pose after = getPose(true, true);
        const lemlib::Pose change = after - before;

        std::uniform_real_distribution<> xDistribution{change.x - DRIVE_NOISE * std::fabs(change.x),
                                                          change.x + DRIVE_NOISE * std::fabs(change.x)};
        std::uniform_real_distribution<> yDistribution{change.y - DRIVE_NOISE * std::fabs(change.y),
                                                    change.y + DRIVE_NOISE * std::fabs(change.y)};
        std::uniform_real_distribution angleDistribution(change.theta - ANGLE_NOISE * std::fabs(change.theta),
                                                          change.theta + ANGLE_NOISE * std::fabs(change.theta));

        static auto& randomGen = pf.getRandomGen();

        pf.update([&]() {
            const auto noisyX = xDistribution(randomGen);
            const auto noisyY = yDistribution(randomGen);
            const auto noisyTheta = angleDistribution(randomGen);

            return Eigen::Vector3f(noisyX, noisyY, noisyTheta);
        });

        // Set the pose to be the filters prediction
        const auto prediction = pf.getPrediction();
        setPose(prediction.x(), prediction.y(), M_PI_2 - prediction.z(), true, false);

        updateTimeMicros = pros::micros() - start;
    }

    [[nodiscard]] bool isOdomTooSlow() const {
        // If the odom is taking more than 10ms it is too slow
        return updateTimeMicros > 10000;
    }

protected:
    ParticleFilter<N> pf;
    unsigned long long updateTimeMicros = 0;
};



