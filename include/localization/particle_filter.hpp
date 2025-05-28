#pragma once

#include <random>
#include <utility>
#include "distance.hpp"
#include "constants.hpp"


typedef struct {
    Eigen::Vector2f location = Eigen::Vector2f::Zero();
    double weight = 1.0;
} Particle;


template<size_t N>
class ParticleFilter {
public:
    explicit ParticleFilter(std::vector<std::unique_ptr<Distance>> sensors, std::function<float()> get_angle)
        : sensors(std::move(sensors)), get_angle(std::move(get_angle)) {}

    Eigen::Vector3f get_prediction() {
        return prediction;
    }

    void update(
        const std::function<Eigen::Vector2f()>& get_prediction
    ) {
        // Add the prediction to each particle
        for (auto&& particle : particles) {
            particle.location += get_prediction();
        }

        distance_since_update = get_prediction().norm();

        // Return if robot hasn't moved to avoid particle convergence
        if (
            distance_since_update < max_distance_since_update &&
            max_update_interval > pros::millis() - last_update_time
        ) {
            return;
        }

        // Update readings on all sensors
        update_sensors();

        float angle = get_angle();
        for (size_t i = 0; i < N; i++) {
            // Place particle at random point in field if out of field
            if (!is_particle_in_field(particles[i])) {
                particles[i].location.x() = field_distribution(random_gen);
                particles[i].location.y() = field_distribution(random_gen);
            }

            // Convert the 2d particle to a 3d particle
            auto particle_vector = Eigen::Vector3f();
            particle_vector.head<2>() = particles[i].location;
            particle_vector.z() = angle;

            // Weight the particle
            particles[i].weight = weight_particle(particle_vector);
        }

        resample();
        
        last_update_time = pros::millis();
        distance_since_update = 0.0;
    }

    double weight_particle(const Eigen::Vector3f& particle_vector) {
        double combined_weight = 1.0;

        // Multiply the combined weight by the probability of each sensor
        for (const auto& sensor : sensors) {
            auto weight = sensor->get_probability(particle_vector);
            if (weight.has_value()) {
                combined_weight *= weight.value();
            }
        }

        return combined_weight;
    }

    void update_sensors() {
        for (const auto& sensor : sensors) {
            sensor->update();
        }
    }

    static bool is_particle_in_field(Particle particle) {
        return (
            particle.location.x() < WALL &&
            particle.location.y() < WALL &&
            particle.location.x() > -WALL &&
            particle.location.y() > -WALL
        );
    }

    void resample() {
        double total_weight = 0.0;

        // Sum all particle weights and make a copy of the current set.
        // Think of each particle's weight as a slice in a pie chart of "trust" — bigger slices = more believable.
        for (size_t i = 0; i < N; i++) {
            total_weight += particles[i].weight;
            old_particles[i] = particles[i];
        }

        // Compute the average weight across all particles.
        // This defines the spacing between our sampling points on the pie chart.
        const double avg_weight = total_weight / static_cast<double>(N);

        // Choose a random starting offset within [0, avg_weight)
        // This is like randomly placing the first pointer on the pie chart.
        std::uniform_real_distribution<> distribution{0.0, avg_weight};
        const double random_weight = distribution(random_gen);

        size_t j = 0;
        auto cumulative_weight = 0.0;

        float x_sum = 0.0;
        float y_sum = 0.0;

        // Resample N particles using low-variance resampling.
        // We place N evenly spaced markers on the cumulative weight pie chart, starting from random_weight.
        for (size_t i = 0; i < N; i++) {
            const auto weight = static_cast<double>(i) * avg_weight + random_weight;

            // Walk through the cumulative weights until we find the particle that spans the current weight.
            // Particles with bigger weight slices will get hit more often — they're more likely to survive.
            while (cumulative_weight < weight) {
                if (j >= N) {
                    break;
                }
                cumulative_weight += particles[j].weight;
                j++;
            }

            // Copy the selected particle's location into the new set.
            // This is effectively cloning a "good guess" multiple times.
            particles[i].location.x() = old_particles[j-1].location.x();
            particles[i].location.y() = old_particles[j-1].location.y();

            // Sum the resampled particle positions for the final prediction.
            x_sum += particles[i].location.x();
            y_sum += particles[i].location.y();
        }

        // Estimate the new predicted position as the mean of all resampled particles.
        prediction = Eigen::Vector3f(x_sum / static_cast<float>(N), y_sum / static_cast<float>(N), get_angle());
    }

protected:
    Eigen::Vector3f prediction{};
    std::function<float()> get_angle;

    std::array<Particle, N> particles;
    std::array<Particle, N> old_particles;
    std::vector<std::unique_ptr<Distance>> sensors;

    std::ranlux24_base random_gen;
    std::uniform_real_distribution<> field_distribution{-WALL, WALL};

    double distance_since_update = 0.0;
    int32_t last_update_time = 0.0;
    double max_distance_since_update = 2.0;
    int32_t max_update_interval = 2000;
};
