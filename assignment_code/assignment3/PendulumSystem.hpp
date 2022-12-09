#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO
{
    class PendulumSystem : public ParticleSystemBase
    {
    public:
        // method for adding particles
        void AddParticle(bool fixed) {
            std::vector<SpringInfo> new_spring_info_vector;
            particle_springs.push_back(new_spring_info_vector);
            is_fixed.push_back(fixed);
        }

        // adding spring to existing particle (adds it for both particles connected)
        void AddSpring(int my_index, int other_index, float rl, float sc) {
            particle_springs[my_index].push_back(SpringInfo{other_index, rl, sc});
            particle_springs[other_index].push_back(SpringInfo{my_index, rl, sc});
        }

        ParticleState ComputeTimeDerivative(const ParticleState &state, float time) const override
        {
            ParticleState new_state = ParticleState();
            // velocities become the new positions
            new_state.positions = state.velocities;
            // initialize the new velocities to 0, since we add components to them for each force
            new_state.velocities = std::vector<glm::vec3>(new_state.positions.size(), glm::vec3(0., 0., 0.));

            // velocities become A(x(t), v(t)): gravity, viscous drag, and spring force
            // we just add each force component to the new velocity, and divide by mass
            for (int i = 0; i < state.positions.size(); i++) {
                if (is_fixed[i]) {
                    new_state.positions[i] = glm::vec3(0., 0., 0.);
                    continue;
                }
                // gravity
                new_state.velocities[i] += glm::vec3(0., gravity, 0.) * mass;

                // viscous drag
                new_state.velocities[i] += state.velocities[i] * (-1.0f * drag_constant);

                // spring force
                std::vector<SpringInfo> curr_particle_springs = particle_springs[i];
                // for each spring the particle is connected to, we add the force
                for (auto curr_info : curr_particle_springs) {
                    glm::vec3 d = state.positions[i] - state.positions[curr_info.index];
                    new_state.velocities[i] += -1.0f * curr_info.sc 
                                                * (glm::length(d) - curr_info.rl) 
                                                * glm::normalize(d);
                }
                                               
                // divide this net force by the mass of particle to get acceleration
                new_state.velocities[i] /= mass;
            }

            return new_state;
        }
    private:
        const float mass = 1.;
        const float gravity = -9.8;
        const float drag_constant = 1.0;
        // stores info on spring between 2 particles
        struct SpringInfo
        {
            int index;
            float rl;
            float sc;
        };
        std::vector<std::vector<SpringInfo>> particle_springs;
        std::vector<bool> is_fixed;
    };
} // namespace GLOO

#endif
