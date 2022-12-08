#ifndef SPH_SYSTEM_H_
#define SPH_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO
{
    class SPHSystem : public ParticleSystemBase
    {
    public:
        ParticleState ComputeTimeDerivative(const ParticleState &state, float time) const override
        {
            ParticleState new_state = ParticleState();
            // velocities become the new positions
            new_state.positions = state.velocities;
            // initialize the new velocities to 0, since we add components to them for each force
            new_state.velocities = std::vector<glm::vec3>(new_state.positions.size(), glm::vec3(0., 0., 0.));

            std::vector<float> densities = {};
            for (int i = 0; i < state.positions.size(); i++) {
                float row = 0;
                for (int j = 0; j < state.positions.size(); j++) {
                    if (j != i) {
                        float r = glm::length(state.positions[i] - state.positions[j]);
                        if (r < h) {
                            row += mass * poly6 * std::pow(h * h - r * r, 3);
                        }
                    }
                }
                densities.push_back(row);
            }

            std::vector<float> pressures = {};               
            for (float d : densities) {
                pressures.push_back(k * (d - 1000));
                // std::cout << d << std::endl;
            }

            std::vector<glm::vec3> forces = {};
            for (int i = 0; i < state.positions.size(); i++) {
                glm::vec3 f(0., 0., 0.);
                for (int j = 0; j < state.positions.size(); j++) {
                    if (j != i) {
                        float r = glm::length(state.positions[i] - state.positions[j]);
                        if (r < h) {
                            // glm::vec3 dir = glm::normalize(state.positions[i] - state.positions[j]);
                            glm::vec3 dir = (state.positions[i] - state.positions[j]);
                            f += -dir * mass * poly6grad * (float) std::pow(h * h - r * r, 2);
                        }
                    }
                }
                forces.push_back(f);
            }
            

            // velocities become A(x(t), v(t)): gravity, viscous drag,
            // we just add each force component to the new velocity, and divide by mass
            for (int i = 0; i < state.positions.size(); i++)
            {
                // simple horizontal motion, call it wind - TODO apply only to specific layer/height
                // new_state.velocities[i] += glm::vec3(1., 1., 1.) * mass;

                // gravity
                new_state.velocities[i] += glm::vec3(0., gravity, 0.) * mass;

                // viscous drag
                new_state.velocities[i] += state.velocities[i] * (-1.0f * drag_constant);

                new_state.velocities[i] -= mass/densities[i] * forces[i];

                // divide this net force by the mass of particle to get acceleration
                new_state.velocities[i] /= mass;
            }

            return new_state;
        }

    private:
        const float mass = 1.;
        const float gravity = -9.8;
        const float drag_constant = 1.0;
        // std::vector<float> densities = {};
        // std::vector<float> pressures = {};
        // smoothing width
        const float h = 0.5;
        // water vapor gas constant
        const float k = 461.52;
        const float poly6 = 315. / (64 * M_PI * std::pow(h, 9.));
        const float poly6grad = - 945. / (32 * M_PI * std::pow(h, 9.));
    };
} // namespace GLOO

#endif