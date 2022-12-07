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

                // divide this net force by the mass of particle to get acceleration
                new_state.velocities[i] /= mass;
            }

            return new_state;
        }

    private:
        const float mass = 1.;
        const float gravity = -5.0;
        const float drag_constant = 1.0;
    };
} // namespace GLOO

#endif