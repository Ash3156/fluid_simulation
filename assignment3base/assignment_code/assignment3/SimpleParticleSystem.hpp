#ifndef SIMPLE_PARTICLE_H_
#define SIMPLE_PARTICLE_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO
{
    class SimpleParticleSystem : public ParticleSystemBase
    {
        public:
            ParticleState ComputeTimeDerivative(const ParticleState &state, float time) const override {
                ParticleState new_state = ParticleState();
                new_state.positions.push_back(glm::vec3(-state.positions[0][1], state.positions[0][0], 0.));
                new_state.velocities.push_back(glm::vec3(0., 0., 0.));

                return new_state;
            }
    };
} // namespace GLOO

#endif
