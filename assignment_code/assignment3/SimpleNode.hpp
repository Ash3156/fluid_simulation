#ifndef SIMPLE_NODE_H_
#define SIMPLE_NODE_H_

#include "gloo/SceneNode.hpp"
#include "ParticleState.hpp"
#include "SimpleParticleSystem.hpp"
#include "IntegratorFactory.hpp"

namespace GLOO
{
    class SimpleNode : public SceneNode
    {
    public:
        // require initial position for the particle
        SimpleNode(glm::vec3 init_position, float integration_step, IntegratorType integrator_type);
        void Update(double delta_time) override;

    private:
        float curr_time_ = 0.;
        float integration_step_;
        IntegratorType integrator_type_;
        // choosing to represent state as 1 particle in the ParticleState class
        ParticleState state_ = ParticleState();
        // the new particle system class
        SimpleParticleSystem system_ = SimpleParticleSystem();
        // integrator
        std::unique_ptr<IntegratorBase<SimpleParticleSystem, ParticleState>> integrator_;
    };
}

#endif