#ifndef PENDULUM_NODE_H_
#define PENDULUM_NODE_H_

#include "gloo/SceneNode.hpp"
#include "ParticleState.hpp"
#include "PendulumSystem.hpp"
#include "IntegratorFactory.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"

namespace GLOO
{
    class PendulumNode : public SceneNode
    {
    public:
        // require initial position for all particles
        PendulumNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type);
        void Update(double delta_time) override;

    private:
        float curr_time_ = 0.;
        float integration_step_;
        IntegratorType integrator_type_;
        ParticleState state_ = ParticleState();
        // the new particle system class
        PendulumSystem system_ = PendulumSystem();
        // integrator
        std::unique_ptr<IntegratorBase<PendulumSystem, ParticleState>> integrator_;
        // pointers for particles, used for updating positions
        std::vector<SceneNode*> particle_positions;
        std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(0.3, 30, 30);
        std::shared_ptr<ShaderProgram> shader_ = std::make_shared<PhongShader>();
    };
}

#endif