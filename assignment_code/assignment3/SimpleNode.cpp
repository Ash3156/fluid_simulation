#include "SimpleNode.hpp"

namespace GLOO {
    SimpleNode::SimpleNode(glm::vec3 init_position, float integration_step, IntegratorType integrator_type) {
        state_.positions.push_back(init_position);
        state_.velocities.push_back(glm::vec3(0., 0., 0.));
        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<SimpleParticleSystem, ParticleState>(integrator_type_);
        GetTransform().SetPosition(init_position);
    }

    void SimpleNode::Update(double delta_time) {
        float curr_step = integration_step_;
        while (curr_step <= delta_time) {
            state_ = integrator_->Integrate(system_, state_, curr_time_, integration_step_);
            curr_time_ += integration_step_;
            curr_step += integration_step_;
            GetTransform().SetPosition(state_.positions[0]);
        }
    }
}