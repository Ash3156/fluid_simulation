#include "PendulumNode.hpp"

#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/RenderingComponent.hpp"

namespace GLOO
{
    PendulumNode::PendulumNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type) {
        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type_);

        // add particles to state
        for (int i = 0; i < init_positions.size(); i++) {
            state_.positions.push_back(init_positions[i]);
            state_.velocities.push_back(glm::vec3(0., 0., 0.));
            if (i == 0) {
                system_.AddParticle(true);
            } else {
                system_.AddParticle(false);
            }

            auto particle_node = make_unique<SceneNode>();
            particle_node->GetTransform().SetPosition(init_positions[i]);

            auto sphere_node = make_unique<SceneNode>();
            sphere_node->CreateComponent<ShadingComponent>(shader_);
            sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
            particle_node->AddChild(std::move(sphere_node));

            particle_positions.push_back(particle_node.get());

            AddChild(std::move(particle_node));
        }

        // create springs connecting the particles in order
        for (int i = 0; i < init_positions.size() - 1; i++) {
            system_.AddSpring(i, i + 1, 0.01f, 13.0f);
        }
    }

    void PendulumNode::Update(double delta_time) {
        float curr_step = integration_step_;
        while (curr_step <= delta_time) {
            state_ = integrator_->Integrate(system_, state_, curr_time_, integration_step_);
            curr_time_ += integration_step_;
            curr_step += integration_step_;
            // update position for each particle
            for (int i = 0; i < particle_positions.size(); i++) {
                particle_positions[i]->GetTransform().SetPosition(state_.positions[i]);
            }
        }
    }
}