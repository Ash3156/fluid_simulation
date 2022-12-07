#include "SPHNode.hpp"

#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO
{
    SPHNode::SPHNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type)
    {
        init_positions_ = init_positions;
        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<SPHSystem, ParticleState>(integrator_type_);

        // add particles to the SPH environment
        for (int i = 0; i < init_positions.size(); i++)
        {
            state_.positions.push_back(init_positions[i]);
            state_.velocities.push_back(glm::vec3(0., 0., 0.));

            // each particle is its own scene node represented as sphere
            auto particle_node = make_unique<SceneNode>();
            particle_node->GetTransform().SetPosition(init_positions[i]);

            auto sphere_node = make_unique<SceneNode>();
            sphere_node->CreateComponent<ShadingComponent>(shader_);
            sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
            particle_node->AddChild(std::move(sphere_node));
            auto material = std::make_shared<Material>(turquoise, turquoise, turquoise, 0);
            sphere_node->CreateComponent<MaterialComponent>(material);

            particle_positions.push_back(particle_node.get());
            std::cout << "HERE" << std::endl;

            AddChild(std::move(particle_node));
        }
    }

    void SPHNode::Update(double delta_time)
    {
        // reset on 'R'
        static bool prev_released = true;
        if (InputManager::GetInstance().IsKeyPressed('R'))
        {
            if (prev_released)
            {
                // reset the particle
                state_.positions = init_positions_;
                state_.velocities = std::vector<glm::vec3>(init_positions_.size(), glm::vec3(0., 0., 0.));
            }
            prev_released = false;
        }
        else
        {
            prev_released = true;
        }

        float curr_step = integration_step_;
        while (curr_step <= delta_time)
        {
            state_ = integrator_->Integrate(system_, state_, curr_time_, integration_step_);
            curr_time_ += integration_step_;
            curr_step += integration_step_;
            // update position for each particle
            for (int i = 0; i < particle_positions.size(); i++)
            {
                particle_positions[i]->GetTransform().SetPosition(state_.positions[i]);
                glm::vec3 curr_pos = particle_positions[i]->GetTransform().GetPosition();

                // float boxWidth = 3.f;
                // float elasticity = 0.5f;
                // if (p->position.y < p->size)
                // {
                //     p->position.y = -p->position.y + 2 * p->size + 0.0001f;
                //     p->velocity.y = -p->velocity.y * elasticity;
                // }

                // if (p->position.x < p->size - boxWidth)
                // {
                //     p->position.x = -p->position.x + 2 * (p->size - boxWidth) + 0.0001f;
                //     p->velocity.x = -p->velocity.x * elasticity;
                // }

                // if (p->position.x > -p->size + boxWidth)
                // {
                //     p->position.x = -p->position.x + 2 * -(p->size - boxWidth) - 0.0001f;
                //     p->velocity.x = -p->velocity.x * elasticity;
                // }

                // if (p->position.z < p->size - boxWidth)
                // {
                //     p->position.z = -p->position.z + 2 * (p->size - boxWidth) + 0.0001f;
                //     p->velocity.z = -p->velocity.z * elasticity;
                // }

                // if (p->position.z > -p->size + boxWidth)
                // {
                //     p->position.z = -p->position.z + 2 * -(p->size - boxWidth) - 0.0001f;
                //     p->velocity.z = -p->velocity.z * elasticity;
                // }

                if (particle_positions[i]->GetTransform().GetPosition()[1] < -2.0)
                {
                    glm::vec3 new_pos = curr_pos;
                    new_pos[1] = -2.0;
                    particle_positions[i]->GetTransform().SetPosition(new_pos);
                }
            }
        }
    }
}