#include "ClothNode.hpp"

#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO
{
    ClothNode::ClothNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type) {
        init_positions_ = init_positions;
        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<PendulumSystem, ParticleState>(integrator_type_);

        // add particles to state - cloth is 8x8, top two corners are fixed - reads in row by row
        for (int i = 0; i < init_positions.size(); i++) {
            state_.positions.push_back(init_positions[i]);
            state_.velocities.push_back(glm::vec3(0., 0., 0.));
            // fix the top left and right corners
            if (i == 0 || i == sqrt(init_positions.size()) - 1)
            // if (i < sqrt(init_positions.size()))
            {
                system_.AddParticle(true);
            }
            else
            {
                system_.AddParticle(false);
            }

            auto particle_node = make_unique<SceneNode>();
            particle_node->GetTransform().SetPosition(init_positions[i]);

            auto sphere_node = make_unique<SceneNode>();
            sphere_node->CreateComponent<ShadingComponent>(shader_);
            sphere_node->CreateComponent<RenderingComponent>(sphere_mesh_);
            particle_node->AddChild(std::move(sphere_node));

            particle_positions.push_back(particle_node.get());
            line_positions->push_back(init_positions[i]);

            AddChild(std::move(particle_node));
        }

        // structural springs
        for (int i = 0; i < sqrt(init_positions.size()); i++) {
            for (int j = 0; j < sqrt(init_positions.size()) - 1; j++) {
                // horizontal spring + line
                system_.AddSpring(i * sqrt(init_positions.size()) + j, i * sqrt(init_positions.size()) + j + 1, 2./7., 1000.0f);
                line_indices->push_back(i * sqrt(init_positions.size()) + j);
                line_indices->push_back(i * sqrt(init_positions.size()) + j + 1);
                
                // vertical spring + line
                system_.AddSpring(j * sqrt(init_positions.size()) + i, (j + 1) * sqrt(init_positions.size()) + i, 2./7., 1000.0f);
                line_indices->push_back(j * sqrt(init_positions.size()) + i);
                line_indices->push_back((j + 1) * sqrt(init_positions.size()) + i);
            }
        }

        // shear springs
        for (int i = 0; i < sqrt(init_positions.size()) - 1; i++) {
            // to bottom right
            for (int j = 0; j < sqrt(init_positions.size()) - 1; j++) {
                system_.AddSpring(i * sqrt(init_positions.size()) + j, (i + 1) * sqrt(init_positions.size()) + j + 1, (float) (sqrt(8.)/7.), 1000.0f);
                line_indices->push_back(i * sqrt(init_positions.size()) + j);
                line_indices->push_back((i + 1) * sqrt(init_positions.size()) + j + 1);
            }
            // to bottom left
            for (int j = 1; j < sqrt(init_positions.size()); j++) {
                system_.AddSpring(i * sqrt(init_positions.size()) + j, (i + 1) * sqrt(init_positions.size()) + j - 1, (float) (sqrt(8.)/7.), 1000.0f);
                line_indices->push_back(i * sqrt(init_positions.size()) + j);
                line_indices->push_back((i + 1) * sqrt(init_positions.size()) + j - 1);
            }
        }

        // flex springs
        for (int i = 0; i < sqrt(init_positions.size()); i++) {
            for (int j = 0; j < sqrt(init_positions.size()) - 2; j++) {
                // horizontal spring + line
                system_.AddSpring(i * sqrt(init_positions.size()) + j, i * sqrt(init_positions.size()) + j + 2, 4./7., 1000.0f);
                line_indices->push_back(i * sqrt(init_positions.size()) + j);
                line_indices->push_back(i * sqrt(init_positions.size()) + j + 2);

                // // vertical spring + line
                system_.AddSpring(j * sqrt(init_positions.size()) + i, (j + 2) * sqrt(init_positions.size()) + i, 4./7., 1000.0f);
                line_indices->push_back(j * sqrt(init_positions.size()) + i);
                line_indices->push_back((j + 2) * sqrt(init_positions.size()) + i);
            }
        }

        // draw the lines (springs)
        line_->UpdatePositions(std::move(line_positions));
        line_->UpdateIndices(std::move(line_indices));

        auto cloth_shader = std::make_shared<SimpleShader>();
        auto line_node = make_unique<SceneNode>();
        line_node->CreateComponent<ShadingComponent>(cloth_shader);
        auto &rc_line = line_node->CreateComponent<RenderingComponent>(line_);
        rc_line.SetDrawMode(DrawMode::Lines);
        auto material = std::make_shared<Material>(white, white, white, 0);
        line_node->CreateComponent<MaterialComponent>(material);

        line_ptr = line_node.get();
        AddChild(std::move(line_node));
    }

    void ClothNode::Update(double delta_time) {
        // reset on 'R'
        static bool prev_released = true;
        if (InputManager::GetInstance().IsKeyPressed('R')) {
            if (prev_released){
                // reset the particle
                state_.positions = init_positions_;
                state_.velocities = std::vector<glm::vec3>(init_positions_.size(), glm::vec3(0., 0., 0.));
            }
            prev_released = false;
        }
        else{
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
            }
        }
        // update the line positions (indices don't change)
        std::unique_ptr<PositionArray> new_line_positions = make_unique<PositionArray>(state_.positions);
        line_ptr->GetComponentPtr<RenderingComponent>()->GetVertexObjectPtr()->UpdatePositions(std::move(new_line_positions));
    }
}
