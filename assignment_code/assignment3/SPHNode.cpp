#include "SPHNode.hpp"

#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

uint spatialHash(const glm::ivec3 &cell) {
    return (((uint)(cell.x * 73856093) ^ (uint)(cell.y * 19349663) ^ (uint)(cell.z * 83492791)) % MAP_SIZE) + 1;
}

namespace GLOO
{
    SPHNode::SPHNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type)
    {
        init_positions_ = init_positions;
        integration_step_ = integration_step;
        integrator_type_ = integrator_type;
        integrator_ = IntegratorFactory::CreateIntegrator<SPHSystem, ParticleState>(integrator_type_);

        // initialize particles
        initParticles();

        // make scene nodes for each particle (particle scene node idx == particle ParticleState idx)
        for (int i = 0; i < init_positions.size(); i++)
        {
            // each particle is its own scene node represented as sphere
            auto particle_node = make_unique<SceneNode>();
            particle_node->GetTransform().SetPosition(init_positions[i]);
            particle_node->CreateComponent<ShadingComponent>(shader_);
            particle_node->CreateComponent<RenderingComponent>(sphere_mesh_);
            auto material = std::make_shared<Material>(turquoise, turquoise, turquoise, 0);
            particle_node->CreateComponent<MaterialComponent>(material);

            particle_ptrs.push_back(particle_node.get());
            AddChild(std::move(particle_node));
        }
    }

    void SPHNode::Update(double delta_time)
    {
        // Reset Simulation
        static bool prev_released = true;
        if (InputManager::GetInstance().IsKeyPressed('R'))
        {
            if (prev_released)
            {
                // resets ParticleState to initial setup
                initParticles();
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
            // CORE LOGIC AT EACH TIME STEP - no need for this when doing fixed timesteps!
            state_ = integrator_->Integrate(system_, state_, curr_time_, integration_step_);
            curr_time_ += integration_step_;
            curr_step += integration_step_;
            // update position for each particle
            for (int i = 0; i < particle_ptrs.size(); i++)
            {
                particle_ptrs[i]->GetTransform().SetPosition(state_.positions[i]);
            }
        }
    }

    void SPHNode::initParticles() {
        state_.positions.clear();
        state_.positions.push_back(glm::vec3(0.));
        state_.positions.insert(state_.positions.end(), init_positions_.begin(), init_positions_.end());
        state_.velocities = std::vector<glm::vec3>(init_positions_.size() + 1, glm::vec3(0.));
        state_.forces = std::vector<glm::vec3>(init_positions_.size() + 1, glm::vec3(0.));
        state_.accelerations = std::vector<glm::vec3>(init_positions_.size() + 1, glm::vec3(0.));
        state_.densities = std::vector<float>(init_positions_.size() + 1, 0.);
        state_.pressures = std::vector<float>(init_positions_.size() + 1, 0.);
        state_.next = std::vector<int>(init_positions_.size() + 1, 0);
    }

    void SPHNode::neighborUpdates() {
        // clear all old neighbor info and re-calculate for each iteration

        neighbor_map.clear();
        for (int i = 1; i < particle_ptrs.size() + 1; i++) {
            // get cell hash
            int idx = spatialHash(currCell(state_.positions[i]));

            // no particle put in that cell yet
            if (neighbor_map[idx] == 0) {
                // no other particle is there, so next is empty
                state_.next[i] = 0;
                // update map to point to this particle
                neighbor_map[idx] = i;
            } else {
                // move current mapping to be next of current particle, make curr particle the mapping
                state_.next[i] = neighbor_map[idx];
                neighbor_map[idx] = i;
            }
        }
    }

    glm::ivec3 SPHNode::currCell(glm::vec3 pos) {
        return glm::ivec3(pos.x / h, pos.y / h, pos.z / h);
    }
}