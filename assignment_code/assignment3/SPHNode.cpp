#include "SPHNode.hpp"

#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/InputManager.hpp"

#define MAP_SIZE 1000000

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

        // particle_ptrs.push_back(NULL);
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

        // std::cout << "HERE"
        // std::cout << particle_ptrs.size() << std::endl;

        // for lengths to be equivalent with other vectors, need particle_ptrs to have 
        // filler in 0th index (will just repeat first position - never used)
        // particle_ptrs.insert(particle_ptrs.begin(), particle_ptrs[0]);
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

        // new stuff

        deltaTime = 0.003f;

        // std::cout << "neigh" << std::endl;
        neighborUpdates();

        // calculate densities and pressures for all particles
        for (int i = 1; i < particle_ptrs.size() + 1; i++) {
            float curr_density = 0.;
            glm::ivec3 curr_cell = currCell(state_.positions[i]);
            // check all neighboring cells
            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {
                    for (int z = -1; z < 2; z++) {
                        // j is index of current neighbor
                        int j = neighbor_map[spatialHash(curr_cell + glm::ivec3(x, y, z))];

                        // check all neighbors
                        while (j != 0) {
                            float r = glm::length(state_.positions[i] - state_.positions[j]);
                            if (r < h && j != i) {
                                curr_density += mass * poly6 * std::pow(h * h - r * r, 3);
                            }
                            j = state_.next[j];
                        }
                    }
                }
            }
            state_.densities[i] = curr_density + self_density;
            state_.pressures[i] = k * (state_.densities[i] - rest_density);
        }

        // calculate forces for all particles (from pressure and viscosity)
        for (int i = 1; i < particle_ptrs.size(); i++) {
            state_.forces[i] = glm::vec3(0.);
            glm::ivec3 curr_cell = currCell(state_.positions[i]);

            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {
                    for (int z = -1; z < 2; z++) {
                        // j is index of current neighbor
                        int j = neighbor_map[spatialHash(curr_cell + glm::ivec3(x, y, z))];
                        // check all neighbors
                        while (j != 0) {
                            float r = glm::length(state_.positions[i] - state_.positions[j]);
                            if (r < h && j != i) {
                                glm::vec3 dir = glm::normalize(state_.positions[j] - state_.positions[i]);
                                state_.forces[i] += (-dir * mass * (state_.pressures[i] + state_.pressures[j]) 
                                / (2 * state_.densities[j] + epsilon) * spiky_grad) * (float)std::pow(h - r, 2);

                                // viscosity
                                state_.forces[i] += viscosity * mass * ((state_.velocities[j] - state_.velocities[i]) 
                                / (state_.densities[j] + epsilon)) * (-spiky_grad) * (h - r);
                            }
                            j = state_.next[j];
                        }
                    }
                }
            }
        }

        // update positions for all particles
        for (int i = 1; i < particle_ptrs.size() + 1; i++) {
            glm::vec3 acceleration = state_.forces[i] / state_.densities[i];
            // gravity
            acceleration += glm::vec3(0., -9.8, 0.);

            // update velocity by acceleration * deltaTime
            state_.velocities[i] += acceleration * deltaTime;

            // update position to be velocity * deltaTime
            state_.positions[i] += state_.velocities[i] * deltaTime;

            // Boundary Box checks - particles need to be in cube centered around origin with coord ranges (-2, 2)
            // fix position to be boundary and flip velocity
            for (int j = 0; j < 3; j++) {
                if (state_.positions[i][j] < -2.f) {
                    state_.positions[i][j] = -2.f;
                    state_.velocities[i][j] *= -0.5f;
                }
                if (state_.positions[i][j] > 2.f) {
                    state_.positions[i][j] = 2.f;
                    state_.velocities[i][j] *= -0.5f;
                }
            }

            // std::cout << i << std::endl;
            // std::cout << particle_ptrs[i] << std::endl;
            // update the scene nodes themselves when totally done with position calculations
            particle_ptrs[i]->GetTransform().SetPosition(state_.positions[i]);
            // std::cout << "GOOD" << std::endl;
        }
        // std::cout << "OUTSIDE" << std::endl;
    //     // OLD SHIT

    //     float curr_step = integration_step_;
    //     while (curr_step <= delta_time)
    //     {
    //         // CORE LOGIC AT EACH TIME STEP - no need for this when doing fixed timesteps!
    //         state_ = integrator_->Integrate(system_, state_, curr_time_, integration_step_);
    //         curr_time_ += integration_step_;
    //         curr_step += integration_step_;
    //         // update position for each particle
    //         for (int i = 0; i < particle_ptrs.size(); i++)
    //         {
    //             particle_ptrs[i]->GetTransform().SetPosition(state_.positions[i]);
    //         }
    //     }
    //     std::cout << "HERE" << std::endl;
    }

    void SPHNode::initParticles() {
        state_.positions.clear();
        state_.positions.push_back(glm::vec3(0.));
        state_.positions.insert(state_.positions.end(), init_positions_.begin(), init_positions_.end());
        state_.velocities = std::vector<glm::vec3>(init_positions_.size() + 1, glm::vec3(0.));
        state_.forces = std::vector<glm::vec3>(init_positions_.size() + 1, glm::vec3(0.));
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