#ifndef SPH_NODE_H_
#define SPH_NODE_H_

#include "gloo/SceneNode.hpp"
#include "ParticleState.hpp"
#include "SPHSystem.hpp"
#include "IntegratorFactory.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include <map>

#define MAP_SIZE 1000000

namespace GLOO
{
    class SPHNode : public SceneNode
    {
    public:
        // require initial position for all particles
        SPHNode(std::vector<glm::vec3> init_positions, float integration_step, IntegratorType integrator_type);
        void Update(double delta_time) override;
        void initParticles();
        void neighborUpdates();
        glm::ivec3 currCell(glm::vec3 pos);

    private : float h = 0.15f;
        float curr_time_ = 0.;
        std::vector<glm::vec3> init_positions_;
        float integration_step_;
        IntegratorType integrator_type_;
        ParticleState state_ = ParticleState();
        // the new particle system class
        SPHSystem system_ = SPHSystem();
        // neighbor map
        std::map<int, int> neighbor_map;
        // integrator
        std::unique_ptr<IntegratorBase<SPHSystem, ParticleState>> integrator_;
        // pointers for particle scene nodes, used for updating positions in scene
        std::vector<SceneNode *> particle_ptrs;
        std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(0.05, 30, 30);
        std::shared_ptr<ShaderProgram> shader_ = std::make_shared<PhongShader>();
        glm::vec3 turquoise = glm::vec3(0.18823529411f, 0.83529411764f, 0.78431372549f);
    };
}

#endif