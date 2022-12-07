#include "SimulationApp.hpp"

#include "glm/gtx/string_cast.hpp"

#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/CameraComponent.hpp"
#include "gloo/components/LightComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/debug/AxisNode.hpp"
#include "SimpleNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "PendulumNode.hpp"
#include "ClothNode.hpp"

namespace GLOO {
SimulationApp::SimulationApp(const std::string& app_name,
                             glm::ivec2 window_size,
                             IntegratorType integrator_type,
                             float integration_step)
    : Application(app_name, window_size),
      integrator_type_(integrator_type),
      integration_step_(integration_step) {
  // TODO: remove the following two lines and use integrator type and step to
  // create integrators; the lines below exist only to suppress compiler
  // warnings.
  // UNUSED(integrator_type_);
  // UNUSED(integration_step_);



}

void SimulationApp::SetupScene() {
  SceneNode& root = scene_->GetRootNode();

  auto camera_node = make_unique<ArcBallCameraNode>(45.f, 0.75f, 5.0f);
  scene_->ActivateCamera(camera_node->GetComponentPtr<CameraComponent>());
  root.AddChild(std::move(camera_node));

  root.AddChild(make_unique<AxisNode>('A'));

  auto ambient_light = std::make_shared<AmbientLight>();
  ambient_light->SetAmbientColor(glm::vec3(0.2f));
  root.CreateComponent<LightComponent>(ambient_light);

  auto point_light = std::make_shared<PointLight>();
  point_light->SetDiffuseColor(glm::vec3(0.8f, 0.8f, 0.8f));
  point_light->SetSpecularColor(glm::vec3(1.0f, 1.0f, 1.0f));
  point_light->SetAttenuation(glm::vec3(1.0f, 0.09f, 0.032f));
  auto point_light_node = make_unique<SceneNode>();
  point_light_node->CreateComponent<LightComponent>(point_light);
  point_light_node->GetTransform().SetPosition(glm::vec3(0.0f, 2.0f, 4.f));
  root.AddChild(std::move(point_light_node));

  // Simple Node
  auto simple_node = make_unique<SimpleNode>(glm::vec3(0., 1., 2.), integration_step_, integrator_type_);
  std::shared_ptr<VertexObject> sphere_mesh_ = PrimitiveFactory::CreateSphere(0.3, 30, 30);
  simple_node->CreateComponent<RenderingComponent>(sphere_mesh_);
  std::shared_ptr<ShaderProgram> shader_ = std::make_shared<PhongShader>();
  simple_node->CreateComponent<ShadingComponent>(shader_);
  root.AddChild(std::move(simple_node));

  // Pendulum Node
  std::vector<glm::vec3> pendulum_init_positions = {glm::vec3(-2., 0., 0.), 
                                                    glm::vec3(-7., 2.6, 0.), 
                                                    glm::vec3(1., -0.2, 0.), 
                                                    glm::vec3(-3.0, -1.0, 0.), 
                                                    glm::vec3(-1., -10.5, 0.), 
                                                    glm::vec3(7.0, -5.8, -2.0), 
                                                    glm::vec3(0., -6.5, 0.), 
                                                    glm::vec3(3.5, -8.5, 4.3)};
  auto pendulum_node = make_unique<PendulumNode>(pendulum_init_positions, integration_step_, integrator_type_);
  root.AddChild(std::move(pendulum_node));

  // Cloth Node  
  std::vector<glm::vec3> cloth_init_positions = {
    glm::vec3(1.0, 2.0, 0.0), glm::vec3(9./7., 2.0, 0.0), glm::vec3(11./7., 2.0, 0.0), glm::vec3(13./7., 2.0, 0.0), glm::vec3(15./7., 2.0, 0.0), glm::vec3(17./7., 2.0, 0.0), glm::vec3(19./7., 2.0, 0.0), glm::vec3(3.0, 2.0, 0.0),
    glm::vec3(1.0, 12./7., 0.0), glm::vec3(9./7., 12./7., 0.0), glm::vec3(11./7., 12./7., 0.0), glm::vec3(13./7., 12./7., 0.0), glm::vec3(15./7., 12./7., 0.0), glm::vec3(17./7., 12./7., 0.0), glm::vec3(19./7., 12./7., 0.0), glm::vec3(3.0, 12./7., 0.0),
    glm::vec3(1.0, 10./7., 0.0), glm::vec3(9./7., 10./7., 0.0), glm::vec3(11./7., 10./7., 0.0), glm::vec3(13./7., 10./7., 0.0), glm::vec3(15./7., 10./7., 0.0), glm::vec3(17./7., 10./7., 0.0), glm::vec3(19./7., 10./7., 0.0), glm::vec3(3.0, 10./7., 0.0),
    glm::vec3(1.0, 8./7., 0.0), glm::vec3(9./7., 8./7., 0.0), glm::vec3(11./7., 8./7., 0.0), glm::vec3(13./7., 8./7., 0.0), glm::vec3(15./7., 8./7., 0.0), glm::vec3(17./7., 8./7., 0.0), glm::vec3(19./7., 8./7., 0.0), glm::vec3(3.0, 8./7., 0.0),
    glm::vec3(1.0, 6./7., 0.0), glm::vec3(9./7., 6./7., 0.0), glm::vec3(11./7., 6./7., 0.0), glm::vec3(13./7., 6./7., 0.0), glm::vec3(15./7., 6./7., 0.0), glm::vec3(17./7., 6./7., 0.0), glm::vec3(19./7., 6./7., 0.0), glm::vec3(3.0, 6./7., 0.0),
    glm::vec3(1.0, 4./7., 0.0), glm::vec3(9./7., 4./7., 0.0), glm::vec3(11./7., 4./7., 0.0), glm::vec3(13./7., 4./7., 0.0), glm::vec3(15./7., 4./7., 0.0), glm::vec3(17./7., 4./7., 0.0), glm::vec3(19./7., 4./7., 0.0), glm::vec3(3.0, 4./7., 0.0),
    glm::vec3(1.0, 2./7., 0.0), glm::vec3(9./7., 2./7., 0.0), glm::vec3(11./7., 2./7., 0.0), glm::vec3(13./7., 2./7., 0.0), glm::vec3(15./7., 2./7., 0.0), glm::vec3(17./7., 2./7., 0.0), glm::vec3(19./7., 2./7., 0.0), glm::vec3(3.0, 2./7., 0.0),
    glm::vec3(1.0, 0.0, 0.0), glm::vec3(9./7., 0.0, 0.0), glm::vec3(11./7., 0.0, 0.0), glm::vec3(13./7., 0.0, 0.0), glm::vec3(15./7., 0.0, 0.0), glm::vec3(17./7., 0.0, 0.0), glm::vec3(19./7., 0.0, 0.0), glm::vec3(3.0, 0.0, 0.0)
  };
  auto cloth_node = make_unique<ClothNode>(cloth_init_positions, integration_step_, integrator_type_);
  root.AddChild(std::move(cloth_node));
}
}  // namespace GLOO
