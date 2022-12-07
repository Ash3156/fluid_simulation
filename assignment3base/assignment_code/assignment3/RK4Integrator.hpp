#ifndef RK4_INTEGRATOR_H_
#define RK4_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO
{
    template <class TSystem, class TState>
    class RK4Integrator : public IntegratorBase<TSystem, TState>
    {
    public:
        TState Integrate(const TSystem &system,
                         const TState &state,
                         float start_time,
                         float dt) const override
        {
            std::vector<float> densities = {};
            for (int i = 0; i < state.positions.size(); i++) {
                float row = 0;
                for (int j = 0; j < state.positions.size(); j++) {
                    if (j != i) {
                        float r = glm::length(state.positions[i] - state.positions[j]);
                        
                        row += mass * 1./(std::pow(M_PI, 1.5) * std::pow(h, 3.)) * std::exp(r*r/(h*h));
                    }
                }
                densities.push_back(row);
            }

            std::vector<float> pressures = {};               
            for (float d : densities) {
                pressures.push_back(k * (d - 1000));
                std::cout << d << std::endl;
            }

            TState k1 = system.ComputeTimeDerivative(state, start_time);
            TState k2 = system.ComputeTimeDerivative(state + (dt / 2) * k1, start_time + dt / 2);
            TState k3 = system.ComputeTimeDerivative(state + (dt / 2) * k2, start_time + dt / 2);
            TState k4 = system.ComputeTimeDerivative(state + dt * k3, start_time + dt);

            return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
        }
    private:
        const float mass = 1.;
        const float gravity = -9.8;
        const float drag_constant = 1.0;
        // smoothing width
        const float h = 1.;
        // water vapor gas constant
        const float k = 461.52;
    };
} // namespace GLOO

#endif
