#ifndef FORWARD_EULER_INTEGRATOR_H_
#define FORWARD_EULER_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class ForwardEulerIntegrator : public IntegratorBase<TSystem, TState> {
  public:
    TState Integrate(const TSystem& system,
                    const TState& state,
                    float start_time,
                    float dt) const override {
      return state + (dt * system.ComputeTimeDerivative(state, start_time));
    }
};
}  // namespace GLOO

#endif
