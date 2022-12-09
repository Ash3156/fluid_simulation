#ifndef TRAPEZOID_INTEGRATOR_H_
#define TRAPEZOID_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO
{
    template <class TSystem, class TState>
    class TrapezoidIntegrator : public IntegratorBase<TSystem, TState>
    {
    public:
        TState Integrate(const TSystem &system,
                         const TState &state,
                         float start_time,
                         float dt) const override
        {
            TState f0 = system.ComputeTimeDerivative(state, start_time);
            TState f1 = system.ComputeTimeDerivative(state + (dt * f0), start_time + dt);
            return state + ((dt / 2) * (f0 + f1));
        }
    };
} // namespace GLOO

#endif
