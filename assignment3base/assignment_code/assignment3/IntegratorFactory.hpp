#ifndef INTEGRATOR_FACTORY_H_
#define INTEGRATOR_FACTORY_H_

#include "IntegratorBase.hpp"

#include <stdexcept>

#include "gloo/utils.hpp"

#include "IntegratorType.hpp"

#include "ForwardEulerIntegrator.hpp"
#include "TrapezoidIntegrator.hpp"
#include "RK4Integrator.hpp"

namespace GLOO {
class IntegratorFactory {
 public:
  template <class TSystem, class TState>
  static std::unique_ptr<IntegratorBase<TSystem, TState>> CreateIntegrator(
      IntegratorType type) {
    // do cases for each of the 3 types

    if (type == IntegratorType::Euler) {
      return make_unique<ForwardEulerIntegrator<TSystem, TState>>();
    }
    else if (type == IntegratorType::Trapezoidal) {
      return make_unique<TrapezoidIntegrator<TSystem, TState>>();
    }
    else if (type == IntegratorType::RK4) {
      return make_unique<RK4Integrator<TSystem, TState>>();
    }
    else {
      throw std::runtime_error("Invalid integrator type");
    }
  }
};
}  // namespace GLOO

#endif
