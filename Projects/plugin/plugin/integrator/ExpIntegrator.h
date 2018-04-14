#pragma once

#include "BaseIntegrator.h"

class ExpIntegrator : public BaseIntegrator {

public:

  const int EULER = 1;
  const int RUNGE_KUTTA_2 = 2;
  const int RUNGE_KUTTA_4 = 3;

public:
    ExpIntegrator(std::string name);

    ~ExpIntegrator() = default;

    virtual void integrate(float timeStep, int params, const State<T, dim> &currentState, State<T, dim> &newState);

};
