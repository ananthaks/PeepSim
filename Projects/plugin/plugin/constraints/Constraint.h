#pragma once

#include "../globalincludes.h"
#include "../components/Agents.h"

class Constraint {
  protected:
    const PeepSimConfig& mConfig;

  public:
    Constraint(const PeepSimConfig& config);
    virtual VectorPair evaluate(Agent &x1, Agent &x2) = 0;
};
