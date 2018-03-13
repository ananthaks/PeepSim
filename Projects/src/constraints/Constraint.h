#pragma once

#include "../globalincludes.h"
#include "../components/Agents.h"

class Constraint {

  public:
    Constraint();
    virtual VectorPair evaluate(Agent x1, Agent x2) = 0;
};
