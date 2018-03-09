#pragma once

#include "../globalincludes.h"
#include "../components/Agents.h"

class Constraint {

  public:
    Constraint();
    virtual Vector2 evaluate(Agent x1, Agent x2) = 0;
};
