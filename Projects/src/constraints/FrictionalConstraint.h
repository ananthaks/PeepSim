#pragma once

#include "Constraint.h"
#include "../components/Agents.h"

class FrictionalConstraint : public Constraint {

  public:
    FrictionalConstraint();
    Vector2 evaluate(Agent x1, Agent x2) override;
};
