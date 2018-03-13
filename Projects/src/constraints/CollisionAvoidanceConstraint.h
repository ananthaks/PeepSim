#pragma once

#include "Constraint.h"
#include "../components/Agents.h"

class CollisionAvoidanceConstraint : public Constraint {

  public:
    CollisionAvoidanceConstraint();
    VectorPair evaluate(Agent x1, Agent x2) override;
};
