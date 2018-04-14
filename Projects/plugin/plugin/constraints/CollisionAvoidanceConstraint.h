#pragma once

#include "Constraint.h"
#include "../components/Agents.h"

constexpr float CONSTRAINT_CA_MAX_TAU = 20.0f; // Collision Avoidance Max Tau

class CollisionAvoidanceConstraint : public Constraint {

  public:
    CollisionAvoidanceConstraint(const PeepSimConfig& config);
    VectorPair evaluate(Agent &x1, Agent &x2) override;
};
