#pragma once

#include "Constraint.h"
#include "../components/Agents.h"
#include "../components/Scene.h"

class ColliderConstraint : public Constraint {

  public:
    ColliderConstraint(const PeepSimConfig& config);
    VectorPair evaluate(Agent &x1, Agent &x2) override;
    Vector evaluate(Scene &scene, Agent &agent);
};
