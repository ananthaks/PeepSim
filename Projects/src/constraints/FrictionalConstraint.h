#pragma once

#include "Constraint.h"

class FrictionalConstraint : public Constraint {

  public:
    FrictionalConstraint();
    Vector3d evaluate() override;
};
