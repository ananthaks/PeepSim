#pragma once

#include "../globalincludes.h"

class Constraint {

  public:
    Constraint();
    virtual Vector3d evaluate() = 0;
};
