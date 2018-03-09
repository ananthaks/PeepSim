#include "FrictionalConstraint.h"


FrictionalConstraint::FrictionalConstraint() : Constraint() {
}

Vector3d FrictionalConstraint::evaluate() {
  return Vector3d(0, 0, 0);
}
