#include "FrictionalConstraint.h"

#include "components/Agents.h"

FrictionalConstraint::FrictionalConstraint() : Constraint() {
}

Vector2 FrictionalConstraint::evaluate(Agent x1, Agent x2) {
  return Vector2() << 0, 0;
}
