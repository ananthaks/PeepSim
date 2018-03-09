#include "FrictionalConstraint.h"

#include "components/Agents.h"

FrictionalConstraint::FrictionalConstraint() : Constraint() {
}

Vector2f FrictionalConstraint::evaluate(Agent x1, Agent x2) {
  return Vector2f(0, 0, 0);
}
