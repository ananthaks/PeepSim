#include "FrictionalConstraint.h"

#include "components/Agents.h"

FrictionalConstraint::FrictionalConstraint() : Constraint() {
}

Vector FrictionalConstraint::evaluate(Agent x1, Agent x2) {
  return Vector() << 0, 0;
}
