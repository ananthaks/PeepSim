#include "FrictionalConstraint.h"

FrictionalConstraint::FrictionalConstraint() : Constraint() {
}

Vector FrictionalConstraint::evaluate(Agent x1, Agent x2) {
  // | x1 - x2 | - (r + r)
  Vector distVec = x1.mCurrPosition - x2.mCurrPosition;
  float minDistance = AGENT_RADIUS + AGENT_RADIUS;

  float constraintValue = distVec.norm() - minDistance;

  Vector delta = Vector::Zero();

  if (constraintValue < 0) {
    // Need to apply Constraint to reach minDistance length between x1 & x2.
    Vector n = distVec.normalized();
    float invWeight1 = 1.0f / AGENT_MASS;
    float invWeight2 = 1.0f / AGENT_MASS;

    float s = constraintValue / (invWeight1 + invWeight2);
    delta = (-invWeight1 * s) * n;
  }

  return delta;
}
