#include "FrictionalConstraint.h"

FrictionalConstraint::FrictionalConstraint(const PeepSimConfig& config) : Constraint(config) {
}

VectorPair FrictionalConstraint::evaluate(Agent &x1, Agent &x2) {
  // | x1 - x2 | - (r + r)
  Vector distVec = x1.mCurrPosition - x2.mCurrPosition;

  // TODO: Take Agent Specific Radius
  float minDistance = x1.mRadius + x2.mRadius;

  float constraintValue = distVec.Length() - minDistance;

  VectorPair result = VectorPair(Vector(0, 0), Vector(0, 0));

  if (constraintValue < 0) {
    // Need to apply Constraint to reach minDistance length between x1 & x2.
    Vector n = distVec.Normalized();
    float invWeight1 = 1.0f / x1.mMass;
    float invWeight2 = 1.0f / x2.mMass;

    float s = constraintValue / (invWeight1 + invWeight2);
    result.first = (-invWeight1 * s) * n;
    result.second = (invWeight2 * s) * n;

  }

  return result;
}
