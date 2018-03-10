#include <cmath>
#include "CollisionAvoidanceConstraint.h"

CollisionAvoidanceConstraint::CollisionAvoidanceConstraint() : Constraint() {
}

Vector CollisionAvoidanceConstraint::evaluate(Agent x1, Agent x2) {
  Vector delta = Vector::Zero();

  // TODO: Stiffness Constaint?

  Vector distVec = x1.mCurrPosition - x2.mCurrPosition;
  Vector proposedVec = x1.mProposedPosition - x2.mProposedPosition;

  float a = (1.0f / TIME_STEP_SQ) * (proposedVec.squaredNorm());
  float b = (-1.0f / TIME_STEP) * (distVec.dot(proposedVec));

  // TODO: Take Agent Specific Radius
  float agentRadiusSum = AGENT_RADIUS + AGENT_RADIUS;
  float c = distVec.squaredNorm() - (agentRadiusSum * agentRadiusSum);

  float tau = (b - std::sqrt((b * b) - (a * c))) / a;

  if (tau > 0 && tau < CONSTRAINT_CA_MAX_TAU) {
    float tauPrev = TIME_STEP * std::floor(tau / TIME_STEP);
    float tauNext = tauPrev + TIME_STEP;

    // Position just before Collision
    // xi^
    Vector predictAgent1Pos1 = x1.mCurrPosition + tauPrev * x1.mBlendedVelocity;
    // xj^
    Vector predictAgent2Pos1 = x2.mCurrPosition + tauPrev * x2.mBlendedVelocity;

    // Position with actual Collision
    // xi~
    Vector predictAgent1Pos2 = x1.mCurrPosition + tauNext * x1.mBlendedVelocity;
    // xj~
    Vector predictAgent2Pos2 = x2.mCurrPosition + tauNext * x2.mBlendedVelocity;

    // Relative Displacement
    Vector d = (predictAgent1Pos2 - predictAgent1Pos1) -
      (predictAgent2Pos2 - predictAgent2Pos1);

    // Contact Normal
    Vector n = (predictAgent1Pos2 - predictAgent2Pos2).normalized();

    Vector dn = (d.dot(n)) * n;
    Vector dt = d - dn; // Tangential Displacement needed

    delta = dt;
  }

  return delta;
}
