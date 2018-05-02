#include <cmath>
#include "CollisionAvoidanceConstraint.h"

CollisionAvoidanceConstraint::CollisionAvoidanceConstraint(const PeepSimConfig& config) : Constraint(config)
{
}

VectorPair CollisionAvoidanceConstraint::evaluate(Agent &x1, Agent &x2)
{
  VectorPair result = VectorPair(Vector(0, 0), Vector(0, 0));

  // TODO: Stiffness Constaint?

  Vector distVec = x1.mCurrPosition - x2.mCurrPosition;
  Vector proposedVec = x1.mProposedPosition - x2.mProposedPosition;

  float a = (1.0f / mConfig.mTimeStepSq) * (proposedVec.LengthSquared());
  float b = (-1.0f / mConfig.mTimeStep) * (Vector::DotProduct(distVec, proposedVec));

  float agentRadiusSum = x1.mRadius + x2.mRadius;
  float c = distVec.LengthSquared() - (agentRadiusSum * agentRadiusSum);

  float tau = (b - std::sqrt((b * b) - (a * c))) / a;

  Vector v1 = (x1.mProposedPosition - x1.mCurrPosition) / mConfig.mTimeStep;
  Vector v2 = (x2.mProposedPosition - x2.mCurrPosition) / mConfig.mTimeStep;

  Vector v12 = v1 - v2;

  float a2 = v12.LengthSquared();
  float b2 = -1.0f * (Vector::DotProduct(distVec, v12));
  float c2 = c;

  float tau2 = (b2 - std::sqrt((b2 * b2) - (a2 * c2))) / a2;

  tau = tau2;

  // TODO: remove Experimental code

  if (tau > 0 && tau < mConfig.mAvoidanceMaxTau)
  {

    // std::cout<<"Doing Collision Avoidance: " << tau << " vs " << tau2 << std::endl;
    float tauPrev = mConfig.mTimeStep * std::floor(tau / mConfig.mTimeStep);
    float tauNext = tauPrev + mConfig.mTimeStep;

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
    Vector d1 = (predictAgent1Pos2 - predictAgent1Pos1) -
               (predictAgent2Pos2 - predictAgent2Pos1);

    // Contact Normal
    Vector n = (predictAgent1Pos2 - predictAgent2Pos2).Normalized();

    Vector d2 = (-d1) -  2.0f * Vector::DotProduct(-d1, n) * n;

    Vector d1n = (Vector::DotProduct(d1, n)) * n;
    Vector d2n = (Vector::DotProduct(d2, n)) * n;
    Vector d1t = d1 - d1n; // Tangential Displacement needed
    Vector d2t = d2 - d2n;

    float powVal = (-1.0f * tauPrev * tauPrev) / tau;
    float stiff = mConfig.mAvoidanceStiffness * std::exp(powVal);

    result.first = stiff * d1t;
    result.second = stiff * d2t;
  }

  return result;
}
