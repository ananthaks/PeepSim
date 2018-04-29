#pragma once
#include "../globalincludes.h"
#include <GU/GU_Detail.h>

template <class T, int dim>
struct AgentTemplate {

  int mId;

  T mMass;
  T mRadius;

  Vector mStartPosition;
  Vector mTargetPosition;

  Vector mCurrPosition; // xn
  Vector mProposedPosition; // x*

  Vector mCurrVelocity; // vn
  Vector mPlannerVelocity; // vp
  Vector mBlendedVelocity; // vb

  Vector mForce; // f

  int currTarget;

  std::vector<Vector> mPlannedPath; // Path to Target

  std::vector<Vector> mCachedPos;

  const GU_Detail *mReference;
  GEO_Primitive *mCurrGeo;

};

using Agent = AgentTemplate<T, dim>;

struct AgentGroup {
  Vector3 mDebugColor;
  std::vector<Agent> mAgents;
};
