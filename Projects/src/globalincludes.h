#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include "mathfu/vector.h"
#include "mathfu/matrix.h"

#ifndef DISABLE_PARTIO
#include "Partio.h"
#endif

using T = float;
const int dim = 2;

using Vector = mathfu::Vector<float, 2>;
using Vector3 = mathfu::Vector<float, 3>;
using VectorPair = std::pair<Vector, Vector>;

const int NUM_AGENTS = 50;

constexpr float GROUND_Y_POS = 0.f;
constexpr int PATH_GRID_SIZE = 100;

struct Cell {

  int posX, posZ;
  int parentX, parentZ;

  int GCost;
  int HCost;
  int FCost;

  bool isClosed;
  bool isBlocked;
};

struct PeepSimConfig {
  float mFPS{48.0f};
  float mSimualtionDuration {10.0f};
  float mTimeStep;
  float mTimeStepSq;
  float mVelocityBlend{0.8f};
  float mMaxVelocity{2.0f};
  int mMaxStabilityIterations{10};
  int mMaxIterations{5};
  float mDefaultAgentRadius{0.25f};
  float mDefaultAgentMass{1.0f};
  float mViscosityH{7.0f};
  float mViscosityC{217.0f};
  int mCollisionMarchSteps{1000};
  float mPoly6Kernel{1.566681471f};
  int mPathGridSize{100};
  float mMinDistanceToTarget{0.5f};
  float mAvoidanceMaxTau{20.0f};

  void create() {
    mTimeStep = 1.0f / mFPS;
    mTimeStepSq = mTimeStep * mTimeStep;
  }
};


