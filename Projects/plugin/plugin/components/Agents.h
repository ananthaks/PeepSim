#pragma once
#include "../globalincludes.h"

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

};

using Agent = AgentTemplate<T, dim>;

class Agents {

private:
  int mNumAgents;
  float mMass;
  std::vector<Agent> mAgents;

public:

  Agents(int numAgents);

  void setMass(float mass);

  int getNumAgents() const;

  Agent& getAgent(int index);

  void addAgent(const Vector &startPos, const Vector &target,
                const Vector &plannedVelocity, float mass = 1.0f, float radius = 0.25f);

  void outputFrame(unsigned int frameId);

    std::vector<Vector> getAllPositions();

};
