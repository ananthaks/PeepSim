#pragma once
#include "../globalincludes.h"

template <class T, int dim>
struct AgentTemplate {

  int mId;

  Matrix<T, dim, 1> mStartPosition;
  Matrix<T, dim, 1> mTargetPosition;

  Matrix<T, dim, 1> mCurrPosition; // xn
  Matrix<T, dim, 1> mProposedPosition; // x*

  Matrix<T, dim, 1> mCurrVelocity; // vn
  Matrix<T, dim, 1> mPlannerVelocity; // vp
  Matrix<T, dim, 1> mBlendedVelocity; // vb

  Matrix<T, dim, 1> mForce; // f

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

  Agent& getAgent(unsigned int index);

  void addAgent(const Matrix<T, dim, 1> &startPos, const Matrix<T, dim, 1> &target,
                const Matrix<T, dim, 1> &plannedVelocity);

  void outputFrame(unsigned int frameId);

};
