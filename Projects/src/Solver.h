#pragma once

#include "globalincludes.h"
#include "components/Agents.h"
#include "components/Scene.h"
#include "integrator/ExpIntegrator.h"
#include "constraints/FrictionalConstraint.h"
#include "constraints/CollisionAvoidanceConstraint.h"
#include "constraints/ColliderConstraint.h"

class Solver {

private:
  ExpIntegrator mExplicitIntegrator;
  FrictionalConstraint mFrictionalContraint;
  CollisionAvoidanceConstraint mCollisionAvoidance;
  ColliderConstraint mColliderConstraint;

public:

  Solver();

  void initialize();

  void solve(Scene &scene);

};
