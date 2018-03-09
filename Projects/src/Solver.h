#pragma once

#include "globalincludes.h"
#include "components/Agents.h"
#include "integrator/ExpIntegrator.h"

class Solver {

private:
  Agents mAgents;
  ExpIntegrator mExplicitIntegrator;

public:

  Solver();

  void initialize();

  void solve();

};
