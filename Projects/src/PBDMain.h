#pragma once

#include "globalincludes.h"
#include "components/Agents.h"
#include "integrator/ExpIntegrator.h"

class PBDMain {

private:
  Agents mAgents;
  ExpIntegrator mExplicitIntegrator;

public:

  PBDMain();

  void initialize();

  void solve();

};
