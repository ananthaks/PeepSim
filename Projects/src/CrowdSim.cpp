#include "CrowdSim.h"

CrowdSim::CrowdSim() {
}

void CrowdSim::loadSceneFromFile(String filePath) {
  mScene.loadFromFile(filePath);
  // TODO: Send the Scene to Solver
}

void CrowdSim::loadTestScene() {
  // TODO: Initialize Test Scene with N Agents and other variables Here
  // and Send it to Solver!
  mScene.mAgents.addAgent(Vector(5.f, 5.f), Vector(-5.f, 5.f), Vector::Zero());
  mScene.mAgents.addAgent(Vector(-1.f, 1.f), Vector(1.f, 1.f), Vector::Zero());
}

void CrowdSim::evaluate() {
  mSolver.initialize();
  mSolver.solve(mScene);
}
