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
}

void CrowdSim::evaluate() {
  mSolver.initialize();
  mSolver.solve(mScene);
}
