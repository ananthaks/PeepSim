#include "CrowdSim.h"

CrowdSim::CrowdSim(const PeepSimConfig& config): mConfig(config), mSolver(Solver(config)), mScene(Scene(config)) {
}

void CrowdSim::loadSceneFromFile(String filePath) {
  mScene.loadFromFile(filePath);
  // TODO: Send the Scene to Solver
}

void CrowdSim::loadTestScene() {
  // TODO: Initialize Test Scene with N Agents and other variables Here
  // and Send it to Solver!

  for(int i = 0; i < 10; ++i) {
    mScene.mAgents.addAgent(Vector(5.f, i + 0.5f), Vector(-5.f, i + 0.5f), Vector(0, 0));
  }

  for(int i = 0; i < 10; ++i)  {
    mScene.mAgents.addAgent(Vector(-5.f, i), Vector(5.f, i), Vector(0, 0));
  }


}

Results CrowdSim::evaluate() {
  mSolver.initialize();
  return mSolver.solve(mScene);
}
