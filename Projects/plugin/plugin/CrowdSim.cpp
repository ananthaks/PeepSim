#include "CrowdSim.h"

CrowdSim::CrowdSim(const PeepSimConfig& config): mConfig(config), mSolver(Solver(config)), mScene(Scene(config)) {
}

CrowdSim::CrowdSim(const PeepSimConfig& config, const Scene& scene) : mScene(scene), mConfig(config), mSolver(Solver(config)) {

}

void CrowdSim::loadSceneFromFile(String filePath) {
  mScene.loadFromFile(filePath);
}

Results CrowdSim::evaluate() {
  mSolver.initialize();
  return mSolver.solve(mScene);
}
