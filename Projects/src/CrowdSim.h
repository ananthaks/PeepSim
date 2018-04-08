#pragma once

#include <string>
#include "Solver.h"
#include "components/Scene.h"

class CrowdSim {
  using String = std::string;

  const PeepSimConfig& mConfig;

public:
  CrowdSim(const PeepSimConfig& config);

  void loadSceneFromFile(String filePath);
  void loadTestScene();
  void evaluate();

private:
  Scene mScene;
  Solver mSolver;
};
