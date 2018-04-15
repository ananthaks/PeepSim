#pragma once

#include <string>
#include "Solver.h"
#include "components/Scene.h"

class CrowdSim {
  using String = std::string;

  PeepSimConfig mConfig;

public:
  CrowdSim(const PeepSimConfig& config);
  CrowdSim(const PeepSimConfig& config, const Scene& scene);

  void loadSceneFromFile(String filePath);
  Results evaluate();

private:
  Scene mScene;
  Solver mSolver;
  
};
