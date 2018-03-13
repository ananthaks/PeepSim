#pragma once

#include <string>
#include "Solver.h"
#include "components/Scene.h"

class CrowdSim {
  using String = std::string;

public:
  CrowdSim();

  void loadSceneFromFile(String filePath);
  void loadTestScene();
  void evaluate();

private:
  Scene mScene;
  Solver mSolver;
};
