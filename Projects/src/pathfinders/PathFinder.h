#pragma once

#include "../globalincludes.h"
#include "../components/Scene.h"

using uc = unsigned char;

class PathFinder {

protected:

  int mSceneWidth;
  int mSceneHeight;

public:

  PathFinder(int sceneWidth, int sceneHeight);

  virtual void initialize(Scene &scene) = 0;

  virtual bool getPathToTarget(Vector &currPos, Vector &targetPos, std::vector<Vector> &path) = 0;


};
