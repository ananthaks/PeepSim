#pragma once

#include "../globalincludes.h"
#include "PathFinder.h"


class AStarFinder : public PathFinder {

private:

  int mGridWidth;
  int mGridHeight;

  bool mAllowDiagonal;

  Cell mNodes[PATH_GRID_SIZE][PATH_GRID_SIZE];


public:

  AStarFinder(int sceneWidth, int sceneHeight, const PeepSimConfig& config);

  void initialize(Scene &scene) override;

  bool getPathToTarget(Vector &currPos, Vector &targetPos, std::vector<Vector> &path) override;

};
