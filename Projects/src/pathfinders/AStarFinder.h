#pragma once

#include "PathFinder.h"

struct Cell {

  int posX, posZ;
  int parentX, parentZ;

  int GCost;
  int HCost;
  int FCost;

  bool isClosed;

  Cell();

};


class AStarFinder : public PathFinder {

public:

  AStarFinder(int sceneWidth, int sceneHeight, std::pair<int, int> &resizeFactor);

  void initialize(Scene &scene) override;

  bool getPathToTarget(Vector &currPos, Vector &targetPos, std::vector<Vector> &path) override;

};
