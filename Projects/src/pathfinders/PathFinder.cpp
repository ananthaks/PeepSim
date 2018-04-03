#include "PathFinder.h"

PathFinder::PathFinder(int sceneWidth, int sceneHeight, std::pair<int, int> &resizeFactor) :
  mSceneWidth(sceneWidth), mSceneHeight(sceneHeight), mGridWidth(sceneWidth), mGridHeight(sceneHeight) {}
