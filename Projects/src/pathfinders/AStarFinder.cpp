#include "AStarFinder.h"
#include <limits>
#include <set>

Cell::Cell() : posX(0), posZ(0), isClosed(false), GCost(0), HCost(0), FCost(0) {}

AStarFinder::AStarFinder(int sceneWidth, int sceneHeight, std::pair<int, int> &resizeFactor)
  : PathFinder(sceneWidth, sceneHeight, resizeFactor) {}

// Manhattan Heuristic
inline float H_COST_MAH(int currX, int currZ, int targetX, int targetZ) {
  return std::abs(targetX - currX) + std::abs(targetZ - currZ);
}

// Diagonal Heuristic
inline float H_COST_DIG(int currX, int currZ, int targetX, int targetZ) {
  return std::max(std::abs(targetX - currX), std::abs(targetZ - currZ));
}

void AStarFinder::initialize(Scene &scene) {
  mGrid = MatrixXi(mGridWidth, mGridHeight);
  mGrid.setZero();

  for(auto& collider: scene.mColliders) {
    collider->fillCollisionSpace(mGrid, mGridWidth, mGridHeight);
  }
}

bool AStarFinder::getPathToTarget(Vector &currPos, Vector &targetPos, std::vector<Vector> &path) {

  bool pathFound = false;

  std::pair<int, int> startPos = std::make_pair(std::floor(currPos[0]), std::floor(currPos[2]));
  std::pair<int, int> endPos = std::make_pair(std::floor(targetPos[0]), std::floor(targetPos[2]));

  // target & source at same location
  if(startPos.first == endPos.first && startPos.second == endPos.second) {
    return true;
  }

  int MAX = std::numeric_limits<int>::max();

  Cell mNodes[mGridWidth][mGridHeight];

  for(int i = 0; i < mGridWidth; ++i) {
      for(int j = 0; j < mGridHeight; ++j) {
          mNodes[i][j].GCost = MAX;
          mNodes[i][j].HCost = MAX;
          mNodes[i][j].FCost = MAX;
          mNodes[i][j].parentX = -1;
          mNodes[i][j].parentZ = -1;
          mNodes[i][j].isClosed = false;
      }
  }

  mNodes[startPos.first][startPos.second].GCost = 0;
  mNodes[startPos.first][startPos.second].HCost = 0;
  mNodes[startPos.first][startPos.second].FCost = 0;
  mNodes[startPos.first][startPos.second].parentX = startPos.first;
  mNodes[startPos.first][startPos.second].parentZ = startPos.second;

  std::set<std::pair<int, std::pair<int, int>>> openList;

  openList.insert(std::make_pair(0, startPos));

  int newX, newZ;

  while(!openList.empty()) {

    std::pair<int, std::pair<int, int>> p = *openList.begin();

    openList.erase(openList.begin());

    // Add this vertex to the open list

    std::pair<int, int> cellPos = p.second;

    int i = p.second.first;
    int j = p.second.second;

    mNodes[i][j].isClosed = true;

    int gNew, hNew, fNew;

    // Immediate neighbours
    for(int p = 0; p  < 4; ++p) {

      newX = i - 1 * (p < 2? (p == 0 ? 1 : -1): 0);
      newZ = j - 1 * (p < 2? 0: (p == 2 ? 1 : -1));

      if(newX >= 0 && mGrid(newX, newZ) != 1) {
        if(newX == endPos.first && j == endPos.second) {
          mNodes[newX][newZ].parentX = i;
          mNodes[newX][newZ].parentZ = j;
          pathFound = true;
          break;
        }

        if(!mNodes[newX][newZ].isClosed) {

          gNew = mNodes[i][j].GCost + 1.0;
          hNew = H_COST_MAH(i, j, newX, newZ);
          fNew = gNew + hNew;

          if (mNodes[newX][newZ].FCost == MAX || mNodes[newX][newZ].FCost > fNew) {

              openList.insert(std::make_pair(fNew, std::make_pair(newX, newZ)));

              mNodes[newX][newZ].FCost = fNew;
              mNodes[newX][newZ].GCost = gNew;
              mNodes[newX][newZ].HCost = hNew;
              mNodes[newX][newZ].parentX = i;
              mNodes[newX][newZ].parentZ = j;
          }
        }
      }
    }

    if(!mAllowDiagonal) {
      continue;
    }
    // newX = i - 1 * (p < 2? (p == 0 ? 1 : -1): 0);
    // newZ = j - 1 * (p < 2? 0: (p == 2 ? 1 : -1));

    // Diagonals

  }

  // Trace The path
  if(pathFound) {

  }

  return pathFound;
}

/*std::pair<int, int> startPos = std::make_pair(std::floor(currPos[0]), std::floor(currPos[2]));
std::pair<int, int> endPos = std::make_pair(std::floor(targetPos[0]), std::floor(targetPos[2]));
std::pair<int, int> lowestCost;

// target & source at same location
if(currLoc.first == endPos.first && currLoc.second == endPos.second) {
  return true;
}

MatriXi GCost(mGridWidth, mGridHeight);
MatriXi HCost(mGridWidth, mGridHeight);
MatriXi FCost(mGridWidth, mGridHeight);

GCost.setZero();
HCost.setZero();
FCost.setZero();

std::priority_queue<std::pair<int, int>> nodesToVisit;
nodesToVisit.push_back(startPos);

while(!nodesToVisit.empty()) {

  std::pair<int, int> currLoc = nodesToVisit.top();

  if(currLoc.first == endPos.first && currLoc.second == endPos.second) {
    pathFound = true;
    break;
  }

  nodesToVisit.pop();

  // search all neighbours

  // left
  if(currLoc.first - 1 >= 0 && mGrid[currLoc.first - 1][currLoc.second] != 1) {

  }

  // right
  if(currLoc.first + 1 < mGridWidth && mGrid[currLoc.first + 1][currLoc.second] != 1) {

  }

  // top
  if(currLoc.second - 1 >= 0 && mGrid[currLoc.first][currLoc.second - 1] != 1) {

  }

  // bottom
  if(currLoc.second + 1 < mGridHeight && mGrid[currLoc.first][currLoc.second + 1] != 1) {

  }

  // left-top corner
  if(mAllowDiagonal && currLoc.first - 1 >= 0 && currLoc.second - 1 >= 0 && mGrid[currLoc.first - 1][currLoc.second - 1] != 1) {

  }

  // left-bot corner
  if(mAllowDiagonal && currLoc.first - 1 >= 0 && currLoc.second + 1 < mGridHeight && mGrid[currLoc.first - 1][currLoc.second + 1] != 1) {

  }

  // right-top corner
  if(mAllowDiagonal && currLoc.first + 1 < mGridWidth 0 && currLoc.second - 1 >= 0 && mGrid[currLoc.first + 1][currLoc.second - 1] != 1) {

  }

  // left-top corner
  if(mAllowDiagonal && currLoc.first + 1 < mGridWidth  0 && currLoc.second + 1 < mGridHeight && mGrid[currLoc.first + 1][currLoc.second + 1] != 1) {

  }
}*/
