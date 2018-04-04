#include "AStarFinder.h"
#include <limits>
#include <set>

static int MAX = std::numeric_limits<int>::max();
static int MAX_ITERATIONS = 10000;

AStarFinder::AStarFinder(int sceneWidth, int sceneHeight)
  : PathFinder(sceneWidth, sceneHeight), mGridWidth(sceneWidth), mGridHeight(sceneHeight), mAllowDiagonal(true) {}

// Manhattan Heuristic
inline float H_COST_MAH(int currX, int currZ, int targetX, int targetZ) {
  return std::abs(targetX - currX) + std::abs(targetZ - currZ);
}

// Diagonal Heuristic
inline float H_COST_DIG(int currX, int currZ, int targetX, int targetZ) {
  return std::max(std::abs(targetX - currX), std::abs(targetZ - currZ));
}

void AStarFinder::initialize(Scene &scene) {
  for(auto& collider: scene.mColliders) {
    collider->fillCollisionSpace(mNodes, mGridWidth, mGridHeight);
  }
}

bool AStarFinder::getPathToTarget(Vector &currPos, Vector &targetPos, std::vector<Vector> &path) {

  bool pathFound = false;

  std::pair<int, int> startPos = std::make_pair(std::floor(currPos[0] + PATH_GRID_SIZE / 2.f), std::floor(currPos[1] + PATH_GRID_SIZE / 2.f));
  std::pair<int, int> endPos = std::make_pair(std::floor(targetPos[0] + PATH_GRID_SIZE / 2.f), std::floor(targetPos[1] + PATH_GRID_SIZE / 2.f));

  // target & source at same location
  if(startPos.first == endPos.first && startPos.second == endPos.second) {
    return true;
  }

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
  int itera = 0;
  int i, j;

  while(!openList.empty()) {

    itera++;

    if(itera > 10000) {
      std::cout << "Breaking main from infinite loop " << itera << std::endl;
      break;
    }

    std::pair<int, std::pair<int, int>> p = *openList.begin();

    openList.erase(openList.begin());

    i = p.second.first;
    j = p.second.second;

    mNodes[i][j].isClosed = true;

    int gNew, hNew, fNew;

    // Immediate neighbours
    for(int p = 0; p  < 4; ++p) {

      newX = i - 1 * (p < 2? (p == 0 ? 1 : -1): 0);
      newZ = j - 1 * (p < 2? 0: (p == 2 ? 1 : -1));

      if(newX >= 0 && newX < mGridWidth && newZ >= 0 && newZ < mGridHeight && !mNodes[newX][newZ].isBlocked) {
        if(newX == endPos.first && newZ == endPos.second) {
          mNodes[newX][newZ].parentX = i;
          mNodes[newX][newZ].parentZ = j;
          pathFound = true;
          break;
        }

        if(!mNodes[newX][newZ].isClosed) {

          gNew = mNodes[i][j].GCost + 1.0;
          hNew = H_COST_MAH(newX, newZ, endPos.first, endPos.second);
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

    // Diagonals
    for(int p = 0; p  < 4; ++p) {

      newX = i - 1 * (p < 2? (p == 0 ? 1 : -1): 0);
      newZ = j - 1 * (p < 2? 0: (p == 2 ? 1 : -1));

      if(newX >= 0 && newX < mGridWidth && newZ >= 0 && newZ < mGridHeight && !mNodes[newX][newZ].isBlocked) {
        if(newX == endPos.first && newZ == endPos.second) {
          mNodes[newX][newZ].parentX = i;
          mNodes[newX][newZ].parentZ = j;
          pathFound = true;
          break;
        }

        if(!mNodes[newX][newZ].isClosed) {

          gNew = mNodes[i][j].GCost + 1.0;
          hNew = H_COST_MAH(newX, newZ, endPos.first, endPos.second);
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

  }

  // Trace The path
  if(pathFound) {

    int x = endPos.first;
    int z = endPos.second;
    int tempX, tempZ;

    itera = 0;

    while(x != startPos.first || z != startPos.second) {

      itera++;

      if(itera > MAX_ITERATIONS) {
        std::cout << "Breaking path from infinite loop" << std::endl;
        break;
      }
      path.push_back(Vector(x - PATH_GRID_SIZE / 2.f, z - PATH_GRID_SIZE / 2.f));
      tempX = mNodes[x][z].parentX;
      tempZ = mNodes[x][z].parentZ;
      x = tempX;
      z = tempZ;
    }

  }

  return pathFound;
}
