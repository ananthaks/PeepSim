#include "CrowdSim.h"
#include "pathfinders/AStarFinder.h"
#include <iostream>
#include <vector>

CrowdSim::CrowdSim() {
}

void CrowdSim::loadSceneFromFile(String filePath) {
  mScene.loadFromFile(filePath);
  // TODO: Send the Scene to Solver
}

void CrowdSim::loadTestScene() {
  // TODO: Initialize Test Scene with N Agents and other variables Here
  // and Send it to Solver!

  for(int i = 0; i < 10; ++i) {
    mScene.mAgents.addAgent(Vector(5.f, i + 0.5f), Vector(-5.f, i + 0.5f), Vector::Zero());
  }

  for(int i = 0; i < 10; ++i)  {
    mScene.mAgents.addAgent(Vector(-5.f, i), Vector(5.f, i), Vector::Zero());
  }


}

void CrowdSim::evaluate() {
  mSolver.initialize();
  mSolver.solve(mScene);

  std::cout << "Starting Path finding " << std::endl;

  AStarFinder finder(7, 6);

  std::cout << "Created instance " << std::endl;

  std::vector<Vector> result;
  Vector start = Vector(1, 3);
  Vector end = Vector(5, 4);

  finder.initialize(mScene);

  std::cout << "Generating path " << std::endl;
  bool hit = finder.getPathToTarget(start, end, result);

  std::cout << "The result of path find is " << hit << std::endl;


  for(Vector &vec : result) {
    std::cout << "Path " << vec[0] << " " << vec[1] << std::endl;
  }




}
