#include "Scene.h"
#include "../colliders/BoxCollider.h"

Scene::Scene(const PeepSimConfig& config)
  : mConfig(config) {}

Scene::~Scene() {
  for (auto& collider : mColliders) {
	  if (collider != nullptr) {
		  delete collider;
		  collider = nullptr;
	  }
  }
}

AgentGroup& Scene::getAgentGroup(int index) {
  return mAgentGroups[index];
}

void Scene::addAgent(const Vector& startPos, const Vector& target, const Vector& plannedVelocity,
                     float mass, float radius, AgentGroup* group) {
  Agent agent;
  agent.mStartPosition = Vector(startPos);
  agent.mTargetPosition = Vector(target);
  agent.mCurrPosition = Vector(startPos);
  agent.mCurrVelocity = Vector(0, 0);
  agent.mForce = Vector(0, 0);
  agent.mMass = mass;
  agent.mRadius = radius;

  group->mAgents.push_back(agent);
}

SizeType Scene::getNumAgents() const {
  return mAgentGroups.size();
}

void Scene::loadFromFile(String filePath) {
  std::ifstream inputFileStream(filePath);
  String jsonData;

  // Note: File Load Not Optimized for huge files.
  inputFileStream.seekg(0, std::ios::end);
  jsonData.reserve(inputFileStream.tellg());
  inputFileStream.seekg(0, std::ios::beg);

  jsonData.assign((std::istreambuf_iterator<char>(inputFileStream)),
                  std::istreambuf_iterator<char>());

  auto data = Json::parse(jsonData);

  auto agentGroups = data["agentGroups"];

  mNumAgents = 0;

  for (Json::iterator it = agentGroups.begin(); it != agentGroups.end(); ++it) {
    /* Structure:
     *
     * "size": [1, 10],
     * "spacing": [0, 1],
     * "startPosition": [-5, 0],
     * "relativeTarget": [10, 0]
     */
    auto group = *it;
    int size[2] = {group["size"][0].get<int>(), group["size"][1].get<int>()};
    float spacing[2] = {group["spacing"][0].get<float>(), group["spacing"][1].get<float>()};
    float startPosition[2] = {
      group["startPosition"][0].get<float>(), group["startPosition"][1].get<float>()
    };
    float relativeTarget[2] = {
      group["relativeTarget"][0].get<float>(), group["relativeTarget"][1].get<float>()
    };

    AgentGroup agentGroup;

    float mass = group["mass"].get<float>();
    float radius = group["radius"].get<float>();

    for (int xSize = 0; xSize < size[0]; xSize++) {
      for (int ySize = 0; ySize < size[1]; ySize++) {

        float xOffset = xSize * spacing[0];
        float yOffset = ySize * spacing[1];

        Vector start = Vector(startPosition[0] + xOffset, startPosition[1] + yOffset);
        Vector target = start + Vector(relativeTarget[0], relativeTarget[1]);
        addAgent(start, target, Vector(0, 0), mass, radius, &agentGroup);
        ++mNumAgents;
      }
    }

    mAgentGroups.push_back(agentGroup);
  }

  auto colliders = data["colliders"];

  std::string boxKey = "box";

  for (Json::iterator it = colliders.begin(); it != colliders.end(); ++it) {
    auto collider = *it;

    float origin[2] = {collider["origin"][0].get<float>(), collider["origin"][1].get<float>()};
    if (collider["type"].get<std::string>() == boxKey) {
      float dims[3] = {
        collider["dimensions"][0].get<float>(), collider["dimensions"][1].get<float>(),
        collider["dimensions"][2].get<float>()
      };
      BoxCollider* box = new BoxCollider(Vector(origin[0], origin[1]),
                                         Vector3(dims[0], dims[1], dims[2]));
      mColliders.push_back(box);
    }
  }
}

std::vector<Vector> Scene::getAllPositions() const {
  std::vector<Vector> result;

  for (int i = 0; i < mAgentGroups.size(); i++) {

    auto group = mAgentGroups[i];

    for (int j = 0; j < group.mAgents.size(); j++) {
      result.push_back(group.mAgents[j].mCurrPosition);
    }
  }

  return result;
}

void Scene::addAgentGroups(std::vector<AgentGroup*> &mAllAgentGroups) {
	// TODO
}


void Scene::outputFrame(unsigned int frameId) {

#ifndef DISABLE_PARTIO
  std::string particleFile = "output/frame" + std::to_string(frameId) + ".bgeo";

  Partio::ParticlesDataMutable* parts = Partio::create();
  Partio::ParticleAttribute posH, vH, mH;

  mH   = parts->addAttribute("m", Partio::VECTOR, 1);
  posH = parts->addAttribute("position", Partio::VECTOR, 3);
  vH   = parts->addAttribute("v", Partio::VECTOR, 3);

  for (int i = 0; i < mAgentGroups.size(); i++) {

    auto group = mAgentGroups[i];

    for (int j = 0; j < group.mAgents.size(); j++) {

      int idx  = parts->addParticle();
      float* m = parts->dataWrite<float>(mH, idx);
      float* p = parts->dataWrite<float>(posH, idx);
      float* v = parts->dataWrite<float>(vH, idx);
      m[0]     = group.mAgents[j].mMass;

      p[0] = group.mAgents[j].mCurrPosition(0);
      p[1] = GROUND_Y_POS;
      p[2] = group.mAgents[j].mCurrPosition(1);

      v[0] = group.mAgents[j].mCurrVelocity(0);
      v[1] = 0;
      v[2] = group.mAgents[j].mCurrVelocity(1);
    }
  }

  Partio::write(particleFile.c_str(), *parts);
  parts->release();

#endif

}
