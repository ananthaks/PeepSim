#include "Scene.h"
#include "../colliders/BoxCollider.h"

Scene::Scene() : mAgents(Agents(NUM_AGENTS)) {}

Scene::~Scene() {
  for(auto& collider: mColliders) {
    delete collider;
  }
}

void Scene::loadFromFile(String filePath)
{
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

  for (Json::iterator it = agentGroups.begin(); it != agentGroups.end(); ++it)
  {
    /* Structure:
     *
     * "size": [1, 10],
     * "spacing": [0, 1],
     * "startPosition": [-5, 0],
     * "relativeTarget": [10, 0]
     */
    auto group = *it;
    int size[2] = { group["size"][0].get<int>(), group["size"][1].get<int>() };
    float spacing[2] = { group["spacing"][0].get<float>(), group["spacing"][1].get<float>() };
    float startPosition[2] = { group["startPosition"][0].get<float>(), group["startPosition"][1].get<float>() };
    float relativeTarget[2] = { group["relativeTarget"][0].get<float>(), group["relativeTarget"][1].get<float>() };

    for (int xSize = 0; xSize < size[0]; xSize++) {
      for (int ySize = 0; ySize < size[1]; ySize++) {

        float xOffset = xSize * spacing[0];
        float yOffset = ySize * spacing[1];

        Vector start = Vector(startPosition[0] + xOffset, startPosition[1] + yOffset);
        Vector target = start + Vector(relativeTarget[0], relativeTarget[1]);
        mAgents.addAgent(start, target, Vector::Zero());
      }
    }

    auto colliders = data["colliders"];

    std::string boxKey = "box";

    for (Json::iterator it = colliders.begin(); it != colliders.end(); ++it) {
      auto collider = *it;

      float origin[2] = { collider["origin"][0].get<float>(), collider["origin"][1].get<float>() };
      if (collider["type"].get<std::string>() == boxKey) {
        float dims[3] = { collider["dimensions"][0].get<float>(), collider["dimensions"][1].get<float>(), collider["dimensions"][2].get<float>() };
        BoxCollider* box = new BoxCollider(Vector(origin[0], origin[1]), Vector3(dims[0], dims[1], dims[2]));
        mColliders.push_back(box);
      }
    }
  }
}
