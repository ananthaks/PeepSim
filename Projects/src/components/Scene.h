#pragma once
#include <string>
#include <fstream>
#include <streambuf>

#include "Agents.h"
#include "../colliders/Collider.h"
#include "../external/json.hpp"


// Keep it as a Struct so
// it is just a data container for us.
struct Scene {
  using String = std::string;
  using Json = nlohmann::json;

  const PeepSimConfig& mConfig;

  Scene(const PeepSimConfig& config);
  ~Scene();
  void loadFromFile(String filePath);

  Agents mAgents;
  std::vector<Collider*> mColliders;
};
