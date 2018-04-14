#pragma once
#include <string>
#include <fstream>
#include <streambuf>

#include "Agents.h"
#include "../colliders/Collider.h"
#include "../external/json.hpp"

using SizeType = std::size_t;

// Keep it as a Struct so
// it is just a data container for us.
struct Scene {
  using String = std::string;
  using Json = nlohmann::json;

  const PeepSimConfig& mConfig;

  Scene(const PeepSimConfig& config);
  ~Scene();

  void loadFromFile(String filePath);

  void addAgent(const Vector& startPos, const Vector& target, const Vector& plannedVelocity,
                float mass, float radius, AgentGroup* group);

  AgentGroup& getAgentGroup(int index);
  std::vector<Vector> getAllPositions() const;
  void outputFrame(unsigned int frameId);

  int mNumAgents{0};

  std::vector<AgentGroup> mAgentGroups;
  std::vector<Collider*> mColliders;
  SizeType getNumAgents() const;


};
