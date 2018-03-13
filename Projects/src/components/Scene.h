#pragma once
#include "Agents.h"

#include <string>

// Keep it as a Struct so
// it is just a data container for us.
struct Scene {
  using String = std::string;
  Scene();
  void loadFromFile(String filePath);

  Agents mAgents;
};
