#pragma once

#include "Collider.h"

class BoxCollider : public Collider {

private:
  Vector mDimensions; // Length - x, Height - Y, Breadth - Z

public:

  BoxCollider(Vector &origin, Vector &size);

  ~BoxCollider();

  bool collide(Vector &currPos, Vector &nextPos) override;

};
