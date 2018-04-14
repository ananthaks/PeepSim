#pragma once

#include "Collider.h"

class BoxCollider : public Collider {

private:
  Vector3 mDimensions; // Length - x, Height - Y, Breadth - Z

public:

  BoxCollider(const Vector &origin, const Vector3 &size);

  ~BoxCollider();

  bool collide(const Vector& position, float radius) override;

  void fillCollisionSpace(Cell (&grid)[PATH_GRID_SIZE][PATH_GRID_SIZE], int width, int height) override;

};
