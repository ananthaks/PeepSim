#include "BoxCollider.h"

BoxCollider::BoxCollider(const Vector &origin, const Vector3 &size) : Collider(origin), mDimensions(size) {}

BoxCollider::~BoxCollider() {}

bool BoxCollider::collide(const Vector& position, float radius) {
  float lowX = mOrigin[0];
  float lowY = mOrigin[1];

  float highX = lowX + mDimensions[0];
  float highY = lowY + mDimensions[1];

  bool collideX = false;
  bool collideY = false;

  float centerX = position[0];
  float centerY = position[1];

  lowX -= radius;
  lowY -= radius;

  highX += radius;
  highY += radius;

  if (lowX < centerX && centerX < highX) {
    if (lowY < centerY && centerY < highY) {
      return true;
    }
  }

  return false;
}

void BoxCollider::fillCollisionSpace(Cell (&grid)[PATH_GRID_SIZE][PATH_GRID_SIZE], int width, int height) {

  float lowX = mOrigin[0] + PATH_GRID_SIZE / 2.f;
  float lowY = mOrigin[1] + PATH_GRID_SIZE / 2.f;

  float highX = lowX + mDimensions[0];
  float highY = lowY + mDimensions[1];

  for(int i = lowX; i >=0 && i <= highX && i < PATH_GRID_SIZE; ++i) {
    for(int j = lowY; j >=0 && j <= highY && j < PATH_GRID_SIZE; ++j) {
      grid[i][j].isBlocked = true;
    }
  }
}
