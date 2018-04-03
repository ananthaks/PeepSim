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

void BoxCollider::fillCollisionSpace(MatrixXi &grid, int width, int height) {

}
