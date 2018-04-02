#include "BoxCollider.h"

BoxCollider::BoxCollider(Vector &origin, Vector &size) : Collider(origin), mDimensions(size) {}

BoxCollider::~BoxCollider() {}

bool BoxCollider::collide(Vector &currPos, Vector &nextPos) {
  return false;
}
