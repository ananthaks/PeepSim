#include "Collider.h"

Collider::Collider(const Vector &origin) : mOrigin(origin), mVisible(true),
  mCollide(true), mTranslation(Vector(0, 0)), mScale(Vector(0, 0)), mRotation(Vector(0, 0)) {}

Collider::~Collider() {}

bool Collider::isVisible() const {
  return mVisible;
}

void Collider::setVisible(bool visible) {
  mVisible = visible;
}

bool Collider::isCollisionOn() const {
  return mCollide;
}

void Collider::setCollisionOn(bool collide) {
  mCollide = collide;
}

void Collider::translate(Vector &translate) {

}

void Collider::scale(Vector &scale) {

}

void Collider::rotate(Vector &rotate) {

}
