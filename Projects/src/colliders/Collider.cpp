#include "Collider.h"

Collider::Collider(const Vector &origin) : mOrigin(origin), mVisible(true),
  mCollide(true), mTranslation(Vector::Zero(dim)), mScale(Vector::Zero(dim)), mRotation(Vector::Zero(dim)) {}

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
