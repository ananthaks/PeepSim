#pragma once
#include "../globalincludes.h"
#include "../components/Agents.h"

class Collider {

protected:
  bool mVisible;
  bool mCollide;

  Vector mOrigin;

  Vector mTranslation;
  Vector mRotation;
  Vector mScale;

public:

  Collider(const Vector &origin);

  ~Collider();

  virtual bool collide(const Vector& position, float radius) = 0;

  bool isVisible() const;

  void setVisible(bool visible);

  bool isCollisionOn() const;

  void setCollisionOn(bool collide);

  void translate(Vector &translate);

  void scale(Vector &scale);

  void rotate(Vector &rotate);

};
