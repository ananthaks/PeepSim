#pragma once
#include "../globalincludes.h"

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

  virtual bool collide(Vector &currPos, Vector &nextPos) = 0;

  bool isVisible() const;

  void setVisible(bool visible);

  bool isCollisionOn() const;

  void setCollisionOn(bool collide);

  void translate(Vector &translate);

  void scale(Vector &scale);

  void rotate(Vector &rotate);

};
