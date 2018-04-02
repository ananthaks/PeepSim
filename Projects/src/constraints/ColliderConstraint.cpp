#include "ColliderConstraint.h"

ColliderConstraint::ColliderConstraint() : Constraint() {}

VectorPair ColliderConstraint::evaluate(Agent &x1, Agent &x2) {
}

Vector ColliderConstraint::evaluate(Scene &scene, Agent &agent) {
  // TODO : Iterate through colliders from scene and return the delta pos
  return Vector::Zero(dim);
}
