#include "ColliderConstraint.h"

ColliderConstraint::ColliderConstraint() : Constraint() {}

VectorPair ColliderConstraint::evaluate(Agent &x1, Agent &x2) {
}

Vector ColliderConstraint::evaluate(Scene &scene, Agent &agent) {
  Vector delta = Vector::Zero();

  for(auto& collider: scene.mColliders) {
    // TODO: take Agent Radius
    bool result = collider->collide(agent.mProposedPosition, AGENT_RADIUS);

    if (result) {
      float steps = COLLISION_MARCH_STEPS;
      Vector lengthVec = agent.mCurrPosition - agent.mProposedPosition;
      Vector direction = lengthVec.normalized();

      Vector validPosition = agent.mCurrPosition;
      bool foundCollision = false;

      Vector itrDelta = direction * (lengthVec.norm() / steps);

      // std::cout << "Evalutaing: " << std::endl;
      // std::cout << "Start:" << agent.mCurrPosition[0] << std::endl;
      // std::cout << "End:" << agent.mProposedPosition[0]  << std::endl;
      // std::cout << "Delta:" << itrDelta[0]  << std::endl;

      Vector itrVec = agent.mCurrPosition;

      for(int itr = 0; itr < steps; ++itr) {
        bool marchResult = collider->collide(itrVec, AGENT_RADIUS);

        // std::cout << "Marching " << itr << std::endl;
        if (marchResult) {
          foundCollision = true;
          break;
        }

        validPosition = itrVec;
        itrVec += itrDelta;
      }

      if (foundCollision) {
        delta = validPosition - agent.mProposedPosition;
        // std::cout << "Found Delta: " << delta[0] << std::endl;
      }

      break;
    }
  }

  return delta;
}
