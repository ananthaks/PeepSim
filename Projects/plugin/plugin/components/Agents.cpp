#include "Agents.h"
//
//Agents::Agents(int numAgents) : mNumAgents(numAgents), mMass(1.f) {
//  //mAgentGroups.resize(numAgents);
//}
//
//void Agents::setMass(float mass) {
//  mMass = mass;
//}
//
//int Agents::getNumAgents() const {
//  return mAgents.size();
//}
//
//Agent& Agents::getAgent(int index) {
//  return mAgents[index];
//}
//
//void Agents::addAgent(const Vector &startPos, const Vector &target, const Vector &plannedVelocity, float mass, float radius) {
//
//  Agent agent;
//  agent.mStartPosition = Vector(startPos);
//  agent.mTargetPosition = Vector(target);
//  agent.mCurrPosition = Vector(startPos);
//  agent.mCurrVelocity = Vector(0, 0);
//  agent.mForce = Vector(0, 0);
//  agent.mMass = mass;
//  agent.mRadius = radius;
//
//  mAgents.push_back(agent);
//}
//
//
//
//std::vector<Vector> Agents::getAllPositions() {
//    std::vector<Vector> result;
//    for (int i=0; i < mAgents.size(); i++){
//        result.push_back(mAgents[i].mCurrPosition);
//    }
//    return result;
//}
