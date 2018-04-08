#include "Agents.h"

Agents::Agents(int numAgents) : mNumAgents(numAgents), mMass(1.f) {
  //mAgents.resize(numAgents);
}

void Agents::setMass(float mass) {
  mMass = mass;
}

int Agents::getNumAgents() const {
  return mAgents.size();
}

Agent& Agents::getAgent(int index) {
  return mAgents[index];
}

void Agents::addAgent(const Vector &startPos, const Vector &target, const Vector &plannedVelocity, float mass, float radius) {

  Agent agent;
  agent.mStartPosition = Vector(startPos);
  agent.mTargetPosition = Vector(target);
  agent.mCurrPosition = Vector(startPos);
  agent.mCurrVelocity = Vector(0, 0);
  agent.mForce = Vector(0, 0);
  agent.mMass = mass;
  agent.mRadius = radius;

  mAgents.push_back(agent);
}


void Agents::outputFrame(unsigned int frameId) {

#ifndef DISABLE_PARTIO

  std::string particleFile = "output/frame" + std::to_string(frameId) + ".bgeo";

  Partio::ParticlesDataMutable* parts = Partio::create();
  Partio::ParticleAttribute posH, vH, mH;

  mH = parts->addAttribute("m", Partio::VECTOR, 1);
  posH = parts->addAttribute("position", Partio::VECTOR, 3);
  vH = parts->addAttribute("v", Partio::VECTOR, 3);

  for (int i=0; i < mAgents.size(); i++){
      int idx = parts->addParticle();
      float* m = parts->dataWrite<float>(mH, idx);
      float* p = parts->dataWrite<float>(posH, idx);
      float* v = parts->dataWrite<float>(vH, idx);
      m[0] = mAgents[i].mMass;

      p[0] = mAgents[i].mCurrPosition(0);
      p[1] = GROUND_Y_POS;
      p[2] = mAgents[i].mCurrPosition(1);

      v[0] = mAgents[i].mCurrVelocity(0);
      v[1] = 0;
      v[2] = mAgents[i].mCurrVelocity(1);

  }

  Partio::write(particleFile.c_str(), *parts);
  parts->release();

#endif

}

std::vector<Vector> Agents::getAllPositions() {
    std::vector<Vector> result;
    for (int i=0; i < mAgents.size(); i++){
        result.push_back(mAgents[i].mCurrPosition);
    }
    return result;
}
