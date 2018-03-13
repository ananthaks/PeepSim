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

void Agents::addAgent(const Vector &startPos, const Vector &target, const Vector &plannedVelocity) {

  Agent agent;
  agent.mStartPosition = Vector(startPos);
  agent.mTargetPosition = Vector(target);
  agent.mCurrPosition = Vector(startPos);
  agent.mCurrVelocity = Vector::Zero(dim);
  agent.mForce = Vector::Zero(dim);

  mAgents.push_back(agent);
}


void Agents::outputFrame(unsigned int frameId) {

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
      m[0] = mMass;
      for (int k = 0; k < dim; k++)
          p[k] = mAgents[i].mCurrPosition(k, 0);
      for (int k = 0; k < dim; k++)
          v[k] = mAgents[i].mCurrVelocity(k, 0);
  }

  Partio::write(particleFile.c_str(), *parts);
  parts->release();

}
