#include "Agents.h"

Agents::Agents(int numAgents) : mNumAgents(numAgents), mMass(1.f) {
  mAgents.resize(numAgents);
}

void Agents::setMass(float mass) {
  mMass = mass;
}

int Agents::getNumAgents() const {
  return mNumAgents;
}

Agent<T, dim>& Agents::getAgent(unsigned int index) {
  return mAgents[index];
}

void Agents::addAgent(const Matrix<T, dim, 1> &startPos, const Matrix<T, dim, 1> &target, const Matrix<T, dim, 1> &plannedVelocity) {

  Agent<T, dim> agent;
  agent.mStartPosition = Matrix<T, dim, 1>(startPos);
  agent.mTargetPosition = Matrix<T, dim, 1>(target);
  agent.mCurrPosition = Matrix<T, dim, 1>(startPos);
  agent.mCurrVelocity = Matrix<T, dim, 1>::Zero(dim);
  agent.mForce = Matrix<T, dim, 1>::Zero(dim);

  mAgents.push_back(agent);
}


void Agents::outputFrame(unsigned int frameId) {

  std::string particleFile = "output/frame" + std::to_string(frameId) + ".bgeo";

  Partio::ParticlesDataMutable* parts = Partio::create();
  Partio::ParticleAttribute posH, vH, mH;

  mH = parts->addAttribute("m", Partio::VECTOR, 1);
  posH = parts->addAttribute("position", Partio::VECTOR, 3);
  vH = parts->addAttribute("v", Partio::VECTOR, 3);

  for (int i=0; i < mNumAgents; i++){
      int idx = parts->addParticle();
      float* m = parts->dataWrite<float>(mH, idx);
      float* p = parts->dataWrite<float>(posH, idx);
      float* v = parts->dataWrite<float>(vH, idx);
      m[0] = mMass;
      for (int k = 0; k < 3; k++)
          p[k] = mAgents[i].mCurrPosition[k];
      for (int k = 0; k < 3; k++)
          v[k] = mAgents[i].mCurrVelocity[k];
  }

  Partio::write(particleFile.c_str(), *parts);
  parts->release();

}
