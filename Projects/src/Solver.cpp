#include "Solver.h"

Solver::Solver() :  mAgents(Agents(NUM_AGENTS)), mExplicitIntegrator(ExpIntegrator("explicit")),
  mFrictionalContraint(FrictionalConstraint()), mCollisionAvoidance(CollisionAvoidanceConstraint()) {}

void Solver::initialize() {

  // 1. Initialize each agent

  // 2. Initialize constraints

  // Testing
  for(int i = 0; i < 100000; ++i) {
    mAgents.addAgent(Vector::Zero(), Vector::Zero(), Vector::Zero());
  }

}

// Source : https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf
inline float W(Vector distance, float h) {
  float r = distance.norm();
  return (0.f <= r && r <= h) ? (POLY_6_KERNEL * (pow(h * h -  r * r, 3)) / (pow(h, 9))) : 0.f;
}

void Solver::solve(Scene scene) {
  // TODO: Use this scene when we have loaded scene via CrowdSim class

  int numIterations = FRAMES_PER_SECOND * SIMULATION_DURATION;

  for(int frame = 0; frame < numIterations; ++frame) {

    // Step 0: Write current frame to file
    mAgents.outputFrame(frame);

    // Step 1: Calculate Proposed positions
    for(int i = 0; i < mAgents.getNumAgents(); ++i) {
      Agent& agent = mAgents.getAgent(i);
      agent.mPlannerVelocity = agent.mTargetPosition - agent.mCurrPosition;
      agent.mBlendedVelocity = (1.f - VELOCITY_BLEND) * agent.mCurrVelocity + VELOCITY_BLEND * agent.mPlannerVelocity;
      agent.mProposedPosition = agent.mCurrPosition + TIME_STEP * agent.mBlendedVelocity;
    }

    // Step 2: Project Frictional Contact constraints

    // TODO: Optimization: We can compute neighouring Agents for each agent and
    // use them in the Inner Loop (int b loop).
    // This will speed up the process.

    for(int i = 0; i < MAX_STABILITY_ITERATION; ++i) {
      for(int a = 0; a < mAgents.getNumAgents(); ++a) {
        for(int b = a + 1; b < mAgents.getNumAgents(); ++b) {
            Agent& currAgent = mAgents.getAgent(a);
            Agent& nextAgent = mAgents.getAgent(b);
            VectorPair deltaPos = mFrictionalContraint.evaluate(currAgent, nextAgent);

            // TODO: As per algorithm & discussion, we have to update both
            // xi and xi* after the first FrictionalConstraint Check

            currAgent.mProposedPosition = currAgent.mCurrPosition + deltaPos.first;
            nextAgent.mProposedPosition = nextAgent.mCurrPosition + deltaPos.second;
        }
      }
    }

    // Step 3: Project FC, LRC, AM constraint
    for(int i = 0; i < MAX_ITERATION; ++i) {

      for(int a = 0; a < mAgents.getNumAgents(); ++a) {
        for(int b = a + 1; b < mAgents.getNumAgents(); ++b) {
            Agent& currAgent = mAgents.getAgent(a);
            Agent& nextAgent = mAgents.getAgent(b);
            VectorPair deltaPos = mFrictionalContraint.evaluate(currAgent, nextAgent);
            currAgent.mProposedPosition = currAgent.mCurrPosition + deltaPos.first;
            nextAgent.mProposedPosition = nextAgent.mCurrPosition + deltaPos.second;
        }
      }

      for(int a = 0; a < mAgents.getNumAgents(); ++a) {
        for(int b = a + 1; b < mAgents.getNumAgents(); ++b) {
            Agent& currAgent = mAgents.getAgent(a);
            Agent& nextAgent = mAgents.getAgent(b);
            VectorPair deltaPos = mCollisionAvoidance.evaluate(currAgent, nextAgent);
            currAgent.mProposedPosition = currAgent.mCurrPosition + deltaPos.first;
            nextAgent.mProposedPosition = nextAgent.mCurrPosition + deltaPos.second;
        }
      }
    }

    // Step 4: Update velocity and position
    for(int i = 0; i < mAgents.getNumAgents(); ++i) {
      Agent& agent = mAgents.getAgent(i);
      agent.mCurrVelocity = (agent.mProposedPosition - agent.mCurrPosition) / TIME_STEP;
      Vector viscosityVel = Vector::Zero();

      for(int j = 0; j < mAgents.getNumAgents(); ++j) {
        if(i == j){
          continue;
        }
        viscosityVel += (agent.mCurrVelocity - mAgents.getAgent(j).mCurrVelocity) * W(agent.mCurrPosition - mAgents.getAgent(j).mCurrPosition, VISCOSITY_H);
      }

      agent.mCurrVelocity += VISCOSITY_C * viscosityVel;
      agent.mCurrPosition = agent.mProposedPosition;
    }
  }
}
