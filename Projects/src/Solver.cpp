#include "Solver.h"


Solver::Solver() :  mAgents(Agents(NUM_AGENTS)), mExplicitIntegrator(ExpIntegrator("explicit")) {}

void Solver::initialize() {

  // 1. Initialize each agent

  // 2. Initialize constraints

}


void Solver::solve() {

  int numIterations = FRAMES_PER_SECOND * SIMULATION_DURATION;

  for(int frame = 0; frame < numIterations; ++frame) {

    // Step 0: Write current frame to file
    mAgents.outputFrame(frame);

    // Step 1: Calculate Proposed positions
    for(int i = 0; i < mAgents.getNumAgents(); ++i) {

      Agent<T, dim> &agent = mAgents.getAgent(i);

      Matrix<T, dim, 1> velocityBlend = (1.f - VELOCITY_BLEND) * agent.mCurrVelocity + VELOCITY_BLEND * agent.mPlannerVelocity;
      agent.mProposedPosition = agent.mCurrPosition + TIME_STEP * velocityBlend;
    }

    // Step 2: Calculate nearby agents

    // Step 3: Project constraints

    // Step 4: Correct positions for collision

    // Step 5: Damp velocities

  }

}
