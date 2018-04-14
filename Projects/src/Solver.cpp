#include "Solver.h"

#define PATH_FINDER_ON


Solver::Solver(const PeepSimConfig& config)
  : mExplicitIntegrator(ExpIntegrator("explicit")),
    mFrictionalContraint(FrictionalConstraint(config)),
    mCollisionAvoidance(CollisionAvoidanceConstraint(config)),
    mColliderConstraint(ColliderConstraint(config)),
    mPathFinder(AStarFinder(PATH_GRID_SIZE, PATH_GRID_SIZE, config)),
    mConfig(config) {}

void Solver::initialize() {
  // Initialize Path finding
}

// Source : https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf
inline float W(Vector distance, float h, float kernel) {
  float r = distance.Length();
  return (0.f <= r && r <= h) ? (kernel * (pow(h * h - r * r, 3)) / (pow(h, 9))) : 0.f;
}

Results Solver::solve(Scene& scene) {

  using SizeType = std::size_t;

  Results results = Results();

  std::vector<Agent*> mAgents;
  mAgents.resize(scene.mNumAgents);

  int count = 0;

  for (SizeType groupIdx = 0; groupIdx < scene.mAgentGroups.size(); groupIdx++) {
    auto group = scene.mAgentGroups[groupIdx];

    for (SizeType agentIdx = 0; agentIdx < group.mAgents.size(); agentIdx++) {
      mAgents[count] = &(scene.mAgentGroups[groupIdx].mAgents[agentIdx]);
      ++count;
    }
  }

#ifdef PATH_FINDER_ON
  mPathFinder.initialize(scene);
  for (int i = 0; i < mAgents.size(); ++i) {

    Agent* agent = mAgents[i];
    std::vector<Vector> resultPath;
    bool result = mPathFinder.getPathToTarget(agent->mCurrPosition, agent->mTargetPosition,
                                              resultPath);
    agent->mPlannedPath = resultPath;
    agent->currTarget = 0;
  }
#endif

  const int numIterations = mConfig.mFPS * mConfig.mSimualtionDuration;

  results.mPositions.reserve(numIterations);

  for (int frame = 0; frame < numIterations; ++frame) {

    // For Debug:
    // std::cout << "Processing Frame: " << frame << std::endl;

    // Step 0: Write current frame to file
    scene.outputFrame(frame);

    // Step 1: Calculate Proposed positions
    for (SizeType i = 0; i < mAgents.size(); ++i) {
      Agent* agent = mAgents[i];

#ifdef PATH_FINDER_ON
      Vector target = agent->mPlannedPath[agent->mPlannedPath.size() - agent->currTarget - 1];
      Vector dist = target - agent->mCurrPosition;

      if (dist.Length() < mConfig.mMinDistanceToTarget) {
        agent->currTarget = agent->currTarget + 1;
      }

      if (agent->currTarget < agent->mPlannedPath.size()) {
        target = agent->mPlannedPath[agent->mPlannedPath.size() - agent->currTarget - 1];
      } else {
        target = agent->mTargetPosition;
      }
      agent->mPlannerVelocity = target - agent->mCurrPosition;


#else
      agent->mPlannerVelocity = agent->mTargetPosition - agent->mCurrPosition;

#endif
      agent->mBlendedVelocity = (1.f - mConfig.mVelocityBlend) * agent->mCurrVelocity + mConfig.
                                mVelocityBlend * agent->mPlannerVelocity;
      agent->mProposedPosition = agent->mCurrPosition + mConfig.mTimeStep * agent->mBlendedVelocity;
    }
    // Step 2: Project Frictional Contact constraints

    // TODO: Optimization: We can compute neighouring Agents for each agent and
    // use them in the Inner Loop (int b loop).
    // This will speed up the process.

    for (int i = 0; i < mConfig.mMaxStabilityIterations; ++i) {

      for (SizeType a = 0; a < mAgents.size(); ++a) {
        for (SizeType b = a + 1; b < mAgents.size(); ++b) {
          Agent* currAgent = mAgents[a];
          Agent* nextAgent = mAgents[b];
          VectorPair deltaPos = mFrictionalContraint.evaluate(*currAgent, *nextAgent);

          currAgent->mProposedPosition = currAgent->mProposedPosition + deltaPos.first;
          nextAgent->mProposedPosition = nextAgent->mProposedPosition + deltaPos.second;

          currAgent->mCurrPosition = currAgent->mCurrPosition + deltaPos.first;
          nextAgent->mCurrPosition = nextAgent->mCurrPosition + deltaPos.second;
        }
      }
    }

    // Step 3: Project FC, LRC, AM constraint
    for (int i = 0; i < mConfig.mMaxIterations; ++i) {

      for (SizeType a = 0; a < mAgents.size(); ++a) {
        for (SizeType b = a + 1; b < mAgents.size(); ++b) {
          Agent* currAgent = mAgents[a];
          Agent* nextAgent = mAgents[b];
          VectorPair deltaPos = mFrictionalContraint.evaluate(*currAgent, *nextAgent);
          currAgent->mProposedPosition = currAgent->mProposedPosition + deltaPos.first;
          nextAgent->mProposedPosition = nextAgent->mProposedPosition + deltaPos.second;
        }
      }

      for (SizeType a = 0; a < mAgents.size(); ++a) {
        for (SizeType b = a + 1; b < mAgents.size(); ++b) {
          Agent* currAgent = mAgents[a];
          Agent* nextAgent = mAgents[b];
          VectorPair deltaPos = mCollisionAvoidance.evaluate(*currAgent, *nextAgent);
          currAgent->mProposedPosition = currAgent->mProposedPosition + deltaPos.first;
          nextAgent->mProposedPosition = nextAgent->mProposedPosition + deltaPos.second;
        }
      }

      for (SizeType a = 0; a < mAgents.size(); ++a) {
        Agent* currAgent = mAgents[a];
        Vector deltaPos = mColliderConstraint.evaluate(scene, *currAgent);
        currAgent->mProposedPosition = currAgent->mProposedPosition + deltaPos;
      }
    }

    // Step 4: Update velocity and position

    std::vector<Vector> viscosityVels;

    for (SizeType i = 0; i < mAgents.size(); ++i) {
      Agent* agent = mAgents[i];
      agent->mCurrVelocity = (agent->mProposedPosition - agent->mCurrPosition) / mConfig.mTimeStep;
    }

    // Groups of Agents
    for (SizeType gIdx = 0; gIdx < scene.mAgentGroups.size(); gIdx++) {
      auto& group = scene.mAgentGroups[gIdx];

      // For each Group: Agents i & j get cohesion
      for (SizeType i = 0; i < group.mAgents.size(); ++i) {
        Agent& agent = scene.mAgentGroups[gIdx].mAgents[i];
        Vector viscosityVel = Vector(0, 0);

        for (SizeType j = 0; j < group.mAgents.size(); ++j) {
          if (i == j) {
            continue;
          }

          viscosityVel += (agent.mCurrVelocity - mAgents[j]->mCurrVelocity) *
            W(
              agent.mCurrPosition - mAgents[j]->mCurrPosition,
              mConfig.mViscosityH,
              mConfig.mPoly6Kernel
            );
        }

        viscosityVels.push_back(viscosityVel);
      }
    }

    for (SizeType i = 0; i < mAgents.size(); ++i) {
      Agent* agent = mAgents[i];

      //agent.mCurrVelocity += VISCOSITY_C * viscosityVels[i];
      float speed = agent->mCurrVelocity.Length();
      if (speed > mConfig.mMaxVelocity) {
        agent->mCurrVelocity *= (mConfig.mMaxVelocity / speed);
      }
      agent->mCurrPosition = agent->mProposedPosition;
    }

    results.mPositions.push_back(scene.getAllPositions());
  }

  return results;
}
