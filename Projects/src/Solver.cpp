#include "Solver.h"

#define PATH_FINDER_ON


Solver::Solver(const PeepSimConfig& config) :  mExplicitIntegrator(ExpIntegrator("explicit")),
  mFrictionalContraint(FrictionalConstraint(config)), mCollisionAvoidance(CollisionAvoidanceConstraint(config)),
  mColliderConstraint(ColliderConstraint(config)), mPathFinder(AStarFinder(PATH_GRID_SIZE, PATH_GRID_SIZE, config)), mConfig(config) {}

void Solver::initialize()
{
  // Initialize Path finding
}

// Source : https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf
inline float W(Vector distance, float h, float kernel)
{
  float r = distance.Length();
  return (0.f <= r && r <= h) ? (kernel * (pow(h * h -  r * r, 3)) / (pow(h, 9))) : 0.f;
}

void Solver::solve(Scene &scene)
{

  Agents mAgents = scene.mAgents;

#ifdef PATH_FINDER_ON
  mPathFinder.initialize(scene);
  for (int i = 0; i < mAgents.getNumAgents(); ++i)
  {
    Agent& agent = mAgents.getAgent(i);
    std::vector<Vector> resultPath;
    bool result = mPathFinder.getPathToTarget(agent.mCurrPosition, agent.mTargetPosition, resultPath);
    agent.mPlannedPath = resultPath;
    agent.currTarget = 0;
  }
#endif

  int numIterations = mConfig.mFPS * mConfig.mSimualtionDuration;

  for (int frame = 0; frame < numIterations; ++frame)
  {

    // For Debug:
    // std::cout << "Processing Frame: " << frame << std::endl;

    // Step 0: Write current frame to file
    mAgents.outputFrame(frame);

    // Step 1: Calculate Proposed positions
    for (int i = 0; i < mAgents.getNumAgents(); ++i)
    {
      Agent& agent = mAgents.getAgent(i);

#ifdef PATH_FINDER_ON
      Vector target = agent.mPlannedPath[agent.mPlannedPath.size() - agent.currTarget - 1];
      Vector dist = target - agent.mCurrPosition;

      if (dist.Length() < mConfig.mMinDistanceToTarget)
      {
        agent.currTarget = agent.currTarget + 1;
      }

      if (agent.currTarget < agent.mPlannedPath.size())
      {
        target = agent.mPlannedPath[agent.mPlannedPath.size() - agent.currTarget - 1];
      }
      else
      {
        target = agent.mTargetPosition;
      }
      agent.mPlannerVelocity = target - agent.mCurrPosition;


#else
      agent.mPlannerVelocity = agent.mTargetPosition - agent.mCurrPosition;

#endif
      agent.mBlendedVelocity = (1.f - mConfig.mVelocityBlend) * agent.mCurrVelocity + mConfig.mVelocityBlend * agent.mPlannerVelocity;
      agent.mProposedPosition = agent.mCurrPosition + mConfig.mTimeStep * agent.mBlendedVelocity;
    }
    // Step 2: Project Frictional Contact constraints

    // TODO: Optimization: We can compute neighouring Agents for each agent and
    // use them in the Inner Loop (int b loop).
    // This will speed up the process.

    for (int i = 0; i < mConfig.mMaxStabilityIterations; ++i)
    {

      for (int a = 0; a < mAgents.getNumAgents(); ++a)
      {
        for (int b = a + 1; b < mAgents.getNumAgents(); ++b)
        {
          Agent& currAgent = mAgents.getAgent(a);
          Agent& nextAgent = mAgents.getAgent(b);
          VectorPair deltaPos = mFrictionalContraint.evaluate(currAgent, nextAgent);

          currAgent.mProposedPosition = currAgent.mProposedPosition + deltaPos.first;
          nextAgent.mProposedPosition = nextAgent.mProposedPosition + deltaPos.second;

          currAgent.mCurrPosition = currAgent.mCurrPosition + deltaPos.first;
          nextAgent.mCurrPosition = nextAgent.mCurrPosition + deltaPos.second;
        }
      }

    }

    // Step 3: Project FC, LRC, AM constraint
    for (int i = 0; i < mConfig.mMaxIterations; ++i)
    {

      for (int a = 0; a < mAgents.getNumAgents(); ++a)
      {
        for (int b = a + 1; b < mAgents.getNumAgents(); ++b)
        {
          Agent& currAgent = mAgents.getAgent(a);
          Agent& nextAgent = mAgents.getAgent(b);
          VectorPair deltaPos = mFrictionalContraint.evaluate(currAgent, nextAgent);
          currAgent.mProposedPosition = currAgent.mProposedPosition + deltaPos.first;
          nextAgent.mProposedPosition = nextAgent.mProposedPosition + deltaPos.second;
        }
      }

      for (int a = 0; a < mAgents.getNumAgents(); ++a)
      {
        for (int b = a + 1; b < mAgents.getNumAgents(); ++b)
        {
          Agent& currAgent = mAgents.getAgent(a);
          Agent& nextAgent = mAgents.getAgent(b);
          VectorPair deltaPos = mCollisionAvoidance.evaluate(currAgent, nextAgent);
          currAgent.mProposedPosition = currAgent.mProposedPosition + deltaPos.first;
          nextAgent.mProposedPosition = nextAgent.mProposedPosition + deltaPos.second;
        }
      }

      for (int a = 0; a < mAgents.getNumAgents(); ++a)
      {
        Agent& currAgent = mAgents.getAgent(a);
        Vector deltaPos = mColliderConstraint.evaluate(scene, currAgent);
        currAgent.mProposedPosition = currAgent.mProposedPosition + deltaPos;
      }
    }

    // Step 4: Update velocity and position

    std::vector<Vector> viscosityVels;

    for (int i = 0; i < mAgents.getNumAgents(); ++i)
    {
      Agent& agent = mAgents.getAgent(i);
      agent.mCurrVelocity = (agent.mProposedPosition - agent.mCurrPosition) / mConfig.mTimeStep;
    }

    for (int i = 0; i < mAgents.getNumAgents(); ++i)
    {
      Agent& agent = mAgents.getAgent(i);
      Vector viscosityVel = Vector(0, 0);
      for (int j = 0; j < mAgents.getNumAgents(); ++j)
      {
        if (i == j)
        {
          continue;
        }
        viscosityVel += (agent.mCurrVelocity - mAgents.getAgent(j).mCurrVelocity) *
                        W(agent.mCurrPosition - mAgents.getAgent(j).mCurrPosition, mConfig.mViscosityH, mConfig.mPoly6Kernel);
      }
      viscosityVels.push_back(viscosityVel);
    }

    for (int i = 0; i < mAgents.getNumAgents(); ++i)
    {
      Agent& agent = mAgents.getAgent(i);

      //agent.mCurrVelocity += VISCOSITY_C * viscosityVels[i];
      float speed = agent.mCurrVelocity.Length();
      if (speed > mConfig.mMaxVelocity)
      {
        agent.mCurrVelocity *= (mConfig.mMaxVelocity / speed);
      }
      agent.mCurrPosition = agent.mProposedPosition;
    }


  }
}
