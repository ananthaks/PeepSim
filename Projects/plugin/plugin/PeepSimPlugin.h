#pragma once

#include <SOP/SOP_Node.h>

#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>

#include <DOP/DOP_Node.h>

#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_Geometry.h>

#include "globalincludes.h"
#include "components\Agents.h"
#include "CrowdSim.h"

#define TEST_BASE_PLUGIN

class GEO_PrimParticle;

namespace HDK_Sample {

	class PeepSimSolver : public DOP_Node {

	private:
		bool hasCookedSop;

	public:

		static OP_Node* myConstructor(OP_Network*, const char*, OP_Operator*);

		static PRM_Template	myTemplateList[];

		PeepSimSolver(OP_Network* net, const char* name, OP_Operator* op);

		virtual ~PeepSimSolver();

		void updateScene(fpreal time);

		static int simulateScene(void* data, int index, float time, const PRM_Template*);

	protected:

		virtual void processObjectsSubclass(fpreal time, int foroutputidx, const SIM_ObjectArray &objects, DOP_Engine &engine);

		virtual void getInputInfoSubclass(int inputidx, DOP_InOutInfo &info) const;

		virtual void getOutputInfoSubclass(int inputidx, DOP_InOutInfo &info) const;

	private:
		Results mSimResults;
		Scene mScene;
		PeepSimConfig mConfig;

    fpreal  VELOCITYBLEND(fpreal t) { return evalFloat("velocityBlend", 0, t); }
    fpreal  MAXVELOCITY(fpreal t) { return evalFloat("maxVelocity", 0, t); }
    fpreal  STABILITYITERATIONS(fpreal t) { return evalInt("maxStabilityIterations", 0, t); }
    fpreal  MAXITERATIONS(fpreal t) { return evalInt("maxIterations", 0, t); }
    fpreal  COLLISIONSTEPS(fpreal t) { return evalInt("collisionSteps", 0, t); }
	};

	class AgentNode : public SOP_Node {

	public:

		static OP_Node*  myConstructor(OP_Network*, const char*, OP_Operator*);

		static PRM_Template  mParameterList[];

		static CH_LocalVariable  mLocalVariables[];

	public:

		bool mInitialized;

		AgentNode(OP_Network* net, const char* name, OP_Operator* op);

		virtual ~AgentNode();

		virtual unsigned disableParms();

		static int addAgentCallback(void* data, int index, float time, const PRM_Template*);

		virtual OP_ERROR cookMySop(OP_Context& context);

		virtual bool evalVariableValue(fpreal& val, int index, int thread);

		virtual bool  evalVariableValue(UT_String& v, int i, int thread) {
			return evalVariableValue(v, i, thread);
		}

		void setNumAgents(int numAgents);

		void initialize();

		GEO_Primitive* addAgent(fpreal x, fpreal y, fpreal z);

		void update(fpreal timeStep);

		void updateResults(Results *results);

		void updateScene(Scene *scene);

		const GU_Detail* fetchAgentObj(OP_Context& context);

		AgentGroup* getAgentGroup();

	private:

		Results *mSimResults;

		Scene *mScene;

		AgentGroup mAgentgroup;
		
		int mNumAgents;

		int myCurrPoint;
		int myTotalPoints;

		exint NUM_AGENTS(fpreal n) {
			return evalInt("numAgents", 0, n);
		}

		void TARGET_POS(fpreal t, UT_Vector2R &pos) {
			evalFloats("targetPos", pos.data(), t);
		}

		void SOURCE_POS(fpreal n, UT_Vector2R &pos) {
			evalFloats("sourcePos", pos.data(), n);
		}

		fpreal AGENT_MASS(fpreal n) {
			return evalFloat("agentMass", 0, n);
		}

		fpreal AGENT_RADIUS(fpreal n) {
			return evalFloat("agentMass", 0, n);
		}
	};

	/*class Colliders : public SOP_Node {

	};*/

}
