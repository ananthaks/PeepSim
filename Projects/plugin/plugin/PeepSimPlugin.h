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

	protected:

		virtual void processObjectsSubclass(fpreal time, int foroutputidx, const SIM_ObjectArray &objects, DOP_Engine &engine);

		virtual void getInputInfoSubclass(int inputidx, DOP_InOutInfo &info) const;

		virtual void getOutputInfoSubclass(int inputidx, DOP_InOutInfo &info) const;

	private:
		Results mSimResults;
		Scene mScene;
		PeepSimConfig mConfig;
	};

	class AgentNode : public SOP_Node {

	public:

		static OP_Node*  myConstructor(OP_Network*, const char*, OP_Operator*);

		static PRM_Template  mParameterList[];

		static CH_LocalVariable  mLocalVariables[];

	public:

		AgentNode(OP_Network* net, const char* name, OP_Operator* op);

		virtual ~AgentNode();

		virtual unsigned disableParms();

		virtual OP_ERROR cookMySop(OP_Context& context);

		virtual bool evalVariableValue(fpreal& val, int index, int thread);

		virtual bool  evalVariableValue(UT_String& v, int i, int thread) {
			return evalVariableValue(v, i, thread);
		}

		void setNumAgents(int numAgents);

		void initialize(int numAgents);

		void addAgent(fpreal x, fpreal y, fpreal z);

		void update(int agentId, fpreal x, fpreal y, fpreal z);

		void update(fpreal timeStep);

		void updateResults(Results *results);

		void updateScene(Scene *scene);


	private:

		Results *mSimResults;

		Scene *mScene;

		// Vector of primitives
		std::vector<GEO_Primitive*> mAgents;

		// List of Arguments

		int mNumAgents;
		UT_String mJSONFilePath;
		UT_String mObjPath;

		int myCurrPoint;
		int myTotalPoints;

		exint NUM_AGENTS(fpreal n) {
			return evalInt("numAgents", 0, n);
		}

		void IMPORT_AGENTS(UT_String& label, fpreal t) {
			evalString(label, "filePath", 0, t);
		}

		void OBJ_FILE(UT_String& label, fpreal t) {
			evalString(label, "objFile", 0, t);
		}

	};

	/*class Colliders : public SOP_Node {

	};*/

}
