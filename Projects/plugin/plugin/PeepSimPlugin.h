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

#define TEST_BASE_PLUGIN

class GEO_PrimParticle;

namespace HDK_Sample {


	class PeepSource : public SOP_Node {

	};

	class PeepSimSolver :  public SIM_SingleSolver, public SIM_OptionsUser {

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

		virtual bool evalVariableValue(fpreal& val, int index,	int thread);

		virtual bool  evalVariableValue(UT_String& v, int i, int thread) {
			return evalVariableValue(v, i, thread);
		}

		void initialize(int numAgents);

		void addAgent(fpreal x, fpreal y, fpreal z);

		void update(int agentId, fpreal x, fpreal y, fpreal z);

	private:

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


	class PeepSim : public DOP_Node {

	public:

		static OP_Node* myConstructor(OP_Network*, const char*, OP_Operator*);

		static PRM_Template	myTemplateList[];

		PeepSim(OP_Network* net, const char* name, OP_Operator* op);

		virtual ~PeepSim();

	};

  class PeepSimSolver : public SOP_Node {

   public:

    static OP_Node*  myConstructor(OP_Network*, const char*,
                                     OP_Operator*);

    /// Stores the description of the interface of the SOP in Houdini.
    /// Each parm template refers to a parameter.
    static PRM_Template    mParameterList[];

    /// This optional data stores the list of local variables.
    static CH_LocalVariable  mLocalVariables[];

    static int generateCallback(void *data, int index, float time, const PRM_Template *);
    int startSimulation(OP_Context& context);

   protected:

	PeepSimSolver(OP_Network* net, const char* name, OP_Operator* op);

    virtual ~PeepSimSolver();

    /// Disable parameters according to other parameters.
    virtual unsigned disableParms();


    /// cookMySop does the actual work of the SOP computing
    virtual OP_ERROR cookMySop(OP_Context& context);

    /// This function is used to lookup local variables that you have
    /// defined specific to your SOP.
    virtual bool  evalVariableValue(
          fpreal& val,
          int index,
          int thread);

    // Add virtual overload that delegates to the super class to avoid
    // shadow warnings.
    virtual bool  evalVariableValue(
          UT_String& v,
          int i,
          int thread) {
      return evalVariableValue(v, i, thread);
    }

    void AddAgent(float x, float y, float z);

	void initCrowdSim();

	void update(fpreal frame);

   private:

	   std::vector<GEO_Primitive*> mAgents;
	   UT_String mFilePath;

	   GEO_PrimParticle	*mySystem;
	   fpreal		 myLastCookTime;	// Last cooked time

	   int   myCurrPoint;
	   int   myTotalPoints;


    /// The following list of accessors simplify evaluating the parameters
    /// of the SOP.

    bool initialized = false;
    bool calledFromCallback = false;

    exint NUM_AGENTS(fpreal n) {
      return evalInt("num_agents", 0, n);
    }

    void AGENTS_PATH(UT_String& label, fpreal t) {
      evalString(label, "filePath", 0, t);
    }

    fpreal  VELOCITYBLEND(fpreal t) {
      return evalFloat("velocityBlend", 0, t);
    }

    fpreal  MAXVELOCITY(fpreal t) {
      return evalFloat("maxVelocity", 0, t);
    }

    fpreal  DEFAULTAGENTMASS(fpreal t) {
      return evalFloat("defaultAgentMass", 0, t);
    }

    fpreal  DEFAULTAGENTRADIUS(fpreal t) {
      return evalFloat("defaultAgentRadius", 0, t);
    }

    int  STABILITYITERATIONS(fpreal t) {
      return evalInt("maxStabilityIterations", 0, t);
    }

    int  MAXITERATIONS(fpreal t) {
      return evalInt("maxIterations", 0, t);
    }

    int  COLLISIONSTEPS(fpreal t) {
      return evalInt("collisionSteps", 0, t);
    }
	
  };
} 
