#pragma once

#include <SOP/SOP_Node.h>

namespace HDK_Sample {

  class PeepSimPlugin : public SOP_Node {

   public:
    static OP_Node*    myConstructor(OP_Network*, const char*,
                                     OP_Operator*);

    /// Stores the description of the interface of the SOP in Houdini.
    /// Each parm template refers to a parameter.
    static PRM_Template    mParameterList[];

    /// This optional data stores the list of local variables.
    static CH_LocalVariable  mLocalVariables[];

    static int generateCallback(void *data, int index, float time, const PRM_Template *);
    int startSimulation(OP_Context& context);

   protected:

    PeepSimPlugin(OP_Network* net, const char* name, OP_Operator* op);

    virtual ~PeepSimPlugin();

    /// Disable parameters according to other parameters.
    virtual unsigned disableParms();


    /// cookMySop does the actual work of the SOP computing
    virtual OP_ERROR cookMySop(OP_Context& context);

    /// This function is used to lookup local variables that you have
    /// defined specific to your SOP.
    virtual bool     evalVariableValue(
          fpreal& val,
          int index,
          int thread);

    // Add virtual overload that delegates to the super class to avoid
    // shadow warnings.
    virtual bool     evalVariableValue(
          UT_String& v,
          int i,
          int thread) {
      return evalVariableValue(v, i, thread);
    }

    void AddAgent(float x, float y, float z);

   private:
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    /// Member variables are stored in the actual SOP, not with the geometry
    /// In this case these are just used to transfer data to the local
    /// variable callback.
    /// Another use for local data is a cache to store expensive calculations.

    // NOTE : You can declare local variables here like this
    int   myCurrPoint;
    int   myTotalPoints;
  };
} // End HDK_Sample namespace
