#include <UT/UT_DSOVersion.h>

#include <UT/UT_Math.h>
#include <UT/UT_Interrupt.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <GU/GU_PrimSphere.h>
#include <CH/CH_LocalVariable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>

#include <DOP/DOP_PRMShared.h>
#include <DOP/DOP_InOutInfo.h>
#include <DOP/DOP_Operator.h>
#include <DOP/DOP_Engine.h>

#include <limits.h>
#include "PeepSimPlugin.h"


static float numAgent = 10.0;


// #include "../../src/CrowdSim.h"

using namespace HDK_Sample;

/*
* Register the SOP node
*/
void newSopOperator(OP_OperatorTable* table) {

  table->addOperator(
        new OP_Operator("CrowdSim", // Internal name
                        "PeepSimSolver", // UI name
                        PeepSimSolver::myConstructor, // How to build the SOP
                        PeepSimSolver::mParameterList, // My parameters
                        0, // Min # of sources
                        0, // Max # of sources
                        PeepSimSolver::mLocalVariables, // Local variables
                        OP_FLAG_GENERATOR) // Flag it as generator
  );
}

/*
* Register the PeepSim DOP node
*/

void newDopOperator(OP_OperatorTable *table)
{
	OP_Operator	*op;

	op = new DOP_Operator("PluginBase", "PeepSim",
		PeepSim::myConstructor,
		PeepSim::myTemplateList, 0, 9999, 0,
		0, 1);
	table->addOperator(op);
}

/*
* Defining a PeepSim DOP node Constructor for Houdini to create a plugin
*/
OP_Node* PeepSim::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
	return new PeepSim(net, name, op);
}

static PRM_Name	theInputIndexName("inputindex", "Input Index");

/*
* Passing Possible parameter lists for PeepSim DOP node
*/
PRM_Template PeepSim::myTemplateList[] = {
	
	// Standard activation parameter.
	PRM_Template(PRM_INT_J,	1, &DOPactivationName, &DOPactivationDefault),
	
	// Standard group parameter with group menu.
	PRM_Template(PRM_STRING, 1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu),

	// The input index that determines which data will be attached to
	// each object.
	PRM_Template(PRM_INT_J,	1, &theInputIndexName, PRMzeroDefaults),

	PRM_Template()
};

PeepSim::PeepSim(OP_Network *net, const char *name, OP_Operator *op) : DOP_Node(net, name, op)
{}

PeepSim::~PeepSim() 
{}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static PRM_Name filePath("filePath", "File Path");
static PRM_Name velocityBlendName("velocityBlend", "Velocity Blend");
static PRM_Name maxVelocityName("maxVelocity", "Max Velocity");
static PRM_Name maxStabilityIterationsName("maxStabilityIterations", "Max Stability Iterations");
static PRM_Name maxIterationsName("maxIterations", "Max Iterations");
static PRM_Name defaultAgentMassName("defaultAgentMass", "Default Agent Mass");
static PRM_Name defaultAgentRadiusName("defaultAgentRadius", "Default Agent Radius");
static PRM_Name collisionMarchingStepsName("collisionSteps", "Max Env Collision Steps");
static PRM_Name generateCommandName("generateCommand", "Generate");

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static PRM_Default filePathDefault(0.0, "");
static PRM_Default velocityBlendDefault(0.4);
static PRM_Default maxVelocityDefault(2.0);
static PRM_Default maxStabilityIterationsDefault(10);
static PRM_Default maxIterationsDefault(5);
static PRM_Default defaultAgentMassDefault(1.0);
static PRM_Default defaultAgentRadiusDefault(0.25);
static PRM_Default collisionMarchingStepsDefault(1000);


////////////////////////////////////////////////////////////////////////////////////////

PRM_Template PeepSimSolver::mParameterList[] = {
  PRM_Template(PRM_STRING, PRM_Template::PRM_EXPORT_MIN, 1, &filePath, &filePathDefault, 0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &velocityBlendName, &velocityBlendDefault,
               0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &maxVelocityName, &maxVelocityDefault, 0),
  PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MIN, 1, &maxStabilityIterationsName,
               &maxStabilityIterationsDefault, 0),
  PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MIN, 1, &maxIterationsName, &maxIterationsDefault,
               0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &defaultAgentMassName,
               &defaultAgentMassDefault, 0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &defaultAgentRadiusName,
               &defaultAgentRadiusDefault, 0),
  PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MIN, 1, &collisionMarchingStepsName,
               &collisionMarchingStepsDefault, 0),

  PRM_Template(PRM_CALLBACK, 1, &generateCommandName, 0, 0, 0, PeepSimSolver::generateCallback),

  /////////////////////////////////////////////////////////////////////////////////////////////

  PRM_Template()
};

// Here's how we define local variables for the SOP.
enum {
  VAR_PT,
  // Point number of the star
  VAR_NPT // Number of points in the star
};

CH_LocalVariable PeepSimSolver::mLocalVariables[] = {
  {"PT", VAR_PT, 0}, // The table provides a mapping
  {"NPT", VAR_NPT, 0}, // from text string to integer token
  {0, 0, 0},
};

bool PeepSimSolver::evalVariableValue(fpreal& val, int index, int thread) {
  // myCurrPoint will be negative when we're not cooking so only try to
  // handle the local variables when we have a valid myCurrPoint index.
  if (myCurrPoint >= 0) {
    // Note that "gdp" may be null here, so we do the safe thing
    // and cache values we are interested in.
    switch (index) {
      case VAR_PT:
        val = (fpreal)myCurrPoint;
        return true;

      case VAR_NPT:
        val = (fpreal)myTotalPoints;
        return true;

      default:
        /* do nothing */
        ;
    }
  }

  // Not one of our variables, must delegate to the base class.
  return SOP_Node::evalVariableValue(val, index, thread);
}

OP_Node* PeepSimSolver::myConstructor(OP_Network* net, const char* name, OP_Operator* op) {
  return new PeepSimSolver(net, name, op);
}

int PeepSimSolver::generateCallback(void* data, int index, float time, const PRM_Template*) {
  fpreal now = time;

  PeepSimSolver* me = (PeepSimSolver*)data;

  me->calledFromCallback = true;

  OP_Context myContext(time);
  me->startSimulation(myContext);

  return 1;
}

int HDK_Sample::PeepSimSolver::startSimulation(OP_Context& context) {
  fpreal now = context.getTime();

  // Read input from GUI

  float velocityBlend = VELOCITYBLEND(now);
  float maxVelocity = MAXVELOCITY(now);
  float defaultAgentMass = DEFAULTAGENTMASS(now);
  float defaultAgentRadius = DEFAULTAGENTRADIUS(now);
  int stabilityIterations = STABILITYITERATIONS(now);
  int maxIterations = MAXITERATIONS(now);
  int collisionSteps = COLLISIONSTEPS(now);

  return 0;
}

PeepSimSolver::PeepSimSolver(OP_Network* net, const char* name, OP_Operator* op)
  : SOP_Node(net,
             name, op) {
  myCurrPoint = -1; // To prevent garbage values from being returned

}

PeepSimSolver::~PeepSimSolver() {}

unsigned PeepSimSolver::disableParms() {
  return 0;
}

void PeepSimSolver::AddAgent(float x, float y, float z) {

}

void PeepSimSolver::initCrowdSim() {


#ifdef TEST_BASE_PLUGIN

	printf("Initing Peep Sim!\n");

	gdp->clearAndDestroy();
	mAgents.clear();

	//TODO need to check if we need to destory/free the memory of the object we created
	// I am calling gdp::clearAndDestroy but double check again
	
	float total = 2 * M_PI;

	for (int i = 0; i < numAgent; ++i) {

		float xPos = 5 * std::cos(i * total / (float)numAgent);
		float zPos = 5 * std::sin(i * total / (float)numAgent);

		printf("Pos %f %f\n", xPos, zPos);

		GU_PrimSphereParms parms(gdp);
		parms.freq = 1;

		// Change this to some obj
		GEO_Primitive *sphere = GU_PrimSphere::build(parms, GEO_PRIMSPHERE);

		sphere->setPos3(0, UT_Vector3F(xPos, 0, zPos));

		mAgents.push_back(sphere);

	}
#endif
}

void PeepSimSolver::update(fpreal frame) {

#ifdef TEST_BASE_PLUGIN
	printf("Update for Frame %f \n", frame);

	int period = 360;

	float xPos, zPos, angle;
	float total = 2 * M_PI;

	int frameId = frame;

	for (int i = 0; i < mAgents.size(); ++i) {

		GEO_Primitive *sphere = mAgents[i];

		float angle = i * total / (float)numAgent + frameId % 360;

		float xPos = 5 * std::cos(angle);
		float zPos = 5 * std::sin(angle);

		sphere->setPos3(0, UT_Vector3F(xPos, 0, zPos));
	}
#endif

}


OP_ERROR PeepSimSolver::cookMySop(OP_Context& context) {
  
	fpreal now = context.getTime();
	
  // Read input from GUI

  int numAgents;
  numAgents = 10;// NUM_AGENTS(now);

  AGENTS_PATH(mFilePath, now);

  float velocityBlend      = VELOCITYBLEND(now);
  float maxVelocity        = MAXVELOCITY(now);
  float defaultAgentMass   = DEFAULTAGENTMASS(now);
  float defaultAgentRadius = DEFAULTAGENTRADIUS(now);
  int stabilityIterations  = STABILITYITERATIONS(now);
  int maxIterations        = MAXITERATIONS(now);
  int collisionSteps       = COLLISIONSTEPS(now);

  // CrowdSim simulator;

  // We must lock our inputs before we try to access their geometry.
  // OP_AutoLockInputs will automatically unlock our inputs when we return.
  // NOTE: Don't call unlockInputs yourself when using this!
  OP_AutoLockInputs inputs(this);

  // Check if locking caused an error
  if (inputs.lock(context) >= UT_ERROR_ABORT)
	  return error();

  // Now, indicate that we are time dependent (have to cook every time the current frame changes).
  OP_Node::flags().timeDep = 1;

  // Channel manager has time info for us
  CH_Manager *chman = OPgetDirector()->getChannelManager();

  // This is the frame that we're cooking at...
  fpreal currframe = chman->getSample(context.getTime());

  // Lets assume that we solve all the frames in the first frame itslef
  // We can probably do a frame by frame calculation later on
  fpreal reset = 1; 

  if (currframe <= reset)
  {
	  myLastCookTime = reset;
	  initCrowdSim();
  }
  else
  {
	  update(currframe);
  }

  return error();
}
