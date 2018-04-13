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

#include <SIM/SIM_DopDescription.h>

#include <limits.h>
#include "PeepSimPlugin.h"

#include <UT/UT_NTStreamUtil.h>
#include <UT/UT_IStream.h>
#include <CMD/CMD_Args.h>
#include <PI/PI_ResourceManager.h>
#include <MOT/MOT_Director.h>

static float numAgent = 10.0;

using namespace HDK_Sample;

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Register Nodes <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/*
* Register the SOP nodes
*/
void newSopOperator(OP_OperatorTable* table) {

	table->addOperator(
		new OP_Operator("AgentsNode", // Internal name
			"Agents", // UI name
			AgentNode::myConstructor, // How to build the SOP
			AgentNode::mParameterList, // My parameters
			0, // Min # of sources
			0, // Max # of sources
			AgentNode::mLocalVariables, // Local variables
			OP_FLAG_GENERATOR) // Flag it as generator
	);
}

/*
* Register the DOP nodes
*/
void newDopOperator(OP_OperatorTable *table)
{
	OP_Operator	*op;

	op = new DOP_Operator("PluginBase", "PeepSimSolver",
		PeepSimSolver::myConstructor,
		PeepSimSolver::myTemplateList, 2, 9999, 0,
		0, 1);
	table->addOperator(op);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent Node Start <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


/*
* Declare the GUI parameters to the nodes
*/
static PRM_Name filePath("filePath", "File Path");
static PRM_Name velocityBlendName("velocityBlend", "Velocity Blend");
static PRM_Name maxVelocityName("maxVelocity", "Max Velocity");
static PRM_Name maxStabilityIterationsName("maxStabilityIterations", "Max Stability Iterations");
static PRM_Name maxIterationsName("maxIterations", "Max Iterations");
static PRM_Name defaultAgentMassName("defaultAgentMass", "Default Agent Mass");
static PRM_Name defaultAgentRadiusName("defaultAgentRadius", "Default Agent Radius");
static PRM_Name collisionMarchingStepsName("collisionSteps", "Max Env Collision Steps");
static PRM_Name generateCommandName("generateCommand", "Generate");

/*
* Declare the defaults of the defined parameters
*/
static PRM_Default filePathDefault(0.0, "");
static PRM_Default velocityBlendDefault(0.4);
static PRM_Default maxVelocityDefault(2.0);
static PRM_Default maxStabilityIterationsDefault(10);
static PRM_Default maxIterationsDefault(5);
static PRM_Default defaultAgentMassDefault(1.0);
static PRM_Default defaultAgentRadiusDefault(0.25);
static PRM_Default collisionMarchingStepsDefault(1000);


/*
* Package the declared parameters so that it can be used to add to the operator table
*/
PRM_Template AgentNode::mParameterList[] = {
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

	//PRM_Template(PRM_CALLBACK, 1, &generateCommandName, 0, 0, 0, PeepSimSolver::generateCallback),

	PRM_Template()
};

// Here's how we define local variables for the SOP.
enum {
	VAR_PT,
	// Point number of the star
	VAR_NPT // Number of points in the star
};

CH_LocalVariable AgentNode::mLocalVariables[] = {
	{ "PT", VAR_PT, 0 }, // The table provides a mapping
	{ "NPT", VAR_NPT, 0 }, // from text string to integer token
	{ 0, 0, 0 },
};

bool AgentNode::evalVariableValue(fpreal& val, int index, int thread) {
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

OP_Node* AgentNode::myConstructor(OP_Network* net, const char* name, OP_Operator* op) {
	return new AgentNode(net, name, op);
}


AgentNode::AgentNode(OP_Network* net, const char* name, OP_Operator* op) : SOP_Node(net, name, op), mNumAgents(0) {
	myCurrPoint = -1; // To prevent garbage values from being returned

}

AgentNode::~AgentNode() {}

unsigned AgentNode::disableParms() {
	return 0;
}

void AgentNode::setNumAgents(int numAgents) {
	mNumAgents = numAgents;
}

void AgentNode::initialize(int numAgents) {

	if (mNumAgents == numAgent) {
		return;
	}

	printf("Initializing nodes \n");

	gdp->clearAndDestroy();
	mAgents.clear();

	mNumAgents = numAgents;

	float total = 2 * M_PI;

	for (int i = 0; i < numAgents; ++i) {

		float xPos = 5 * std::cos(i * total / (float)numAgent);
		float zPos = 5 * std::sin(i * total / (float)numAgent);

		GU_PrimSphereParms parms(gdp);
		parms.freq = 1;

		// Change this to some obj
		GEO_Primitive *sphere = GU_PrimSphere::build(parms, GEO_PRIMSPHERE);
		sphere->setPos3(0, UT_Vector3F(xPos, 0, zPos));

		mAgents.push_back(sphere);

	}
}

void AgentNode::addAgent(fpreal x, fpreal y, fpreal z) {

	GU_PrimSphereParms parms(gdp);
	parms.freq = 1;

	GEO_Primitive *sphere = GU_PrimSphere::build(parms, GEO_PRIMSPHERE);

	sphere->setPos3(0, UT_Vector3F(x, y, z));

	mAgents.push_back(sphere);
	mNumAgents++;
}

void AgentNode::update(int agentId, fpreal x, fpreal y, fpreal z) {
	if (agentId < mNumAgents) {
		GEO_Primitive *sphere = mAgents[agentId];
		sphere->setPos3(0, UT_Vector3F(x, y, z));
	}
}

void AgentNode::update(fpreal timeStep) {

	int period = 360;

	float xPos, zPos, angle;
	float total = 2 * M_PI;

	int frameId = timeStep;

	printf("Frame is %d and time step is %f \n", frameId, timeStep);


	for (int i = 0; i < mAgents.size(); ++i) {

		GEO_Primitive *sphere = mAgents[i];

		float angle = i * total / (float)numAgent + frameId % 360;

		float xPos = 5 * std::cos(angle);
		float zPos = 5 * std::sin(angle);

		sphere->setPos3(0, UT_Vector3F(xPos, 0, zPos));
	}

}

OP_ERROR AgentNode::cookMySop(OP_Context& context) {

	printf("Agent Node is Cooked >>>> \n");

	OP_AutoLockInputs inputs(this);

	// Check if locking caused an error
	if (inputs.lock(context) >= UT_ERROR_ABORT)
		return error();

	OP_Node::flags().timeDep = 1;

	CH_Manager *chman = OPgetDirector()->getChannelManager();
	fpreal currframe = chman->getSample(context.getTime());

	fpreal reset = 1;

	if (currframe <= reset)
	{
		initialize(10);
	}
	else
	{
		update(currframe);
	}
	return error();
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent Node End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PeepSimSolver Start <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/*
* Defining a PeepSim DOP node Constructor for Houdini to create a plugin
*/
OP_Node* PeepSimSolver::myConstructor(OP_Network *net, const char *name, OP_Operator *op) {
	return new PeepSimSolver(net, name, op);
}

static PRM_Name	theInputIndexName("inputindex", "Input Index");

/*
* Passing Possible parameter lists for PeepSim DOP node
*/
PRM_Template PeepSimSolver::myTemplateList[] = {

	// Standard activation parameter.
	PRM_Template(PRM_INT_J,	1, &DOPactivationName, &DOPactivationDefault),

	// Standard group parameter with group menu.
	PRM_Template(PRM_STRING, 1, &DOPgroupName, &DOPgroupDefault, &DOPgroupMenu),

	// The input index that determines which data will be attached to
	// each object.
	PRM_Template(PRM_INT_J,	1, &theInputIndexName, PRMzeroDefaults),

	PRM_Template()
};

PeepSimSolver::PeepSimSolver(OP_Network *net, const char *name, OP_Operator *op) : DOP_Node(net, name, op), hasCookedSop(false)
{}

PeepSimSolver::~PeepSimSolver()
{}

void PeepSimSolver::getInputInfoSubclass(int inputidx, DOP_InOutInfo &info) const
{
	// Our first input is an object input.
	// Our remaining inputs are data inputs.
	if (inputidx == 0)
		info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
	else
		info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void PeepSimSolver::getOutputInfoSubclass(int outputidx, DOP_InOutInfo &info) const
{
	// Our single output is an object output.
	info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

/*
* Use this to initialize any crowd sim data
* This will be called only once per frame. So if user scrubs back to the start,
* we do not receive a callback here.
*/
void PeepSimSolver::processObjectsSubclass(fpreal time, int, const SIM_ObjectArray &objects, DOP_Engine &engine)
{
	// Channel manager has time info for us
	CH_Manager *chman = OPgetDirector()->getChannelManager();

	// This is the frame that we're cooking at...
	fpreal currframe = chman->getSample(time);

	//printf("The Frame is %f \n", currframe);

	OP_NodeList crowdData;

	//getParent()->getInput(0)->getAllChildren(crowdData);
	//AgentNode *agents = ((AgentNode*)CAST_SOPNODE(crowdData(0)));

	printf("PeepSimSolver::processObjectsSubclass >>>> \n");
	
	/*if (!hasCookedSop) {
		printf("Cooking Agent >>>> \n");
		OP_Context context;
		context.setFrame(currframe);
		agents->cook(context);
		hasCookedSop = true;
	}
	else {
		printf("Update Frame >>>> \n");
		
	}*/

	

	//agents->update(currframe);

	/*OP_NodeList crowdData;

	OP_Node *parent = getParent();

	printf(" Name of parent is %s\n", parent->getName().toStdString());

	OP_Node *parentInput = parent->getInput(0);

	printf(" Name of Input Node is %s\n", parentInput->getName().toStdString());

	parentInput->getAllChildren(crowdData);

	AgentNode *agents = ((AgentNode*)CAST_SOPNODE(crowdData(0)));

	//getParent()->getInput(0)->getAllNodes(crowdData);

	printf("Size of children list is %d \n", crowdData.size());
	for (int i = 0; i < crowdData.size(); ++i) {
	printf(" Node : %s\n", crowdData[i]->getName().toStdString());
	}*/



}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PeepSimSolver End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

