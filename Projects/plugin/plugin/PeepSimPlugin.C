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
		new OP_Operator("AgentGroup", // Internal name
			"Agents", // UI name
			AgentNode::myConstructor, // How to build the SOP
			AgentNode::mParameterList, // My parameters
			0, // Min # of sources
			2, // Max # of sources
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
		PeepSimSolver::myTemplateList, 0, 9999, 0,
		0, 1);
	table->addOperator(op);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent Node Start <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

static PRM_Name filePath("filePath", "File Path");

/*
* The GUI parameters to the solver node
*/
static PRM_Name velocityBlendName("velocityBlend", "Velocity Blend");
static PRM_Name maxVelocityName("maxVelocity", "Max Velocity");
static PRM_Name maxStabilityIterationsName("maxStabilityIterations", "Max Stability Iterations");
static PRM_Name maxIterationsName("maxIterations", "Max Iterations");
static PRM_Name generateCommandName("generateCommand", "Update");
static PRM_Name simulateSceneCommand("simulateScene", "Simulate Scene");
static PRM_Name loadFileCommand("loadFile", "Load Scene from File");

/*
* The GUI parameters to the agent node
*/
static PRM_Name targetPosition("targetPos", "Target Position");
static PRM_Name sourcePosition("sourcePos", "Source Position");
static PRM_Name agentMassName("agentMass", "Agent Mass");
static PRM_Name agentRadiusName("agentRadius", "Agent Radius");
static PRM_Name addAgentCommand("addAgent", "Add");

/*
* The GUI parameters to the environment node
*/
static PRM_Name collisionMarchingStepsName("collisionSteps", "Max Env Collision Steps");

/*
* Declare the defaults of the defined parameters
*/
static PRM_Default filePathDefault(0.0, "P:\\ubuntu\\PeepSim\\Projects\\src\\scenes\\scene_5.json");
static PRM_Default velocityBlendDefault(0.4);
static PRM_Default maxVelocityDefault(2.0);
static PRM_Default maxStabilityIterationsDefault(10);
static PRM_Default maxIterationsDefault(5);
static PRM_Default defaultAgentMassDefault(1.0);
static PRM_Default defaultAgentRadiusDefault(0.25);
static PRM_Default collisionMarchingStepsDefault(1000);
static PRM_Default targetPositionDefault[] = {
	PRM_Default(0.0),
	PRM_Default(0.0),
};
static PRM_Default sourcePositionDefault[] = {
	PRM_Default(0.0),
	PRM_Default(0.0),
};

/*
* Package the declared parameters so that it can be used to add to the operator table
*/
PRM_Template AgentNode::mParameterList[] = {

	PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &agentMassName,
	&defaultAgentMassDefault, 0),
	PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &agentRadiusName,
	&defaultAgentRadiusDefault, 0),

	PRM_Template(PRM_XYZ_J,  2, &targetPosition, targetPositionDefault, 0),
	PRM_Template(PRM_XYZ_J,  2, &sourcePosition, sourcePositionDefault, 0),

	PRM_Template(PRM_CALLBACK, 1, &addAgentCommand, 0, 0, 0, AgentNode::addAgentCallback),

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


AgentNode::AgentNode(OP_Network* net, const char* name, OP_Operator* op)
	: SOP_Node(net, name, op), mNumAgents(0), mSimResults(nullptr), mScene(nullptr), mAgentgroup(AgentGroup()), mInitialized(false) {
	myCurrPoint = -1; // To prevent garbage values from being returned

}

AgentNode::~AgentNode() {}

unsigned AgentNode::disableParms() {
	return 0;
}

AgentGroup* AgentNode::getAgentGroup() {
	return &mAgentgroup;
}

void AgentNode::setNumAgents(int numAgents) {
	mNumAgents = numAgents;
}

void AgentNode::updateResults(Results *results) {
	mSimResults = results;
}

void AgentNode::updateScene(Scene *scene) {
	mScene = scene;
}

int AgentNode::addAgentCallback(void* data, int index, float time, const PRM_Template*) {

	fpreal now = time;

	// Fetch an instance of self
	AgentNode* me = (AgentNode*)data;

	OP_Context myContext(time);

	const GU_Detail *geometry = me->fetchAgentObj(myContext);

	float mass = me->AGENT_MASS(now);
	float radius = me->AGENT_RADIUS(now);

	UT_Vector2R target;
	UT_Vector2R source;

	me->TARGET_POS(now, target);
	me->SOURCE_POS(now, source);

	Agent newAgent;

	newAgent.mMass = mass;
	newAgent.mRadius = radius;

	newAgent.mStartPosition = Vector(source[0], source[1]);
	newAgent.mCurrPosition = newAgent.mStartPosition;
	newAgent.mTargetPosition = Vector(target[0], target[1]);
	newAgent.mReference = geometry;
	
	me->getAgentGroup()->mAgents.push_back(newAgent);

	printf("AgentNode :: The position of the node was %f %f \n", source[0], source[1]);
	printf("AgentNode :: The position of the node was %f %f \n", me->getAgentGroup()->mAgents[0].mCurrPosition.x, me->getAgentGroup()->mAgents[0].mCurrPosition.y);


	me->mInitialized = false;

	// For some reason, we cannot add geometry from this thread

	//me->cookMySop(myContext); // recooks your sop, so you can read inputs again and do your thing

	return 1;

}

void AgentNode::initialize() {

	if (mNumAgents == mAgentgroup.mAgents.size()) {
		return;
	}

	mNumAgents = mAgentgroup.mAgents.size();

	printf("Initializing nodes %d \n ", mNumAgents);

	gdp->clearAndDestroy();

	for (int i = 0; i < mNumAgents; ++i) {

		float xPos = mAgentgroup.mAgents[i].mCurrPosition.x;
		float yPos = 0;
		float zPos = mAgentgroup.mAgents[i].mCurrPosition.y;

		mAgentgroup.mAgents[i].mCurrGeo = addAgent(xPos, yPos, zPos);
	}

}

GEO_Primitive* AgentNode::addAgent(fpreal x, fpreal y, fpreal z) {

	GU_PrimSphereParms parms(gdp);
	parms.freq = 1;

	GEO_Primitive *sphere = GU_PrimSphere::build(parms, GEO_PRIMSPHERE);

	sphere->setPos3(0, UT_Vector3F(x, y, z));

	return sphere;
}


void AgentNode::update(fpreal timeStep) {

	int frameId = timeStep;



	for (int i = 0; i < mAgentgroup.mAgents.size(); ++i) {

		GEO_Primitive *sphere = mAgentgroup.mAgents[i].mCurrGeo;

		if (sphere == nullptr || frameId >= mAgentgroup.mAgents[i].mCachedPos.size()) {
			continue;
		}
		
		float xPos = mAgentgroup.mAgents[i].mCachedPos[frameId].x;
		float yPos = 0;
		float zPos = mAgentgroup.mAgents[i].mCachedPos[frameId].y;

		printf("Agent Update  Geometry found %f %f >> \n", xPos, zPos);

		sphere->setPos3(0, UT_Vector3F(xPos, yPos, zPos));

	}
}

OP_ERROR AgentNode::cookMySop(OP_Context& context) {

	printf("Agent Node is Cooked >>>> \n");

	if (mSimResults != nullptr) {
		int size = mSimResults->mPositions.size();
		printf("Cooking Sop with >>>> %d \n", size);
	}

	OP_Node::flags().timeDep = 1;

	CH_Manager *chman = OPgetDirector()->getChannelManager();
	fpreal currframe = chman->getSample(context.getTime());

	fpreal reset = 1;

	if (!mInitialized)
	{
		initialize();
		mInitialized = true;
	}
	else
	{
		update(currframe);
	}

	return error();
}

const GU_Detail* AgentNode::fetchAgentObj(OP_Context& context) {

	OP_Node *inputGeometry = getInput(0);

	printf("Input Geometry is %s \n", inputGeometry->getOperator()->getName().toStdString());

	SOP_Node *geo = ((SOP_Node*)CAST_SOPNODE(inputGeometry));

	return geo->getCookedGeo(context);

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

  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &velocityBlendName, &velocityBlendDefault, 0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &maxVelocityName, &maxVelocityDefault, 0),
  PRM_Template(PRM_INT_J, 1, &maxStabilityIterationsName, &maxStabilityIterationsDefault),
  PRM_Template(PRM_INT_J, 1, &maxIterationsName, &maxIterationsDefault),
  PRM_Template(PRM_INT_J, 1, &collisionMarchingStepsName, &collisionMarchingStepsDefault),

	PRM_Template(PRM_CALLBACK, 1, &simulateSceneCommand, 0, 0, 0, PeepSimSolver::simulateScene),

  PRM_Template(PRM_STRING, PRM_Template::PRM_EXPORT_MIN, 1, &filePath, &filePathDefault, 0),

  PRM_Template(PRM_CALLBACK, 1, &loadFileCommand, 0, 0, 0, PeepSimSolver::loadFromFileCallback),

	PRM_Template()
};

PeepSimSolver::PeepSimSolver(OP_Network *net, const char *name, OP_Operator *op) 
	: DOP_Node(net, name, op), hasCookedSop(false), mConfig(PeepSimConfig()), mScene(mConfig)
{
	mConfig.create();
	mScene = Scene(mConfig);
}

PeepSimSolver::~PeepSimSolver()
{}

int PeepSimSolver::simulateScene(void* data, int index, float time, const PRM_Template*) {
	fpreal now = time;

	// Fetch an instance of self
	PeepSimSolver* me = (PeepSimSolver*)data;
	me->updateScene(now);

	return 0;
}

int PeepSimSolver::loadFromFileCallback(void* data, int index, float time, const PRM_Template*) {
  fpreal now = time;

  // Fetch an instance of self
  PeepSimSolver* me = (PeepSimSolver*)data;
  me->loadFromFile(now);

  return 0;
}

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
	
}

void PeepSimSolver::loadFromFile(fpreal time) {
  UT_String filePath;
  FILEPATH(filePath, time);

  OP_Context myContext(time);

  OP_Node *parent = getParent();
  if (parent == nullptr) {
    printf("Load Failed: Can't find parent \n");
    return;
  }

  parent = parent->getParent();

  OP_NodeList crowdData;
  OP_NodeList siblings;
  OP_NodeList agentGroups;

  parent->getAllChildren(siblings);

  OP_Node *crowdSourceNode = nullptr;
  OP_Node *agentMergeNode = nullptr;

  // Get the CrowdSource Node
  for (OP_Node* node : siblings) {
    if (node->getName().compare("CrowdSource", false) == 0) {
      crowdSourceNode = node;
      break;
    }
  }

  // Get the agent group merge Node
  if (crowdSourceNode == nullptr) {
    printf("Load Failed: Can't find crowdSource Node \n");
    return;
  }

  crowdSourceNode->getAllChildren(crowdData);
  for (OP_Node* node : crowdData) {
    if (node->getOperator()->getName().compare("merge", false) == 0) {
      agentMergeNode = node;
      break;
    }
  }

  if (agentMergeNode == nullptr) {
    printf("Load Failed: Can't find Merge Node \n");
    return;
  }

  std::string path = filePath.toStdString();

  printf("Loading File From: %s \n", path.c_str());

  Scene tempScene = Scene(mConfig);
  tempScene.loadFromFile(path);

  int count = 1;

  /*for (auto& group : tempScene.mAgentGroups) {
    std::string nodeName = "AgentGroup_1" + count;

    OP_Node* node = ((OP_Network*)crowdSourceNode)->createNode("AgentGroup", nodeName);

    if (node == nullptr) {
      printf("Load Failed: Can't create node \n");
      return;
    }

    if (!node->runCreateScript()) {
      printf("Load Failed: Can't create script error \n");
      return;
    }

    node->moveToGoodPosition();

    printf("Load Passed: Created Node \n");
  }*/
}

void PeepSimSolver::updateScene(fpreal time) {
	// Channel manager has time info for us
	CH_Manager *chman = OPgetDirector()->getChannelManager();

	// This is the frame that we're cooking at...
	fpreal currframe = chman->getSample(time);

	// Fetch the input Crowd Source node

	std::vector<AgentNode*> mAgentGroupNodes;

	AgentNode *agents = nullptr;

	OP_Context myContext(time);

  float velocityBlend = VELOCITYBLEND(time);
  float maxVelocity = MAXVELOCITY(time);
  int stabilityIterations = STABILITYITERATIONS(time);
  int maxIterations = MAXITERATIONS(time);
  int collisionSteps = COLLISIONSTEPS(time);

  mConfig = PeepSimConfig();
  mConfig.mMaxVelocity = maxVelocity;
  mConfig.mVelocityBlend = velocityBlend;
  mConfig.mMaxStabilityIterations = stabilityIterations;
  mConfig.mMaxIterations = maxIterations;
  mConfig.mCollisionMarchSteps = collisionSteps;
  mConfig.create();

	// Start Fetching the agent data from Houdini
	OP_Node *parent = getParent();
	if (parent != nullptr) {

		parent = parent->getParent();

		OP_NodeList crowdData;
		OP_NodeList siblings;
		OP_NodeList agentGroups;

		parent->getAllChildren(siblings);

		OP_Node *crowdSourceNode = nullptr;
		OP_Node *agentMergeNode = nullptr;

		// Get the CrowdSource Node
		for (OP_Node* node : siblings) {
			if (node->getName().compare("CrowdSource", false) == 0) {
				crowdSourceNode = node;
				break;
			}
		}

		// Get the agent group merge Node
		if (crowdSourceNode != nullptr) {
			crowdSourceNode->getAllChildren(crowdData);
			for (OP_Node* node : crowdData) {
				if (node->getOperator()->getName().compare("merge", false) == 0) {
					agentMergeNode = node;
					break;
				}
			}
		}
		else {
			printf("PeepSimSolver::updateScene: Can't find Crowd Source Node \n");
		}

		// Get all the agent groups which needs to be shown
		if (agentMergeNode != nullptr) {

			int m = agentMergeNode->getInputsArraySize();

			printf("PeepSimSolver::updateScene: Number of nodes in parent is %d \n", m);

			for (int i = 0; i < m; ++i) {
				OP_Node *node = agentMergeNode->getInput(i);
				AgentNode *agentNode = ((AgentNode*)CAST_SOPNODE(node));
				mAgentGroupNodes.push_back(agentNode);
			}
		}
		else {
			printf("PeepSimSolver::updateScene: Can't find `Merge Node \n");
		}
	}

	std::vector<AgentGroup*> mAllAgentGroups;


	int numAgents = 0;


	for (AgentNode *node : mAgentGroupNodes) {
		printf("Agent Group in Node Size: %d \n", node->getAgentGroup()->mAgents.size());
		mAllAgentGroups.push_back(node->getAgentGroup());
		numAgents += node->getAgentGroup()->mAgents.size();
	}

	printf("PeepSimSolver::updateScene >>>> \n");
	// Run the Crowd Simulation and cook the agents node

	mScene = Scene(mConfig);
	mScene.addAgentGroups(mAllAgentGroups);
	mScene.mNumAgents = numAgents;

	// TODO: add colliders

	// We can remove this later
	//mScene.loadFromFile("E:\\Git\\PeepSim\\Projects\\plugin\\plugin\\scenes\\scene_5.json");

	CrowdSim simulation = CrowdSim(mConfig, mScene);
	//simulation.loadSceneFromFile("E:\\Git\\PeepSim\\Projects\\plugin\\plugin\\scenes\\scene_5.json");
	mSimResults = simulation.evaluate();

	printf("Number of Frames in results is %d \n", mSimResults.mPositions.size());
	printf("Number of Agents in results is %d \n", mSimResults.mPositions[0].size());
	hasCookedSop = true;

	for (AgentNode *agent : mAgentGroupNodes) {
		if (agent != nullptr) {
			agent->updateScene(&mScene);
			agent->mInitialized = false;

			printf("Number of Cached Positions in this Group Node %d \n", agent->getAgentGroup()->mAgents[0].mCachedPos.size());

			OP_Context context;
			context.setFrame(currframe);
			agent->cook(context);
		}
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PeepSimSolver End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

