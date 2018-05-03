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

#include "external/json.hpp"

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
static PRM_Name maxStabilityIterationsName("maxStabilityIterations", "Constraints Stability Iterations");
static PRM_Name maxIterationsName("maxIterations", "Max Constraint Iterations");
static PRM_Name generateCommandName("generateCommand", "Update");
static PRM_Name simulateSceneCommand("simulateScene", "Simulate Scene");
static PRM_Name loadFileCommand("loadFile", "Load Scene from File");
static PRM_Name simDurationName("simDuration", "Simulation Duration (s)");

/*
* The GUI parameters to the agent node
*/

static PRM_Name numAgentsName("numAgents", "Number of Agents");
static PRM_Name agentMassName("agentMass", "Agent Mass");
static PRM_Name agentRadiusName("agentRadius", "Agent Radius");
static PRM_Name agentSpacingName("agentSpacing", "Agent Spacing");

static PRM_Name samplingShapeChoiceName("shapeMenu", "Sampling Shape");
static PRM_Name samplingMethodChoiceName("methodMenu", "Sampling Method");

static PRM_Name samplingShapeChoice[] =
{
	PRM_Name("shapechoice1", "Square"),
	PRM_Name("shapechoice2", "Disc"),
	PRM_Name(0)
};
static PRM_ChoiceList samplingShapeChoiceMenu(PRM_CHOICELIST_SINGLE, samplingShapeChoice);

static PRM_Name samplingMethodChoice[] =
{
	PRM_Name("sampchoice2", "Grid"),
	PRM_Name("sampchoice1", "Random"),
	PRM_Name("sampchoice3", "Stratified"),
	PRM_Name(0)
};
static PRM_ChoiceList samplingMethodChoiceMenu(PRM_CHOICELIST_SINGLE, samplingMethodChoice);

static PRM_Name shapeSize("shapeSize", "Shape Size");

static PRM_Name targetPosition("targetPos", "Target Position");
static PRM_Name sourcePosition("sourcePos", "Source Position");


/*
* The GUI parameters to the environment node
*/
static PRM_Name collisionMarchingStepsName("collisionSteps", "Max Env Collision Steps");

/*
* Declare the defaults of the defined parameters
*/

static PRM_Default numAgentsDefault(1);
static PRM_Default defaultAgentMassDefault(1.0);
static PRM_Default defaultAgentRadiusDefault(0.25);
static PRM_Default shapeSizeDefault[] = {
	PRM_Default(5.0),
	PRM_Default(5.0),
};

static PRM_Default filePathDefault(0.0, "P:\\ubuntu\\PeepSim\\Projects\\src\\scenes\\scene_7.json");
static PRM_Default velocityBlendDefault(0.4);
static PRM_Default maxVelocityDefault(2.0);
static PRM_Default maxStabilityIterationsDefault(10);
static PRM_Default simDurationDefault(10.0);
static PRM_Default maxIterationsDefault(5);

static PRM_Default collisionMarchingStepsDefault(1000);
static PRM_Default targetPositionDefault[] = {
	PRM_Default(10.0),
	PRM_Default(10.0),
};
static PRM_Default sourcePositionDefault[] = {
	PRM_Default(0.0),
	PRM_Default(0.0),
};

static PRM_Default agentSpacingDefault[] = {
  PRM_Default(1.0),
  PRM_Default(1.0),
};

/*
* Package the declared parameters so that it can be used to add to the operator table
*/
PRM_Template AgentNode::mParameterList[] = {

	PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &agentMassName, &defaultAgentMassDefault, 0),

	PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &agentRadiusName, &defaultAgentRadiusDefault, 0),

	PRM_Template(PRM_INT, PRM_Template::PRM_EXPORT_MIN, 1, &numAgentsName, &numAgentsDefault, 0),

	PRM_Template(PRM_ORD, 1, &samplingShapeChoiceName, 0, &samplingShapeChoiceMenu),

	PRM_Template(PRM_ORD, 1, &samplingMethodChoiceName, 0, &samplingMethodChoiceMenu),

	PRM_Template(PRM_XYZ_J,  2, &shapeSize, shapeSizeDefault, 0),

  PRM_Template(PRM_XYZ_J,  2, &agentSpacingName, agentSpacingDefault, 0),

	PRM_Template(PRM_XYZ_J,  2, &targetPosition, targetPositionDefault, 0),

	PRM_Template(PRM_XYZ_J,  2, &sourcePosition, sourcePositionDefault, 0),

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

  printf("AgentNode :: The mass & radius was %f %f \n", mass, radius);
	printf("AgentNode :: The position of the node was %f %f \n", source[0], source[1]);
  printf("AgentNode :: The target of the node was %f %f \n", target[0], target[1]);
	printf("AgentNode :: The position of the node was %f %f \n", me->getAgentGroup()->mAgents[0].mCurrPosition.x, me->getAgentGroup()->mAgents[0].mCurrPosition.y);


	me->mInitialized = false;

	// For some reason, we cannot add geometry from this thread

	//me->cookMySop(myContext); // recooks your sop, so you can read inputs again and do your thing

	return 1;

}

void AgentNode::initialize(fpreal frame) {
	
	OP_Context myContext(frame);

	int numAgents = NUM_AGENTS(frame);
	float mass = AGENT_MASS(frame);
	float radius = AGENT_RADIUS(frame);

	UT_Vector2R size;
	UT_Vector2R source;
	UT_Vector2R target;
  UT_Vector2R spacing;
	
	SHAPE_SIZE(frame, size);
	SOURCE_POS(frame, source);
	 AGENT_SPACING(frame, spacing);
	TARGET_POS(frame, target);

	int sampleShape = SAMPLE_SHAPE(frame);
	int sampleMethod = SAMPLE_METHOD(frame);

	mNumAgents = numAgents;

	printf("Initializing nodes %d \n ", mNumAgents);

	gdp->clearAndDestroy();

	float width = size[0];
	float height = size[1];

  float xSpace = spacing[0];
  float ySpace = spacing[0];

	const GU_Detail *geometry = fetchAgentObj(myContext);

  float currentX = 0.0f;
  float currentZ = 0.0f;

  int deployedAgents = 0;

  for (int w = 0; w < width; ++w) {
    for (int h = 0; h < height; ++h) {
      if (deployedAgents >= mNumAgents) {
        // Deployed all agents inside grid box
        break;
      }

      float xPos = source[0] + w * xSpace;
      float yPos = 0;
      float zPos = source[1] + h * ySpace;

      float xTargetPos = target[0] + w * xSpace;
      float zTargetPos = target[1] + h * ySpace;

      Agent newAgent;

      newAgent.mMass = mass;
      newAgent.mRadius = radius;

      newAgent.mCurrVelocity = Vector(0, 0);
      newAgent.mForce = Vector(0, 0);

      newAgent.mStartPosition = Vector(xPos, zPos);
      newAgent.mCurrPosition = newAgent.mStartPosition;
      newAgent.mTargetPosition = Vector(xTargetPos, zTargetPos);
      newAgent.mReference = geometry;

      newAgent.mCurrGeo = addAgent(xPos, yPos, zPos, newAgent.mRadius);

      mAgentgroup.mAgents.push_back(newAgent);

      ++deployedAgents;
    }
  }

	/*for (int i = 0; i < mNumAgents; ++i) {

		float xPos = source[0] +  i * width;
		float yPos = 0;
		float zPos = source[1] + i * height;

		float xTargetPos = target[0] + i * width;
		float zTargetPos = target[1] + i * height;

		Agent newAgent;

		newAgent.mMass = mass;
		newAgent.mRadius = radius;

		newAgent.mCurrVelocity = Vector(0, 0);
		newAgent.mForce = Vector(0, 0);

		newAgent.mStartPosition = Vector(xPos, zPos);
		newAgent.mCurrPosition = newAgent.mStartPosition;
		newAgent.mTargetPosition = Vector(xTargetPos, zTargetPos);
		newAgent.mReference = geometry;

		newAgent.mCurrGeo = addAgent(xPos, yPos, zPos);

		mAgentgroup.mAgents.push_back(newAgent);
	}*/

}

GEO_Primitive* AgentNode::addAgent(fpreal x, fpreal y, fpreal z, float radius) {

	GU_PrimSphereParms parms(gdp);
	parms.xform.scale(radius, radius, radius);
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

	if (currframe == reset)
	{
		initialize(currframe);
	}
	else
	{
		update(currframe);
	}

	return error();
}

const GU_Detail* AgentNode::fetchAgentObj(OP_Context& context) {

	OP_Node *inputGeometry = getInput(0);

	if (inputGeometry == nullptr) {
		return nullptr;
	}

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

  PRM_Template(PRM_FLT, 1, &simDurationName, &simDurationDefault),

  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &velocityBlendName, &velocityBlendDefault, 0),
  PRM_Template(PRM_FLT, PRM_Template::PRM_EXPORT_MIN, 1, &maxVelocityName, &maxVelocityDefault, 0),
  PRM_Template(PRM_INT_J, 1, &maxStabilityIterationsName, &maxStabilityIterationsDefault),
  PRM_Template(PRM_INT_J, 1, &maxIterationsName, &maxIterationsDefault),
  PRM_Template(PRM_INT_J, 1, &collisionMarchingStepsName, &collisionMarchingStepsDefault),

  PRM_Template(PRM_STRING, PRM_Template::PRM_EXPORT_MIN, 1, &filePath, &filePathDefault, 0),

  PRM_Template(PRM_CALLBACK, 1, &loadFileCommand, 0, 0, 0, PeepSimSolver::loadFromFileCallback),

  PRM_Template(PRM_CALLBACK, 1, &simulateSceneCommand, 0, 0, 0, PeepSimSolver::simulateScene),

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
  using Json = nlohmann::json;

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
  OP_NodeList envData;
  OP_NodeList siblings;
  OP_NodeList agentGroups;

  parent->getAllChildren(siblings);

  OP_Node *crowdSourceNode = nullptr;
  OP_Node *agentMergeNode = nullptr;
  OP_Node *environmentNode = nullptr;
  OP_Node *envMergeNode = nullptr;

  // Get the CrowdSource Node
  for (OP_Node* node : siblings) {
    if (node->getName().compare("CrowdSource", false) == 0) {
      crowdSourceNode = node;
      break;
    }
  }

  for (OP_Node* node : siblings) {
    if (node->getName().compare("Environment", false) == 0) {
      environmentNode = node;
      break;
    }
  }

  // Get the agent group merge Node
  if (crowdSourceNode == nullptr) {
    printf("Load Failed: Can't find crowdSource Node \n");
    return;
  }

  if (environmentNode == nullptr) {
    printf("Load Failed: Can't find environment Node \n");
    return;
  }

  crowdSourceNode->getAllChildren(crowdData);
  for (OP_Node* node : crowdData) {
    if (node->getOperator()->getName().compare("merge", false) == 0) {
      agentMergeNode = node;
      break;
    }
  }

  environmentNode->getAllChildren(envData);
  for (OP_Node* node : envData) {
    if (node->getOperator()->getName().compare("merge", false) == 0) {
      envMergeNode = node;
      break;
    }
  }

  if (envMergeNode == nullptr) {
    printf("Load Failed: Can't find Environment Merge Node \n");
    return;
  }

  if (agentMergeNode == nullptr) {
    printf("Load Failed: Can't find Merge Node \n");
    return;
  }

  // Delete all existing children of Agent Merge node
  OP_NodeList existingNodes;
  int inputsize = agentMergeNode->getInputsArraySize();
  std::vector<OP_Node*> nodesToDelete;
  std::vector<OP_Node*> envNodesToDelete;

  for (int i = 0; i < inputsize; ++i) {
	  OP_Node *node = agentMergeNode->getInput(i);
	  nodesToDelete.push_back(node);
  }

  for (OP_Node *node : nodesToDelete) {
	  ((OP_Network*)crowdSourceNode)->destroyNode(node);
  }

  inputsize = envMergeNode->getInputsArraySize();

  for (int i = 0; i < inputsize; ++i) {
    OP_Node *node = envMergeNode->getInput(i);
    envNodesToDelete.push_back(node);
  }

  for (OP_Node *node : envNodesToDelete) {
    ((OP_Network*)environmentNode)->destroyNode(node);
  }

  inputsize = agentMergeNode->getInputsArraySize();
  
  std::string path = filePath.toStdString();

  printf("Loading File From: %s \n", path.c_str());

  std::ifstream inputFileStream(filePath);
  std::string jsonData;

  // Note: File Load Not Optimized for huge files.
  inputFileStream.seekg(0, std::ios::end);
  jsonData.reserve(inputFileStream.tellg());
  inputFileStream.seekg(0, std::ios::beg);

  jsonData.assign((std::istreambuf_iterator<char>(inputFileStream)),
    std::istreambuf_iterator<char>());

  auto data = Json::parse(jsonData);

  auto jsonGroups = data["agentGroups"];
  auto jsonColliders = data["colliders"];

  int count = 1;

  for (Json::iterator it = jsonGroups.begin(); it != jsonGroups.end(); ++it) {
    // Structure
    /*{
      "numAgents": 10,
      "size" : [1, 10],
      "sourcePos" : [5, 0.5],
      "targetPos" : [-5, 0.5],
      "mass" : 1.0,
      "radius" : 0.25,
      "sampler" : "NONE",
      "samplerShape" : "SQUARE",
	  "debugColor" : [255, 0, 0]
	  }*/
	  auto group = *it;

	  int jsonNumAgents = group["numAgents"].get<int>();
	  int jsonSizeX = group["size"][0].get<int>();
	  int jsonSizeY = group["size"][1].get<int>();

	  float jsonSourcePos[2] = {
		  group["sourcePos"][0].get<float>(), group["sourcePos"][1].get<float>()
	  };

	  float jsonSpacing[2] = {
		  group["spacing"][0].get<float>(), group["spacing"][1].get<float>()
	  };

	  float jsonTargetPos[2] = {
		  group["targetPos"][0].get<float>(), group["targetPos"][1].get<float>()
	  };

	  float mass = group["mass"].get<float>();
	  float radius = group["radius"].get<float>();

	  std::string nodeName = "AgentGroup_" + std::to_string(count);

	  OP_Node* node = ((OP_Network*)crowdSourceNode)->createNode("AgentGroup", nodeName.c_str());

	  if (node == nullptr) {
		  printf("Load Failed: Can't create node \n");
		  return;
	  }

	  if (!node->runCreateScript()) {
		  printf("Load Failed: Can't create script error \n");
		  return;
	  }

	  int sampler = 0;
	  int samplerShape = 0;

	  if (group["sampler"].get<std::string>() == "GRID") {
		  sampler = 0;
	  }
	  else if (group["sampler"].get<std::string>() == "RANDOMIZED") {
		  sampler = 1;
	  }
	  else if (group["sampler"].get<std::string>() == "STRATIFIED") {
		  sampler = 2;
	  }

	  if (group["samplerShape"].get<std::string>() == "SQUARE") {
		  samplerShape = 0;
	  }
	  else if (group["samplerShape"].get<std::string>() == "DISC") {
		  samplerShape = 1;
	  }

	  node->setInt("numAgents", 0, time, jsonNumAgents);
	  node->setFloat("agentMass", 0, time, mass);
	  node->setFloat("agentRadius", 0, time, radius);

	  node->setFloat("shapeSize", 0, time, jsonSizeX);
	  node->setFloat("shapeSize", 1, time, jsonSizeY);

	  node->setFloat("agentSpacing", 0, time, jsonSpacing[0]);
	  node->setFloat("agentSpacing", 1, time, jsonSpacing[1]);

	  node->setFloat("targetPos", 0, time, jsonTargetPos[0]);
	  node->setFloat("targetPos", 1, time, jsonTargetPos[1]);
	  node->setFloat("targetPos", 2, time, jsonTargetPos[2]);

	  node->setFloat("sourcePos", 0, time, jsonSourcePos[0]);
	  node->setFloat("sourcePos", 1, time, jsonSourcePos[1]);
	  node->setFloat("sourcePos", 2, time, jsonSourcePos[2]);


	  node->setFloat("shapeMenu", 0, time, samplerShape);
	  node->setFloat("methodMenu", 0, time, sampler);

	  node->cook(myContext);

	  agentMergeNode->setInput(agentMergeNode->getInputsArraySize(), node);
	  node->moveToGoodPosition();
  }

  int envCount = 0;

  for (Json::iterator it = jsonColliders.begin(); it != jsonColliders.end(); ++it) {
    /*{
      "origin": [0, 0],
      "dimensions" : [1, 4, 1],
      "type" : "box"
    }*/
    auto collider = *it;

    float jsonOrigin[2] = {
      collider["origin"][0].get<float>(), collider["origin"][1].get<float>()
    };

    float jsonDimensions[3] = {
      collider["dimensions"][0].get<float>(), collider["dimensions"][1].get<float>(), collider["dimensions"][2].get<float>()
    };

    std::string nodeName = "BoxCollider_" + std::to_string(envCount);

    OP_Node* node = ((OP_Network*)environmentNode)->createNode("box", nodeName.c_str());

    if (node == nullptr) {
      printf("Load Failed: Can't create node \n");
      return;
    }

    if (!node->runCreateScript()) {
      printf("Load Failed: Can't create script error \n");
      return;
    }

    node->setFloat("size", 0, time, jsonDimensions[0]);
    node->setFloat("size", 1, time, jsonDimensions[2]);
    node->setFloat("size", 2, time, jsonDimensions[1]);

    node->setFloat("t", 0, time, jsonOrigin[0] + jsonDimensions[0] / 2.0f);
    node->setFloat("t", 1, time, jsonDimensions[2] / 2.0f);
    node->setFloat("t", 2, time, jsonOrigin[1] + jsonDimensions[1] / 2.0f);

    ++envCount;

    envMergeNode->setInput(envMergeNode->getInputsArraySize(), node);
    node->moveToGoodPosition();
  }

  printf("Loading JSON Completed\n");
}

void PeepSimSolver::updateScene(fpreal time) {
	// Channel manager has time info for us
	CH_Manager *chman = OPgetDirector()->getChannelManager();

	// This is the frame that we're cooking at...
	fpreal currframe = chman->getSample(time);

	// Fetch the input Crowd Source node

	std::vector<AgentNode*> mAgentGroupNodes;
	std::vector<std::pair<Vector, Vector>> mColliders;

	AgentNode *agents = nullptr;

	OP_Context myContext(time);

	float velocityBlend = VELOCITYBLEND(time);
	float maxVelocity = MAXVELOCITY(time);
	int stabilityIterations = STABILITYITERATIONS(time);
	int maxIterations = MAXITERATIONS(time);
	int collisionSteps = COLLISIONSTEPS(time);
  int simDuration = DURATION(time);

	mConfig = PeepSimConfig();
	mConfig.mMaxVelocity = maxVelocity;
	mConfig.mVelocityBlend = velocityBlend;
	mConfig.mMaxStabilityIterations = stabilityIterations;
	mConfig.mMaxIterations = maxIterations;
	mConfig.mCollisionMarchSteps = collisionSteps;
  mConfig.mSimualtionDuration = simDuration;
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
		OP_Node *environmentNode = nullptr;
		OP_Node *agentMergeNode = nullptr;
		OP_Node *envMergeNode = nullptr;

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

			for (int i = 0; i < m; ++i) {
				OP_Node *node = agentMergeNode->getInput(i);
				AgentNode *agentNode = ((AgentNode*)CAST_SOPNODE(node));
				mAgentGroupNodes.push_back(agentNode);
			}
		}
		else {
			printf("PeepSimSolver::updateScene: Can't find `Merge Node \n");
		}


		// Get the Environment Node
		for (OP_Node* node : siblings) {
			if (node->getName().compare("Environment", false) == 0) {
				environmentNode = node;
				break;
			}
		}

		crowdData.clear();

		// Get the agent group merge Node
		if (environmentNode != nullptr) {
			environmentNode->getAllChildren(crowdData);
			for (OP_Node* node : crowdData) {
				if (node->getOperator()->getName().compare("merge", false) == 0) {
					envMergeNode = node;
					break;
				}
			}
		}
		else {
			printf("PeepSimSolver::updateScene: Can't find Environment Merge Node \n");
		}

		// Get all the agent groups which needs to be shown
		if (envMergeNode != nullptr) {

			int m = envMergeNode->getInputsArraySize();

			for (int i = 0; i < m; ++i) {
				OP_Node *node = envMergeNode->getInput(i);
				// Allow only box colliders for now
				if (node->getOperator()->getName().compare("box", false) == 0) {
					SOP_Node *boxNode = ((SOP_Node*)CAST_SOPNODE(node));
					UT_BoundingBoxF mBoundingBox;
					boxNode->getBoundingBox(mBoundingBox, myContext);


					float minx = mBoundingBox.minvec()[0];
					float minz = mBoundingBox.minvec()[2];

					float sizex = mBoundingBox.size()[0];
					float sizeZ = mBoundingBox.size()[2];

					printf("The collider size is %f %f %f %f \n", minx, minz, sizex, sizeZ);

					std::pair<Vector, Vector> mBoundingPair = std::make_pair(Vector(minx, minz), Vector(sizex, sizeZ));

					mColliders.push_back(mBoundingPair);
				}
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
	mScene.addColliders(mColliders);
	mScene.mNumAgents = numAgents;

  //printf("YOLO %f, %f \n", mAllAgentGroups[0]->mAgents[0].mMass, mAllAgentGroups[0]->mAgents[0].mRadius);

	// TODO: add colliders

	// We can remove this later
	//mScene.loadFromFile("E:\\Git\\PeepSim\\Projects\\plugin\\plugin\\scenes\\scene_5.json");
  // mScene.loadFromFile("P:\\ubuntu\\PeepSim\\Projects\\src\\scenes\\scene_5.json");

	CrowdSim simulation = CrowdSim(mConfig, mScene);
	//simulation.loadSceneFromFile("E:\\Git\\PeepSim\\Projects\\plugin\\plugin\\scenes\\scene_5.json");
	mSimResults = simulation.evaluate();

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

