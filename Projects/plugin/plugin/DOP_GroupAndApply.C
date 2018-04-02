/*
 * Copyright (c) 2017
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */

#include "DOP_GroupAndApply.h"
#include <UT/UT_DSOVersion.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipGroup.h>
#include <OP/OP_OperatorTable.h>
#include <DOP/DOP_PRMShared.h>
#include <DOP/DOP_InOutInfo.h>
#include <DOP/DOP_Operator.h>
#include <DOP/DOP_Engine.h>

using namespace HDK_Sample;

void
newDopOperator(OP_OperatorTable *table)
{
    OP_Operator	*op;

    // Create a new DOP_Operator which describes the operator we are
    // building. The parameters to this function are similar to the
    // OP_Operator constructor except for the last parameter, which
    // specifies the number of outputs (up to 4) from this operator.
    op = new DOP_Operator("hdk_groupandapply", "Group and Apply",
			  DOP_GroupAndApply::myConstructor,
			  DOP_GroupAndApply::myTemplateList, 1, 9999, 0,
			  0, 1);
    table->addOperator(op);
}

static PRM_Name		 theInputIndexName("inputindex", "Input Index");

PRM_Template
DOP_GroupAndApply::myTemplateList[] = {
    // Standard activation parameter.
    PRM_Template(PRM_INT_J,	1, &DOPactivationName,
				&DOPactivationDefault),
    // Standard group parameter with group menu.
    PRM_Template(PRM_STRING,	1, &DOPgroupName, &DOPgroupDefault,
				&DOPgroupMenu),
    // The input index that determines which data will be attached to
    // each object.
    PRM_Template(PRM_INT_J,	1, &theInputIndexName, PRMzeroDefaults),
    PRM_Template()
};

OP_Node *
DOP_GroupAndApply::myConstructor(OP_Network *net, const char *name,
				 OP_Operator *op)
{
    return new DOP_GroupAndApply(net, name, op);
}

DOP_GroupAndApply::DOP_GroupAndApply(OP_Network *net, const char *name,
				     OP_Operator *op)
    : DOP_Node(net, name, op)
{
}

DOP_GroupAndApply::~DOP_GroupAndApply()
{
}

void
DOP_GroupAndApply::processObjectsSubclass(fpreal time, int,
					  const SIM_ObjectArray &objects,
					  DOP_Engine &engine)
{
    SIM_ObjectArray	 filtered;
    UT_String		 group;
    int			 i, inputindex;

    // Grab the group string and filter our incoming objects using that
    // string. This narrows down the set of objects that we actually want
    // to operate on. The filtered array will contain only those objects
    // from the original objects array that match the supplied string.
    GROUP(group, time);
    SIM_DataFilterRootData	 filter(group);
    objects.filter(filter, filtered);

    // Loop through all the objects that passed the filter.
    for( i = 0; i < filtered.entries(); i++ )
    {
	// Set information about the object we are going to process.
	// The first argument is the index of the current object within the
	// full list of objects we are going to process. The second
	// argument is the total number of objects we are going to process.
	// The last argument is a pointer to the actual object we are
	// processing.
	setCurrentObject(i, filtered.entries(), filtered(i));

	// The isActive function checks both the bypass flag and the
	// activation parameter on the node (if there is one, which there
	// is in this case). We call this function after calling
	// setCurrentObject and we call it for each object in case the
	// activation parameter uses some object-specific variables
	// like OBJID in an expression.
	if( isActive(time) )
	{
	    // Evaluate the input index. Also called after setCurrentObject
	    // to properly evaluate objects specific local variables. Then
	    // make sure there is something connected to the requested input.
	    // We add one to the returned value to skip over the object
	    // input.
	    inputindex = INPUTINDEX(time) + 1;
	    if( inputindex != 0 && getInput(inputindex) )
	    {
		SIM_Relationship	*relationship;
		UT_String		 addtogroup;
		char			 numstr[UT_NUMBUF];

		// This function attaches the data connected to input
		// number inputindex to the current object.
		applyDataFromInput(time, inputindex, inputindex,
				   *filtered(i),
				   0, engine, 0, true);
		// Create a group name based on the input index.
		UT_String::itoa(numstr, inputindex);
		addtogroup = "applygroup_";
		addtogroup += numstr;

		// Create the relationship if it doesn't already exist,
		// and add the object to the group.
		relationship = engine.addRelationship(addtogroup,
				      SIM_DATA_RETURN_EXISTING);
		if( relationship )
		{
		    relationship->addGroup(filtered(i));
		    SIM_DATA_CREATE(*relationship, SIM_RELGROUP_DATANAME,
				    SIM_RelationshipGroup,
				    SIM_DATA_RETURN_EXISTING);
		}
		else
		    addError(DOP_CANTCREATERELATIONSHIP);
	    }
	}
    }
}

void
DOP_GroupAndApply::getInputInfoSubclass(int inputidx,
	DOP_InOutInfo &info) const
{
    // Our first input is an object input.
    // Our remaining inputs are data inputs.
    if( inputidx == 0 )
	info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
    else
	info = DOP_InOutInfo(DOP_INOUT_DATA, true);
}

void
DOP_GroupAndApply::getOutputInfoSubclass(int outputidx,
	DOP_InOutInfo &info) const
{
    // Our single output is an object output.
    info = DOP_InOutInfo(DOP_INOUT_OBJECTS, false);
}

void
DOP_GroupAndApply::GROUP(UT_String &str, fpreal t)
{
    evalString(str, DOPgroupName.getToken(), 0, t);
}

int
DOP_GroupAndApply::INPUTINDEX(fpreal t)
{
    return evalInt(theInputIndexName.getToken(), 0, t);
}

