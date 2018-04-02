#pragma once

#include <DOP/DOP_Node.h>

namespace HDK_Sample {

	class DOP_PeepSim : public DOP_Node
	{
	public:

		DOP_PeepSim(OP_Network *net, const char *name, OP_Operator *op);

		virtual ~DOP_PeepSim();

		static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op);

		static PRM_Template	myTemplateList[];

	protected:
		virtual void processObjectsSubclass(fpreal time,
			int foroutputidx,
			const SIM_ObjectArray &objects,
			DOP_Engine &engine);
		virtual void	 getInputInfoSubclass(int inputidx,
			DOP_InOutInfo &info) const;
		virtual void	 getOutputInfoSubclass(int inputidx,
			DOP_InOutInfo &info) const;

	private:
		void		 GROUP(UT_String &str, fpreal t);
		int			 INPUTINDEX(fpreal t);
	};

}