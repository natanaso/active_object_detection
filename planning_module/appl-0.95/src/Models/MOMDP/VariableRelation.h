#ifndef VariableRelation_H
#define VariableRelation_H


#include <vector>
#include <map>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
#include "VariableValue.h"
#include "IVariable.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// relation entry
	class RelEntry : public MObject
	{
	public:
		//map<string, applSharedPointer<IVariableValue> > sourceValues;
		map<string, applSharedPointer<IVariableValue> > destValues;
		double prob;
	};
	// generic class for variable, holds user defined variable value
	class VariableRelation : public MObject
	{
	protected:
		vector<applSharedPointer<IVariable> > srcVars;
		applSharedPointer<IVariable> destVar;
		
	public:
		VariableRelation();
		virtual ~VariableRelation(void);
		
		virtual void addSourceVar(applSharedPointer<IVariable> var);
		virtual void setDestVariable(applSharedPointer<IVariable> var);
		virtual vector<applSharedPointer<IVariable> > getSourceVars();
		virtual applSharedPointer<IVariable> getDestVariable();
		
		virtual vector<applSharedPointer<RelEntry> > getProb(map<string, applSharedPointer<IVariableValue> > sourceVals) = 0;
		
	};

	

}

#endif

