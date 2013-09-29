#include "VariableRelation.h"

VariableRelation::VariableRelation()
{
}

VariableRelation::~VariableRelation(void)
{
}
void VariableRelation::addSourceVar(applSharedPointer<IVariable> var)
{
	srcVars.push_back(var);
}
void VariableRelation::setDestVariable(applSharedPointer<IVariable> var)
{
	destVar = var;
}
vector<applSharedPointer<IVariable> > VariableRelation::getSourceVars()
{
	return srcVars;
}
applSharedPointer<IVariable> VariableRelation::getDestVariable()
{
	return destVar;
}
