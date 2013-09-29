#include "BooleanVariable.h"
#include "VariableValue.h"
#include <exception>


BooleanVariable::BooleanVariable(string name, double initProbTrue, double initProbFalse)
{
	this->variableName = name;
	addValue("true", initProbTrue);
	addValue("false", initProbFalse);
}

BooleanVariable::~BooleanVariable(void)
{
}
vector<applSharedPointer<IVariableValue> > BooleanVariable::getValues()
{
	vector<applSharedPointer<IVariableValue> > result;
	for(int i = 0; i < values.size(); i++)
	{
		result.push_back(values[i]);
	}
	return result;
}

void  BooleanVariable::addValue(applSharedPointer<VariableValue> value)
{
	values.push_back(value);
}
void  BooleanVariable::addValue(string value, double initialProb)
{
	applSharedPointer<VariableValue> newValue (new VariableValue(this->getVariableName(), value, values.size(), initialProb));
	addValue(newValue);
}
int BooleanVariable::getNumValues()
{
	return values.size();
}
string BooleanVariable::getVariableName()
{
	return variableName;
}

vector<applSharedPointer<IVariableValue> > BooleanVariable::getInitialValues()
{
	vector<applSharedPointer<IVariableValue> > result;
	for(int i = 0; i < values.size(); i++)
	{
		if(values[i]->getProb() > 0.0001)
		{
			result.push_back(values[i]);
		}
	}
	return result;
}

applSharedPointer<IVariableValue> BooleanVariable::getTrueValue()
{
	return values[0];
}

applSharedPointer<IVariableValue> BooleanVariable::getFalseValue()
{
	return values[1];
}
applSharedPointer<IVariableValue> BooleanVariable::getValueByName(string valName)
{
	for(int i = 0; i < values.size(); i++)
	{
		if(values[i]->getValueName().compare(valName) == 0)
		{
			return values[i];
		}
	}

	throw runtime_error("Cannot find value : " + valName + " in variable : " + this->getVariableName());
}

