#ifndef BooleanVariable_H
#define BooleanVariable_H


#include <vector>
#include <string>
#include "MOMDP.h"
#include "MObject.h"
#include "IVariableValue.h"
#include "VariableValue.h"
#include "IVariable.h"
using namespace std;
using namespace momdp;

namespace momdp
{


	// Boolean Variable
	class BooleanVariable : public IVariable
	{
	private:
		string variableName;
		vector<applSharedPointer<VariableValue> > values;
		virtual void addValue(applSharedPointer<VariableValue> value);
		virtual void addValue(string value, double initialProb = 0.0);

	public:
		static const int TrueValueIndex = 0;
		static const int FalseValueIndex = 1;
		BooleanVariable(string name,double initProbTrue, double initProbFalse);
		virtual ~BooleanVariable(void);
		virtual applSharedPointer<IVariableValue> getValueByName(string valName);
		virtual applSharedPointer<IVariableValue> getTrueValue();
		virtual applSharedPointer<IVariableValue> getFalseValue();
		virtual string getVariableName();
		virtual vector<applSharedPointer<IVariableValue> > getInitialValues();
		virtual int getNumValues();
		virtual vector<applSharedPointer<IVariableValue> > getValues();
	};

}

#endif

