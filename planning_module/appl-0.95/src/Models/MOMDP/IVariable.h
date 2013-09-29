#ifndef IVariable_H
#define IVariable_H


#include <vector>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
using namespace std;
using namespace momdp;
namespace momdp
{
	// pure interface for Variable
	class IVariable : public MObject
	{
	public:
		IVariable(void);
		virtual ~IVariable(void);

		virtual int getNumValues() = 0;
		virtual applSharedPointer<IVariableValue> getValueByName(string valName) = 0;

		virtual vector<applSharedPointer<IVariableValue> > getInitialValues() = 0;

		virtual string getVariableName() = 0;
		virtual vector<applSharedPointer<IVariableValue> > getValues() = 0;
	};
}

#endif

