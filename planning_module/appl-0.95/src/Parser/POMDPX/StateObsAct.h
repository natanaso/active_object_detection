// written by png shao wei
// StateObsAct is for base class
// ObsAct is for Observation and Action
// State inherits StateObsAct and it has additional previous and current values

#ifndef StateObsAct_H
#define StateObsAct_H

#include <iostream>
#include <vector>
#include <iterator>
using namespace std;

class StateObsAct {

    protected:
        vector<string> valueEnum;

    public:
        void print();
        void setValueEnum(vector<string> ve);
        vector<string> getValueEnum() const;
        const bool containsInstance(string instanceName);
};

#endif


