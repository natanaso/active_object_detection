// written by png shao wei

#include "StateObsAct.h"
#include <string>
using namespace std;

vector<string> StateObsAct::getValueEnum() const {
    return valueEnum;
}

void StateObsAct::setValueEnum(vector<string> ve) {
    valueEnum = ve;
}

void StateObsAct::print() {
    for (unsigned int i = 0; i < valueEnum.size(); i++) {
        cout << "i: " << i << " " << valueEnum[i] << endl;
    }
    cout << endl;
}

const bool StateObsAct::containsInstance(string instanceName) {
    for (unsigned int i=0; i < valueEnum.size(); i++)
        if (valueEnum[i] == instanceName) return true;
    return false;
}
