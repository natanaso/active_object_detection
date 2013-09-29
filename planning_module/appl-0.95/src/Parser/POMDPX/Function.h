// written by png shao wei
// Function is for ObservationFunction and RewardFunction

#ifndef Function_H
#define Function_H

#include <iostream>
#include <vector>
#include <iterator>
#include <map>
#include <cassert>
#include <cmath>
#include "StateObsAct.h"
#include "SparseTable.h"
#include "SparseEntry.h"

using namespace std;

class Function {

    protected:
        string vnameCurr;
        vector<string> parents;

    public:

        void setVNameCurr(string soa);
        string getVNameCurr() const;

        void setParents(vector<string> p);
        vector<string> getParents() const;

        vector<vector<double> > getDynamicArray();

        void initSparseTables(map<string, StateObsAct*>* mymap);
        void fillTables(map<string, StateObsAct*> mymap, vector<string> insttokens,
                vector<double> probttokensdouble);

        void simpleDenseInsert(map<string, StateObsAct*> mymap,
                vector<string> insttokens, string myself, double prob);
        void simpleSparseInsert(map<string, StateObsAct*> mymap,
                vector<string> insttokens, string myself, double prob);

        double simpleGet(map<string, StateObsAct*> mymap,
                vector<string> insttokens, string myself);

        vector<string> cloneVectorList(vector<string> vectorList);
        vector<double> splitTable(vector<double> probttokensdouble, int size,
                int position);

        int getPosition(vector<string> vlist, string str);
        //toberemoved
        //std::ostream& writeDense(std::ostream& out);
        //endoftoberemoved
        //////////////////////////
        // for sparse
        applSharedPointer<SparseTable> sparseT;
		Function();
        // write sparse
        std::ostream& write(std::ostream& out);

        bool checkNoMissingEntries(map<string, StateObsAct*> mymap, string& info);

};

#endif
