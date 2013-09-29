/********** tell emacs we use -*- c++ -*- style comments *******************
  $Revision: 2.10 $  $Author: duyanzhu $  $Date: 2008/06/14 01:41:16 $

  @file    oldPOMDP.cc
  @brief   No brief

  Copyright (c) 2002-2005, Trey Smith. All rights reserved.

  Licensed under the Apache License, Version 2.0 (the "License"); you may
  not use this file except in compliance with the License.  You may
  obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
implied.  See the License for the specific language governing
permissions and limitations under the License.

 ***************************************************************************/

/***************************************************************************
 * INCLUDES
 ***************************************************************************/

#include <assert.h>
#include <stdlib.h>
#ifdef _MSC_VER
#else
#include <unistd.h>
#include <sys/time.h>
#endif

#include <stdio.h>


#include <iostream>
#include <fstream>
#include <stdexcept>
#include "Const.h"
#include "pomdpCassandraWrapper.h"
#include "oldPOMDP.h"
#include "MathLib.h"
//#include "UpperBounds.h"
//#include "LowerBounds.h"

//#include "MaxPlanesLowerBound.h"
//#include "SawtoothUpperBound.h"

using namespace std;
using namespace momdp;

namespace momdp {

    /***************************************************************************
     * STATIC HELPER FUNCTIONS
     ***************************************************************************/

    static void reaDenseVector(char *data, DenseVector& b, int numValues)
    {
        int i;
        char *inp = data;
        char *tok;

        for (i=0; i < numValues; i++) 
        {
            tok = strtok(inp," ");
            if (0 == tok) {
                cout << "ERROR: not enough entries in initial belief distribution"
                    << endl;
                exit(EXIT_FAILURE);
            }
            inp = 0;

            b(i) = atof(tok);
        }
    }

    static void trimTrailingWhiteSpace(char *s)
    {
        int n = strlen(s);
        int i;
        for (i = n-1; i >= 0; i--) {
            if (!isspace(s[i])) break;
        }
        s[i+1] = '\0';
    }

    /***************************************************************************
     * oldPOMDP FUNCTIONS
     ***************************************************************************/

    oldPOMDP::oldPOMDP(const std::string& fileName, bool useFastParser)
    {
        this->fileName = fileName;//added Xan 12082007
        size_t backslash = this->fileName.rfind( "/", this->fileName.length() );
        if ( backslash != std::string::npos ) this->fileName = this->fileName.substr(backslash+1);

        readFromFile(fileName, useFastParser);
    }

    void oldPOMDP::readFromFile(const std::string& fileName,
            bool useFastParser)
    {
        if (useFastParser) {
            readFromFileFast(fileName);
        } else {
            readFromFileCassandra(fileName);
        }

        // post-process: calculate isPOMDPTerminalState
#if USE_DEBUG_PRINT
        printf("oldPOMDP::readFromFile: marking zero-reward absorbing states as terminal\n");
#endif
        isPOMDPTerminalState.resize(numStates, /* initialValue = */ true);
        FOR (s, numStates) {
            FOR (a, numActions) {
                if ((fabs(1.0 - T[a](s,s)) > OBS_IS_ZERO_EPS) || R(s,a) != 0.0) {
                    isPOMDPTerminalState[s] = false;
                    break;
                }
            }
        }
    }

    //********added on 30/03/2007 rn
    //	for implementation of the access functions
    const SparseMatrix& oldPOMDP::getTransitionMatrix(int a) const{
        return T[a];
    }//end getTransitionMatrix

    const SparseMatrix& oldPOMDP::getTransposedTransitionMatrix(int a) const{
        return Ttr[a];
    }//end getTransposedTransitionMatrix

    const SparseMatrix& oldPOMDP::getObservationMatrix(int a) const{
        return O[a];
    }//end getObservationMatrix

    const SparseMatrix& oldPOMDP::getTransposedObservationMatrix(int a) const{
        return Otr[a];
    }//end getTransposedObservationMatrix

    const SparseMatrix& oldPOMDP::getRewardMatrix() const{
        return R;
    }//end getRewardMatrix

    const double oldPOMDP::getMaxReward() const{
        double maxVal = R.data.begin()->value;
        double val;
        FOREACH(SparseVector_Entry, entry,  R.data) {
            val = entry->value;
            if(val>maxVal){
                maxVal = val;
            }
        }//end FOR_EACH
        return maxVal;
    }

    //********end added


    const belief_vector& oldPOMDP::getInitialBelief(void) const
    {
        return initialBelief;
    }

    obs_prob_vector& oldPOMDP::getObsProbVector(obs_prob_vector& result,
            const belief_vector& b,
            int a) const
    {
        DenseVector tmp, tmp2;
        // --- overall: result = O_a' * T_a' * b
        // tmp = T_a' * b
        mult( tmp, Ttr[a], b );
        // result = O_a' * tmp
        mult( tmp2, tmp, O[a] );

        copy( result, tmp2);


        return result;
    }

    // T[a](s,s'), Ttr[a](s',s), O[a](s',o)

    belief_vector& oldPOMDP::getNextBelief(belief_vector& result,
            const belief_vector& b,
            int a, int o) const
    {
        belief_vector tmp;

        // result = O_a(:,o) .* (T_a * b)
        mult( tmp, Ttr[a], b );
        emult_column( result, O[a], o, tmp );

        // renormalize
        result *= (1.0/result.norm_1());

        return result;
    }

    double oldPOMDP::getReward(const belief_vector& b, int a) const
    {
        return inner_prod_column( R, a, b );
    }

    bool oldPOMDP::getIsTerminalState(const state_vector& s) const
    {
        double nonTerminalSum = 0.0;
        FOR_CV (s) {
            if (!isPOMDPTerminalState[CV_INDEX(s)]) {
                nonTerminalSum += CV_VAL(s);
            }
        }
        return (nonTerminalSum < 1e-10);
    }

    void oldPOMDP::readFromFileCassandra(const string& fileName) {
#if USE_DEBUG_PRINT
        timeval startTime, endTime;
        cout << "reading problem from " << fileName << endl;
        gettimeofday(&startTime,0);
#endif

        PomdpCassandraWrapper p;
        p.readFromFile(fileName);

        numStates = p.getNumStates();
        setBeliefSize(numStates);
        numActions = p.getNumActions();
        numObservations = p.getNumObservations();
        discount = p.getDiscount();

        // convert R to sla format
        kmatrix Rk;
        copy(Rk, p.getRTranspose(), numStates);
        kmatrix_transpose_in_place(Rk);
        copy(R, Rk);

        // convert T, Tr, and O to sla format
        kmatrix Tk, Ok;
        T.resize(numActions);
        Ttr.resize(numActions);
        O.resize(numActions);
        Otr.resize(numActions);  
        FOR (a, numActions) {
            copy(Tk, p.getT(a), numStates);
            copy(T[a], Tk);
            kmatrix_transpose_in_place(Tk);
            copy(Ttr[a], Tk);
            //   copy(O[a], p.getO(a), numObservations);
            copy(Ok, p.getO(a), numObservations);
            copy(O[a], Ok);
            kmatrix_transpose_in_place(Ok);
            copy(Otr[a], Ok);
        }

        // convert initialBelief to sla format
        DenseVector initialBeliefD;
        initialBeliefD.resize(numStates);
        FOR (s, numStates) {
            initialBeliefD(s) = p.getInitialBelief(s);
        }
        copy(initialBelief, initialBeliefD);

#if 0
        DenseVector initialBeliefx;
        std::vector<bool> isPOMDPTerminalStatex;
        kmatrix Rx;
        std::vector<kmatrix> Tx, Ox;

        // pre-process
        initialBeliefx.resize(numStates);
        set_to_zero(initialBeliefx);
        isPOMDPTerminalStatex.resize(numStates, /* initialValue = */ false);
        Rx.resize(numStates, numActions);
        Tx.resize(numActions);
        Ox.resize(numActions);
        FOR (a, numActions) {
            Tx[a].resize(numStates, numStates);
            Ox[a].resize(numStates, numObservations);
        }

        // copy
        FOR (s, numStates) {
            initialBeliefx(s) = p.getInitialBelief(s);
            isPOMDPTerminalStatex[s] = p.isTerminalState(s);
            FOR (a, numActions) {
                kmatrix_set_entry( Rx, s, a, p.R(s,a) );
                FOR (sp, numStates) {
                    kmatrix_set_entry( Tx[a], s, sp, p.T(s,a,sp) );
                }
                FOR (o, numObservations) {
                    kmatrix_set_entry( Ox[a], s, o, p.O(s,a,o) );
                }
            }
        }

        // post-process
        copy( initialBelief, initialBeliefx );
        isPOMDPTerminalState = isPOMDPTerminalStatex;
        copy( R, Rx );
        Ttr.resize(numActions);
        O.resize(numActions);
        T.resize(numActions);
        FOR (a, numActions) {
            copy( T[a], Tx[a] );
            kmatrix_transpose_in_place( Tx[a] );
            copy( Ttr[a], Tx[a] );
            copy( O[a], Ox[a] );
        }
#endif // if 0

#if USE_DEBUG_PRINT
        gettimeofday(&endTime,0);
        double numSeconds = (endTime.tv_sec - startTime.tv_sec)
            + 1e-6 * (endTime.tv_usec - startTime.tv_usec);
        cout << "[file reading took " << numSeconds << " seconds]" << endl;

        debugDensity();
#endif
    }

    // this is functionally similar to readFromFile() but much faster.
    // the oldPOMDP file must obey a restricted syntax.
    void oldPOMDP::readFromFileFast(const std::string& fileName)
    {
        char buf[1<<20];
        int lineNumber;
        ifstream in;
        char sbuf[512];
        int numSizesSet = 0;
        bool inPreamble = true;
        char *data;

#if USE_DEBUG_PRINT
        timeval startTime, endTime;
        cout << "reading problem (in fast mode) from " << fileName << endl;
        gettimeofday(&startTime,0);
#endif

        in.open(fileName.c_str());
        if (!in) 
        {
            cerr << "ERROR: couldn't open " << fileName << " for reading: " << endl;
            exit(EXIT_FAILURE);
        }

        DenseVector initialBeliefx;
        kmatrix Rx;
        std::vector<kmatrix> Tx, Ox;

#define PM_PREFIX_MATCHES(X) \
        (0 == strncmp(buf,(X),strlen(X)))

        lineNumber = 1;
        while (!in.eof()) {
            in.getline(buf,sizeof(buf));
            if (in.fail() && !in.eof()) {
                cerr << "ERROR: readFromFileFast: line too long for buffer"
                    << " (max length " << sizeof(buf) << ")" << endl;
                exit(EXIT_FAILURE);
            }

            if ('#' == buf[0]) continue;
            trimTrailingWhiteSpace(buf);
            if ('\0' == buf[0]) continue;

            if (inPreamble) {
                if (PM_PREFIX_MATCHES("discount:")) {
                    if (1 != sscanf(buf,"discount: %lf", &discount)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in discount statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                } else if (PM_PREFIX_MATCHES("values:")) {
                    if (1 != sscanf(buf,"values: %s", sbuf)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in values statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    if (0 != strcmp(sbuf,"reward")) {
                        cerr << "ERROR: line " << lineNumber
                            << ": can only handle values of type reward"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                } else if (PM_PREFIX_MATCHES("actions:")) {
                    if (1 != sscanf(buf,"actions: %d", &numActions)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in actions statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    numSizesSet++;
                } else if (PM_PREFIX_MATCHES("observations:")) {
                    if (1 != sscanf(buf,"observations: %d", &numObservations)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in observations statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    numSizesSet++;
                } else if (PM_PREFIX_MATCHES("states:")) {
                    if (1 != sscanf(buf,"states: %d", &numStates)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in states statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    numSizesSet++;
                } else {
                    cerr << "ERROR: line " << lineNumber
                        << ": got unexpected statement type while parsing preamble"
                        << endl;
                    exit(EXIT_FAILURE);
                }

                if (3 == numSizesSet) {
                    // pre-process
                    setBeliefSize(numStates);
                    initialBeliefx.resize(numStates);
                    set_to_zero(initialBeliefx);
                    Rx.resize(numStates, numActions);
                    Tx.resize(numActions);
                    Ox.resize(numActions);
                    FOR (a, numActions) {
                        Tx[a].resize(numStates, numStates);
                        Ox[a].resize(numStates, numObservations);
                    }

                    inPreamble = false;
                }

            } else {

                if (PM_PREFIX_MATCHES("start:")) {
                    data = buf + strlen("start: ");
                    reaDenseVector(data,initialBeliefx,numStates);
                } else if (PM_PREFIX_MATCHES("R:")) {
                    int s, a;
                    double reward;
                    if (3 != sscanf(buf,"R: %d : %d : * : * %lf", &a, &s, &reward)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in R statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    kmatrix_set_entry( Rx, s, a, reward );
                } else if (PM_PREFIX_MATCHES("T:")) {
                    int s, a, sp;
                    double prob;
                    if (4 != sscanf(buf,"T: %d : %d : %d %lf", &a, &s, &sp, &prob)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in T statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    kmatrix_set_entry( Tx[a], s, sp, prob );
                } else if (PM_PREFIX_MATCHES("O:")) {
                    int s, a, o;
                    double prob;
                    if (4 != sscanf(buf,"O: %d : %d : %d %lf", &a, &s, &o, &prob)) {
                        cerr << "ERROR: line " << lineNumber
                            << ": syntax error in O statement"
                            << endl;
                        exit(EXIT_FAILURE);
                    }
                    kmatrix_set_entry( Ox[a], s, o, prob );
                } else {
                    cerr << "ERROR: line " << lineNumber
                        << ": got unexpected statement type while parsing body"
                        << endl;
                    exit(EXIT_FAILURE);
                }
            }

            lineNumber++;
        }

        in.close();

        // post-process
        copy( initialBelief, initialBeliefx );
        copy( R, Rx );
        Ttr.resize(numActions);
        O.resize(numActions);
        T.resize(numActions);
        FOR (a, numActions) {
            copy( T[a], Tx[a] );
            kmatrix_transpose_in_place( Tx[a] );
            copy( Ttr[a], Tx[a] );
            copy( O[a], Ox[a] );
        }

#if USE_DEBUG_PRINT
        gettimeofday(&endTime,0);
        double numSeconds = (endTime.tv_sec - startTime.tv_sec)
            + 1e-6 * (endTime.tv_usec - startTime.tv_usec);
        cout << "[file reading took " << numSeconds << " seconds]" << endl;
#endif

        debugDensity();
    }

    void oldPOMDP::debugDensity(void) {
        int Ttr_size = 0;
        int Ttr_filled = 0;
        int O_size = 0;
        int O_filled = 0;
        FOR (a, numActions) {
            Ttr_size += Ttr[a].size1() * Ttr[a].size2();
            O_size += O[a].size1() * O[a].size2();
            Ttr_filled += Ttr[a].filled();
            O_filled += O[a].filled();
        }
        cout << "T density = " << (((double) Ttr_filled) / Ttr_size)
            << ", O density = " << (((double) O_filled) / O_size)
            << endl;
    }

    void oldPOMDP::getPossibleObservations(vector<int>& result, vector<double>&
            resultProbs, const belief_vector& bel, int act) const{
        belief_vector obsBel;
        const SparseMatrix obsMat = Otr[act];//problem.getTransposedObservationMatrix(act);
        mult(obsBel, obsMat, bel); 

        //now to put this info into the vectors
        for(vector<SparseVector_Entry>::const_iterator iter = obsBel.data.begin(); 
                iter != obsBel.data.end(); iter++){
            result.push_back(iter->index);
            resultProbs.push_back(iter->value);
        }
    }

    void oldPOMDP::getPossibleActions(vector<int>& result, const belief_vector bel){
        int numActs = getNumActions(); 
        result.reserve(numActs);
        const vector<SparseVector_Entry>& activeIndices = bel.data;
        vector<SparseVector_Entry>::const_iterator iter;
        for(int i = 0; i< numActs; i++){
            const SparseMatrix transMat = Ttr[i];//problem.getTransposedTransitionMatrix(i);
            for(iter = activeIndices.begin(); iter != activeIndices.end(); iter++){
                if(transMat.col_starts[(iter->index)] != transMat.col_starts[(iter->index)+1]){
                    result.push_back(i);
                    break;
                }
            }
        }
    }

	bool oldPOMDP::getIsTerminalState(const BeliefWithState&) const {throw runtime_error("");}
	outcome_prob_vector& oldPOMDP::getOutcomeProbVector(outcome_prob_vector&, const BeliefWithState&, int, int) const {throw runtime_error("");}
	BeliefWithState& oldPOMDP::getNextState(BeliefWithState&, const BeliefWithState&, int, int, int) const {throw runtime_error("");}
	double oldPOMDP::getReward(const BeliefWithState&, int) const {throw runtime_error("");}


}; // namespace momdp

