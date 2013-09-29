/********** tell emacs we use -*- c++ -*- style comments *******************
  $Revision: 2.10 $  $Author: duyanzhu $  $Date: 2008/06/14 01:41:17 $

  @file    POMDP.h
  @brief   Describes data structure of POMDP object which defines the
  @	  pomdp problems

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

#ifndef OLDPOMDP_H
#define OLDPOMDP_H

#include <iostream>
#include <string>
#include <vector>
#include <string.h>
#include <errno.h>

#include "Const.h"
#include "MDP.h"
#include "BeliefWithState.h"

namespace momdp {

    class oldPOMDP : public MDP 
    {
        public:
            int numStates, numObservations;
            std::string fileName;//added Xan 12082007

            // initialBelief(s)
            SparseVector initialBelief;
            // R(s,a)
            SparseMatrix R;
            // T[a](s,s'), Ttr[a](s',s), O[a](s',o)
            std::vector<SparseMatrix> T, Ttr, O, Otr;
            //std::vector<bool> isPOMDPTerminalState;
            std::vector<int> isPOMDPTerminalState;

            oldPOMDP(void) {}
            oldPOMDP(const std::string& fileName, bool useFastParser = false);

            ~oldPOMDP(void) {}

            void readFromFile(const std::string& fileName,
                    bool useFastParser = false);

            /** added on 29th June ajain
             * @brief this calculates the list of possible observations
             * 
             * @param result the plausible observations being returned
             * @param resultProbs the probability of witnessing an observation
             * @param bel current belief 
             * @param act action taken to reach the belief
             */
            void getPossibleObservations(std::vector<int>& result, std::vector<double>& resultProbs, 
                    const belief_vector& bel, int act) const;

            /** added on 20th june ajain
             * @brief this gets the list of possible actions which can be taken 
             * 
             * @param result the list of action ids
             * @param bel current belief vector
             */
            void getPossibleActions(std::vector<int>& result, const belief_vector bel);

            //*******added on 30/03/2007 rn
            //	for the access methods
            //returns the transition matrix for action a
            const SparseMatrix& getTransitionMatrix(int a) const ;

            //returns the transposed transition matrix for action a
            const SparseMatrix& getTransposedTransitionMatrix(int a) const;

            //returns the observation matrix for action a
            const SparseMatrix& getObservationMatrix(int a) const;

            //returns the transposed observation matrix for action a
            const SparseMatrix& getTransposedObservationMatrix(int a) const;

            //returns the reward matrix
            const  SparseMatrix& getRewardMatrix() const;

            //returns the max reward value 24/04/2007
            const double getMaxReward()const;
            //*******end added

            // returns the initial belief
            const belief_vector& getInitialBelief(void) const;

            // sets result to be the vector of observation probabilities when from
            // belief b action a is selected
            obs_prob_vector& getObsProbVector(obs_prob_vector& result, const belief_vector& b,
                    int a) const;

            // sets result to be the next belief when from belief b action a is
            // selected and observation o is observed
            belief_vector& getNextBelief(belief_vector& result, const belief_vector& b,
                    int a, int o) const;

            // returns the expected immediate reward when from belief b action a is selected
            double getReward(const belief_vector& b, int a) const;

            AbstractBound* newLowerBound(void) const;
            AbstractBound* newUpperBound(void) const;

            // oldPOMDP-as-belief-MDP aliases for functions implemented in MDP
            int getBeliefSize(void) { return getNumStateDimensions(); }
            int getNumObservations(void) const { return numObservations; }
            void setBeliefSize(int beliefSize) { numStateDimensions = beliefSize; }
            void setNumObservations(int _numObservations) { numObservations = _numObservations; }

            // oldPOMDP-as-belief-MDP implementations for virtual functions declared in MDP
            const state_vector& getInitialState(void) const { return getInitialBelief(); }
            bool getIsTerminalState(const state_vector& s) const;
            outcome_prob_vector& getOutcomeProbVector(outcome_prob_vector& result, const state_vector& b,
                    int a) const
            { return getObsProbVector(result,b,a); }
            state_vector& getNextState(state_vector& result, const state_vector& s,
                    int a, int o) const
            { return getNextBelief(result,s,a,o); }

		bool getIsTerminalState(const BeliefWithState&) const;
		outcome_prob_vector& getOutcomeProbVector(outcome_prob_vector&, const BeliefWithState&, int, int) const;
		BeliefWithState& getNextState(BeliefWithState&, const BeliefWithState&, int, int, int) const;
		double getReward(const BeliefWithState&, int) const;



        protected:
            void readFromFileCassandra(const std::string& fileName);
            void readFromFileFast(const std::string& fileName);

            void debugDensity(void);
    };

}; // namespace momdp

#endif // INCPOMDP_h


