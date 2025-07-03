/*
 * Maintenance problem, Simulation of an INCORAP-inspired worker
 * Implements a Markov chain or policy graph for the non-uniform random policy of an equivalent MDP
 * 
 * Juan Carlos Saborio,
 * DFKI NI (2023)
*/

#ifndef MAINTENANCE_WORKER_H
#define MAINTENANCE_WORKER_H

#include <cstring>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>

#include "utils.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using UTILS::Random;

struct MAINTENANCE_PARAMS{
    vector<string> elements, parts;
	vector<int> element_priority;
	vector<double> part_costs;
	double expertise = 0.75;
};

struct PART{
	string name;
	double cost;
};

// An element has a priority and may need to be replaced with a given part
struct ELEMENT{
	string name = "";
	int priority;
	int part;
	bool OK;
	bool inspected;
	
	// Prob estimation... Would need to turn into POMDP with belief state
	double likelihoodOK;
	double likelihoodNOTOK;
	double probabilityOK;
};

class MAINTENANCE_STATE{
	public:
		vector<PART> parts;
		vector<ELEMENT> elements;
		int activity;
		int pose;
		int outcome;
		int partAvailable; //-1 = nothing, 0...n_parts = part type
		
		MAINTENANCE_STATE(){}
		
		~MAINTENANCE_STATE(){
			parts.clear();
			elements.clear();
		}
				
		MAINTENANCE_STATE(const MAINTENANCE_STATE& ws){
			activity = ws.activity;
			pose = ws.pose;
			outcome = ws.outcome;
			partAvailable = ws.partAvailable;
			
			parts = ws.parts;
			elements = ws.elements;
		}
		
		void copy(const MAINTENANCE_STATE& ws){
			activity = ws.activity;
			pose = ws.pose;
			outcome = ws.outcome;
			partAvailable = ws.partAvailable;
			
			parts = ws.parts;
			elements = ws.elements;
		}
};

class MAINTENANCE_WORKER{

		private:
			friend class MAINTENANCE_ASSISTANT;
			vector<string> parts_str, elements_str;
			vector<int> element_priority;
			vector<double> part_costs;
			int N_PARTS, N_ELEMENTS;
			
			double expertise; //Worker expertise 0-1
			
			MAINTENANCE_STATE currentState;
			MAINTENANCE_STATE startState; //Use previously created start state for consistency
			
			double Bernoulli(double p) const;
			void createMWE();
		
		public:
			MAINTENANCE_WORKER();
			MAINTENANCE_WORKER(MAINTENANCE_PARAMS& params);
			
			void setProblem(MAINTENANCE_PARAMS& params);
			
			MAINTENANCE_STATE* createStartState();
			bool Step(MAINTENANCE_STATE& state);
			bool Step(int& activity, int& pose, int& outcome, vector<PART>& parts, vector<ELEMENT>& elements) const;
			
			void generateOutcomes(MAINTENANCE_STATE& state);
			void generateOutcomes(int& activity, int& pose, int& outcome, vector<PART>& parts, vector<ELEMENT>& elements) const;
			
			const MAINTENANCE_STATE getCurrentState();

			void DisplayCurrentState();
			void DisplayState(const MAINTENANCE_STATE& state, std::ostream& ostr) const;
			void setStartState(const MAINTENANCE_STATE state) const;
			
			//Interaction with robot assistant
			void getProblemDescription(vector<string>& parts, vector<string>& activities, vector<string>& poses, vector<string>& elements);
			void makePartAvailable(MAINTENANCE_STATE& state, int part);
			
		protected:
			//List of variables specific to this problem layout
			
			//ACTIVITIES
			enum{
				A_NONE,
				A_INSPECT,
				A_REPLACE,
				A_RECEIVE
			};
			
			//POSES - element i = i - P_ELEMENT
			enum{
				P_NONE,
				P_ELEMENT
			};
			
			//OUTCOMES
			enum{
				O_NOT_OK,
				O_OK,
				O_FAIL
			};
			
			// REMOVE - OBJECTS
			enum{
				O_SCREW,
				O_MULTI,
				O_PART
			};
			
};

#endif
