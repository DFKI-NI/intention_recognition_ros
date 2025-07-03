/*
 * Simulation of an INCORAP worker
 * Implements a Markov chain or policy graph for the non-uniform random policy of an equivalent MDP
 * 
 * Juan Carlos Saborio,
 * DFKI NI (2022)
*/

#ifndef INCORAP_WORKER_H
#define INCORAP_WORKER_H

#include <cstring>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>

using std::vector;
using std::string;
using std::cout;
using std::endl;

struct WORKER_PARAMS{
    vector<string> objects, activities, poses, elements;
	vector<double> obj_costs;
	vector<vector<int>> transitionProbs;
	double expertise = 0.75;
};

struct OBJECT{
	string name;
	bool present;
};

struct ELEMENT{
	string name;
	bool OK;
	bool inspected;
};

class WORKER_STATE{
	public:
		vector<OBJECT> objects;
		vector<ELEMENT> elements;
		int activity;
		int pose;
		int outcome;
		
		WORKER_STATE(){}
		
		~WORKER_STATE(){
			objects.clear();
			elements.clear();
		}
				
		WORKER_STATE(const WORKER_STATE& ws){
			activity = ws.activity;
			pose = ws.pose;
			outcome = ws.outcome;
			
			objects = ws.objects;
			elements = ws.elements;
		}
		
		void copy(const WORKER_STATE& ws){
			activity = ws.activity;
			pose = ws.pose;
			outcome = ws.outcome;
			
			objects = ws.objects;
			elements = ws.elements;
		}
};

class INCORAP_WORKER{

		private:
			friend class INCORAPMWE;
			vector<string> objects_str, activities_str, poses_str, elements_str;
			vector<vector<int>> transitionTable;
			double expertise; //Worker expertise 0-1
			
			void createMWE();
			WORKER_STATE currentState;
			WORKER_STATE startState; //Use previously created start state for consistency
			
			double Bernoulli(double p) const;
		
		public:
			INCORAP_WORKER();
			INCORAP_WORKER(WORKER_PARAMS& params);
			
			void setProblem(WORKER_PARAMS& params);
			
			WORKER_STATE* createStartState();
			bool Step(WORKER_STATE& state);
			bool Step(int& activity, int& pose, int& outcome, vector<OBJECT>& objects, vector<ELEMENT>& elements) const;
			
			void generateOutcomes(WORKER_STATE& state);
			void generateOutcomes(int& activity, int& pose, int& outcome, vector<OBJECT>& objects, vector<ELEMENT>& elements) const;
			
			const WORKER_STATE getCurrentState();

			void DisplayCurrentState();
			void DisplayState(const WORKER_STATE& state, std::ostream& ostr) const;
			void setStartState(const WORKER_STATE state) const;
			
			//Interaction with robot assistant
			void getProblemDescription(vector<string>& objects, vector<string>& activities, vector<string>& poses, vector<string>& elements);
			bool makeObjectAvailable(WORKER_STATE& state, int object);
			
		protected:
			//List of variables specific to this problem layout	
			enum{
				O_SCREW,
				O_MULTI,
				O_PART
			};
			
			enum{
				A_NONE,
				A_INSPECT,
				A_SCREW,
				A_REPLACE,
				A_DONE
			};
			
			enum{
				P_BOARD,
				P_COMPARTMENT
			};
			
			enum{
				O_NOT_OK,
				O_OK,
				O_NONE,
				O_FAIL
			};
};

#endif
