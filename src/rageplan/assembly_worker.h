/*
 * Assembly problem, Simulation of an INCORAP-based worker
 * Implements a Markov chain or policy graph for the non-uniform random policy of an equivalent MDP
 * 
 * Task is to assemble a number of objects (e.g. toy trucks) using a series of parts and finishing them with a specific type of glue. Multiple truck types require different parts. The correspnding assistant must maintin all parts in stock so the worker does not have to wait and must bring the correct glue to finish assembling the truck.
 * 
 * Juan Carlos Saborio,
 * DFKI NI (2023)
*/

#ifndef ASSEMBLY_WORKER_H
#define ASSEMBLY_WORKER_H

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

//TODO inherit from WORKER_PARAMS

    
struct ASSEMBLY_PARAMS{
    int n_objs;
	int n_types;
	int n_parts;
	double expertise = 0.75;
	
	//Add possible specific configurations?
	vector<string> objects_str, parts_str;
	vector<int> part_priority, types;
	vector<double> part_cost;
	vector<vector<int>> type_map;
	vector<int> storage;
	vector<bool> needsGlue;
	
	//Setup MWE
	void ASSEMBLY_MWE(){
		objects_str = {"Blue Truck", "Red Truck"};
		types = {0,1};
		parts_str = {"Chassis", "Wheels", "Blue cabin", "Yellow cabin", "Red cabin", "Container"};
		part_priority = {3,2,2,2,2,3};
		part_cost = {0,0,0,0,0,0}; //{2,2,2,0,2,1};
		storage = {1,2,2,0,2,1};
		
		// Truck parts mapping
		vector<int> blue1 = {0,2,1,5};
		type_map.push_back(blue1);
		vector<int> red = {0,4,1,5};
		type_map.push_back(red);
		
		n_objs = objects_str.size();
		n_types = types.size();
		n_parts = parts_str.size();
	}

	void ASSEMBLY_INCORAP(){
		objects_str = {"Yellow Truck", "Blue Truck"};
		types = {0,1};
		parts_str = {"Front Chassis", "Rear Chassis", "Cabin", "Blue Trailer", "Blue Lid", "Yellow Trailer", "Yellow Lid"};
		part_priority = {1,1,1,1,1,1,1};
		part_cost = {0,0,0,0,0,0,0};
		storage = {2,2,1,1,1,1,1};

		//Truck parts mapping:
		vector<int> yellow = {0,1,2,5,6};
		vector<int> blue = {0,1,2,3,4};		
		type_map.push_back(yellow);
		type_map.push_back(blue);

		n_objs = objects_str.size();
		n_types = types.size();
		n_parts = parts_str.size();
	}
};

// Each part belongs to one storage element
struct PART{
	string name = "";
	int number;
	int priority;
	bool assembled = false;
};

struct TRUCK{
	string name = "";
	int type;
	bool needsGlue = true;
	bool complete = false;
	vector<PART> parts;
	double percentageComplete = 0;
};

class ASSEMBLY_STATE{
	public:
		vector<TRUCK> trucks; //The set of trucks to build
				
		int activity;
		int pose;
		int outcome;
		
		int truck; //The current truck/task
		//int currentPart; //-1 = nothing, 0...n_parts = part type
		int glueType;

		ASSEMBLY_STATE(){}
		
		~ASSEMBLY_STATE(){
			for(auto& t : trucks)
				t.parts.clear();
			trucks.clear();
		}
		
		void copy(const ASSEMBLY_STATE& s){
			activity = s.activity;
			pose = s.pose;
			outcome = s.outcome;
			
			truck = s.truck;
			glueType = s.glueType;
			
			trucks = s.trucks;
		}
};

class ASSEMBLY_WORKER{

		private:
			friend class ASSEMBLY_ROBOT;
			
			//Problem definition params
			vector<string> objects_str, parts_str;
			vector<int> part_priority, types;
			vector<vector<int>> type_map;
			vector<bool> needsGlue;
			
			int N_OBJECTS, N_PARTS, N_TYPES;
			
			double expertise; //Worker expertise 0-1
			
			ASSEMBLY_STATE currentState;
			ASSEMBLY_STATE startState; //Use previously created start state for consistency
			
			double Bernoulli(double p) const;
			void createMWE();
		
		public:
			ASSEMBLY_WORKER();
			ASSEMBLY_WORKER(ASSEMBLY_PARAMS& params);
			
			void setProblem(ASSEMBLY_PARAMS& params);
			
			ASSEMBLY_STATE* createStartState() const;
			bool Step(ASSEMBLY_STATE& state) const;
			void generateOutcomes(ASSEMBLY_STATE& state, vector<int>& storage, double& reward) const;
			
			const ASSEMBLY_STATE getCurrentState();

			void DisplayCurrentState();
			void DisplayState(const ASSEMBLY_STATE& state, std::ostream& ostr) const;
			void DisplayStateRobot(const ASSEMBLY_STATE& state, std::ostream& ostr) const;
			void setStartState(const ASSEMBLY_STATE state) const;
			
			//Interaction with robot assistant
			int restockPart(ASSEMBLY_STATE& state, int part, int amount) const;
			int bringGlue(ASSEMBLY_STATE& state, int type) const;
			void getProblemDescription(vector<string>& parts, vector<string>& activities, vector<string>& poses, vector<string>& outcomes);
			void pickPart(ASSEMBLY_STATE& state, int part);
			
		protected:
			//List of variables specific to this problem layout
			int DEFAULT_GLUE = -1; //INCORAP DEMO = yellow = none

			//ACTIVITIES
			enum{
				A_NONE,
				A_WAIT,
				A_ASSEMBLE,
				A_GLUE
			};
			
			//POSES - element i = i - P_ELEMENT
			int P_NONE, P_GLUE, P_PART;
			
			//OUTCOMES
			enum{
				O_FAIL,
				O_OK
			};

};

#endif
