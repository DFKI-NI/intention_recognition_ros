/*
 * Insect Hotel worker
 * Implements a Markov chain or policy graph for the non-uniform random policy of an equivalent MDP
 * 
 * Task is to assemble one of two possible types of insect hotel, by randomly selecting one part at a time from a stash. Each hotel has some unique parts determined by type.
 *  
 * Juan Carlos Saborio,
 * PBR @ DFKI-NI (2023)
 * Joint Lab KI & DS, UOS (2025)
*/

#ifndef HOTEL_WORKER_H
#define HOTEL_WORKER_H

#include <cstring>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "utils.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using UTILS::Random;

//TODO inherit from WORKER_PARAMS

    
struct HOTEL_PARAMS{
    int n_objs = 1; //No. of hotels to assemble
	int n_types = 2;
	int n_parts; //Total parts in storage regardless of hotel type
	double expertise = 0.75;
	
	//Add possible specific configurations?
	vector<string> objects_str, parts_str;
	vector<int> part_priority, types;
	vector<double> part_cost;
	vector<vector<int>> type_map;
	vector<int> storage;
	
	//Setup MWE
	void HOTEL_AIDEMO(){
		//objects_str = {"Black Hotel", "White Hotel"};
		objects_str = {"Insect Hotel"};
		types = {0}; //list of types, not number
		//parts_str = {"Bright green", "Dark Green", "Magenta", "Purple", "Red", "Yellow", "Black", "Orange"};
		parts_str = {"Green", "Purple", "Orange", "Black", "Yellow"};
		part_priority = {1,1,1,1,1};
		part_cost = {0,0,0,0,0};
		storage = {1,1,1,1,1,1};

		//Hotel definitions, i.e. object-parts mapping:
		vector<int> hotelA = {0,1,2,3};
		vector<int> hotelB = {0,2,3,4};
		type_map.push_back(hotelA);
		type_map.push_back(hotelB);

		n_objs = objects_str.size();
		n_parts = parts_str.size();
	}
};

// Each part belongs to one storage element
struct H_PART{
	string name = "";
	int number;
	int priority;
	bool assembled = false;
};

struct HOTEL{
	string name = "";
	int type;	
	bool complete = false;
	double percentageComplete = 0;
};

class HOTEL_STATE{
	public:
		vector<HOTEL> hotels; //List of hotels to assemble
		vector<H_PART> allParts; //List of all possible parts and their current status (name, priority, assembled?, etc.)

		int action, object, result;

		int hotelStage;
		int hotel; //The current object		

		HOTEL_STATE(){}
		
		~HOTEL_STATE(){
			allParts.clear();
			hotels.clear();
		}
		
		void copy(const HOTEL_STATE& s){
			action = s.action;
            object = s.object;
			result = s.result;
			
			hotelStage = s.hotelStage;
			
			hotel = s.hotel;						
			hotels = s.hotels;
			allParts = s.allParts;
		}
};

class HOTEL_WORKER{

		private:
			friend class HOTEL_ROBOT;
			
			//Problem definition params
			vector<string> objects_str, parts_str;
			vector<int> part_priority, types;
			vector<vector<int>> type_map; //Contains the definition of hotel types (i.e. their corresponding parts)
			vector<int> allPartNumbers; //For vector/set operations
			
			int N_OBJECTS, N_PARTS;
			int N_TYPES = 2;
			
			double expertise; //Worker expertise 0-1
			
			HOTEL_STATE currentState;
			HOTEL_STATE startState; //Use previously created start state for consistency
			
			double Bernoulli(double p) const;
			void createMWE();
		
		public:
			HOTEL_WORKER();
			HOTEL_WORKER(HOTEL_PARAMS& params);
			
			void setProblem(HOTEL_PARAMS& params);
			
			HOTEL_STATE* createStartState() const;
			bool Step(HOTEL_STATE& state) const;
			void generateOutcomes(HOTEL_STATE& state, vector<bool>& storage, double& reward) const;
			
			const HOTEL_STATE getCurrentState();

			void DisplayCurrentState();
			void DisplayState(const HOTEL_STATE& state, std::ostream& ostr) const;
			void DisplayStateRobot(const HOTEL_STATE& state, std::ostream& ostr) const;
			void setStartState(const HOTEL_STATE state) const;
			
			//Interaction with robot assistant
			int restockPart(HOTEL_STATE& state, int part, int amount) const;			
			void getProblemDescription(vector<string>& parts, vector<string>& activities, vector<string>& poses, vector<string>& outcomes);
			void pickPart(HOTEL_STATE& state, int part);
			
		protected:
			double P_WORKER_DELAY = 0.25; //Prob. of worker getting "stuck" on doing nothing
			double P_REMOVE_PART = 0.85; //Prob. of worker "noticing" and removing wrong part from hotel

			/*
				For the insect hotel we do not observe worker activities directly they help simulate worker dynamics
			*/

			//ACTIVITIES or worker actions
			enum{
				A_NONE,
				A_WAIT,
				A_ASSEMBLE,
				A_REMOVE
			};
			
			//Objects - element i = i - P_ELEMENT
			int P_NONE, P_PART;
			
			//OUTCOMES or results
			enum{
				O_FAIL,
				O_OK,
				O_DONE
			};

};

#endif
