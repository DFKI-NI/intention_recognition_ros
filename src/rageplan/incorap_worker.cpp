#include "incorap_worker.h"

/*
	MWE/default constructor
*/
INCORAP_WORKER::INCORAP_WORKER(){
	createMWE();
}

/*
	Main constructor. Receives parameter struct and initializes everything.
 */
INCORAP_WORKER::INCORAP_WORKER(WORKER_PARAMS& params){
	this->activities_str = params.activities;
	this->objects_str = params.objects;
	this->poses_str = params.poses;
	this->elements_str = params.elements;
}

void INCORAP_WORKER::createMWE(){
	objects_str = {"screwdriver", "multimeter", "part"};
	activities_str = {"none", "inspect", "screw/unscrew", "replace part", "done"};
	poses_str = {"board", "compartment"};
	elements_str = {"board", "compartment"};
	
	//srand(time(NULL));
	
	//currentState.copy(createStartState());
}

void INCORAP_WORKER::setProblem(WORKER_PARAMS& params){
	this->activities_str = params.activities;
	this->objects_str = params.objects;
	this->poses_str = params.poses;
	this->elements_str = params.elements;
	this->expertise = params.expertise;
}

WORKER_STATE* INCORAP_WORKER::createStartState(){
	WORKER_STATE * ws = new WORKER_STATE();
	
	ws->objects.clear();
	for(auto o : objects_str){
		OBJECT obj;
		obj.name = o;
		obj.present = false;

		ws->objects.push_back(obj);
	}
		
	ws->activity = 0; //Start in "none"
	ws->pose = 0; //Start at board I guess?
	ws->outcome = O_NONE;

	ws->elements.clear();
	for(auto e : elements_str){
		ELEMENT elem;
		elem.name = e;
		elem.OK = Bernoulli(0.5); //Random true/false
		elem.inspected = false;
		ws->elements.push_back(elem);
	}
		
	return ws;
}

double INCORAP_WORKER::Bernoulli(double p) const{    
    return rand() < p * RAND_MAX;
}

//Step and outcome functions w/o state
bool INCORAP_WORKER::Step(int& activity, int& pose, int& outcome, vector<OBJECT>& objects, vector<ELEMENT>& elements) const{
	bool terminal = true;
	
	//See if terminal conditions are met
	for(auto e : elements)
		terminal &= (e.OK && e.inspected);
	
	if(terminal){
		activity = A_DONE;
		return true;
	}
	
	//Neutral activity	
	if(activity == A_NONE){
		/// Test!!
		// Use expertise to avoid revisiting poses
		/*if(elements[P_BOARD].OK && elements[P_BOARD].inspected){
			pose = Bernoulli(expertise)? P_COMPARTMENT : P_BOARD;
		}
		else if(elements[P_COMPARTMENT].OK && elements[P_COMPARTMENT].inspected){
			pose = Bernoulli(expertise)? P_BOARD : P_COMPARTMENT;
		}*/
		
		//If nothing is checked/OK yet, then simply choose randomly
		if(Bernoulli(0.5)){
			pose = P_BOARD;
		}
		else{
			pose = P_COMPARTMENT;
		}
		
		activity = A_INSPECT;
		
		return false;
	}
	
	//All compartment functions
	if(pose == P_COMPARTMENT){
		if(activity == A_INSPECT){
			if(outcome == O_OK){
				if(Bernoulli(expertise)) activity = A_NONE; //TODO: parameter
			}
			else if(outcome == O_NOT_OK){
				if(Bernoulli(expertise)) activity = A_SCREW; //TODO: parameter
			}
			return false;
		}
		
		if(activity == A_SCREW){
			activity = A_INSPECT;
			
			return false;
		}
		
		return false;
	}
	
	//All board functions
	if(pose == P_BOARD){
		if(activity == A_INSPECT){
			if(outcome == O_OK)
				activity = A_NONE;
						
			if(outcome == O_NOT_OK)
				activity = A_REPLACE;
			
			return false;
		}
		
		if(activity == A_REPLACE){
			activity = A_INSPECT;
			
			return false;
		}
		
		return false;
	}
	
	return false;
}

void INCORAP_WORKER::generateOutcomes(int& activity, int& pose, int& outcome, vector<OBJECT>& objects, vector<ELEMENT>& elements) const{
	outcome = O_NONE; //Default outcome is none
	
//At compartment:
	if(pose == P_COMPARTMENT){
		if(activity == A_INSPECT){
			elements[P_COMPARTMENT].inspected = true;
			outcome = elements[P_COMPARTMENT].OK; //Reveal true state of element
		}
			
		if(activity == A_SCREW){
			if(objects[O_SCREW].present){
				if(Bernoulli(expertise)){ //TODO: parameter
					outcome = O_OK;
					elements[P_COMPARTMENT].OK = true; //Chance of fixing compartment
				}
			}
			else
				outcome = O_FAIL; //If no screwdriver, fail
		}
	}
	
//At board:
	if(pose == P_BOARD){
		if(activity == A_INSPECT){
			if(objects[O_MULTI].present){
				elements[P_BOARD].inspected = true;
				outcome = elements[P_BOARD].OK;
			}
			else outcome = O_FAIL;
		}
		
		if(activity == A_REPLACE){
			if(objects[O_PART].present){
				outcome = O_OK;
				elements[P_BOARD].OK = true; //If part, board is fixed
			}
			else{
				outcome = O_FAIL; //If no part, fail
			}
		}
	}
}



//Simulate transitions with given probabilities...
bool INCORAP_WORKER::Step(WORKER_STATE& state){
	return Step(state.activity, state.pose, state.outcome, state.objects, state.elements);
	
/*	bool terminal = true;
	
	//See if terminal conditions are met
	for(auto e : state.elements)
		terminal &= (e.OK & e.inspected);
	
	if(terminal){
		state.activity = A_DONE;
		return true;
	}
	
	//Neutral activity	
	if(state.activity == A_NONE){
		if(Bernoulli(0.5)){
			state.pose = P_BOARD;
		}
		else{
			state.pose = P_COMPARTMENT;
		}
		state.activity = A_INSPECT;
		
		return false;
	}
	
	//All compartment functions
	if(state.pose == P_COMPARTMENT){
		if(state.activity == A_INSPECT){
			if(state.outcome == O_OK)
				if(Bernoulli(0.7)) state.activity = A_NONE; //TODO: parameter
						
			if(state.outcome == O_NOT_OK)
				if(Bernoulli(0.7)) state.activity = A_SCREW; //TODO: parameter
			
			return false;
		}
		
		if(state.activity == A_SCREW){
			state.activity = A_INSPECT;
			
			return false;
		}
		
		return false;
	}
	
	//All board functions
	if(state.pose == P_BOARD){
		if(state.activity == A_INSPECT){
			if(state.outcome == O_OK)
				state.activity = A_NONE;
						
			if(state.outcome == O_NOT_OK)
				state.activity = A_REPLACE;
			
			return false;
		}
		
		if(state.activity == A_REPLACE){
			state.activity = A_INSPECT;
			
			return false;
		}
		
		return false;
	}
	
	return false;*/
}

//Create function to fill out outcomes
void INCORAP_WORKER::generateOutcomes(WORKER_STATE& state){
	
	generateOutcomes(state.activity, state.pose, state.outcome, state.objects, state.elements);
/*	state.outcome = O_NONE; //Default outcome is none
	
//At compartment:
	if(state.pose == P_COMPARTMENT){
		if(state.activity == A_INSPECT){
			state.elements[P_COMPARTMENT].inspected = true;
			state.outcome = state.elements[P_COMPARTMENT].OK; //Reveal true state of element
		}
			
		if(state.activity == A_SCREW){
			if(state.objects[O_SCREW].present){
				if(Bernoulli(0.8)){ //TODO: parameter
					state.outcome = O_OK;
					state.elements[P_COMPARTMENT].OK = true; //Chance of fixing compartment
				}
			}
			else
				state.outcome = O_FAIL; //If no screwdriver, fail
		}
	}
	
//At board:
	if(state.pose == P_BOARD){
		if(state.activity == A_INSPECT){
			if(state.objects[O_MULTI].present){
				state.elements[P_BOARD].inspected = true;				
				state.outcome = state.elements[P_BOARD].OK;
			}
			else state.outcome = O_FAIL;
		}
		
		if(state.activity == A_REPLACE){
			if(state.objects[O_PART].present){
				state.outcome = O_OK;
				state.elements[P_BOARD].OK = true; //If part, board is fixed
			}
			else state.outcome = O_FAIL; //If no part, fail
		}
	}*/
}

const WORKER_STATE INCORAP_WORKER::getCurrentState(){
	return currentState;
}

void INCORAP_WORKER::DisplayCurrentState(){
	DisplayState(currentState, cout);
}

void INCORAP_WORKER::DisplayState(const WORKER_STATE& state, std::ostream& ostr) const{
	ostr << "<Activity: " << activities_str[state.activity] << " | ";
	ostr << "Pose: " << poses_str[state.pose] << " | ";	
	ostr << "Outcome: ";
	switch(state.outcome){
		case O_NOT_OK:
			ostr << "Not OK";
			break;
		case O_OK:
			ostr << "OK";
			break;
		case O_NONE:
			ostr << "N/A";
			break;
		case O_FAIL:
			ostr << "Failed";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "Objects: " << endl;
	int o_c = 0;
	for(auto o : state.objects)
		ostr << "\t[" << o_c++ << "] " << o.name << " (" << (o.present ? "Y" : "N") << ")" << endl;

	ostr << "Elements: " << endl;
	for(auto e : state.elements)
		ostr << "\t" << e.name << " - " << (e.OK? "OK" : "Not OK") << " - Inspected? " << (e.inspected? "Y" : "N") << endl;
}

/*** POMDP interface ***/
bool INCORAP_WORKER::makeObjectAvailable(WORKER_STATE& state, int object){
	bool result = false;
	if(object >= 0 && object < state.objects.size()){
		state.objects[object].present = true;
		result = true;
	}
	return result;
}

void INCORAP_WORKER::getProblemDescription(vector<string>& objects, vector<string>& activities, vector<string>& poses, vector<string>& elements){
	objects.clear();
	objects = objects_str;

	activities.clear();
	activities = activities_str;

	poses.clear();
	poses = poses_str;

	elements.clear();
	elements = elements_str;
}
