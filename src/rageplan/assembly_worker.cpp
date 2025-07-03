#include "assembly_worker.h"
#include <iomanip>

/*
	MWE/default constructor
*/
ASSEMBLY_WORKER::ASSEMBLY_WORKER(){}

/*
	Main constructor. Receives parameter struct and initializes everything.
 */
ASSEMBLY_WORKER::ASSEMBLY_WORKER(ASSEMBLY_PARAMS& params){
	setProblem(params);
}

//Set all problem variables from "params"
void ASSEMBLY_WORKER::setProblem(ASSEMBLY_PARAMS& params){	
	N_OBJECTS = params.n_objs;
	N_TYPES = params.n_types;
	N_PARTS = params.n_parts;
	expertise = params.expertise;
	
	this->objects_str = params.objects_str;
    this->types = params.types;
	this->parts_str = params.parts_str;
	this->part_priority = params.part_priority;	
	this->type_map = params.type_map;
	this->needsGlue = params.needsGlue;
	
	//Initialize pose markers
	P_NONE = 0;
	P_GLUE = P_NONE + 1;
	P_PART = P_GLUE + N_TYPES;
}

void ASSEMBLY_WORKER::getProblemDescription(vector<string>& parts, vector<string>& activities, vector<string>& poses, vector<string>& outcomes){
	parts = this->parts_str;
	activities = {"NONE", "WAIT", "ASSEMBLE", "GLUE"};
	outcomes = {"FAIL","OK"};
	
	//List all poses, begin with NONE and then list glue tpyes and parts
	poses = {"NONE"};
	for(int i=0; i<N_TYPES; i++)
		poses.push_back("GLUE " + std::to_string(i));
	for(int i=0; i<N_PARTS; i++){
	//	poses.push_back("PART " + std::to_string(i));
		poses.push_back(parts_str[i]);
	}
	
	
}
   
void ASSEMBLY_WORKER::createMWE(){
    this->objects_str = {"Blue Truck", "Yellow Truck", "Red Truck", "Blue Truck"};
    this->types = {0,1,2,0};
	this->parts_str = {"Cabin", "Wheels", "Blue chassis", "Yellow chassis", "Red chassis"};
	this->part_priority = {3,3,2,2,2};
	// Truck parts mapping
	vector<int> blue1 = {0,1,2};
	type_map.push_back(blue1);
	vector<int> yellow = {0,1,3};
	type_map.push_back(yellow);
	vector<int> red = {0,1,4};
	type_map.push_back(red);
	vector<int> blue2 = {0,1,2};
	type_map.push_back(blue2);	
	
	this->expertise = 0.95;
	
	N_OBJECTS = objects_str.size();
	N_TYPES = 3;
	N_PARTS = parts_str.size();
	//srand(time(NULL));
		
	//currentState.copy(createStartState());
}

ASSEMBLY_STATE* ASSEMBLY_WORKER::createStartState() const{
	ASSEMBLY_STATE * s = new ASSEMBLY_STATE();
		
	s->activity = A_NONE; //Start in "none"
	s->pose = P_NONE; //Start in pose "none"
	s->outcome = O_OK;	
	s->glueType = DEFAULT_GLUE;
	
	s->truck = 0; //Start with the first truck?
	
	//Initialize trucks/objects vector
	for(int i=0; i<N_OBJECTS; i++){
		TRUCK o;
		
		o.name = objects_str[i];
		o.type = Random(N_TYPES); //types[i];
		o.complete = false; //default value already
		o.needsGlue = needsGlue[i];
		
		//Iterate over each object part
		for(auto j : type_map[i]){
			PART p;
			p.name = parts_str[j];
			p.number = j;
			p.priority = part_priority[j];
			p.assembled = false;
			
			//Add part to object
			o.parts.push_back(p);
		}
		
		s->trucks.push_back(o);
	}

	return s;
}

double ASSEMBLY_WORKER::Bernoulli(double p) const{    
    return rand() < p * RAND_MAX;
}

//Step and outcome functions w/o state
bool ASSEMBLY_WORKER::Step(ASSEMBLY_STATE& state) const{
	bool terminal = true;
	
	//See if terminal conditions are met
	for(auto t : state.trucks){
		//If ALL are complete, problem is solved
		terminal &= (t.complete);
	}
	
	if(terminal){
		state.activity = A_NONE;
		state.pose = P_NONE;
		return true;
	}
	
	if(state.activity == A_NONE){
		state.pose = P_NONE;
		
		//Look at parts of current object/truck IN ORDER
		for(auto p : state.trucks[state.truck].parts){
			//Choose first unassembled part
			if(!p.assembled){
				state.activity = A_ASSEMBLE;
				state.pose = P_PART + p.number;
				break;
			}
		}
		
		//If all parts are already assembled
		if(state.pose == P_NONE){
			//If truck needs glue, use it
			if(state.trucks[state.truck].needsGlue){
				state.activity = A_GLUE;
				//Attempt to use the corresponding type of glue
				state.pose = P_GLUE + state.trucks[state.truck].type;
			}
			//If truck does not need glue, we're done
			else{
				state.trucks[state.truck].complete = true;
				if(state.truck < N_OBJECTS - 1){						
					state.truck++; //Move on to the next truck			
				}
				state.activity = A_NONE;
			}
		}

		return false;
	}
	
	if(state.activity == A_ASSEMBLE){
		if(state.outcome == O_OK){
			state.activity = A_NONE;
		}
		else{
			state.activity = A_WAIT;
		}
		
		return false;
	}
	
	if(state.activity == A_WAIT){
		state.activity = A_NONE;
		
		return false;
	}
	
	if(state.activity == A_GLUE){
		if(state.outcome == O_OK){
			state.activity = A_NONE;
			state.pose = P_NONE;
			
			if(state.trucks[state.truck].complete && state.truck < N_OBJECTS - 1)
				state.truck++; //Move on to the next truck
		}
		
		if(state.outcome == O_FAIL){
			state.activity = A_WAIT;
		}
		
		return false;
	}
		
	return false;
}

void ASSEMBLY_WORKER::generateOutcomes(ASSEMBLY_STATE& state, vector<int>& storage, double& reward) const
{
	
	int reward_part_missing = -5;
	int reward_wrong_glue = -5;
	int reward_good_glue = 5;
	int reward_wait = -2;
	
	state.outcome = O_OK;
	
	if(state.activity == A_WAIT){
		reward = reward_wait;
	}
	
	if(state.activity == A_ASSEMBLE){
		int part = state.pose - P_PART;
		//If part is available, take one successfully
		if(storage[part]){
			storage[part]--;

			//Find corresponding part and update
			for(auto& p : state.trucks[state.truck].parts){
				if(p.number == part){
					p.assembled = true;
					break;
				}
			}
			
			state.trucks[state.truck].percentageComplete += (100.0 / state.trucks[state.truck].parts.size());
			state.outcome = O_OK;
		}
		//Otherwise assemble fails
		else{
			state.outcome = O_FAIL;
			reward = reward_part_missing;
		}
	}
	
	if(state.activity == A_GLUE){
		int glue = state.pose - P_GLUE;
		//If glue needed is available and required
		if(state.glueType == glue && state.trucks[state.truck].needsGlue){
			state.outcome = O_OK;
			reward = reward_good_glue;
			
			state.glueType = DEFAULT_GLUE; //TODO: Glue is used up?
			
			//Determine if truck is complete
			bool assembled = true;
			for(auto p : state.trucks[state.truck].parts){
				//If ALL are assembled, truck is complete
				assembled &= p.assembled;
			}
			//If ALL assembled and glued (outcome == OK), truck is complete
			if(assembled) state.trucks[state.truck].complete = true;
		}
		else{
			state.outcome = O_FAIL;
			reward = reward_wrong_glue;
		}
	}

}

const ASSEMBLY_STATE ASSEMBLY_WORKER::getCurrentState(){
	return currentState;
}

void ASSEMBLY_WORKER::DisplayCurrentState(){
	DisplayState(currentState, cout);
}

/*void ASSEMBLY_WORKER::DisplayState(const ASSEMBLY_STATE& state, std::ostream& ostr) const{
	ostr << "<Activity: ";
	switch(state.activity){
		case A_NONE:
			ostr << "NONE";
			break;
		case A_WAIT:
			ostr << "WAIT";
			break;
		case A_ASSEMBLE:
			ostr << "ASSEMBLE";
			break;
		case A_GLUE:
			ostr << "GLUE";
			break;
	}
	ostr << " | ";

	ostr << "Pose: ";
	
	if(state.pose >= P_PART) ostr << "Part " << (state.pose - P_PART);
	else ostr << "none";
	ostr << " | ";
	
	ostr << "Outcome: ";
	switch(state.outcome){		
		case O_OK:
			ostr << "OK";
			break;		
		case O_FAIL:
			ostr << "FAILED";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "Working on truck no. " << state.truck << endl;
	ostr << "Current glue type: " << state.glueType << endl;
	
	ostr << endl;
	int count = 0;
	bool names = false;
	if(parts_str.size() == N_PARTS) names = true;
	ostr << "Containers:" <<endl;
		ostr << "\t";
		ostr << std::left << std::setw(8) << "Part No." << " | ";
		ostr << std::left << std::setw(16) << "Name" << " | ";
		ostr << std::left << std::setw(8) << "Quantity" << " | ";
		ostr << endl;
		
	for(auto p : state.storage){
		ostr << "\t";
		ostr << std::left << std::setw(8) << count << " | ";
		if(names)
			ostr << std::left << std::setw(16) << parts_str[count];
		else
			ostr << std::left << std::setw(6) << "Part " << count;
		ostr << " | ";
		//the actual amount
		ostr << std::left << std::setw(8) << p << " | " << endl;
		count++;
	}
	
	ostr << endl;
	
	ostr << "Truck list: " << endl;
	count = 0;
	ostr << "\t[";
	ostr << std::left << std::setw(3) << "No." << " | ";
	if(objects_str.size() == N_OBJECTS) ostr << std::left << std::setw(12) << "Description" << " | ";
	ostr << std::left << std::setw(4) << "Type" << " | ";
	ostr << std::left << std::setw(10) << "Complete?" << " | ";
	ostr << std::left << std::setw(16) << "Parts";
	ostr << "]" << endl;
	
	std::stringstream ss;
	
	for(auto e : state.trucks){
		ostr << "\t[";
		ostr << std::left << std::setw(3) << count++ << " | ";
		if(e.name != "") ostr << std::left << std::setw(12) << e.name << " | ";
		ostr << std::left << std::setw(4) << e.type << " | ";
		ostr << std::left << std::setw(10) << (e.complete? "Y" : "N") << " | ";
		
		ss.str("");
		for(auto p : e.parts){
			if(p.assembled)
				ss << "{" << p.number << "}";
			else
				ss << "(" << p.number << ")";
		}
		
		ostr << std::left << std::setw(16) << ss.str();
		ostr << "]" << endl;
	}
	
}*/

void ASSEMBLY_WORKER::DisplayState(const ASSEMBLY_STATE& state, std::ostream& ostr) const{
	ostr << "<Activity: ";
	switch(state.activity){
		case A_NONE:
			ostr << "NONE";
			break;
		case A_WAIT:
			ostr << "WAIT";
			break;
		case A_ASSEMBLE:
			ostr << "ASSEMBLE";
			break;
		case A_GLUE:
			ostr << "GLUE";
			break;
	}
	ostr << " | ";

	ostr << "Pose: ";
		
	if(state.pose >= P_PART) ostr << "Part " << (state.pose - P_PART);
	else if(state.pose >= P_GLUE) ostr << "Glue " << (state.pose - P_GLUE);
	else ostr << "none";
	ostr << " | ";
	
	ostr << "Outcome: ";
	switch(state.outcome){
		case O_OK:
			ostr << "OK";
			break;		
		case O_FAIL:
			ostr << "FAILED";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "Working on truck no. " << state.truck << ", " << state.trucks[state.truck].percentageComplete << " \% complete." << endl;
	ostr << "Current glue type: ";
	
	if(state.glueType == DEFAULT_GLUE)
		ostr << "none";
	else 
		ostr << state.glueType;
	ostr << endl;
			
	ostr << endl;
	
	ostr << "Truck list: " << endl;
	int count = 0;
	ostr << "\t[";
	ostr << std::left << std::setw(3) << "No." << " | ";
	if(objects_str.size() == N_OBJECTS) ostr << std::left << std::setw(12) << "Description" << " | ";
	ostr << std::left << std::setw(4) << "Type" << " | ";
	ostr << std::left << std::setw(5) << "Glue?" << " | ";
	ostr << std::left << std::setw(10) << "Complete?" << " | ";
	ostr << std::left << std::setw(16) << "Parts";
	ostr << "]" << endl;
	
	std::stringstream ss;
	
	for(auto e : state.trucks){
		ostr << "\t[";
		ostr << std::left << std::setw(3) << count++ << " | ";
		if(e.name != "") ostr << std::left << std::setw(12) << e.name << " | ";
		ostr << std::left << std::setw(4) << e.type << " | ";
		ostr << std::left << std::setw(5) << (e.needsGlue? "Y" : "N") << " | ";
		ostr << std::left << std::setw(10) << (e.complete? "Y" : "N") << " | ";
		
		ss.str("");
		for(auto p : e.parts){
			if(p.assembled)
				ss << "{" << p.number << "}";
			else
				ss << "(" << p.number << ")";
		}
		
		ostr << std::left << std::setw(16) << ss.str();
		ostr << "]" << endl;
	}
	
}

/*** POMDP interface ***/
//TODO: Can glue fail or not be accepted?
int ASSEMBLY_WORKER::bringGlue(ASSEMBLY_STATE& state, int type) const{
	if(type >= 0 && type < N_TYPES)
		state.glueType = type;
	
	return 1;
}
