#include "hotel_worker.h"
#include <iomanip>

/*
	MWE/default constructor
*/
HOTEL_WORKER::HOTEL_WORKER(){}

/*
	Main constructor. Receives parameter struct and initializes everything.
 */
HOTEL_WORKER::HOTEL_WORKER(HOTEL_PARAMS& params){
	setProblem(params);
}

//Set all problem variables from "params"
void HOTEL_WORKER::setProblem(HOTEL_PARAMS& params){
	N_OBJECTS = params.n_objs;
	//N_TYPES = params.n_types;
	N_TYPES = 2; //We will always use 2 types of hotels
	N_PARTS = params.n_parts;
	expertise = params.expertise;
	
	this->objects_str = params.objects_str;
    this->types = params.types;
	this->parts_str = params.parts_str;
	this->part_priority = params.part_priority;	
	this->type_map = params.type_map;

	for(int i=0; i<N_PARTS; i++) allPartNumbers.push_back(i);

	//Initialize pose/object markers
	P_NONE = 0;
	P_PART = P_NONE + 1;
}

void HOTEL_WORKER::getProblemDescription(vector<string>& parts, vector<string>& activities, vector<string>& poses, vector<string>& outcomes){
	parts = this->parts_str;
	activities = {"NONE", "WAIT", "ASSEMBLE", "REMOVE"};
	outcomes = {"FAIL","OK"};
	
	//List all poses/objects, begin with NONE and then parts
	poses = {"NONE"};
	for(int i=0; i<N_PARTS; i++){
		poses.push_back(parts_str[i]);
	}
}

HOTEL_STATE* HOTEL_WORKER::createStartState() const{
	HOTEL_STATE * s = new HOTEL_STATE();
		
	s->action = A_NONE; //Start in "none"
	s->object = P_NONE; //Start in pose "none"
	s->result = O_OK;	
	
	s->hotel = 0; //Initial hotel is always 0

	//Iterate over each object part
	for(int i=0; i < N_PARTS; i++){
		H_PART p;
		p.name = parts_str[i];
		p.number = i;
		p.priority = part_priority[i];
		p.assembled = false;
		
		//Add part to state list
		s->allParts.push_back(p);
	}
	
	//Initialize objects vector
	for(int i=0; i<N_OBJECTS; i++){
		HOTEL o;
		
		o.name = objects_str[i];
		o.type = Random(N_TYPES); //Type is random/unknown
		o.complete = false; //default value already
		
		o.percentageComplete = 0;
		
		s->hotels.push_back(o);
	}

	return s;
}

double HOTEL_WORKER::Bernoulli(double p) const{    
    return rand() < p * RAND_MAX;
}

//Step function takes a previous state and changes variables (action, object) to a new state
bool HOTEL_WORKER::Step(HOTEL_STATE& state) const{
	bool terminal = true;
	
	//See if terminal conditions are met

	//If all hotels are complete
	for(auto h : state.hotels){
		//If ALL are complete, problem is solved
		terminal &= (h.complete);
	}
	
	//Then we are done
	if(terminal){
		state.action = A_NONE;
		state.object = P_NONE;
		return true;
	}

	//If not done, continue with state transition...

	//Check if current hotel is fully assembled
	int hotelType = state.hotels[state.hotel].type;
	bool fullyAssembled = true;
	vector<int> hotelPartList = type_map[hotelType];		
	//Are all parts of THIS hotel assembled?
	for(auto p : hotelPartList) fullyAssembled &= state.allParts[p].assembled;

	//Check for parts NOT in this hotel -- Set diff. between hotel and ALL parts
	bool wrongParts = false;			
	vector<int> diff;
	diff.clear();
	std::set_difference(allPartNumbers.begin(), allPartNumbers.end(), hotelPartList.begin(), hotelPartList.end(), std::inserter(diff, diff.begin()));
	for(auto p : diff){
		//cout << "Diff part: " << p << ", assembled? " << state.allParts[p].assembled << endl;
		wrongParts |= state.allParts[p].assembled;
	}
	//cout << "FullyAssembled is: " << fullyAssembled << endl;

	//If all the right parts and *no wrong parts* are already assembled, hotel is complete
	if(fullyAssembled && !wrongParts){
		state.hotels[state.hotel].complete = true;
		if(state.hotel < N_OBJECTS - 1){
			state.hotel++; //Move on to the next truck			
		}
		state.action = A_NONE;
		state.object = P_NONE;
		return false;
	}
	
	//Transition from A_NONE
	if(state.action == A_NONE){
		//Transition to A_REMOVE
		if(wrongParts){
			//cout << "There are wrong parts!" << endl;

			//With p(expertise), worker removes a wrong part
			if(Bernoulli(expertise)){
				//Remove one ASSEMBLED part from diff set
				for(auto dP : diff){
					if(state.allParts[dP].assembled){
						state.action = A_REMOVE;
						state.object = P_PART + state.allParts[dP].number;
						return false;
					}
				}
			}
		}

		//Transition to A_ASSEMBLE
		state.object = P_NONE;
		
		//Worker may hang out in "none" pose with prob.
		if(Bernoulli(P_WORKER_DELAY))
			return false;

		//Select unassembled part randomly from corresponding hotel type
		//With P = 1 - expertise, choose from wrong set of parts
		if(Bernoulli(1 - expertise))
			hotelType = !hotelType;

		int numHotelParts = type_map[hotelType].size();

		bool partFound = false;		
		int trials = numHotelParts; //TODO: how many trials should we give the worker?
		do{
			int randNo = Random(numHotelParts);
			int rand_part = type_map[hotelType][randNo];
			
			if(!state.allParts[rand_part].assembled){
				partFound = true;
				state.action = A_ASSEMBLE;
				state.object = P_PART + state.allParts[rand_part].number; //Use the PART number, not its position within the array
				//cout << "Unassembled random part found: " << p.name << " " << endl;
			}
		}
		while(!partFound && trials--);

		//We can also return to the ordered assembly following the new hotel type scheme
		/*for(auto p : state.hotels[state.hotel].parts){
			//Choose first unassembled part
			if(!p.assembled){
				state.activity = A_ASSEMBLE;
				state.pose = P_PART + p.number;
				break;
			}
		}*/

		return false;
	}
	
	//Transition from A_ASSEMBLE to NONE or WAIT
	if(state.action == A_ASSEMBLE){
		if(state.result == O_OK){
			state.action = A_NONE;
		}
		else if(state.result == O_FAIL){
			state.action = A_WAIT;
		}
		
		return false;
	}
	
	// WAIT lasts one turn and worker moves to NONE
	if(state.action == A_WAIT){
		state.action = A_NONE;
		
		return false;
	}

	//Transition from REMOVE back to NONE
	if(state.action == A_REMOVE){
		state.action = A_NONE;

		return false;
	}
		
	return false;
}

//Take the current state and storage status and generate the result and reward
void HOTEL_WORKER::generateOutcomes(HOTEL_STATE& state, vector<bool>& storage, double& reward) const
{
	
	int reward_part_missing = -2;
	int reward_assemble = 2;
	int reward_hotel_complete = 5;	
	int reward_wait = 0;
	
	state.result = O_OK;

	if(state.action == A_NONE){
		if(state.hotels[state.hotel].complete){
			state.result = O_DONE;
			reward = reward_hotel_complete;		
		}
	}
	
	if(state.action == A_WAIT){
		reward = reward_wait;
	}
	
	if(state.action == A_ASSEMBLE){
		int part = state.object - P_PART;
		//If part is available, take one successfully
		if(storage[part]){
			storage[part] = false; //TODO: Empty container with e.g., p = 0.5

			//Find corresponding part and update
			state.allParts[part].assembled = true;			
			
			state.hotels[state.hotel].percentageComplete += (100.0 / type_map[state.hotels[state.hotel].type].size());
			state.result = O_OK;
			reward = reward_assemble;
		}
		//Otherwise assemble fails
		else{
			state.result = O_FAIL;
			reward = reward_part_missing;
		}
	}

	if(state.action == A_REMOVE){
		int part = state.object - P_PART;
		if(state.allParts[part].assembled){
			storage[part] = true; //Restock container

			state.allParts[part].assembled = false;
			state.hotels[state.hotel].percentageComplete -= (100.0 / type_map[state.hotels[state.hotel].type].size());
			state.result = O_OK;
		}
		else{
			state.result = O_FAIL; //Attempting to remove a part that is not assembled: Not our problem, just a dumb worker
		}

	}
}

const HOTEL_STATE HOTEL_WORKER::getCurrentState(){
	return currentState;
}

void HOTEL_WORKER::DisplayCurrentState(){
	DisplayState(currentState, cout);
}

void HOTEL_WORKER::DisplayState(const HOTEL_STATE& state, std::ostream& ostr) const{
	ostr << "<Action: ";
	switch(state.action){
		case A_NONE:
			ostr << "NONE";
			break;
		case A_WAIT:
			ostr << "WAIT";
			break;
		case A_ASSEMBLE:
			ostr << "ASSEMBLE";
			break;
		case A_REMOVE:
			ostr << "REMOVE";
			break;
	}
	ostr << " | ";

	ostr << "Object: ";
		
	if(state.object >= P_PART){
		ostr << "Part " << parts_str[(state.object - P_PART)];
	}
	else ostr << "none";
	ostr << " | ";
	
	ostr << "Result: ";
	switch(state.result){
		case O_OK:
			ostr << "OK";
			break;		
		case O_FAIL:
			ostr << "FAILED";
			break;
		case O_DONE:
			ostr << "DONE";
			break;
	}
	ostr << ">" << endl;
	
	ostr << "Working on object no. " << state.hotel << ", " << state.hotels[state.hotel].percentageComplete << " \% complete." << endl;
	
	ostr << "Object list: " << endl;
	int count = 0;
	ostr << "\t[";
	ostr << std::left << std::setw(3) << "No." << " | ";
	if(objects_str.size() == N_OBJECTS) ostr << std::left << std::setw(12) << "Description" << " | ";
	ostr << std::left << std::setw(4) << "Type" << " | ";
	//ostr << std::left << std::setw(5) << "Glue?" << " | ";
	ostr << std::left << std::setw(10) << "Complete?" << " | ";
	ostr << std::left << std::setw(16) << "Parts";
	ostr << "]" << endl;
	
	std::stringstream ss;
	
	for(auto e : state.hotels){
		ostr << "\t[";
		ostr << std::left << std::setw(3) << count++ << " | ";
		if(e.name != "") ostr << std::left << std::setw(12) << e.name << " | ";
		ostr << std::left << std::setw(4) << (e.type? "B" : "A") << " | ";
		//ostr << std::left << std::setw(5) << (e.needsGlue? "Y" : "N") << " | ";
		ostr << std::left << std::setw(10) << (e.complete? "Y" : "N") << " | ";
		
		ss.str("");
		
		for(auto p : type_map[state.hotels[state.hotel].type]){
			if(state.allParts[p].assembled)
				ss << "{" << p << "}";
			else
				ss << "(" << p << ")";
		}
		
		ostr << std::left << std::setw(16) << ss.str();
		ostr << "]" << endl;		
	}

	cout << endl;
}
