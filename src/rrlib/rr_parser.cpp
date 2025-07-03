#include "rr_parser.h"

namespace RR_PARSER{
       
    bool parseCommandLine(char ** argv, int argc, string& paramFile, string& problemFile, string& domainFile, bool& use_mockup_gui){
        string param, value;
        int paramsComplete = 0;        
        for(int i=1; i<argc; i+=2){
            param = argv[i];
            
            if(argc > i+1) value = argv[i+1];
            if(param == "--help"){
                cout << "RAGE: Relevance-Aware GEnerative Planning" << endl;
                cout << "Parameters" << endl;
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--paramFile";
                cout << std::left << std::setw(100) << "Search parameters file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--problemFile";
                cout << std::left << std::setw(100) <<"Problem specification file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--domainFile";
                cout << std::left << std::setw(100) << "Domain description file" << endl;                                

                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--use_mockup_gui";
                cout << std::left << std::setw(100) << "true or false" << endl;

                exit(0);
            }
            if(param == "--about"){
                cout << "RAGEPlan Interface Lib v0.1" << endl;
                cout << "By Juan Carlos Saborio-Morales" << endl;
                cout << "2023, DFKI-NI" << endl;
                exit(0);
            }
            
            if(param == "--paramFile"){
                paramFile = value;
                paramsComplete++;
            }
            else if(param == "--problemFile"){
                problemFile = value;
                paramsComplete++;
            }
            else if(param == "--domainFile"){
                domainFile = value;
                paramsComplete++;
            }
            else if(param == "--use_mockup_gui"){
                if(value == "true")
                    use_mockup_gui = true;
                else
                    use_mockup_gui = false;
            }
            else
                cout << "Unrecognized parameter \"" << param << "\"" << endl;
        }
        
        if(paramsComplete != 3){
            cout << "Too few (or too many) parameters. Run with --help for syntax." << endl;
            return false;
        }
        else return true;
    }

    bool parseParamsFile(string inputFile, COMMAND_LINE& cl){
        std::ifstream infile(inputFile);
        string param;
        string value;

        //TODO: figure out how to specify Worker MDP parameters in file
        
        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
                
        while(infile >> param >> value){
            if(param == "problem")
                cl.problem = value;
            else if(param == "problemFile")
                cl.inputFile = value;
            else if(param == "outputFile")
                cl.outputFile = value;
            else if(param == "nSims")
                cl.simDoubles = stoi(value);
            else if(param == "timeout")
                cl.timeout = stoi(value);
            else if(param == "verbose")
                cl.verbose = stoi(value);
            else if(param == "treeKnowledge")
                cl.treeKnowledge = stoi(value);
            else if(param == "policy")
                cl.rolloutKnowledge = stoi(value);
            else if(param == "ire")
                cl.fTable = stoi(value);
            else                
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();

        return true;
    }

    bool parseAssemblyFile(ASSEMBLY_ROBOT_PARAMS& problem_params, ASSEMBLY_PARAMS& worker_params, string& domainFilename, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        //TODO: figure out how to specify Worker MDP parameters in file
        
        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "problem")
                problem_params.problem = s_value;
            else if(param == "domainFile")
                domainFilename = s_value;
            else if(param == "perceive")
                problem_params.perceive = stof(s_value);
            else if(param == "activation")                
                problem_params.activation = stof(s_value);
            else if(param == "PGSAlpha")
                problem_params.PGSAlpha = stof(s_value);
            else if(param == "discount")
                problem_params.discount = stof(s_value);
            else if(param == "fDiscount")
                problem_params.fDiscount = stof(s_value);
            else if(param == "transitionRate")
                problem_params.transitionRate = stof(s_value);
            else if(param == "expertise")
                worker_params.expertise = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();
        
        problem_params.description = "Manual Assembly - Expertise lvl. " + std::to_string(worker_params.expertise);
        return true;
    }

    /*
        Manual Assembly Task problem specification
    */
    bool ParseManualAssembly(string problemFile, ASSEMBLY_PARAMS& ap){
        cout << "Opening JSON file: " << problemFile << endl;

        //Open JSON file and load into string        
        std::ifstream ifs(problemFile, std::ifstream::in);    
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        ifs.close();

        //Extract string with contents
        std::string str = buffer.str();
        const char * json_str = str.c_str();
        
        //Parse JSON string
        Document document;
        document.Parse(json_str);

        auto trucks = document["objects"].GetArray();
        auto parts = document["parts"].GetArray();
        
        //Process trucks, types and their list of parts
        for(auto& t : trucks){
            //Truck/object name string
            assert(t.HasMember("name"));
            string name = t["name"].GetString();
            ap.objects_str.push_back(name);
            
            //Object type
            assert(t.HasMember("type"));
            int type = t["type"].GetInt();
            ap.types.push_back(type);
                    
            bool glue = true;
            if(t.HasMember("needsGlue"))
                glue = t["needsGlue"].GetBool();
            ap.needsGlue.push_back(glue);

            //Get part array
            assert(t.HasMember("parts"));
            auto truckParts = t["parts"].GetArray();
            vector<int> temp_parts;
            for(auto& p : truckParts){
                int temp_part = p.GetInt();
                temp_parts.push_back(temp_part);
            }
            ap.type_map.push_back(temp_parts);      
        }
        
        //Process parts, their names, priority, cost and starting inventory
        for(auto& p : parts){
            assert(p.HasMember("name"));
            string name = p["name"].GetString();
            ap.parts_str.push_back(name);

            assert(p.HasMember("priority"));
            int priority = p["priority"].GetInt();
            ap.part_priority.push_back(priority);

            assert(p.HasMember("cost"));
            int cost = p["cost"].GetInt();
            ap.part_cost.push_back(cost);

            assert(p.HasMember("storage"));
            int storage = p["storage"].GetInt();
            ap.storage.push_back(storage);
        }

        //Initialize variables. VERY important, used to define actions and observations in robot POMDP
        ap.n_objs = ap.objects_str.size();
        ap.n_types = ap.types.size();
        ap.n_parts = ap.parts_str.size();

        return true;
    }

    bool parseHotelFile(HOTEL_ROBOT_PARAMS& problem_params, HOTEL_PARAMS& worker_params, string& domainFilename, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        //TODO: figure out how to specify Worker MDP parameters in file
        
        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "problem")
                problem_params.problem = s_value;
            else if(param == "domainFile")
                domainFilename = s_value;
            else if(param == "perceive")
                problem_params.perceive = stof(s_value);
            else if(param == "bringSuccess")
                problem_params.bring_success = stof(s_value);
            else if(param == "activation")                
                problem_params.activation = stof(s_value);
            else if(param == "PGSAlpha")
                problem_params.PGSAlpha = stof(s_value);
            else if(param == "discount")
                problem_params.discount = stof(s_value);
            else if(param == "fDiscount")
                problem_params.fDiscount = stof(s_value);
            else if(param == "transitionRate")
                problem_params.transitionRate = stof(s_value);
            else if(param == "expertise")
                worker_params.expertise = stof(s_value);
            else if(param == "entropy")
                problem_params.entropy = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();
        
        problem_params.description = "Insect Hotel - Perception " + std::to_string(problem_params.perceive) + ", Bring " + std::to_string(problem_params.bring_success) + "\nBin.Entropy Limit = " + std::to_string(problem_params.entropy);
        return true;
    }

    bool ParseHotel(string problemFile, HOTEL_PARAMS& ap){
        cout << "Opening JSON file: " << problemFile << endl;

        //Open JSON file and load into string        
        std::ifstream ifs(problemFile, std::ifstream::in);    
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        ifs.close();

        //Extract string with contents
        std::string str = buffer.str();
        const char * json_str = str.c_str();
        
        //Parse JSON string
        Document document;
        document.Parse(json_str);

        //int n_types = document["num_types"].GetInt();
        auto objects = document["objects"].GetArray();
        auto parts = document["parts"].GetArray();
        
        //Process trucks, types and their list of parts
        for(auto& t : objects){
            //Object name string
            assert(t.HasMember("name"));
            string name = t["name"].GetString();
            ap.objects_str.push_back(name);
            
            //Object type
            /*if(t.HasMember("type")){
                int type = t["type"].GetInt();
                //ap.types.push_back(type);
            }*/
            
            //Get part array
            assert(t.HasMember("parts"));
            auto hParts = t["parts"].GetArray();
            vector<int> temp_parts;
            for(auto& p : hParts){
                int temp_part = p.GetInt();
                temp_parts.push_back(temp_part);
            }
            ap.type_map.push_back(temp_parts);
            temp_parts.clear();
        }
        
        //Process parts, their names, priority, cost and starting inventory
        for(auto& p : parts){
            assert(p.HasMember("name"));
            string name = p["name"].GetString();
            ap.parts_str.push_back(name);

            assert(p.HasMember("priority"));
            int priority = p["priority"].GetInt();
            ap.part_priority.push_back(priority);

            assert(p.HasMember("cost"));
            int cost = p["cost"].GetInt();
            ap.part_cost.push_back(cost);

            /*if(p.HasMember("storage")){
                int storage = p["storage"].GetInt();
                //ap.storage.push_back(storage);
            }*/
        }

        //Initialize variables. VERY important, used to define actions and observations in robot POMDP
        ap.n_objs = 1; //ap.objects_str.size();
        //ap.n_types = n_types; //Must be set separately to avoid conflicts, e.g. multiple objs of same type, or one type == one tool
        ap.n_parts = ap.parts_str.size();

        cout << "Problem parsed. Objects = " << ap.n_objs << ", Types = " << ap.n_types << ", Parts = " << ap.n_parts << endl;

        return true;
    }
}
