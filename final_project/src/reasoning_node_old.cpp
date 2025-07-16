#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>
#include <final_project/checkObjExistence.h>
#include <final_project/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    //Variable to save our preference to save or not the asserted queries
    bool m_query_flag_save;

    std::string srv_check_existence_name_;
    ros::ServiceServer check_existence_srv_;
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        this->m_query_flag_save=false;

        // In constructor:
        srv_check_existence_name_ = "check_object_existence";
        check_existence_srv_ = nh.advertiseService(
            srv_check_existence_name_,
            &Reasoner::checkObjExistenceCallback,
            this
        );
    };

    ~Reasoner(){

    };

   //TODO A04.T02: This function should open a file if the path of the file is found, otherwise it should print out that the file or directory were not found (1 pt)
    void setOutQueriesFile(string QueryfileName){

        std::ifstream checkFile(QueryfileName);
        if (checkFile) {
            checkFile.close();
            std::ofstream outFile(QueryfileName);

            this->m_query_flag_save = true; // to save if the file exists and can be opened

        } else {
            ROS_INFO_STREAM("The file or directory was not found");
        }


    }

private:    


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(final_project::UpdateObjectList::Request &req,
                             final_project::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        //TODO: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful.

        object = req.object_name;
        getClass(object);  // verify if the seen object has a class
        res.confirmation = assertKnowledge(object);  // assert it into the knowledge base


        return res.confirmation;
    }


    void getClass(std::string className)
    {
        
        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils"
        std:string query= "query";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool res = false;
        for (auto &it : bdgs) 
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }

       
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        // TODO: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" 
        std:string query= "query";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //TODO: Retrive the value from Prolog
                instanceName = val.second.toString();
                ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
            }
            
        }

        bdgs.finish();
        
        return true;
    }


    // // service callback to check if object exists

    // bool checkObjExistenceCallback(checkObjExistence::Request &req,checkObjExistence::Response &res) 
    // {
    //     // Is this a location query?
    //     auto location_it = location_poses_.find(req.object_name);
    //     if (location_it != location_poses_.end()) {
    //         // It's a location - return its coordinates directly
    //         res.exists = true;
    //         res.location = req.object_name;
    //         res.pose = location_it->second;
    //         return true;
    //     }

    //     // Not a location - check if it's an object in ontology
    //     std::stringstream query;
    //     query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, _)";
        
    //     PrologQuery bdgs = pl_.query(query.str());
    //     res.exists = false;
        
    //     for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++) {
    //         res.exists = true;
    //         ROS_INFO_STREAM("Object exists in ontology");
    //         res.location = getObjectLocation(req.object_name);
            
    //         if(!res.location.empty()) {
    //             // Look up location's coordinates in our map
    //             auto pose_it = location_poses_.find(res.location);
    //             if(pose_it != location_poses_.end()) {
    //                 res.pose = pose_it->second;
    //                 ROS_INFO_STREAM("Found coordinates for: " << res.location);
    //             }
    //         }
    //         break;
    //     }
        
    //     return true;
    // }

    bool checkObjExistenceCallback(final_project::checkObjExistence::Request &req,
                                final_project::checkObjExistence::Response &res) 
    {
        std::stringstream query;
        // Need to check if object exists as instance in ontology
        query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, _)";
        
        PrologQuery bdgs = pl_.query(query.str());
        res.exists = false;
        
        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++) {
            res.exists = true;
            ROS_INFO_STREAM("Object exists in ontology");

            // If exists, try to get location
            res.location = getObjectLocation(req.object_name);

            if(!res.location.empty()) {
                ROS_INFO_STREAM("Object's location is obtained: " << res.location);
                // res.pose = getObjectPose(req.object_name, res.pose); // needs to be implemented
            }
            break;  // We found the object, no need to continue loop
        }
        
        return true;
    }


    std::string getObjectLocation(const std::string& object_name) {
        // Build a query to find the surface where our object is located
        std::stringstream query;
        query << "getClassPath('" << object_name << "', ObjPath), "
            << "surface_of_object(ObjPath, Surface), "
            << "atom_to_string(Surface, SurfaceStr)";

        // Execute the Prolog query
        PrologQuery bdgs = pl_.query(query.str());

        // Check if the query returned any results
        if (bdgs.begin() == bdgs.end()) {
            // If no results, log a warning and return an empty string
            ROS_WARN_STREAM("Could not find location for object: " << object_name);
            return "";
        }

        // Look through query results
        for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); ++it) {
            // Iterate through the Prolog bindings directly (no need for const_iterator)
            for (auto& binding : *it) {
                // When we find the surface string binding
                if (binding.first == "SurfaceStr") {
                    std::string location = binding.second.toString();

                    // Clean up the location string by removing the ontology prefix
                    size_t hashPos = location.find('#');
                    if (hashPos != std::string::npos) {
                        location = location.substr(hashPos + 1);  // Remove the ontology prefix part
                    }

                    // Log and return the found location
                    ROS_INFO_STREAM("Found object " << object_name << " on surface: " << location);
                    return location;
                }
            }
        }

        // If no location was found, log a warning and return empty string
        ROS_WARN_STREAM("Could not find location for object: " << object_name);
        return "";
    }



}; //class Reasoner

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");

  ros::NodeHandle nh;   
  
  Reasoner myReasoner(nh);

   //+ Information about the path for the file that will save the queries
  std::string saveFilePath;
  saveFilePath = argv[1]; // This means that the node expects a path value as input. This means that we need to run this node as follows: rosrun world_percept

 
   bool saveQueries_flag; //variable that will receive the value from the yaml file

   //TODO A04.T02: Retrieve the variable from the yaml file and save it in the new variable "saveQueries_flag" (0.25 pts) 
   std::string savedQueryFile;

   nh.getParam("/read_prolog_queries/save_flag",saveQueries_flag);
   
   if(saveQueries_flag)
    { //If the flag is true, then we will configure the file to save the asserted queries
         //TODO A04.T02: Include the code to load the rosparam from a yaml file (0.75 pt)
         // First define a new string variable "savedQueryFile"
         // Then load the yaml parameter in the new variable
       
        //This node now needs a path as input when we run it. This path is addedd to the obtain variable from the yaml file, as follows
        
        std::string savedQueryFile; //defining again
        nh.getParam("/read_prolog_queries/saved_query_file",savedQueryFile);

        savedQueryFile= saveFilePath+savedQueryFile;
        ROS_INFO_STREAM("query_file: "<< savedQueryFile);

        //Now we call a new function which will create and open the new file
        myReasoner.setOutQueriesFile(savedQueryFile);
     
    }

  ros::spin();

  
  return 0;
}