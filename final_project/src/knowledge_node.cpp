#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <final_project/UpdateObjectList.h>

#include <final_project/loadKnowledge.h>

using namespace std;

class Knowledge
{
private:
    // mentioned vars
    PrologClient pl_;
    std::string srv_load_knowledge_name_;
    ros::ServiceServer load_knowledge_srv_;

    // file to be queried
    std::ifstream q_file;

public:

    Knowledge(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer()){
            pl_ = PrologClient("/rosprolog", true);
        }

        srv_load_knowledge_name_ = "load_knowledge";

        ROS_INFO("Advertising service: %s", srv_load_knowledge_name_.c_str()); //added
        load_knowledge_srv_ = nh.advertiseService(srv_load_knowledge_name_,
                                                 &Knowledge::callback_load_knowledge,
                                                 this);

        ROS_INFO_STREAM("Service advertised.");
    }

    ~Knowledge(){

    };

    void setQueryFile(std::string fileName_Q){

        q_file.open(fileName_Q);
        if (!q_file.is_open()) {
            ROS_INFO_STREAM("file not found and thus exitting");
            return;
        }

    }

    bool callback_load_knowledge(final_project::loadKnowledge::Request &req, final_project::loadKnowledge::Response &res) {
        // can start be any number??
        if (req.start){
            loadQueries();
            res.confirm = true;
        }
        else{
            res.confirm = false;
        }

        return res.confirm;
    }

    void loadQueries(){
        std::string line;
        while (getline(q_file, line)) {
            pl_.query(line);
        }

    }

};
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "knowledge_node");

    ros::NodeHandle nh;   
  
    Knowledge knowledge(nh);

    //as in reasoning node
    std::string saveFilePath;
    saveFilePath = argv[1]; 

    bool saveQueries_flag;
    nh.getParam("/save_flag", saveQueries_flag);
    
    if(saveQueries_flag) {
        std::string savedQueryFile;
        nh.getParam("/saved_query_file", savedQueryFile);
        savedQueryFile = saveFilePath + savedQueryFile;
        ROS_INFO_STREAM("query_file: " << savedQueryFile);
        knowledge.setQueryFile(savedQueryFile);
    } 

    ros::spin();

  
  return 0;
}