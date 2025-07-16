#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <rosprolog/rosprolog_client/PrologClient.h>
#include <final_project/checkObjExistence.h>
#include <final_project/UpdateObjectList.h>

#include <final_project/PredictedLocation.h> // *Receiving predictions
#include <final_project/DetectedObject.h>   // *YOLO detection
#include <geometry_msgs/Pose.h>

using namespace std;

class Reasoner {
private:
    PrologClient pl_;
    int ID_;

    std::string target_object_;
    std::string predicted_location_;
    geometry_msgs::Pose detected_pose_;

    // Service names
    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;
    std::string srv_check_existence_name_;
    ros::ServiceServer check_existence_srv_;

    // Subscribers
    ros::Subscriber predicted_location_sub_;
    ros::Subscriber yolo_detection_sub_;


public:
    Reasoner(ros::NodeHandle &nh) {
        ROS_INFO("Waiting for Prolog service...");
        pl_ = PrologClient("/rosprolog", true);

        if (!pl_.waitForServer(ros::Duration(10.0))) {
            ROS_ERROR("Prolog service not available. Shutting down...");
            ros::shutdown();
            return;
        }

        ID_ = 0; // Initialize instance ID counter

        // Initialize services
        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(
            srv_assert_knowledge_name_, 
            &Reasoner::srv_assert_callback, 
            this
        );

        srv_check_existence_name_ = "check_object_existence";
        check_existence_srv_ = nh.advertiseService(
            srv_check_existence_name_, 
            &Reasoner::checkObjExistenceCallback, 
            this
        );

        // Initialize subscribers
        predicted_location_sub_ = nh.subscribe(
            "/predicted_location", 
            10, 
            &Reasoner::predictedLocationCallback, 
            this
        );

        yolo_detection_sub_ = nh.subscribe(
            "/yolo_detection", 
            10, 
            &Reasoner::yoloDetectionCallback, 
            this
        );

        ROS_INFO("Reasoning node services are ready.");
    }

private:
    // Service callback to assert knowledge
    bool srv_assert_callback(final_project::UpdateObjectList::Request &req,
                             final_project::UpdateObjectList::Response &res) {
        ROS_INFO_STREAM("Received object: " << req.object_name);

        getClass(req.object_name);
        res.confirmation = assertKnowledge(req.object_name);

        return res.confirmation;
    }

    // Service callback to check object existence
    // bool checkObjExistenceCallback(final_project::checkObjExistence::Request &req,
    //                                final_project::checkObjExistence::Response &res) {
    //     ROS_INFO_STREAM("Checking existence of object: " << req.object_name);

    //     std::stringstream query;
    //     query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, _)";
    //     PrologQuery bdgs = pl_.query(query.str());
    //     res.exists = false;

    //     if (bdgs.begin() != bdgs.end()) {
    //         res.exists = true;
    //         ROS_INFO_STREAM("Object exists in ontology: " << req.object_name);

    //         res.location = getObjectLocation(req.object_name);
    //         if (!res.location.empty()) {
    //             ROS_INFO_STREAM("Object location retrieved: " << res.location);
    //         } else {
    //             ROS_WARN_STREAM("Location for object " << req.object_name << " not found.");
    //         }
    //     } else {
    //         ROS_WARN_STREAM("Object " << req.object_name << " does not exist in ontology.");
    //         target_object_ = req.object_name;
    //     }

    //     return true;
    // }

    // bool checkObjExistenceCallback(final_project::checkObjExistence::Request &req,
    //                            final_project::checkObjExistence::Response &res) {
    //     ROS_INFO_STREAM("Checking existence of object: " << req.object_name);

    //     res.exists = false;

    //     // First, check if an individual (instance) of the object exists
    //     std::stringstream instance_query;
    //     instance_query << "owl_individual_of(ssy236Ontology:'" << req.object_name << "', _).";
    //     PrologQuery instance_bdgs = pl_.query(instance_query.str());

    //     if (instance_bdgs.begin() != instance_bdgs.end()) {
    //         res.exists = true;
    //         ROS_INFO_STREAM("Object instance exists in ontology: " << req.object_name);

    //         res.location = getObjectLocation(req.object_name);
    //         if (!res.location.empty()) {
    //             ROS_INFO_STREAM("Object location retrieved: " << res.location);
    //         } else {
    //             ROS_WARN_STREAM("Location for object " << req.object_name << " not found.");
    //         }
    //         return true;
    //     }

    //     // If not found as an instance, check if it's a class
    //     std::stringstream class_query;
    //     class_query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, owl:'Class').";
    //     PrologQuery class_bdgs = pl_.query(class_query.str());

    //     if (class_bdgs.begin() != class_bdgs.end()) {
    //         res.exists = true;
    //         ROS_INFO_STREAM("Object exists as a class in ontology: " << req.object_name);
    //         return true;
    //     }

    //     // If neither instance nor class found, return not found
    //     ROS_WARN_STREAM("Object " << req.object_name << " does not exist in ontology.");
    //     return true;
    // }


    bool checkObjExistenceCallback(final_project::checkObjExistence::Request &req,
                               final_project::checkObjExistence::Response &res) {
        ROS_INFO_STREAM("Checking existence of object: " << req.object_name);
        res.exists = false;

        // First, check if an individual (instance) of the object exists in ontology
        std::stringstream instance_query;
        instance_query << "owl_individual_of(ssy236Ontology:'" << req.object_name << "', _).";
        PrologQuery instance_bdgs = pl_.query(instance_query.str());

        if (instance_bdgs.begin() != instance_bdgs.end()) {
            res.exists = true;
            ROS_INFO_STREAM("Object instance exists in ontology: " << req.object_name);

            // Get location from ontology
            res.location = getObjectLocation(req.object_name);

            if (!res.location.empty()) {
                ROS_INFO_STREAM("Object location retrieved: " << res.location);
            } else {
                ROS_WARN_STREAM("Location for object " << req.object_name << " not found in ontology. Using hardcoded data...");

                // Fetch hardcoded location
                res.location = getHardcodedLocation(req.object_name);
                if (!res.location.empty()) {
                    res.pose = getHardcodedPose(res.location);
                    ROS_INFO_STREAM("Assigned hardcoded location: " << res.location);
                } else {
                    ROS_WARN_STREAM("No hardcoded location found for object: " << req.object_name);
                }
            }
            return true;
        }

        // If not found as an instance, check if it's a class
        std::stringstream class_query;
        class_query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, owl:'Class').";
        PrologQuery class_bdgs = pl_.query(class_query.str());

        if (class_bdgs.begin() != class_bdgs.end()) {
            res.exists = true;
            ROS_INFO_STREAM("Object exists as a class in ontology: " << req.object_name);
            return true;
        }

        // If neither instance nor class found, return not found
        ROS_WARN_STREAM("Object " << req.object_name << " does not exist in ontology.");
        return true;
    }


    // Callback for receiving predicted location
    void predictedLocationCallback(const final_project::PredictedLocation &msg) {
        if (msg.object_name == target_object_) {
            predicted_location_ = msg.location;
            ROS_INFO_STREAM("Predicted location for " << target_object_ << ": " << predicted_location_);
        }
    }

    // Callback for receiving YOLO detection results
    void yoloDetectionCallback(const final_project::DetectedObject &msg) {
        if (msg.object_name == target_object_) {
            detected_pose_ = msg.pose;
            ROS_INFO_STREAM("Object detected by YOLO: " << target_object_);
            ROS_INFO_STREAM("Updating ontology with detected pose and location...");
            addObjectToOntology(target_object_, predicted_location_, detected_pose_);
        }
    }

    // Add object to ontology
    bool addObjectToOntology(const std::string &object_name, const std::string &location, const geometry_msgs::Pose &pose) {
        std::string instance_name = "ssy236Ontology:'" + object_name + "_instance'";
        std::string location_class = "ssy236Ontology:'" + location + "'";

        // Add object instance
        std::string query = "rdf_assert(" + instance_name + ", rdf:type, ssy236Ontology:'" + object_name + "')";
        pl_.query(query);

        // Link object to its location
        query = "rdf_assert(" + instance_name + ", ssy236Ontology:'isLocatedAt', " + location_class + ")";
        pl_.query(query);

        // Add pose details as datatype properties (pseudo-query for Prolog extension)
        std::stringstream pose_query;
        pose_query << "add_pose_details('" << instance_name << "', " 
                   << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")";
        pl_.query(pose_query.str());

        ROS_INFO_STREAM("Object " << object_name << " added to ontology at location " << location);
        return true;
    }

    // Helper to retrieve or create a class
    void getClass(const std::string &className) {
        std::string query = "rdf_has(ssy236Ontology:'" + className + "', rdf:type, owl:'Class')";
        PrologQuery bdgs = pl_.query(query);

        if (bdgs.begin() == bdgs.end()) {
            query = "rdf_assert(ssy236Ontology:'" + className + "', rdf:type, owl:'Class')";
            pl_.query(query);
            ROS_INFO_STREAM("Created new class in ontology: " << className);
        } else {
            ROS_INFO_STREAM("Class already exists: " << className);
        }
    }

    // Assert knowledge into ontology
    // bool assertKnowledge(const std::string &objectName) {
    //     std::string instanceName = "ssy236Ontology:'" + objectName + "_instance'";
    //     // std::string query = "rdf_assert(" + instanceName + ", rdf:type, ssy236Ontology:'" + objectName + "')";
    //     query = "rdf_assert(ssy236Ontology:'" + objectName + "_instance', rdf:type, owl:'NamedIndividual')";
    //     pl_.query(query);
    //     ROS_INFO_STREAM("Object instance added to ontology: " << instanceName);
    //     return true;
    // }

    bool assertKnowledge(const std::string &objectName) {
        std::string instanceName = "ssy236Ontology:'" + objectName + "_instance'";
        
        std::string query = "rdf_assert(" + instanceName + ", rdf:type, ssy236Ontology:'" + objectName + "')";
        pl_.query(query);

        query = "rdf_assert(ssy236Ontology:'" + objectName + "_instance', rdf:type, owl:'NamedIndividual')";
        pl_.query(query);

        ROS_INFO_STREAM("Object instance added to ontology: " << instanceName);
        return true;
    }


    // Get object location
    std::string getObjectLocation(const std::string &object_name) {
        std::stringstream query;
        // query << "getClassPath('" << object_name << "', ObjPath), "
        //       << "surface_of_object(ObjPath, Surface), "
        //       << "atom_to_string(Surface, SurfaceStr)";

        query << "rdf_has(ssy236Ontology:'" << object_name << "', ssy236Ontology:'isLocatedAt', Location), "
          << "atom_to_string(Location, LocStr)";

        PrologQuery bdgs = pl_.query(query.str());

        if (bdgs.begin() == bdgs.end()) {
            return "";
        }

        // for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); ++it) {
        //     for (const auto &binding : *it) {
        //         if (binding.first == "SurfaceStr") {
        //             std::string location = binding.second.toString();
        //             size_t hashPos = location.find('#');
        //             if (hashPos != std::string::npos) {
        //                 return location.substr(hashPos + 1);
        //             }
        //             return location;
        //         }
        //     }
        // }

        // directly queries whether an object has an "isLocatedAt" relation (avoids surface_of_object/2).
        for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); ++it) {
            for (const auto &binding : *it) {
                if (binding.first == "LocStr") {
                    std::string location = binding.second.toString();
                    size_t hashPos = location.find('#');
                    if (hashPos != std::string::npos) {
                        return location.substr(hashPos + 1);
                    }
                    return location;
                }
            }
        }

        return "";
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning_node");
    ros::NodeHandle nh;

    Reasoner myReasoner(nh);

    ros::spin();
    return 0;
}



// OLD WORKING VERSION:
// #include <ros/ros.h>
// #include <iostream>
// #include <string>
// #include <sstream>
// #include <fstream>
// #include <rosprolog/rosprolog_client/PrologClient.h>
// #include <final_project/checkObjExistence.h>
// #include <final_project/UpdateObjectList.h>

// using namespace std;

// class Reasoner {
// private:
//     PrologClient pl_;
//     int ID_;
//     std::string srv_assert_knowledge_name_;
//     ros::ServiceServer assert_knowledge_srv_;
//     std::string srv_check_existence_name_;
//     ros::ServiceServer check_existence_srv_;

// public:
//     Reasoner(ros::NodeHandle &nh) {
//         ROS_INFO("Waiting for Prolog service...");
//         pl_ = PrologClient("/rosprolog", true);

//         if (!pl_.waitForServer(ros::Duration(10.0))) {
//             ROS_ERROR("Prolog service not available. Shutting down...");
//             ros::shutdown();
//             return;
//         }

//         ID_ = 0; // Initialize instance ID counter

//         srv_assert_knowledge_name_ = "assert_knowledge";
//         assert_knowledge_srv_ = nh.advertiseService(
//             srv_assert_knowledge_name_, 
//             &Reasoner::srv_assert_callback, 
//             this
//         );

//         srv_check_existence_name_ = "check_object_existence";
//         check_existence_srv_ = nh.advertiseService(
//             srv_check_existence_name_, 
//             &Reasoner::checkObjExistenceCallback, 
//             this
//         );

//         ROS_INFO("Reasoning node services are ready.");
//     }

// private:
//     bool srv_assert_callback(final_project::UpdateObjectList::Request &req,
//                              final_project::UpdateObjectList::Response &res) {
//         ROS_INFO_STREAM("Received object: " << req.object_name);

//         getClass(req.object_name);
//         res.confirmation = assertKnowledge(req.object_name);

//         return res.confirmation;
//     }

//     bool checkObjExistenceCallback(final_project::checkObjExistence::Request &req,
//                                    final_project::checkObjExistence::Response &res) {
//         ROS_INFO_STREAM("Checking existence of object: " << req.object_name);

//         std::stringstream query;
//         query << "rdf_has(ssy236Ontology:'" << req.object_name << "', rdf:type, _)";
//         PrologQuery bdgs = pl_.query(query.str());
//         res.exists = false;

//         if (bdgs.begin() != bdgs.end()) {
//             res.exists = true;
//             ROS_INFO_STREAM("Object exists in ontology: " << req.object_name);

//             res.location = getObjectLocation(req.object_name);
//             if (!res.location.empty()) {
//                 ROS_INFO_STREAM("Object location retrieved: " << res.location);
//             } else {
//                 ROS_WARN_STREAM("Location for object " << req.object_name << " not found.");
//             }
//         } else {
//             ROS_WARN_STREAM("Object " << req.object_name << " does not exist in ontology.");
//         }

//         return true;
//     }

//     void getClass(const std::string &className) {
//         std::string query = "rdf_has(ssy236Ontology:'" + className + "', rdf:type, owl:'Class')";
//         PrologQuery bdgs = pl_.query(query);

//         if (bdgs.begin() == bdgs.end()) {
//             query = "rdf_assert(ssy236Ontology:'" + className + "', rdf:type, owl:'Class')";
//             pl_.query(query);
//             ROS_INFO_STREAM("Created new class in ontology: " << className);
//         } else {
//             ROS_INFO_STREAM("Class already exists: " << className);
//         }
//     }

//     bool assertKnowledge(const std::string &objectName) {
//         std::string instanceName = "ssy236Ontology:'" + objectName + "_instance'";
//         std::string query = "rdf_assert(" + instanceName + ", rdf:type, ssy236Ontology:'" + objectName + "')";
//         pl_.query(query);
//         ROS_INFO_STREAM("Object instance added to ontology: " << instanceName);
//         return true;
//     }

//     std::string getObjectLocation(const std::string &object_name) {
//         std::stringstream query;
//         query << "getClassPath('" << object_name << "', ObjPath), "
//               << "surface_of_object(ObjPath, Surface), "
//               << "atom_to_string(Surface, SurfaceStr)";

//         PrologQuery bdgs = pl_.query(query.str());

//         if (bdgs.begin() == bdgs.end()) {
//             return "";
//         }

//         for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); ++it) {
//             for (const auto &binding : *it) {
//                 if (binding.first == "SurfaceStr") {
//                     std::string location = binding.second.toString();
//                     size_t hashPos = location.find('#');
//                     if (hashPos != std::string::npos) {
//                         return location.substr(hashPos + 1);
//                     }
//                     return location;
//                 }
//             }
//         }

//         return "";
//     }
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "reasoning_node");
//     ros::NodeHandle nh;

//     Reasoner myReasoner(nh);

//     ros::spin();
//     return 0;
// }


