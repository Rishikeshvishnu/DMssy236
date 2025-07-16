#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <final_project/MovementCommand.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

class TiagoControl
{
private:
    // Core ROS components
    ros::Subscriber sub_gazebo_data_;
    ros::Publisher pub_key_vel_;
    ros::ServiceServer movement_server_;
    
    // Robot state
    geometry_msgs::Twist tiago_twist_;
    geometry_msgs::Pose tiago_position_;
    geometry_msgs::Pose target_position_;
    bool movement_active_ = false;
    bool position_initialized_ = false;
    
    // Control parameters
    double threshold_dist_ = 1.0;
    double Kvx = 0.5;
    double Kwz = 1.0;
    const int MAX_TIMEOUT = 200;  // 20 seconds at 10Hz
    
public:
    TiagoControl(ros::NodeHandle& nh)
    {
        // Initialize ROS interfaces
        sub_gazebo_data_ = nh.subscribe("/gazebo/model_states", 100, 
                                      &TiagoControl::gazeboDataCallback, this);
        pub_key_vel_ = nh.advertise<geometry_msgs::Twist>("key_vel", 100);
        movement_server_ = nh.advertiseService("movement_command", 
                                             &TiagoControl::locationReachedCallback, this);
        ROS_INFO("TiagoControl initialized");
    }
    
    ~TiagoControl() 
    {
        // Ensure robot stops on shutdown
        stopMovement();
    }

private:
    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion &quaternion)
    {
        Eigen::Quaterniond eigenQuaternion(quaternion.w, quaternion.x, 
                                         quaternion.y, quaternion.z);
        return eigenQuaternion.toRotationMatrix().block<2, 2>(0, 0);
    }
    
    double calDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
    {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    void gazeboDataCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        // Find robot in model states
        auto tiago_it = std::find(msg->name.begin(), msg->name.end(), "tiago");
        if (tiago_it != msg->name.end())
        {
            // Update robot position
            int tiago_index = std::distance(msg->name.begin(), tiago_it);
            tiago_position_ = msg->pose[tiago_index];
            position_initialized_ = true;
            
            // If movement is active, update control
            if (movement_active_)
            {
                updateMovement();
            }
        }
        else if (movement_active_)
        {
            ROS_WARN_THROTTLE(5.0, "Lost robot position tracking while moving!");
            stopMovement();
        }
    }
    
    void updateMovement()
    {
        double distance = calDistance(tiago_position_, target_position_);
        
        if (distance > threshold_dist_)
        {
            // Continue movement
            tiago_twist_ = calculateVelocities();
            pub_key_vel_.publish(tiago_twist_);
            ROS_INFO_THROTTLE(2.0, "Distance to target: %.2f meters", distance);
        }
        else
        {
            ROS_INFO("Target reached!");
            stopMovement();
        }
    }
    
    bool locationReachedCallback(final_project::MovementCommand::Request &req,
                               final_project::MovementCommand::Response &res)
    {
        if (!position_initialized_)
        {
            ROS_ERROR("Cannot start movement - robot position unknown");
            res.success = false;
            res.message = "Robot position not initialized";
            return true;
        }
        
        ROS_INFO("Received new movement command");
        target_position_ = req.target_pose;
        movement_active_ = true;
        
        // Wait for movement completion or timeout
        ros::Rate rate(10);  // 10Hz
        int timeout_counter = 0;
        
        while (movement_active_ && timeout_counter < MAX_TIMEOUT && ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
            timeout_counter++;
        }
        
        // Check completion status
        if (movement_active_)
        {
            ROS_WARN("Movement timed out!");
            stopMovement();
            res.success = false;
            res.message = "Movement timed out";
        }
        else
        {
            res.success = true;
            res.message = "Target reached successfully";
        }
        
        return true;
    }
    
    geometry_msgs::Twist calculateVelocities()
    {
        geometry_msgs::Twist calculated_twist;
        
        // Convert positions to Eigen vectors
        Eigen::Vector2d tiago_pos(tiago_position_.position.x, tiago_position_.position.y);
        Eigen::Vector2d obj_pos(target_position_.position.x, target_position_.position.y);
        
        // Calculate transformation and error
        Eigen::Matrix2d R_tiago_w = q2Rot2D(tiago_position_.orientation);
        Eigen::Vector2d delta_pos_w = obj_pos - tiago_pos;
        Eigen::Vector2d delta_pos_tiago = R_tiago_w.inverse() * delta_pos_w;
        
        // Calculate control inputs
        double distance = delta_pos_tiago.norm();
        double theta = std::atan2(delta_pos_tiago(1), delta_pos_tiago(0));
        
        // Set velocities with safety bounds
        calculated_twist.linear.x = std::min(Kvx * distance, 0.5);  // Max 0.5 m/s
        calculated_twist.angular.z = std::min(std::max(Kwz * theta, -1.0), 1.0);  // Bound to ±1.0 rad/s
        
        return calculated_twist;
    }
    
    void stopMovement()
    {
        tiago_twist_.linear.x = 0.0;
        tiago_twist_.angular.z = 0.0;
        pub_key_vel_.publish(tiago_twist_);
        movement_active_ = false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_control_node");
    ros::NodeHandle nh;
    
    try 
    {
        TiagoControl tiago_control(nh);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in tiago_control_node: %s", e.what());
        return 1;
    }
    
    return 0;
}