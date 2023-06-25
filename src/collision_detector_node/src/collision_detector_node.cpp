#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

std::vector<double> joint_values(6);
std::vector<double> ft_wrenches(6) ;
std::vector<double> joint_torques(6);
ros::Publisher* g_marker_array_publisher = 0;
visualization_msgs::MarkerArray g_collision_points;
visualization_msgs::Marker marker;


void publishMarkers()
{
  // // delete old markers  
  // if (g_collision_points.markers.size())
  // {
  //   for (int i = 0; i < g_collision_points.markers.size(); i++)
  //     g_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

  //   g_marker_array_publisher->publish(g_collision_points);
  // }

  // // move new markers into g_collision_points
  // std::swap(g_collision_points.markers, markers.markers);

  // // draw new markers (if there are any)
  // if (g_collision_points.markers.size())
  //   g_marker_array_publisher->publish(g_collision_points);
  // ROS_INFO("marker publisher called");
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "/";
  marker.id = 0;
  marker.type = 10;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.0012;
  marker.scale.y = 0.0012;
  marker.scale.z = 0.0012;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "package://robot_arm_description/meshes/base_link.stl";
  g_marker_array_publisher->publish(marker);
}

void differenceCompute(){
    for(unsigned int i = 0 ; i < 5 ; ++i){
      double diff = abs(joint_torques[i]) - abs(ft_wrenches[i]);
      ROS_INFO_STREAM("Torque difference in joint " << i << ": " << diff << "   joint_values:"<< joint_values[i] <<"  joint_torques:" << joint_torques[i] << "  ft_wrenches: " << ft_wrenches[i] << std::endl);
    if (abs(diff) > 1.0)
    {
      ROS_INFO("-------- COLLISION DETECTED -------->>> OUTPUT HAPTIC FEEDBACK");
      
      publishMarkers();
   }
   else{
    marker.action = visualization_msgs::Marker::DELETE;
   }
  }
  

}


void jointStateCallback(const sensor_msgs::JointState& joint_msg){
    //ROS_INFO("jointStateCallback started!");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");   
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotState state(kinematic_model); 
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    std::string group_name = "arm_group";
    geometry_msgs::Vector3 gravity_vector;
    gravity_vector.x = 0.0;
    gravity_vector.y = 0.0;
    gravity_vector.z = -9.81;    
    double payload = 1.00;

    dynamics_solver::DynamicsSolver solver(kinematic_model, group_name, gravity_vector);
    std::map<std::string, double> metrics;
    double max_payload;
    unsigned int saturated_joint;
    joint_values.clear();
    joint_values.resize(6);
    // state.copyJointGroupPositions(group_name, joint_values);
    
    // for (std::size_t i = 0 ; i < joint_torques.size() ; ++i)
    //   {
    //     std::stringstream stream;
    //     stream << "torque[" << i << "]";
    //     metrics[stream.str()] = joint_torques[i];
    //   }


  for(unsigned int i = 0 ; i < 6 ; ++i) {
       joint_values[i] = joint_msg.position[i];
        //ROS_INFO_STREAM("joint_values for joint " << i << ": " << joint_values[i] << std::endl);
  }
    solver.getMaxPayload(joint_values, max_payload, saturated_joint);
    joint_torques.clear();
    joint_torques.resize(6);
    solver.getPayloadTorques(joint_values, payload, joint_torques);
  differenceCompute();
}

void Revolute1_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute1_ft_wrenches){
  // ft_wrenches.push_back(revolute1_ft_wrenches->wrench.torque.x);
  // ft_wrenches.push_back(revolute1_ft_wrenches->wrench.torque.y);
  ft_wrenches[0] = revolute1_ft_wrenches->wrench.torque.z;
 // ROS_INFO_STREAM("Torque from Revolute1_ftCallback : " << ft_wrenches[0] << std::endl);
}
void Revolute2_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute2_ft_wrenches){
  // ft_wrenches.push_back(revolute2_ft_wrenches->wrench.torque.x);
  ft_wrenches[1] = revolute2_ft_wrenches->wrench.torque.y;
  // ft_wrenches.push_back(revolute2_ft_wrenches->wrench.torque.z);
  //ROS_INFO_STREAM("Torque from Revolute2_ftCallback : " << ft_wrenches[1] << std::endl);
}
void Revolute3_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute3_ft_wrenches){
  // ft_wrenches.push_back(revolute3_ft_wrenches->wrench.torque.x);
  ft_wrenches[2] = revolute3_ft_wrenches->wrench.torque.y;
  // ft_wrenches.push_back(revolute3_ft_wrenches->wrench.torque.z);
  //ROS_INFO_STREAM("Torque from Revolute3_ftCallback : " << ft_wrenches[2] << std::endl);
}
void Revolute4_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute4_ft_wrenches){
  // ft_wrenches.push_back(revolute4_ft_wrenches->wrench.torque.x);
  // ft_wrenches.push_back(revolute4_ft_wrenches->wrench.torque.y);
  ft_wrenches[3] = revolute4_ft_wrenches->wrench.torque.z;
  //ROS_INFO_STREAM("Torque from Revolute4_ftCallback : " << ft_wrenches[3] << std::endl);
}
void Revolute5_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute5_ft_wrenches){
  // ft_wrenches.push_back(revolute5_ft_wrenches->wrench.torque.x);
  ft_wrenches[4] = revolute5_ft_wrenches->wrench.torque.y;
  // ft_wrenches.push_back(revolute5_ft_wrenches->wrench.torque.z);
  //ROS_INFO_STREAM("TorROS_INFO("Node started!");que from Revolute5_ftCallback : " << ft_wrenches[4] << std::endl);
}
void Revolute6_ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& revolute6_ft_wrenches){
  // ft_wrenches.push_back(revolute6_ft_wrenches->wrench.torque.x);
  // ft_wrenches.push_back(revolute6_ft_wrenches->wrench.torque.y);
  ft_wrenches[5] = revolute6_ft_wrenches->wrench.torque.z;
  //ROS_INFO_STREAM("Torque from Revolute6_ftCallback : " << ft_wrenches[5] << std::endl);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detector_node");
    ros::NodeHandle n("~");
     
    ros::Rate loop_rate(100);
    ROS_INFO("Node started!");
    g_marker_array_publisher = new ros::Publisher(n.advertise<visualization_msgs::Marker>("visualization_marker", 0));
    ros::Subscriber sub = n.subscribe("/joint_states", 0, jointStateCallback);

    ros::Subscriber revolute1 = n.subscribe("/Revolute1_ft", 0, Revolute1_ftCallback);
    ros::Subscriber revolute2 = n.subscribe("/Revolute2_ft", 0, Revolute2_ftCallback);
    ros::Subscriber revolute3 = n.subscribe("/Revolute3_ft", 0, Revolute3_ftCallback);
    ros::Subscriber revolute4 = n.subscribe("/Revolute4_ft", 0, Revolute4_ftCallback);
    ros::Subscriber revolute5 = n.subscribe("/Revolute5_ft", 0, Revolute5_ftCallback);
    ros::Subscriber revolute6 = n.subscribe("/Revolute6_ft", 0, Revolute6_ftCallback);

    // collision_detection::CollisionResult result;
    // collision_detection::CollisionResult::ContactMap contact_map;
    // contact_map = collision_detection::Contact(1, 2, 3);
    // for (auto it = contact_map.begin(); it != contact_map.end(); it++) {
    // cout << "Contact " << it->first << ": " << it->second << endl;
    // }

    // ros::Publisher joint_values_pub = n.advertise<std_msgs::String>("modified_joint_angles", 1000);

    
    
    // joint_values_pub.publish(metrics);

    // while (ros::ok()) {

      
      
      
      // loop_rate.sleep();
    // }
      ros::spin();
}




//ROS_INFO("Robot Model: %s", kinematic_model);
    // ROS_DEBUG_STREAM_NAMED("test_only", "Hello " << kinematic_model);
    // cout << "Robort Model:" << kinematic_model;

      // Set the joint angles.
  // std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // // Set the joint velocities.
  // std::vector<double> joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // // Set the joint accelerations.
  // std::vector<double> joint_accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // // Set the external wrenches.
  // std::vector<geometry_msgs::Wrench> wrenches;
  //   //bool success = DynamicsSolver.getTorques();
  // std::vector<double> torques;
  // double payload = 1.00;
  // std::vector<double> joint_torques;
  // joint_torques.resize(6);
  // bool success = solver.getPayloadTorques(joint_angles, payload, joint_torques);

  // // Check for errors.
  // if (!success) {
  //   ROS_ERROR("getTorques failed!");
  //   return 1;
  // }
  // else{
  //   ROS_ERROR("getTorques succeeeeeeeeeesss");
  // }

  // // Do something with the torques vector.
  // for (int i = 0; i < torques.size(); i++) {
  //   ROS_ERROR("torque printer on");
  //   std::cout << "Torque for joint " << i << ": " << joint_torques[i] << std::endl;
  //   ROS_INFO_STREAM("Torque for joint " << i << ": " << joint_torques[i] << std::endl);
  // }
