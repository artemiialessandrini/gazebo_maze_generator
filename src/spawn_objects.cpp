#include <ros/ros.h>

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "gazebo_msgs/SpawnModel.h"
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

#include "geometry_msgs/PoseStamped.h"
#include <cstdlib>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


using namespace std;

//int to string converter

  std::string intToString (int a) {
     std::stringstream ss;
     ss << a;
     return ss.str();
  }

int main(int argc, char **argv) {
    ros::init(argc, argv, "spawn_objects"); // Node setup
    ros::NodeHandle nh;


    ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    // ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

    gazebo_msgs::SpawnModel::Request  spawn_model_req;
    gazebo_msgs::SpawnModel::Response spawn_model_resp;

    // geometry_msgs::Pose pose;

    bool service_ready = false;

    while (!service_ready){
      service_ready = ros::service::exists("/gazebo/spawn_urdf_model", true);
      // service_ready = ros::service::exists("/gazebo/spawn_sdf_model", true);
      // ROS_INFO("waiting for spawn_urdf_model service");
      ROS_INFO("waiting for spawn_sdf_model service");
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("spawn_urdf_model service is ready");
    // ROS_INFO("spawn_sdf_model service is ready");

    std::string red_box_path;
    bool get_red_path;
    get_red_path = nh.getParam("/red_box_path", red_box_path);

    if (!(get_red_path)) {
      return 0;
    }

    else {
      ROS_INFO_STREAM(red_box_path << " has been extracted");
    }

    std::ifstream red_inXml(red_box_path.c_str());
    std::stringstream red_strStream;
    std::string red_xmlStr;

    /*red_inXml.open(red_box_path.c_str());*/

    red_strStream << red_inXml.rdbuf();
    red_xmlStr = red_strStream.str();


    spawn_model_req.initial_pose.position.x= 0.0;
    spawn_model_req.initial_pose.position.y= 0.0;
    spawn_model_req.initial_pose.position.z= 0.0;
    spawn_model_req.initial_pose.orientation.x= 0;
    spawn_model_req.initial_pose.orientation.y= 0;
    spawn_model_req.initial_pose.orientation.z= 0;
    spawn_model_req.initial_pose.orientation.w= 1.0;

    spawn_model_req.reference_frame = "world";

    gazebo_msgs::SpawnModel spawn;

    // Change to any generated maze matrix and it's dimension

    Eigen::MatrixXf Maze_m(30,30);
    Maze_m  <<  1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1,
                1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0,
                1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0,
                0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1,
                1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0,
                1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0,
                1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1,
                1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1,
                0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1,
                1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1,
                0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1,
                1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1,
                1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0,
                1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0,
                0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1,
                1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1,
                1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1,
                0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1,
                1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1,
                1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1,
                1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0,
                1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1,
                1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1,
                1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1,
                1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1,
                0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1,
                1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1,
                1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1;

    while (ros::ok()){

      ros::Duration(3.0).sleep();  // some time for gazebo init

      for (int i = 0; i < Maze_m.rows(); i++)
      {

        for (int j = 0 ; j < Maze_m.cols(); j++)
        {

          if (Maze_m(i,j) == 1)
          {
            std::string index = intToString(i);
            std::string column = intToString(j);

            std::string model_name;


            spawn_model_req.initial_pose.position.x= 0.5 * (float)(i);
            spawn_model_req.initial_pose.position.y= 0.5 * (float)(j);

            model_name = "box_" + index + "_" + column;    // e.g: box_1_1
            spawn_model_req.model_name = model_name;
            spawn_model_req.robot_namespace = model_name;
            spawn_model_req.model_xml = red_xmlStr;

            bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
            if (call_service) {
                if (spawn_model_resp.success) {
                    // ROS_INFO_STREAM(model_name << " has been spawned");
                    ROS_INFO_STREAM("Block pose: " << i << " ; " << j );
                    continue;
                }
                else {
                  ROS_INFO_STREAM(model_name << " spawn failed, since it's already exist.");
                  ROS_INFO_STREAM("Stop.");
                  return 0; }
            }

            else {
                ROS_INFO("Fail in first call");
                ROS_ERROR("Fail to connect with gazebo server"); }

            ros::Duration(0.3).sleep();  // frequency control

          }
        }
      }

    // ros::spinOnce();
    // ros::Duration(1.0).sleep();  // frequency control
    }

    return 0;
}
