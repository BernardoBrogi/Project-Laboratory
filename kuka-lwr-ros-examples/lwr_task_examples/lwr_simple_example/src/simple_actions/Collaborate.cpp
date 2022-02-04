#include "simple_actions/Collaborate.h"
#include "robot_motion_generation/angular_velocity.h"
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>
//#include "gazebo_msgs/ApplyBodyWrench.h"
/*void update_wrench_msg(void )
{
    geometry_msgs::Wrench wrench;
    geometry_msgs::Vector3  force;
    geometry_msgs::Vector3  torque;
    geometry_msgs::Point ref_point;
    force.x=100;
    force.y=0;
    force.z=0;
    torque.x=0;
    torque.y=0;
    torque.z=0;
    wrench.force=force;
    wrench.torque=torque;
    ref_point.x=0;
    ref_point.y=0.5;
    ref_point.z=0;
    wrench_msg.request.body_name= "single_lwr_robot::lwr_7_link";
    wrench_msg.request.wrench=geometry_msgs::Wrench(wrench);
    wrench_msg.request.reference_frame="single_lwr_robot::lwr_7_link";
    wrench_msg.request.duration=ros::Duration(10,0);
    wrench_msg.request.reference_point=ref_point;
    wrench_msg.request.start_time=ros::Time(2);
}*/
namespace simple_actions {



    Collaborate_action::Collaborate_action(ros::NodeHandle& nh):
    Ros_ee_j(nh),
    /*
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");////
    gazebo_msgs::SpawnModel spawn_stick;
    spawn_stick.request.initial_pose=
    spawn_stick.request.model_name=
    spawn_stick.request.reference_frame=
    spawn_stick.request.robot_namespace=
    spawn_stick.request.model_xml=
    if (client.call(spawn_stick))
{
ROS_INFO("SPAWN STICK SUCESS");
ROS_INFO(start_pose.position.x);
}
else
{
ROS_ERROR("Failed to SPAWN STICK");
return 1;
}*/

    switch_controller(nh)
{
    joint_sensor_sub     = nh.subscribe("/lwr/joint_states",10,&Collaborate_action::jStateCallback,this);
    stick_sensor_sub     = nh.subscribe("/stick_wrench_sensor",100,&Collaborate_action::StickCallback,this);
    linear_cddynamics   = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,4) );
    angular_cddynamics  = std::unique_ptr<motion::CDDynamics>( new motion::CDDynamics(3,0.01,1) );
    wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    motion::Vector velLimits(3);
    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 1; // x ms^-1
    }
    linear_cddynamics->SetVelocityLimits(velLimits);

    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 0.04; // x ms^-1
    }
    angular_cddynamics->SetVelocityLimits(velLimits);


    b_run       = false;
    b_position  = false;
        
    bFirst		= false;
    bSwitch		= false;
/*
    //update_wrench_msg(wrench_msg);
    geometry_msgs::Wrench wrench;
    geometry_msgs::Vector3  force;
    geometry_msgs::Vector3  torque;
    geometry_msgs::Point ref_point;
    force.x=100;
    force.y=0;
    force.z=0;
    torque.x=0;
    torque.y=0;
    torque.z=0;
    wrench.force=force;
    wrench.torque=torque;
    ref_point.x=0;
    ref_point.y=0.5;
    ref_point.z=0;
    wrench_msg.request.body_name= "single_lwr_robot::lwr_7_link";
    wrench_msg.request.wrench=geometry_msgs::Wrench(wrench);
    wrench_msg.request.reference_frame="single_lwr_robot::lwr_7_link";
    wrench_msg.request.duration=ros::Duration(10,0);
    wrench_msg.request.reference_point=ref_point;
    wrench_msg.request.start_time=ros::Time(2);
    if (wrench_client.call(wrench_msg))
    {
        ROS_INFO("APPLY WRENCH SUCESS");
        //ROS_INFO(start_pose.position.x);
    }
    else {
        ROS_ERROR("Failed to APPLY WRENCH");
    }
*/
    target_id = 0;
    target_id_tmp = 0;
    dist_target = 0;

    loop_rate_hz = 100;

    joint_sensed.resize(KUKA_NUM_JOINTS);
    joint_sensed.setZero();
}

bool Collaborate_action::update(){

    if(!switch_controller.activate_controller("joint_controllers"))
    {
        ROS_WARN_STREAM("failed to start controller [Joint_action::update()]!");
        return false;
    }
    ros::spinOnce();

    tf::Vector3     current_origin  = ee_pose_current.getOrigin();
    tf::Quaternion  current_orient  = ee_pose_current.getRotation();
    tf::Quaternion  orient=current_orient;
    static tf::TransformBroadcaster br1;
    tf::Transform transform;

    transform.setOrigin(current_origin);
    transform.setRotation(current_orient);
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

    //first_origin        = current_origin+ tf::Vector3(0.1,0.1,0.1);
    target_p1           = first_origin + tf::Vector3(0,0.1,0);
    target_p2           = first_origin - tf::Vector3(0,0.1,0);
    //std::cout<<(first_origin==tf::Vector3(0.3,0.4,0.35))<<std::endl;
    tf::Matrix3x3 tmp1,tmp2;
    double roll, pitch, yaw;


    tmp2.setRPY(M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p1.setRPY(roll,pitch,yaw);

    tmp2.setRPY(M_PI/10,0,0);
    tmp1.setRotation(current_orient);
    tmp1 = tmp2 * tmp1;
    tmp1.getRPY(roll,pitch,yaw);
    target_R_p2.setRPY(roll,pitch,yaw);

    //target_origin       = target_p1;
    //target_orientation  = target_R_p1;

    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    double control_rate = 100;
    ros::Rate loop_rate(control_rate);
    bool success = true;

    static tf::TransformBroadcaster br;

    motion::Vector filter_vel(3);
    filter_vel.setZero();

    linear_cddynamics->SetState(filter_vel);
    linear_cddynamics->SetDt(1.0/control_rate);

    angular_cddynamics->SetState(filter_vel);
    angular_cddynamics->SetDt(1.0/control_rate);


    transform.setOrigin(target_origin);
    transform.setRotation(target_orientation);



    ROS_INFO("starting Linear Action");
    b_run   = true;
    bSwitch = true;
    target_id = 0;
    target_id_tmp = target_id;
    while(b_run) {
       /*if(buffer.empty()){


            ee_vel_msg.linear.x  = 0;
            ee_vel_msg.linear.y  = 0;
            ee_vel_msg.linear.z  = 0;
            ee_vel_msg.angular.x = 0;
            ee_vel_msg.angular.y = 0;
            ee_vel_msg.angular.z = 0;
            sendCartVel(ee_vel_msg);
            std::cout<<"asdhofhaodsifhdoiashfoiuasdhvoiasdhvoisaduasoidahioduv"<<std::endl;
            b_run   = false;


            return success;
        }
        else{*/

            current_origin = ee_pose_current.getOrigin();
            current_orient = ee_pose_current.getRotation();
        //compensate gravity, the others must be command from human
            tf::Vector3 gravity,gravity_relative,g_touque,g_touque_relative;
            gravity=tf::Vector3(0,0,-12.0*9.8);
            g_touque=tf::Vector3(0,0,-9.8);
            gravity_relative=tf::quatRotate(current_orient,gravity);
            g_touque_relative=tf::quatRotate(current_orient,g_touque);
            tf::Vector3 error;
            //error=stick_force-gravity_relative;
            error=stick_force;
            //ROS_INFO_STREAM_THROTTLE(1.0,"raw: " << msg.wrench.force.x<<","<< msg.wrench.force.y<<","<< msg.wrench.force.z<<","<<std::endl;);
            ROS_INFO_STREAM_THROTTLE(1.0,"error: " << error.getX()<<","<< error.getY()<<","<< error.getZ()<<","<<std::endl);
            target_origin=current_origin+error/100.0;

            //ansmp2.setRPY(M_PI/10,0,0);
            //tmp1.setRotation(current_orient);
            //tmp1 = tmp2 * tmp1;
            //tmp1.getRPY(roll,pitch,yaw);
            //target_R_p1.setRPY(roll,pitch,yaw);
            target_orientation  = current_orient;
            transform.setOrigin(target_origin);
            transform.setRotation(target_orientation);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));


            transform.setOrigin(current_origin);
            transform.setRotation(current_orient);
            br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "x_"));

            simple_line_policy(linear_velocity,angular_velocity,current_origin,current_orient,control_rate);

            /// Filter linear velocity
            filter_vel(0) = linear_velocity(0);
            filter_vel(1) = linear_velocity(1);
            filter_vel(2) = linear_velocity(2);

            linear_cddynamics->SetTarget(filter_vel);
            linear_cddynamics->Update();
            linear_cddynamics->GetState(filter_vel);

            ee_vel_msg.linear.x  = filter_vel(0);
            ee_vel_msg.linear.y  = filter_vel(1);
            ee_vel_msg.linear.z  = filter_vel(2);

            /// Filter angular velocity
            filter_vel(0) = angular_velocity(0);
            filter_vel(1) = angular_velocity(1);
            filter_vel(2) = angular_velocity(2);

            angular_cddynamics->SetTarget(filter_vel);
            angular_cddynamics->Update();
            angular_cddynamics->GetState(filter_vel);

            ee_vel_msg.angular.x = angular_velocity(0);
            ee_vel_msg.angular.y = angular_velocity(1);
            ee_vel_msg.angular.z = angular_velocity(2);

            if(b_position)
            {
                if(bSwitch){
                    ros_controller_interface::tf2msg(target_origin,target_orientation,ee_pos_msg);
                    sendCartPose(ee_pos_msg);
                    //std::cout<<"Joint Position: "<<joint_sensed[0]<<" "<<joint_sensed[1]<<" "<<joint_sensed[2]<<" "<<joint_sensed[3]<<" "<<joint_sensed[4]<<" "<<joint_sensed[5]<<" "<<joint_sensed[6]<<std::endl;
                    //ee_pose_current.getOrigin()
                    //std::ofstream outfile;
                    //outfile.open("/home/origin/data/data.txt",std::ios::app);
                    //outfile<<joint_sensed[0]<<" "<<joint_sensed[1]<<" "<<joint_sensed[2]<<" "<<joint_sensed[3]<<" "<<joint_sensed[4]<<" "<<joint_sensed[5]<<" "<<joint_sensed[6]<<"\n";
                    //outfile.close();
                }

            }else{
                sendCartVel(ee_vel_msg);
            }

            ros::spinOnce();
            loop_rate.sleep();
        //}

    }


}

bool Collaborate_action::stop(){
    ee_vel_msg.linear.x  = 0;
    ee_vel_msg.linear.y  = 0;
    ee_vel_msg.linear.z  = 0;
    ee_vel_msg.angular.x = 0;
    ee_vel_msg.angular.y = 0;
    ee_vel_msg.angular.z = 0;
    sendCartVel(ee_vel_msg);
    b_run   = false;
    return true;
}

void Collaborate_action::simple_line_policy(Eigen::Vector3d& linear_velocity,
                                            Eigen::Vector3d& angular_velocity,
                                            const tf::Vector3 &current_origin,
                                            const tf::Quaternion &current_orient,
                                            double rate)
{

   tf::Vector3 velocity = (target_origin - current_origin);
              velocity  = (velocity.normalize()) * 0.05; // 0.05 ms^-1

     linear_velocity(0) = velocity.x();
     linear_velocity(1) = velocity.y();
     linear_velocity(2) = velocity.z();

     tf::Quaternion qdiff =  target_orientation - current_orient;
     Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
     Eigen::Quaternion<double>   q(current_orient.getW(),current_orient.getX(),current_orient.getY(), current_orient.getZ());

     angular_velocity   = motion::d2qw<double>(q,dq);
    dist_target = (current_origin - target_origin).length();
    //ROS_INFO_STREAM_THROTTLE(1.0,"distance: " << dist_target);
    //ROS_INFO_STREAM_THROTTLE(1.0,"Joint Position: "<<joint_sensed[0]<<" "<<joint_sensed[1]<<" "<<joint_sensed[2]<<" "<<joint_sensed[3]<<" "<<joint_sensed[4]<<" "<<joint_sensed[5]<<" "<<joint_sensed[6]);

    /*if((current_origin - target_origin).length() < 0.005)
     {
         ROS_INFO_STREAM_THROTTLE(1.0,"next point"<<target_origin);
        buffer.pop_front();
     }*/


}

    void Collaborate_action::jStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            //std::cout<<"Succ"<<std::endl;
            joint_sensed[i]     = msg->position[i];
        }
        /*
        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            std::cout<<joint_sensed[i];
        }
        std::cout<<std::endl;*/
    }
    void Collaborate_action::StickCallback(const geometry_msgs::WrenchStamped& msg){
        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            //std::cout<<"Succ"<<std::endl

            stick_force.setValue(msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z);
            stick_touque.setValue(msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z);
            //joint_sensed[i]     = msg->position[i];
        }
        /*
        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            std::cout<<joint_sensed[i];
        }
        std::cout<<std::endl;*/
    }
}
