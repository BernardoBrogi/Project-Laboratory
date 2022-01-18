#include <ros/ros.h>
#include "lwr_ros_client/kuka_action_console.h"


int main(int argc, char** argv)
{

    ros::init(argc, argv,"action_console");
    ros::NodeHandle nh("action_console_node");

    ac::Action_client_console client_console(nh);
    //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("cart_pos", 1000);
    //ros::Rate loop_rate(10);
    client_console.AddConsoleCommand("go_home");
    client_console.AddConsoleCommand("go_front");
    client_console.AddConsoleCommand("go_left");
    client_console.AddConsoleCommand("linear");
    client_console.AddConsoleCommand("candle");
    client_console.AddConsoleCommand("Record");
    client_console.AddConsoleCommand("Replay");
    client_console.AddConsoleCommand("LfD");


    client_console.start();

      ros::Rate rate(50);
      while(ros::ok()){

          client_console.ConsoleUpdate();
          //chatter_pub.publish()
          rate.sleep();
          ros::spinOnce();
      }

    return 0;
}
