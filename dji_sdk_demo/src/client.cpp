#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "dji_sdk_demo/major.h"

using namespace DJI::onboardSDK;

MajorNode* major_node;

int main(int argc, char *argv[])
{
  //初始化过程
  ros::init(argc, argv, "sdk_client");
  ROS_INFO("multi UAV system");
  ros::NodeHandle nh;
  major_node = new MajorNode(nh);

  while(major_node->drone->request_sdk_permission_control() == false)
  {
    ros::spinOnce();
    sleep(2);
  }

  major_node->publish_InitShake();      //发送握手包并等待应答
  major_node->publish_LocalFramAck();   //接收LocalFrame并返回应答
  //计算LocalFrame原点与单机home的坐标之差
  major_node->gps_convert_ned
  ( major_node->delta_posi.x,major_node->delta_posi.y,
    major_node->drone->global_position_ref.longitude,major_node->drone->global_position_ref.latitude,
    major_node->LocalFrame_value.longitude,major_node->LocalFrame_value.latitude
  );
  major_node->wait_newshape();          //等待初次队形接收OK
  ROS_INFO("multi_UAV system initialization finished");

  


  return 0;
}
