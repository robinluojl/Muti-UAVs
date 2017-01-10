#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "dji_sdk_demo/major.h"
#include <pthread.h>

using namespace DJI::onboardSDK;

MajorNode* major_node;
DJIDrone* drone;

static void* Thread_1(void* param)
{
  MajorNode* p_tmp = (MajorNode* )param;
  while(true)
  {
    switch(p_tmp->fly_mode)
    {
      case Mode_TakeOff:
        p_tmp->TakeOff();
      break;
      case Mode_Meet:
        p_tmp->Meet();
      break;
      case Mode_Stop:
        //锁定在固定位置
        //drone->local_position_control(major_node->local_pos_lock.x ,major_node->local_pos_lock.y ,major_node->local_pos_lock.z, major_node->local_pos_lock.fi);
      break;
      default:
      break;
    }
    usleep(20000);
  }
}

int main(int argc, char *argv[])
{
  pthread_t m_Tid; int ret;

  //初始化过程
  ros::init(argc, argv, "sdk_client");
  ROS_INFO("multi UAV system");
  ros::NodeHandle nh;
  major_node = new MajorNode(nh);
  drone = new DJIDrone(nh);

  while(drone->request_sdk_permission_control() == false)
  {
    ros::spinOnce();
    sleep(2);
  }

  major_node->publish_InitShake();      //发送握手包并等待应答
  major_node->publish_LocalFramAck();   //接收LocalFrame并返回应答，更新单机局部坐标系原点与多机局部坐标系原点的差值 delta_posi
  major_node->wait_newshape();          //等待初次队形接收OK
  ROS_INFO("multi_UAV system initialization finished");

  //创建线程
  ret = pthread_create(&m_Tid, 0,Thread_1,(void*)major_node);
  //四个线程用于callback调度
  ros::AsyncSpinner spinner(4); // Use 4 threads for callback
	spinner.start();
	ros::waitForShutdown();

  //销毁线程

  delete major_node;
  delete drone;
  return 0;
}
