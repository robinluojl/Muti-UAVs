#ifndef _MAJOR_H
#define _MAJOR_H

#include <ros/ros.h>

#include <math.h>
#include <iostream>
#include <memory.h>



#include "zigbee/GPS.h"
#include "std_msgs/Empty.h"
#include "zigbee/ShapeConfig.h"
#include "zigbee/Posi.h"
#include "zigbee/Ack.h"
#include "zigbee/Attitude.h"

#include "dji_sdk_demo/user_type.h"
#include "dji_sdk_demo/shape.h"

using namespace std;
using namespace zigbee;

#define C_EARTH (double) 6378137.0   //meter:radius of earth
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))


class MajorNode
{
public:
  Posi delta_posi;
  FLY_MODE fly_mode = Mode_Null;              //模式
  Posi local_pos_now;                         //本机在LocalFrame下的坐标，使用之前一定要调用更新函数
  Posi local_pos_lock;                        //要锁定的位置 x\y\z

  SHAPE shape_buf;                            //队形信息临时缓冲
  SHAPE_MANAGE shape_manage;                  //当前队形管理类，包含队形数据以及操作函数等
  SHAPE_MATRIX Meet_TargetPosi;               //本机的目标会和点

  //flag
  int f_InitShakeAck = 0;
  int f_LocalFrame = 0;
  int f_MeetOK = 0;


public:
  //value
  GPS LocalFrame_value;
	ShapeConfig ShapeConfig_value;
	Posi TakeOff_value;
  SHAPE_MATRIX Meet_value;
//	Ack NoArguCmd_value;

	// Posi OtherPosi_value;
	// Posi OtherVel_value;
	// Attitude OtherAttitude_value;
  //Subscriber
  ros::Subscriber InitShakeAck_sub;
	ros::Subscriber LocalFrame_sub;
	ros::Subscriber ShapeConfig_sub;
	ros::Subscriber TakeOff_sub;
  ros::Subscriber Meet_sub;
	ros::Subscriber NoArguCmd_sub;

	// ros::Subscriber OtherPosi_sub;
	// ros::Subscriber OtherVel_sub;
	// ros::Subscriber OtherAttitude_sub;
  //callback
public:
  void InitShakeAck_sub_callback(std_msgs::Empty tmp);
  void LocalFrame_sub_callback(GPS tmp);
  void ShapeConfig_sub_callback(ShapeConfig tmp);
  void TakeOff_sub_callback(Posi tmp);
  void Meet_sub_callback(ShapeConfig tmp);
  void NoArguCmd_sub_callback(Ack tmp);
/*********************************************/
private:
  ros::Publisher InitShake_pub;
	ros::Publisher Ack_pub;
	//对外广播
	// ros::Publisher OwnPosi_pub;			//本机的局部位置（统一）
	// ros::Publisher OwnVel_pub;				//本机速度
	// ros::Publisher	OwnAttitude_pub;	//本机的姿态

private:
  void init_publisher(ros::NodeHandle& nh);
  void init_subscriber(ros::NodeHandle& nh);

  void gps_convert_ned(float &ned_x, float &ned_y,
  double gps_t_lon, double gps_t_lat,
  double gps_r_lon, double gps_r_lat);

public:
  void publish_InitShake(void);
  void publish_LocalFramAck(void);
  void wait_newshape(void);
  //动作函数
  void TakeOff(void);
  void Meet(void);

//将飞机的局部坐标系坐标转换到多机LocalFrame下
  void update_local_pos_now(void);
//由多机LocalFrame下坐标计算在本机的局部坐标系下坐标
  Posi count_OwnLocal_position(Posi tmp);
  //通过多机局部坐标系下坐标控制飞机位置
  void local_pos_control(Posi tmp,float fi);

  MajorNode(ros::NodeHandle& nh);
  ~MajorNode();
};

#endif
