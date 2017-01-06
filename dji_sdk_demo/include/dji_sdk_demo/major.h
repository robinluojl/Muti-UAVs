#ifndef _MAJOR_H
#define _MAJOR_H

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <math.h>
#include <iostream>
#include <memory.h>


#include "zigbee/GPS.h"
#include "std_msgs/Empty.h"
#include "zigbee/ShapeConfig.h"
#include "zigbee/Posi.h"
#include "zigbee/Ack.h"
#include "zigbee/Attitude.h"

using namespace std;
using namespace zigbee;

#define C_EARTH (double) 6378137.0   //meter:radius of earth
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

typedef struct SHAPE_MATRIX
{
  float x;
  float y;
  float z;
  float fi;
}SHAPE_MATRIX;

class SHAPE{
public:
  int total_num = 0;
  int lead_id = 0;
  int uavID_serial[10];         //记录
  SHAPE_MATRIX shape_matrix[10][10];

  void clear(void)
  {
    int total_num = 0;
    int lead_id = 0;
    for(int i=0;i<10;i++)
    {
      uavID_serial[i]=0;
    }
    memset(shape_matrix,0,sizeof(shape_matrix));
  }
};

class MajorNode
{
public:
  DJIDrone* drone;

  SHAPE shape_temp;             //队形信息二级缓冲

  //flag
  int f_InitShakeAck;
  int f_LocalFrame;
  int f_NewShapeOk;

public:
  //value
  Posi delta_posi;

  GPS LocalFrame_value;
	ShapeConfig ShapeConfig_value;
	Posi TakeOff_value;
	Ack NoArguCmd_value;

	// Posi OtherPosi_value;
	// Posi OtherVel_value;
	// Attitude OtherAttitude_value;
  //Subscriber
  ros::Subscriber InitShakeAck_sub;
	ros::Subscriber LocalFrame_sub;
	ros::Subscriber ShapeConfig_sub;
	ros::Subscriber TakeOff_sub;
	ros::Subscriber NoArguCmd_sub;

	// ros::Subscriber OtherPosi_sub;
	// ros::Subscriber OtherVel_sub;
	// ros::Subscriber OtherAttitude_sub;
  //callback
public:
  void InitShakeAck_sub_callback(std_msgs::Empty tmp);
  void LocalFrame_sub_callback(GPS tmp);
  void ShapeConfig_sub_callback(ShapeConfig tmp);
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



public:
  void publish_InitShake(void);
  void publish_LocalFramAck(void);
  void wait_newshape(void);
  void gps_convert_ned(float &ned_x, float &ned_y,
  double gps_t_lon, double gps_t_lat,
  double gps_r_lon, double gps_r_lat);

  MajorNode(ros::NodeHandle& nh);
  ~MajorNode();
};

#endif
