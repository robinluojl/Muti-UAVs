#include "dji_sdk_demo/major.h"

MajorNode::MajorNode(ros::NodeHandle& nh)
{
  drone = new DJIDrone(nh);
  f_InitShakeAck = 0;
  f_LocalFrame = 0;
  delta_posi.x = 0;delta_posi.y = 0;delta_posi.z = 0;

  init_subscriber(nh);
  init_publisher(nh);
}

MajorNode::~MajorNode()
{
  delete drone;
}

void MajorNode::init_subscriber(ros::NodeHandle& nh)
{
  InitShakeAck_sub = nh.subscribe<std_msgs::Empty>("GS/InitShakeAck",10,&MajorNode::InitShakeAck_sub_callback,this);
  LocalFrame_sub = nh.subscribe<GPS>("GS/LocalFrame",10,&MajorNode::LocalFrame_sub_callback,this);
}

void MajorNode::init_publisher(ros::NodeHandle& nh)
{
  InitShake_pub = nh.advertise<GPS>("UAV/InitShake",10);
  Ack_pub = nh.advertise<Ack>("UAV/Ack",10);
}


void MajorNode::InitShakeAck_sub_callback(std_msgs::Empty tmp)
{
  f_InitShakeAck = 1;
}

void MajorNode::LocalFrame_sub_callback(GPS tmp)
{
  LocalFrame_value = tmp;
  f_LocalFrame = 1;
}

void MajorNode::publish_InitShake(void)
{
  GPS tmp;

  tmp.latitude = drone->global_position_ref.latitude;
  tmp.longitude = drone->global_position_ref.longitude;
  tmp.altitude = drone->global_position_ref.altitude;

  do
  {
    InitShake_pub.publish(tmp);
    sleep(2);
    ros::spinOnce();
  }while(f_InitShakeAck==0);

  f_InitShakeAck = 0;
}

void MajorNode::publish_LocalFramAck(void)
{
  Ack tmp;
  tmp.msgID = 0x42;tmp.targetID = 0x00;

  do{
    ros::spinOnce();
    sleep(2);
  }while(f_LocalFrame == 0);

  Ack_pub.publish(tmp);
  f_LocalFrame = 0;
}

void MajorNode::gps_convert_ned(float &ned_x, float &ned_y,
  double gps_t_lon, double gps_t_lat,
  double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
	double d_lat = gps_t_lat - gps_r_lat;
	ned_x = DEG2RAD(d_lat) * C_EARTH;
	ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}
