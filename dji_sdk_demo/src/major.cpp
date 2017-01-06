#include "dji_sdk_demo/major.h"

MajorNode::MajorNode(ros::NodeHandle& nh)
{
  drone = new DJIDrone(nh);
  f_InitShakeAck = 0;
  f_LocalFrame = 0;
  f_NewShapeOk = 0;
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
  ShapeConfig_sub = nh.subscribe<ShapeConfig>("GS/ShapeConfig",10,&MajorNode::ShapeConfig_sub_callback,this);
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

void MajorNode::ShapeConfig_sub_callback(ShapeConfig tmp)
{
  Ack ack_tmp;ack_tmp.msgID = 0x43;ack_tmp.targetID = 0x00;     //ShapeConfigAck

  static int f_updating_count = 0;
  if(tmp.i== tmp.j && f_updating_count == 0)        //开始存储队形数据
  {
    shape_temp.clear();                         //清除缓冲区
    shape_temp.total_num = tmp.totol_uav_num;
    shape_temp.lead_id = tmp.i;
    shape_temp.uavID_serial[0] = tmp.i;
    f_updating_count++;
  }
  else if(f_updating_count!=0 && tmp.i != tmp.j)
  {
    shape_temp.uavID_serial[f_updating_count] = tmp.j;

    shape_temp.shape_matrix[tmp.i][tmp.j].x = tmp.x;
    shape_temp.shape_matrix[tmp.i][tmp.j].y = tmp.y;
    shape_temp.shape_matrix[tmp.i][tmp.j].z = tmp.z;
    shape_temp.shape_matrix[tmp.i][tmp.j].fi = tmp.fi;

    f_updating_count++;
    //判断计数是否达到,数组收全
    if(f_updating_count>=shape_temp.total_num)
    {
      /*检查队形数据的完整性*/
      f_updating_count = 0;
      f_NewShapeOk = 1;
      Ack_pub.publish(ack_tmp);
    }
  }
  else              //error
  {
    shape_temp.clear();
    f_updating_count = 0;
  }

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
    sleep(0.5);
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
    sleep(0.5);
  }while(f_LocalFrame == 0);

  Ack_pub.publish(tmp);
  f_LocalFrame = 0;
}

void MajorNode::wait_newshape(void)
{
  while(f_NewShapeOk == 0)
  {
    ros::spinOnce();
    sleep(0.5);
  }
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
