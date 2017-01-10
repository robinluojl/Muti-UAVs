#include "dji_sdk_demo/major.h"
#include <dji_sdk/dji_drone.h>
#include "../../zigbee/include//zigbee/MsgCode.h"
//#include "zigbee/MsgCode.h"

extern DJIDrone* drone;

MajorNode::MajorNode(ros::NodeHandle& nh)
{
  delta_posi.x = 0;
  delta_posi.y = 0;
  delta_posi.z = 0;

  init_subscriber(nh);
  init_publisher(nh);
}

MajorNode::~MajorNode()
{

}

void MajorNode::init_subscriber(ros::NodeHandle& nh)
{
  InitShakeAck_sub = nh.subscribe<std_msgs::Empty>("GS/InitShakeAck",10,&MajorNode::InitShakeAck_sub_callback,this);
  LocalFrame_sub = nh.subscribe<GPS>("GS/LocalFrame",10,&MajorNode::LocalFrame_sub_callback,this);
  ShapeConfig_sub = nh.subscribe<ShapeConfig>("GS/ShapeConfig",10,&MajorNode::ShapeConfig_sub_callback,this);
  TakeOff_sub = nh.subscribe<Posi>("GS/TakeOff",10,&MajorNode::TakeOff_sub_callback,this);
  Meet_sub = nh.subscribe<ShapeConfig>("GS/Meet",10,&MajorNode::Meet_sub_callback,this);
  NoArguCmd_sub = nh.subscribe<Ack>("GS/NoArgueCmd",10,&MajorNode::NoArguCmd_sub_callback,this);
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
  if(f_LocalFrame == 0)
  {
    LocalFrame_value = tmp;
    //计算LocalFrame原点与单机home的坐标之差
    gps_convert_ned
    ( delta_posi.x,delta_posi.y,
      drone->global_position_ref.longitude,drone->global_position_ref.latitude,
      LocalFrame_value.longitude,LocalFrame_value.latitude
    );
    f_LocalFrame = 1;                                                             //LocalFrame更新过一次之后就上锁，不需要再次更新了
  }

}

void MajorNode::ShapeConfig_sub_callback(ShapeConfig tmp)
{
  Ack ack_tmp;ack_tmp.msgID = 0x43;ack_tmp.targetID = 0x00;     //ShapeConfigAck

  static int f_updating_count = 0;
  if(tmp.i== tmp.j && f_updating_count == 0)        //开始存储队形数据
  {
    shape_buf.clear();                         //清除缓冲区
    shape_buf.total_num = tmp.totol_uav_num;
    shape_buf.lead_id = tmp.i;
    shape_buf.uavID_serial[0] = tmp.i;
    f_updating_count++;
  }
  else if(f_updating_count!=0 && tmp.i != tmp.j)
  {
    shape_buf.uavID_serial[f_updating_count] = tmp.j;

    shape_buf.shape_matrix[tmp.i][tmp.j].x = tmp.x;
    shape_buf.shape_matrix[tmp.i][tmp.j].y = tmp.y;
    shape_buf.shape_matrix[tmp.i][tmp.j].z = tmp.z;
    shape_buf.shape_matrix[tmp.i][tmp.j].fi = tmp.fi;

    f_updating_count++;
    //判断计数是否达到,数组收全
    if(f_updating_count>=shape_buf.total_num)
    {
      /*检查队形数据的完整性*/
      f_updating_count = 0;
      shape_buf.f_NewShapeOk = 1;
      Ack_pub.publish(ack_tmp);
    }
  }
  else              //error
  {
    shape_buf.clear();
    f_updating_count = 0;
  }
}

void MajorNode::TakeOff_sub_callback(Posi tmp)
{
  TakeOff_value = tmp;
  if(fly_mode == Mode_Null)fly_mode = Mode_TakeOff;
}

void MajorNode::Meet_sub_callback(ShapeConfig tmp)
{
  Meet_value.x = tmp.x;
  Meet_value.y = tmp.y;
  Meet_value.x = tmp.z;
  Meet_value.fi = tmp.fi;
  fly_mode = (fly_mode == Mode_TakeOff)?Mode_Meet:fly_mode;
}

void MajorNode::NoArguCmd_sub_callback(Ack tmp)
{
  switch(tmp.msgID)
  {
    case msgID_Fly:
      fly_mode = (fly_mode == Mode_Meet)?Mode_Fly:fly_mode;
      break;
    case msgID_Stop:
      fly_mode = Mode_Stop;
      //使用当前位置更新local_pos_lock
      break;
    case msgID_Return:
      fly_mode = (fly_mode == Mode_Stop)?Mode_Return:fly_mode;
      break;
    case msgID_Land:
      fly_mode = (fly_mode == Mode_Stop||fly_mode == Mode_Return)?Mode_Land:fly_mode;
      break;
    default:break;
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
  while(shape_buf.f_NewShapeOk == 0)
  {
    ros::spinOnce();
    sleep(0.5);
  }
}

//从当前位置起飞到一定高度
void MajorNode::TakeOff(void)
{
  static int f_TmpForTakeoff = 0;       //强制stop时，清除该标志存在bug------bug
  float x,y,z;
  if(f_TmpForTakeoff==0)                //记录初始位置
  {
    x = drone->local_position.x;
    y = drone->local_position.y;
    z = drone->local_position.z;
    f_TmpForTakeoff = 1;
  }
  //计算飞机位置并更新

  // //1m/s速度
  // int k = abs(TakeOff_value.z - drone->local_position.z)/0.02;
  //
  // for(int i=0;i<k;i++){
  // z = (z_tmp < TakeOff_value.z)?(z+0.02):(z-0.02);
  // drone->local_position_control(x ,y ,z, 0);
  // usleep(20000);
  // }

  float tmp = (local_pos_now.z - TakeOff_value.z)*(local_pos_now.z - TakeOff_value.z);
  if(tmp<0.5)
  {
    local_pos_lock.x = x;
    local_pos_lock.y = y;
    local_pos_lock.x = TakeOff_value.z;
    fly_mode = Mode_Stop;
    f_TmpForTakeoff = 0;
  }
}

//静态集结的功能函数
void MajorNode::Meet(void)
{
  //队形数据OK且未使用过，则首先进行队形的装载和处理
  if(shape_buf.f_ShapeMsgUsed == 0 && shape_buf.f_NewShapeOk == 1)              //如果在meet的过程中又来了新的队形，实际上会立即生效了----bug
  {
    shape_buf.f_ShapeMsgUsed = 1;
    shape_manage.shape_real = shape_buf;
    shape_manage.Update();
    //（计算）本机的理想汇合点
    Meet_TargetPosi = (shape_manage.OwnID == shape_manage.shape_real.lead_id)?Meet_value:(Meet_value + shape_manage.IdealDelta_with_leader);
  }
  //计算位置并更新

  //判断是否到达meet位置
  float tmp = (local_pos_now.x - Meet_TargetPosi.x)*(local_pos_now.x - Meet_TargetPosi.x) +
              (local_pos_now.y - Meet_TargetPosi.y)*(local_pos_now.y - Meet_TargetPosi.y) +
              (local_pos_now.z - Meet_TargetPosi.z)*(local_pos_now.z - Meet_TargetPosi.z);
  if(tmp<=0.5)      //距离小于一定阈值认为到达
  {
    local_pos_lock = Meet_TargetPosi;
    fly_mode = Mode_Stop;                   //退出Meet模式
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
