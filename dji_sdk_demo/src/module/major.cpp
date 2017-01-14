#include "dji_sdk_demo/major.h"
#include <zigbee/MsgCode.h>


const string target_ID_string[10]={"zigbee_target_ID0",
															"zigbee_target_ID1",
															"zigbee_target_ID2",
															"zigbee_target_ID3",
															"zigbee_target_ID4",
															"zigbee_target_ID5",
															"zigbee_target_ID6",
															"zigbee_target_ID7",
															"zigbee_target_ID8",
															"zigbee_target_ID9"

};

MajorNode::MajorNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  delta_posi.x = 0;
  delta_posi.y = 0;
  delta_posi.z = 0;
/***************这一部分和zigbee node 重复，暂时就这样吧****************************/
  nh_private.param("zigbee_own_ID",shape_manage.OwnID, 0);
  nh_private.param("zigbee_target_ID_num",shape_manage.target_ID_num, 0);
  int tmp;
  for(int i = 0;i<shape_manage.target_ID_num;i++)
	{
		nh_private.param(target_ID_string[i],tmp, i);
    shape_manage.target_ID[i] = (unsigned char)tmp;
	}
/******************************************************/
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
  NoArguCmd_sub = nh.subscribe<Ack>("GS/NoArguCmd",10,&MajorNode::NoArguCmd_sub_callback,this);
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
  if(0== tmp.j && f_updating_count == 0)        //开始存储队形数据
  {
    shape_buf.clear();                         			//清除缓冲区
    shape_buf.total_num = tmp.totol_uav_num;
    shape_buf.lead_id = tmp.i;
    shape_buf.uavID_serial[0] = tmp.i;
    f_updating_count++;

		if(shape_buf.total_num == 1)													//集群就一架飞机,正确则该飞机就是头机
		{
			if(shape_buf.lead_id == shape_manage.OwnID)					//判断一下是不是本机ID
			{
				f_updating_count = 0;
	      shape_buf.f_NewShapeOk = 1;
	      Ack_pub.publish(ack_tmp);
			}
			else              																	//error
		  {
		    shape_buf.clear();
		    f_updating_count = 0;
		  }
		}
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
/******************指令更改模式代码段***************************************/
void MajorNode::TakeOff_sub_callback(Posi tmp)
{
  if(fly_mode == Mode_Null)
  {
    TakeOff_value = tmp;
    fly_mode = Mode_TakeOff;
    f_finish = 0;                 //清flag
    update_local_pos_now();
    local_pos_lock.x = local_pos_now.x;
    local_pos_lock.y = local_pos_now.y;
    local_pos_lock.z = local_pos_now.z + 1;
		drone->drone_arm();											//电机起转
		ROS_INFO("Enter TakeOff mode");
  }
}

void MajorNode::Meet_sub_callback(ShapeConfig tmp)
{
  if(fly_mode == Mode_TakeOff && f_finish == 1)     //起飞完成方可进入会和模式
  {
    f_finish = 0;
    fly_mode = Mode_Meet;
    Meet_value.x = tmp.x;
    Meet_value.y = tmp.y;
    Meet_value.x = tmp.z;
    Meet_value.fi = tmp.fi;
		ROS_INFO("Enter Meet mode");
  }

}

void MajorNode::NoArguCmd_sub_callback(Ack tmp)
{
  switch(tmp.msgID)
  {
    case msgID_Fly:
    //  if(fly_mode == Mode_Meet && f_finish == 1)
			if(fly_mode == Mode_TakeOff && f_finish == 1)
      {
				ROS_INFO("Enter Fly mode");
        fly_mode = Mode_Fly;
        f_finish = 0;
      }
      else if(fly_mode == Mode_Fly)       //本来就是飞行模式
      {
        //检查队形缓冲是否有新队形
        //如果有则装载并置重构标志有效
        //否则重构标志无效，--->保持原有队形飞
      }
      break;
    case msgID_Stop:
			ROS_INFO("Enter Stop mode");
      fly_mode = Mode_Stop;
      f_finish = 0;
      update_local_pos_now();
      local_pos_lock = local_pos_now;
      break;
    case msgID_Return:
      if((fly_mode == Mode_Stop)||(fly_mode == Mode_Fly && f_finish == 1))
      {
				ROS_INFO("Enter Return mode");
        f_finish = 0;
        fly_mode = Mode_Return;
      }
      break;
    case msgID_Land:
      if((fly_mode == Mode_Stop)||(fly_mode == Mode_Return && f_finish == 1))
      {
				ROS_INFO("Enter Land mode");
        fly_mode = Mode_Land;
        f_finish = 0;
      }
      break;
    default:break;
  }
}
/********************************************************************************************/
void MajorNode::publish_InitShake(void)
{
  GPS tmp;
	//没有GPS则一直等待
	while(drone->global_position_ref.latitude==0)
	{
		sleep(2);
		ros::spinOnce();
	}
  tmp.latitude = drone->global_position_ref.latitude;
  tmp.longitude = drone->global_position_ref.longitude;
  tmp.altitude = drone->global_position_ref.altitude;

	ROS_INFO("GPS has been getted!");
	cout<<"global_position_ref:"<<tmp.latitude<<" "<<tmp.longitude<<endl;

  do
  {
		InitShake_pub.publish(tmp);
    sleep(2);
    ros::spinOnce();
  }while(f_InitShakeAck==0);
	f_InitShakeAck = 0;

	ROS_INFO("InitShake succeed!");
}

void MajorNode::publish_LocalFramAck(void)
{
  Ack tmp;
  tmp.msgID = 0x42;tmp.targetID = 0x00;

  do{
    ros::spinOnce();
    sleep(0.5);
  }while(f_LocalFrame == 0);

	ROS_INFO("LocalFrame succeed!");
	cout<<"delta_posi:"<<delta_posi.x<<" "<<delta_posi.y<<" "<<delta_posi.z<<endl;
	cout<<"LocalFrame_value:"<<LocalFrame_value.longitude<<" "<<LocalFrame_value.latitude<<endl;

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


//将飞机的局部坐标系坐标转换到多机LocalFrame下
void MajorNode::update_local_pos_now(void)
{
 local_pos_now.x = drone->local_position.x + delta_posi.x;
 local_pos_now.y = drone->local_position.y + delta_posi.y;
 local_pos_now.z = drone->local_position.z + delta_posi.z;
}

//由多机LocalFrame下坐标计算在本机的局部坐标系下坐标
Posi MajorNode::count_OwnLocal_position(Posi tmp)
{
  Posi ret;
  ret.x = tmp.x - delta_posi.x;
  ret.y = tmp.y - delta_posi.y;
  ret.z = tmp.z - delta_posi.z;
  return ret;
}

void MajorNode::local_pos_control(Posi tmp,float fi)
{
  Posi tmp1 = count_OwnLocal_position(tmp);   //坐标转换
  drone->local_position_control(tmp1.x,tmp1.y,tmp1.z,fi);
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
