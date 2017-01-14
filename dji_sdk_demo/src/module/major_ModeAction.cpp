#include "dji_sdk_demo/major.h"

void MajorNode::TakeOff(void)
{
  update_local_pos_now();

  if(!f_finish)
  {
    float tmp = (local_pos_now.z - TakeOff_value.z)*(local_pos_now.z - TakeOff_value.z);
    if(tmp<0.9)     //距离目标起飞高度小于阈值,认为到达
    {
      local_pos_lock.z = TakeOff_value.z;
      f_finish = 1;
      ROS_INFO("Take off succeed");
    }
    else
      local_pos_lock.z+=0.01;             //缓慢上升
  }
  local_pos_control(local_pos_lock,0);
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
  update_local_pos_now();
  /********计算位置并更新 ****************/
  /*暂时采用先水平移动再垂直运动的会和策略*/
  /*此段代码可用会和算法替代*/
  if(!f_finish)
  {
    //x方向
    float tmp1 = (local_pos_now.x - Meet_TargetPosi.x)*(local_pos_now.x - Meet_TargetPosi.x);
    if(tmp1>0.5)
      local_pos_lock.x = (local_pos_now.x < Meet_TargetPosi.x)?(local_pos_now.x + 0.01):(local_pos_now.x - 0.01);
    else
      local_pos_lock.x = Meet_TargetPosi.x;
    //y方向
    float tmp2 = (local_pos_now.y - Meet_TargetPosi.y)*(local_pos_now.y - Meet_TargetPosi.y);
    if(tmp2>0.5)
      local_pos_lock.y = (local_pos_now.y < Meet_TargetPosi.y)?(local_pos_now.y + 0.01):(local_pos_now.y - 0.01);
    else
      local_pos_lock.y = Meet_TargetPosi.x;
    //z方向
    float tmp =  tmp1 + tmp2;
    if(tmp<1)         //水平方向到达预定位置(小于阈值)
    {
      float tmp3 = (local_pos_now.z - Meet_TargetPosi.z)*(local_pos_now.z - Meet_TargetPosi.z);
      if(tmp3<0.5)     //垂直方向距离差小于阈值
      {
        local_pos_lock.x = Meet_TargetPosi.x;
        local_pos_lock.y = Meet_TargetPosi.y;
        local_pos_lock.z = Meet_TargetPosi.z;
        f_finish = 1;
        ROS_INFO("Meet succeed");
      }
      else
      {
        local_pos_lock.z = (local_pos_now.z < Meet_TargetPosi.z)?(local_pos_now.z + 0.01):(local_pos_now.z - 0.01);
      }
    }
  }
  local_pos_control(local_pos_lock,0);
  /**************************************/
}

void MajorNode::Fly()
{
  int circleRadius;
  int circleHeight;
  float Phi, circleRadiusIncrements;
  int x_center, y_center, yaw_local;

  static float x;
  static float y;
  circleRadius = 15;
  circleHeight = 10;


  if(!f_finish)
  {
    x_center = drone->local_position.x;
    y_center = drone->local_position.y;

    circleRadiusIncrements = 0.01;

    for(int j = 0; j < 750; j ++)
    {
        x =  x_center + circleRadiusIncrements;
        y =  y_center;
        circleRadiusIncrements = circleRadiusIncrements + 0.02;
        drone->local_position_control(x ,y ,circleHeight, 0);
        usleep(10000);
    }

    /* start to draw circle */
    for(int i = 0; i < 1890; i ++)
    {
      x =  x_center + circleRadius*cos((Phi/300));
      y =  y_center + circleRadius*sin((Phi/300));
      Phi = Phi+1;
      drone->local_position_control(x ,y ,circleHeight, 0);
      usleep(10000);
    }

    for(int j = 0; j < 750; j ++)
    {
        x -=  0.02;
        y =  y_center;
        drone->local_position_control(x ,y ,circleHeight, 0);
        usleep(10000);
    }

    f_finish = 1;
    ROS_INFO("Fly succeed");
  }
}

void MajorNode::Return()
{
	if(!f_finish)
	{
		drone->gohome();
		f_finish = 1;
	}
}

void MajorNode::Land(void)
{
  drone->landing();
  fly_mode = Mode_Null;
  f_finish = 0;
}
