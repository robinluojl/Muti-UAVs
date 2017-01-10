#ifndef _SHAPE_H
#define _SHAPE_H

class SHAPE_MATRIX
{
public:
  float x;
  float y;
  float z;
  float fi;

  SHAPE_MATRIX operator+(SHAPE_MATRIX other)
  {
    SHAPE_MATRIX tmp;
    tmp.x = this->x + other.x;
    tmp.y = this->y + other.y;
    tmp.z = this->z + other.z;
    tmp.fi = this->fi + other.fi;
    return tmp;
  }

  SHAPE_MATRIX operator-(SHAPE_MATRIX other)
  {
    SHAPE_MATRIX tmp;
    tmp.x = this->x - other.x;
    tmp.y = this->y - other.y;
    tmp.z = this->z - other.z;
    tmp.fi = this->fi - other.fi;
    return tmp;
  }
};

class SHAPE{
public:
  int total_num = 0;
  int lead_id = 0;
  int uavID_serial[10];         //记录
  SHAPE_MATRIX shape_matrix[10][10];

  int f_ShapeMsgUsed = 0;     //本组队形数据是否被装载过？
  int f_NewShapeOk = 0;       //本组队形数据是否有效

  void clear(void)
  {
    int total_num = 0;
    int lead_id = 0;
    for(int i=0;i<10;i++)
    {
      uavID_serial[i]=0;
    }
    memset(shape_matrix,0,sizeof(shape_matrix));
    f_ShapeMsgUsed = 0;
    f_NewShapeOk = 0;
  }

  // void MajorNode::wait_newshape(void)
  // {
  //   while(shape_buf.f_NewShapeOk == 0)
  //   {
  //     ros::spinOnce();
  //     sleep(0.5);
  //   }
  // }
};

class SHAPE_MANAGE{
public:
  SHAPE shape_real;                        //当前的有效队形信息

  int OwnID;                                   //自己的ID
  int target_ID_num;						            	 //需要几架飞机信息？
	unsigned char target_ID[10];		             //需要哪几架飞机的ID？

  SHAPE_MATRIX IdealDelta_with_leader;         //与头机的理想位置差
  SHAPE_MATRIX IdealDelta_with_neighbors[10];  //与众邻居的理想位置差

  void Update(void)
  {
    //向量正方向定义为lead指向follower 例如：x13向量代表1为头机，3为从机
    if(OwnID !=shape_real.lead_id)                        //如果本机不是头机
    {
      //更新本机与头机 以及邻居飞机的理想位置向量
      IdealDelta_with_leader = shape_real.shape_matrix[shape_real.lead_id][OwnID];
      for(int i=0;i++;i<target_ID_num){
        //邻居们为头机，本机为从机，计算位置向量
        IdealDelta_with_neighbors[i] = IdealDelta_with_leader - shape_real.shape_matrix[shape_real.lead_id][target_ID[i]] ;
      }
    }
  }
};

#endif
