/***************************************************************************************************************************
*
* Author: bingo
* Email: 1554459957@qq.com
* Time: 2019.10.14
* Description: lidar collision v1.0
*  
***************************************************************************************************************************/

//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>


//topic 头文件
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
Eigen::Vector3d pos_drone;                                      //无人机当前位置 (来自fcu)
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
float target_z;                                                 //期望位置_z
float target_temp_x;
float target_temp_y;
//--------------------------------------------算法相关--------------------------------------------------
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
//--------------------------------------------输出--------------------------------------------------
Eigen::Vector3d vel_sp_body;                                           //总速度
float vel_sp_max;                                               //总速度限幅
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float barriar_threshols = 3; //设置判断是否为障碍物距离
float pi = 3.1415926;
typedef struct
{
	int amount; 				//障碍物数量
	float distan_min;			//据某个障碍物最短距离
	float direc_start;			//障碍物起始方向
	float distan_start;			//障碍物起始距离 
	float direc_stop;			//障碍物结束方向	
	float distan_stop;			//障碍物结束距离
	float size; 				//障碍物大小
	float frame_x;				//障碍物相当飞机的坐标
	float frame_y;
	float world_x;				//世界坐标系ENU
	float world_y;				
	float distan_min_vert_range; //某个障碍物最短距离点据最短路经的垂直距离
	float distan_min_away_target;//某个障碍物据目标点的距离
}S_BARRIER;
int barrier_num = 0;
int barrier_obj = 0;
float uav2targetDistan;
S_BARRIER s_barrier[10];
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    	Laser = *scan;
    	int count = 359; 
	float distan_min_direc = 0;   
	for(int i=0; i<= barrier_num; i++)
	{
		s_barrier[i].distan_min = 0;
 		s_barrier[i].direc_start = 0;
		s_barrier[i].direc_stop = 0;
 		s_barrier[i].distan_start = 0;
		s_barrier[i].distan_stop = 0;
		s_barrier[i].frame_x = 0;
		s_barrier[i].frame_y = 0;
		s_barrier[i].world_x = 0;
		s_barrier[i].world_y = 0;
		s_barrier[i].distan_min_vert_range = 0;
		s_barrier[i].size = 0;
		
	}
	barrier_num = 0;
	/*检测障碍物*/
    for(int temp = 0; temp <= count; temp++)
    {
		if(isinf(Laser.ranges[temp]))
		{
			
		}
		else if(Laser.ranges[temp] <= barriar_threshols)
		{
				barrier_num++;
				s_barrier[barrier_num].direc_start = temp;
				s_barrier[barrier_num].distan_min = Laser.ranges[temp];
				s_barrier[barrier_num].distan_start = Laser.ranges[temp];
				distan_min_direc = temp;
			while(1)
			{
				//cout << "Laser.ranges[temp] : " << Laser.ranges[temp] << endl;
				temp++;
				if(s_barrier[barrier_num].distan_min > Laser.ranges[temp])
				{
					s_barrier[barrier_num].distan_min = Laser.ranges[temp];
					distan_min_direc = temp;
					//cout << "s_barrier[barrier_num].distan_min : " << s_barrier[barrier_num].distan_min << endl;
				}
				if(Laser.ranges[temp] > barriar_threshols || temp >= 359)
				{
					s_barrier[barrier_num].direc_stop = temp;
					s_barrier[barrier_num].distan_stop = Laser.ranges[temp-1];
					//余弦定理
					s_barrier[barrier_num].size =  sqrt(s_barrier[barrier_num].distan_start*s_barrier[barrier_num].distan_start+s_barrier[barrier_num].distan_stop*s_barrier[barrier_num].distan_stop-2*s_barrier[barrier_num].distan_stop*s_barrier[barrier_num].distan_start*cos((s_barrier[barrier_num].direc_stop- s_barrier[barrier_num].direc_start)*pi/180));
					s_barrier[barrier_num].frame_x = -s_barrier[barrier_num].distan_min * cos(distan_min_direc/180*3.1415926);
					s_barrier[barrier_num].frame_y = -s_barrier[barrier_num].distan_min * sin(distan_min_direc/180*3.1415926);
					s_barrier[barrier_num].world_x = s_barrier[barrier_num].frame_x + pos_drone[0];
					s_barrier[barrier_num].world_y = s_barrier[barrier_num].frame_y + pos_drone[1];
					//点到直线距离公式
					float k = (pos_drone[1] - target_y)/(pos_drone[0] - target_x);
					s_barrier[barrier_num].distan_min_vert_range = (s_barrier[barrier_num].world_y-k*s_barrier[barrier_num].world_x+k*target_x-target_y)/sqrt(1+k*k);
					//勾股定理
					s_barrier[barrier_num].distan_min_away_target = sqrt((s_barrier[barrier_num].world_x-target_x)*(s_barrier[barrier_num].world_x-target_x)+(s_barrier[barrier_num].world_y-target_y)*(s_barrier[barrier_num].world_y-target_y));
					break;
				}
			}
		}
		else
		{
			
		}
	
    }	
	/*避障策略*/
	if(barrier_num >0)
	{	
		//cout << "barrier_num : " << barrier_num  <<endl;
		float barrier_distan_min = 1;
		int barrier_temp = 0;
		for(int temp=1;temp<=barrier_num;temp++)
		{
			//cout << "barrier_num : " << barrier_num  <<endl;
			uav2targetDistan = sqrt((target_x - pos_drone[0])*(target_x - pos_drone[0]) + (target_y - pos_drone[1])*(target_y - pos_drone[1]));//飞机与目标点之间的距离
			if(s_barrier[temp].distan_min_vert_range < 0.5 && s_barrier[temp].distan_min_vert_range > -0.5 && s_barrier[temp].distan_min_away_target < uav2targetDistan)//找到满足条件的障碍物点
			{
				// cout << "temp : " << temp <<endl;
				if(barrier_distan_min > s_barrier[temp].distan_min)//寻找离飞机最近的障碍物，前提要障碍物据飞机与目标点直线距离不小于1m,而且障碍物在飞机的前方。
				{
					barrier_distan_min = s_barrier[temp].distan_min;
					barrier_temp = temp;
   					cout << "barrier_temp : " << barrier_temp <<endl;
				}
				
			}

		}
		if(barrier_temp > 0)
		{
 			if (s_barrier[barrier_temp].distan_min_vert_range > 0)			//正负代表障碍物在uav到目标点的直线的哪一侧
			{
				target_temp_x =  s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
			}
			else 
			{
				target_temp_x = s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
			}
			vel_sp_body[0] = 1 * (target_temp_x - pos_drone[0]);
			vel_sp_body[1] = 1 * (target_temp_y - pos_drone[1]);	
			cout << "target_temp_x : " << target_temp_x << " m "<<endl;
    			cout << "target_temp_y : " << target_temp_y << " m "<<endl;		
		}
		else
		{
			vel_sp_body[0] = p_xy * (target_x - pos_drone[0]);
			vel_sp_body[1] = p_xy * (target_x - pos_drone[1]);
		}
	}
	else
	{
		vel_sp_body[0] = p_xy * (target_x - pos_drone[0]);
		vel_sp_body[1] = p_xy * (target_y - pos_drone[1]);
	}
	
	cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>s_barrier<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
	if(barrier_num > 0)
	{
		for(int i=1;i<=barrier_num;i++)
		{
			cout << "s_barrier_number : " << i << endl;
			cout <<	"s_barrier_distan_min : " << s_barrier[i].distan_min << endl;
			cout <<	"s_barrier_direc_start : " << s_barrier[i].direc_start << endl;
			cout <<	"s_barrier_direc_stop: " << s_barrier[i].direc_stop << endl;
			cout <<	"s_barrier_distan_stop: " << s_barrier[i].distan_stop << endl;
			cout <<	"s_barrier_distan_start: " << s_barrier[i].distan_start << endl;
			cout <<	"s_barrier_size: " << s_barrier[i].size << endl;
			cout <<	"s_barrier_frame_x: " << s_barrier[i].frame_x << endl;
			cout <<	"s_barrier_frame_y: " << s_barrier[i].frame_y << endl;
			cout <<	"s_barrier_world_x: " << s_barrier[i].world_x << endl;
			cout <<	"s_barrier_world_y: " << s_barrier[i].world_y << endl;
			cout <<	"s_barrier_distan_min_vert_range: " << s_barrier[i].distan_min_vert_range << endl;
			cout <<	"s_barrier_distan_min_away_target: " << s_barrier[i].distan_min_away_target << endl;
		}
	}

}


void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    float p_temp_xy;
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);


    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0);
    nh.param<float>("target_y", target_y, 1.0);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    //打印现实检查参数
    printf_param();
    int check_flag;
 
    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;
    //输出指令初始化
    int comid = 1;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

        ros::spinOnce();
       for (int i = 0; i < 2; i++)
	{
	    vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
	}
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp_body[0];
    pos_setpoint.velocity.y = vel_sp_body[1];
    pos_setpoint.velocity.z = 0;

//    pos_setpoint.yaw = 1.5;

    setpoint_raw_local_pub.publish(pos_setpoint);
     //   printf();
        rate.sleep();

    }

    return 0;

}

//饱和函数
float satfunc(float data, float Max)
{
    if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

void printf()
{

    cout <<  "uav2targetDistan :" << uav2targetDistan << "m" <<endl;
    cout << "vel_sp_x : " << vel_sp_body[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_body[1] << " [m/s] "<<endl;

}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;

    cout << "p_xy : "<< p_xy << endl;

    cout << "vel_sp_max : "<< vel_sp_max << endl;

}


