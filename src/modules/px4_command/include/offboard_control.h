#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
class OffboardControl {
 public:
    /**
     *默认构造函数
     */
  OffboardControl(void):
    offboard_nh_("~") {
  mavros_setpoint_pos_pub_ = offboard_nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

}
    void send_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z);
    void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);
  private:
    ros::NodeHandle offboard_nh_;
    ros::Publisher mavros_setpoint_pos_pub_;

};

//发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
void OffboardControl::send_velxy_posz_setpoint(const Eigen::Vector3d& vel_sp, float desire_z)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 1 + 2 + /*4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.position.z = desire_z;

    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void OffboardControl::send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    mavros_setpoint_pos_pub_.publish(pos_setpoint);
}
