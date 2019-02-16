#include "vel2odom.h"
#include <iostream>
namespace autocar
{
namespace odom_mul
{
vel2odom::vel2odom():x(0.0), y(0.0), th(0.0), vx(0.0), vy(0.0), vth(0.0)
{
}

void vel2odom::cal_odom()
{
  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx*cos(th)-vy*sin(th)) * dt;
  double delta_y = (vx*sin(th)+vy*cos(th)) * dt;
  x += delta_x;
  y += delta_y;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(th);

  std::cout<<"Vx  "<<vx<<std::endl;
  std::cout<<"Vy  "<<vy<<std::endl;
  
  int thetaPeriodCount = (int)(th/2/3.1415926);
  float thetaNormal = th - thetaPeriodCount*(2*3.1415926);
  float thetaNormalDegree = thetaNormal * (180/3.1415926);
  
  std::cout<<"theta   "<<thetaNormalDegree<<std::endl;
  std::cout<<"X  "<<x<<std::endl;
  std::cout<<"Y  "<<y<<std::endl;

}
}
}

