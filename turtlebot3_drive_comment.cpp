// turtlebot3_drive.cpp 코드 공부
// 코드 출처 : https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/src

/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

// turtlebot 센서가 받은 데이터를 turtlebot3_drive.cpp가 subscribe
// cpp는 봇이 어디로 가야하는지 계산해서 봇한테 다시 publish

#include "turtlebot3_gazebo/turtlebot3_drive.h"
#include <iostream>

using namespace std;


Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
// “cmd_vel_topic_name” param으로 받아온 값 저장 ( cmd_vel_topic_name = “cmd_vel” 
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");  //cmd_vel

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers - “cmd_vel"으로 publisher 생성
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
// topic “scan”으로 받아온 msg 저장 + callback 함수 호출 	
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
		
// topic “odom”으로 받아온 msg 저장 + callback함수 호출
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

  tb3_pose_ = atan2(siny, cosy); // turtlebot의 각도 구해서 저장, atan2는 파라미터로 음수 허용, -pi~pi 라디안 값 리턴
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
// scan_angle의 0 , 30, 330 ⇒ 정면, 왼쪽, 오른쪽 각도 의미
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
// msg -> ranges( ) ⇒  sensor_msgs::LaserScan::ranges - 특정 각도에서 측정된 거리를 msg 데이터에서 읽어옴
// isinf ⇒ 해당 데이터가 무한대인지 판단, 무한대가 아니면 0, 무한대면 0이 아닌 값 리턴
    if (std::isinf(msg->ranges.at(scan_angle[num]))) // 거리가 무한대일 경우에는 최댓값(limit) 저장
    {
      scan_data_[num] = msg->range_max;
    }
    else // 측정값 저장
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

 // 업데이트한 내역 publish
  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:

    // print scan_data (center, left, right)
    // maximum distance is 3.5
      for (int i = 0; i < 3; i++){
        cout << scan_data_[i] << ' ';
      }      
      cout << endl;
      
	// 로봇이 가야하는 방향 정함
	// 현재 각도 업데이트, 상태 업데이트
      if (scan_data_[CENTER] > check_forward_dist_) // 정면에는 거리가 많이 남은 경우
      {
        if (scan_data_[LEFT] < check_side_dist_) // 왼쪽에 여유가 없는 경우 오른쪽으로 턴
        {	  
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_) // 오른쪽에 여유가 없는 경우 왼쪽으로 턴
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
	 // 그냥 정면 전진
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

	 // 정면 여유가 없는 경우 오른쪽으로 턴(계속 각도 오른쪽으로 돌려도 막힌 경우 계속 돌려서 뒤로 돌거임)
      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

	 // 속도, 각도 업데이트
    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_) // fabs : 
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
