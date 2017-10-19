#include "ros/ros.h"
#include "robotbucks_coffeeplease/PadOrder.h"
#include "robotbucks_coffeeplease/AvailableItemList.h"
#include "robotbucks_coffeeplease/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"

#include <string.h>
#include <sstream>

#define ROBOT_NUMBER_TB3G 1

class ServiceCore
{
public:
  ServiceCore()
  {
    fnInitParam();

    pubServiceStatusPadtb3g = nh_.advertise<std_msgs::String>("/tb3g/service_status", 1);
    pub_is_item_available = nh_.advertise<robotbucks_coffeeplease::AvailableItemList>("/is_item_available", 1);
    pub_play_sound_tb3g = nh_.advertise<std_msgs::String>("/tb3g/play_sound_file", 1);
    pubPoseStampedTb3g = nh_.advertise<geometry_msgs::PoseStamped>("/tb3g/move_base_simple/goal", 1);
    sub_pad_order_tb3g = nh_.subscribe("/tb3g/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
    sub_arrival_status_tb3g = nh_.subscribe("/tb3g/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusTB3G, this);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
      fnPubServiceStatus();

      fnPubPose();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void fnInitParam()
  {

    nh_.getParam("table_pose_tb3g/position", target_pose_position);
    nh_.getParam("table_pose_tb3g/orientation", target_pose_orientation);

    poseStampedTable[1].header.frame_id = "map";
    poseStampedTable[1].header.stamp = ros::Time::now();

    poseStampedTable[1].pose.position.x = target_pose_position[0];
    poseStampedTable[1].pose.position.y = target_pose_position[1];
    poseStampedTable[1].pose.position.z = target_pose_position[2];

    poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_table1/position", target_pose_position);
    nh_.getParam("counter_pose_table1/orientation", target_pose_orientation);

    poseStampedCounter[0].header.frame_id = "map";
    poseStampedCounter[0].header.stamp = ros::Time::now();

    poseStampedCounter[0].pose.position.x = target_pose_position[0];
    poseStampedCounter[0].pose.position.y = target_pose_position[1];
    poseStampedCounter[0].pose.position.z = target_pose_position[2];

    poseStampedCounter[0].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[0].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[0].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[0].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_table2/position", target_pose_position);
    nh_.getParam("counter_pose_table2/orientation", target_pose_orientation);

    poseStampedCounter[1].header.frame_id = "map";
    poseStampedCounter[1].header.stamp = ros::Time::now();

    poseStampedCounter[1].pose.position.x = target_pose_position[0];
    poseStampedCounter[1].pose.position.y = target_pose_position[1];
    poseStampedCounter[1].pose.position.z = target_pose_position[2];

    poseStampedCounter[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[1].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_table3/position", target_pose_position);
    nh_.getParam("counter_pose_table3/orientation", target_pose_orientation);

    poseStampedCounter[2].header.frame_id = "map";
    poseStampedCounter[2].header.stamp = ros::Time::now();

    poseStampedCounter[2].pose.position.x = target_pose_position[0];
    poseStampedCounter[2].pose.position.y = target_pose_position[1];
    poseStampedCounter[2].pose.position.z = target_pose_position[2];

    poseStampedCounter[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[2].pose.orientation.w = target_pose_orientation[3];

  }


  void fnPublishVoiceFilePath(int robot_num, const char* file_path)
  {
    std_msgs::String str;

    str.data = file_path;

    if (robot_num == ROBOT_NUMBER_TB3G)
    {
      pub_play_sound_tb3g.publish(str);
    }
  
  }
  

  void cbCheckArrivalStatusTB3G(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[ROBOT_NUMBER_TB3G] = true;
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusTB3G : %d", rcvMoveBaseActionResult.status.status);
    }
  }


  void fnPubPose()
  {

      if (is_robot_reached_target[ROBOT_NUMBER_TB3G])
      {
        if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 1)
        {
          robot_service_sequence[ROBOT_NUMBER_TB3G] = 2;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 2)
        {
          /* go to table */
          pubPoseStampedTb3g.publish(poseStampedCounter[item_num_chosen_by_pad[ROBOT_NUMBER_TB3G]]);

          is_robot_reached_target[ROBOT_NUMBER_TB3G] = false;

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 3;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 3)
        {
          robot_service_sequence[ROBOT_NUMBER_TB3G] = 4;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 4)
        {
          /* return to turtlebotGREEN */
          pubPoseStampedTb3g.publish(poseStampedTable[ROBOT_NUMBER_TB3G]);

          is_robot_reached_target[ROBOT_NUMBER_TB3G] = false;

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 5;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 5)
        {
          robot_service_sequence[ROBOT_NUMBER_TB3G] = 0;

          is_item_available[item_num_chosen_by_pad[ROBOT_NUMBER_TB3G]] = 1;

          item_num_chosen_by_pad[ROBOT_NUMBER_TB3G] = -1;
        }
      }

  }

  void cbReceivePadOrder(const std_msgs::String padOrder)
  {
    std::string str = padOrder.data;
    std::string delimiter = ",";

    size_t pos = 0;

    int num = 0;
    int input_numbers[2] = {-2, -2};

    while ((pos = str.find(delimiter)) != std::string::npos)
    {
        input_numbers[num] = atoi(str.substr(0, pos).c_str());
        str.erase(0, pos + delimiter.length());

        num++;
    }

    input_numbers[num] = atoi(str.substr(0, str.size()).c_str());

    int pad_number = input_numbers[0];
    int item_number = input_numbers[1];

    if (is_item_available[item_number] != 1)
    {
      ROS_INFO("Chosen item is currently unavailable");
      return;
    }

    if (robot_service_sequence[pad_number] != 0)
    {
      ROS_INFO("Your TurtleBot is currently on servicing");
      return;
    }

    if (item_num_chosen_by_pad[pad_number] != -1)
    {
      ROS_INFO("Your TurtleBot is currently on servicing");
      return;
    }

    item_num_chosen_by_pad[pad_number] = item_number;

    robot_service_sequence[pad_number] = 1; // just left from the table

    is_item_available[item_number] = 0;

  }

  void fnPubServiceStatus()
  {

      std::string str;
      std_msgs::String serviceStatus;
      std::ostringstream oss;

      oss << item_num_chosen_by_pad[0] << "," << item_num_chosen_by_pad[1] << "," << item_num_chosen_by_pad[2] << ",";
      oss << is_item_available[0] << "," << is_item_available[1] << "," << is_item_available[2] << ",";
      oss << robot_service_sequence[1];

      str = oss.str();
      serviceStatus.data = str;
      pubServiceStatusPadtb3p.publish(serviceStatus);
      pubServiceStatusPadtb3g.publish(serviceStatus);
      pubServiceStatusPadtb3r.publish(serviceStatus);

  }

  
private:

  ros::NodeHandle nh_;

  
  // Publisher
  // turtlebot r,g,p
  ros::Publisher pubServiceStatusPadtb3g;
  ros::Publisher pub_is_item_available;

  ros::Publisher pub_play_sound_tb3g;
  
  ros::Publisher pubPoseStampedTb3g;


  // Subscriber
  ros::Subscriber sub_pad_order_tb3g;

  ros::Subscriber sub_arrival_status_tb3g;

  // msgs
  geometry_msgs::PoseStamped poseStampedTable[1];
  geometry_msgs::PoseStamped poseStampedCounter[3];

  std::vector<double> target_pose_position;
  std::vector<double> target_pose_orientation;

  //pad로 부터 선택 된 item_num [0][1][2] 원래는 bread, waffle, drink인데 우리는 table1,2,3임
  boost::array<int, 3> item_num_chosen_by_pad = { {-1, -1, -1} }; 
  //가능한 item 갯수, table1,2,3에 해당하는 주문 가능 음료 갯수,100=무한대로 봄
  boost::array<int, 3> is_item_available = { {100, 100, 100} };
  //로봇 서비스 상태, 0이면 서비스 끝났다는 뜻, 상태는 1,2,3,4,5가 있고 이에 해당하는 voice가 있음
  boost::array<int, 1> robot_service_sequence = { {0} };
  //robot이 목적지에 도착했을 때, true
  bool is_robot_reached_target[3] = {true, true, true};
};

int main(int argc, char **argv)
{
  /* initiate ROS */
  ros::init(argc, argv, "service_core");

  /* create an object of class ServiceCore that will take care of everything */
  ServiceCore serviceCore;

  ros::spin();

  return 0;
}
